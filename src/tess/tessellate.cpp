// SPDX-License-Identifier: GPL-3.0-or-later
#include "tess/tessellate.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <Poly_Polygon3D.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "core/profile.hpp"
#include "kernel/internal/progress.hpp"
#include "kernel/occt_access.hpp"

namespace iw::tess {
namespace {

Error cancelledError() { return Error{ErrorCode::Cancelled, "tessellate cancelled"}; }

Error occtError(const Standard_Failure& e) {
    return Error{ErrorCode::KernelFailure,
                 std::string("tessellate: ") + (e.GetMessageString() ? e.GetMessageString() : "OCCT exception")};
}

/// Normalises (x, y, z) in place. Returns false when the vector is numerically zero (left untouched).
bool normaliseInPlace(double& x, double& y, double& z) {
    const double len = std::sqrt(x * x + y * y + z * z);
    if (!(len > 1e-12)) return false;
    x /= len;
    y /= len;
    z /= len;
    return true;
}

} // namespace

Result<MeshData> tessellate(const kernel::Shape& shape, const TessParams& params, CancelToken cancel) {
    IW_ZONE("tess::tessellate");
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "tessellate: null shape");
    if (!(params.linearDeflection > 0) || !(params.angularDeflection > 0))
        return makeError(ErrorCode::InvalidArgument, "tessellate: deflections must be > 0");
    if (cancel.cancelled()) return std::unexpected(cancelledError());

    try {
        // Stage 1: private copy. Meshing writes triangulations into the TShapes, so the immutable input stays clean.
        TopoDS_Shape work;
        {
            IW_ZONE("tess::copy");
            BRepBuilderAPI_Copy copy(kernel::occt::get(shape), /*copyGeom=*/Standard_False,
                                     /*copyMesh=*/Standard_False);
            work = copy.Shape();
        }
        if (cancel.cancelled()) return std::unexpected(cancelledError());

        // Stage 2: parallel incremental meshing, observing the cancel token through the progress indicator.
        {
            IW_ZONE("tess::mesh");
            Handle(Message_ProgressIndicator) pi = new kernel::CancelProgress(cancel);
            Message_ProgressScope scope(pi->Start(), "tess", 1);
            IMeshTools_Parameters meshParams;
            meshParams.Deflection = params.linearDeflection;
            meshParams.Angle = params.angularDeflection;
            meshParams.Relative = params.relative ? Standard_True : Standard_False;
            meshParams.InParallel = Standard_True;
            BRepMesh_IncrementalMesh mesh(work, meshParams, scope.Next());
        }
        if (cancel.cancelled()) return std::unexpected(cancelledError());

        // Sub-shape indices: same depth-first ordering contract as kernel::Shape (see kernel/shape.hpp).
        TopTools_IndexedMapOfShape faceMap;
        TopTools_IndexedMapOfShape edgeMap;
        TopExp::MapShapes(work, TopAbs_FACE, faceMap);
        TopExp::MapShapes(work, TopAbs_EDGE, edgeMap);
        if (static_cast<std::uint32_t>(faceMap.Extent()) != shape.subShapeCount(kernel::SubShapeKind::Face) ||
            static_cast<std::uint32_t>(edgeMap.Extent()) != shape.subShapeCount(kernel::SubShapeKind::Edge))
            return makeError(ErrorCode::KernelFailure, "tessellate: sub-shape count mismatch after copy");

        MeshData out;

        // Stage 3: per-face triangles, normals and face ranges. Face i (1-based in OCCT) -> faceRanges[i-1].
        {
            IW_ZONE("tess::faces");
            out.faceRanges.resize(static_cast<std::size_t>(faceMap.Extent()));
            for (Standard_Integer i = 1; i <= faceMap.Extent(); ++i) {
                if (cancel.cancelled()) return std::unexpected(cancelledError());
                MeshData::FaceRange& range = out.faceRanges[static_cast<std::size_t>(i - 1)];
                range.firstIndex = static_cast<std::uint32_t>(out.indices.size());
                range.indexCount = 0;

                const TopoDS_Face face = TopoDS::Face(faceMap(i));
                TopLoc_Location loc;
                Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
                if (tri.IsNull()) continue; // failed to mesh: empty range

                // Triangulation normals come from the surface (D1U x D1V). A REVERSED face or a mirroring
                // location flips the outward direction; mirroring also reverses the triangle winding.
                BRepLib_ToolTriangulatedShape::ComputeNormals(face, tri);
                const gp_Trsf trsf = loc.Transformation();
                const bool reversed = face.Orientation() == TopAbs_REVERSED;
                const bool flipWinding = reversed != trsf.IsNegative();

                const Standard_Integer nNodes = tri->NbNodes();
                const std::size_t base = out.positions.size() / 3;
                std::vector<gp_Pnt> pts;
                pts.reserve(static_cast<std::size_t>(nNodes));
                for (Standard_Integer k = 1; k <= nNodes; ++k) {
                    const gp_Pnt p = tri->Node(k).Transformed(trsf);
                    pts.push_back(p);
                    out.bounds.extend(iw::Vec3{p.X(), p.Y(), p.Z()});
                    out.positions.push_back(static_cast<float>(p.X()));
                    out.positions.push_back(static_cast<float>(p.Y()));
                    out.positions.push_back(static_cast<float>(p.Z()));
                }

                // Accumulated geometric normals: fallback for nodes where the surface normal is degenerate (poles).
                std::vector<gp_Vec> accum(static_cast<std::size_t>(nNodes), gp_Vec(0.0, 0.0, 0.0));
                const Standard_Integer nTris = tri->NbTriangles();
                for (Standard_Integer t = 1; t <= nTris; ++t) {
                    Standard_Integer a = 0, b = 0, c = 0;
                    tri->Triangle(t).Get(a, b, c);
                    if (flipWinding) std::swap(b, c);
                    out.indices.push_back(static_cast<std::uint32_t>(base + static_cast<std::size_t>(a - 1)));
                    out.indices.push_back(static_cast<std::uint32_t>(base + static_cast<std::size_t>(b - 1)));
                    out.indices.push_back(static_cast<std::uint32_t>(base + static_cast<std::size_t>(c - 1)));
                    const gp_Vec fn = gp_Vec(pts[static_cast<std::size_t>(a - 1)], pts[static_cast<std::size_t>(b - 1)])
                                          .Crossed(gp_Vec(pts[static_cast<std::size_t>(a - 1)],
                                                          pts[static_cast<std::size_t>(c - 1)]));
                    accum[static_cast<std::size_t>(a - 1)] += fn;
                    accum[static_cast<std::size_t>(b - 1)] += fn;
                    accum[static_cast<std::size_t>(c - 1)] += fn;
                }

                for (Standard_Integer k = 1; k <= nNodes; ++k) {
                    gp_Vec v(0.0, 0.0, 0.0);
                    if (tri->HasNormals()) {
                        v = gp_Vec(tri->Normal(k)).Transformed(trsf);
                        if (reversed) v.Reverse();
                    }
                    double x = v.X(), y = v.Y(), z = v.Z();
                    if (!normaliseInPlace(x, y, z)) {
                        const gp_Vec& g = accum[static_cast<std::size_t>(k - 1)];
                        x = g.X();
                        y = g.Y();
                        z = g.Z();
                        if (!normaliseInPlace(x, y, z)) {
                            x = 0.0;
                            y = 0.0;
                            z = 1.0; // degenerate node with no triangle normal at all; keeps the unit-length invariant
                        }
                    }
                    out.normals.push_back(static_cast<float>(x));
                    out.normals.push_back(static_cast<float>(y));
                    out.normals.push_back(static_cast<float>(z));
                }
                range.indexCount = static_cast<std::uint32_t>(out.indices.size()) - range.firstIndex;
            }
        }
        if (cancel.cancelled()) return std::unexpected(cancelledError());

        // Stage 4: edge polylines. Edge i (1-based in OCCT) -> edgePolylines[i-1]. Uses the mesher's 3D polygon when
        // present, otherwise a tangential-deflection discretisation of the edge curve.
        {
            IW_ZONE("tess::edges");
            double absLinear = params.linearDeflection;
            if (params.relative) {
                const Box3 b = shape.boundingBox();
                const double dx = b.max.x - b.min.x, dy = b.max.y - b.min.y, dz = b.max.z - b.min.z;
                absLinear = params.linearDeflection * std::sqrt(dx * dx + dy * dy + dz * dz);
            }
            out.edgePolylines.resize(static_cast<std::size_t>(edgeMap.Extent()));
            for (Standard_Integer i = 1; i <= edgeMap.Extent(); ++i) {
                if (cancel.cancelled()) return std::unexpected(cancelledError());
                MeshData::EdgePolyline& poly = out.edgePolylines[static_cast<std::size_t>(i - 1)];
                const TopoDS_Edge edge = TopoDS::Edge(edgeMap(i));
                if (BRep_Tool::Degenerated(edge)) continue;

                TopLoc_Location eloc;
                Handle(Poly_Polygon3D) p3 = BRep_Tool::Polygon3D(edge, eloc);
                if (!p3.IsNull()) {
                    const gp_Trsf t = eloc.Transformation();
                    const auto& nodes = p3->Nodes();
                    for (Standard_Integer k = nodes.Lower(); k <= nodes.Upper(); ++k) {
                        const gp_Pnt p = nodes(k).Transformed(t);
                        poly.points.push_back(static_cast<float>(p.X()));
                        poly.points.push_back(static_cast<float>(p.Y()));
                        poly.points.push_back(static_cast<float>(p.Z()));
                    }
                } else {
                    BRepAdaptor_Curve curve(edge);
                    GCPnts_TangentialDeflection discret(curve, params.angularDeflection, absLinear);
                    for (Standard_Integer k = 1; k <= discret.NbPoints(); ++k) {
                        const gp_Pnt p = discret.Value(k);
                        poly.points.push_back(static_cast<float>(p.X()));
                        poly.points.push_back(static_cast<float>(p.Y()));
                        poly.points.push_back(static_cast<float>(p.Z()));
                    }
                }
            }
        }
        if (cancel.cancelled()) return std::unexpected(cancelledError());

        return out;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string("tessellate: ") + e.what());
    }
}

} // namespace iw::tess
