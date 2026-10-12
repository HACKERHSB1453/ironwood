// SPDX-License-Identifier: GPL-3.0-or-later
#include "kernel/ops.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include <BOPAlgo_Options.hxx>
#include <BRepAlgoAPI_BooleanOperation.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Message_ProgressRange.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include "kernel/internal/profile_support.hpp"
#include "kernel/internal/progress.hpp"
#include "kernel/internal/shape_impl.hpp"

namespace iw::kernel {
namespace {

bool finitePositive(double v) { return std::isfinite(v) && v > 0.0; }
bool finiteVec(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

Error occtError(const Standard_Failure& e, const char* what) {
    return Error{ErrorCode::KernelFailure, std::string(what) + ": " + (e.GetMessageString() ? e.GetMessageString() : "OCCT exception")};
}

/// Collapses compounds that wrap exactly one child (BOP always returns a compound) so that e.g. a fused
/// pair of touching boxes is reported as a Solid. Empty compounds stay empty (caller rejects them).
TopoDS_Shape unwrapSingle(TopoDS_Shape s) {
    while (!s.IsNull() && s.ShapeType() == TopAbs_COMPOUND) {
        TopoDS_Iterator it(s); // cumulates orientation and location
        if (!it.More()) break;
        TopoDS_Shape child = it.Value();
        it.Next();
        if (it.More()) break;
        s = child;
    }
    return s;
}

bool hasSolid(const TopoDS_Shape& s) {
    if (s.IsNull()) return false;
    if (s.ShapeType() == TopAbs_SOLID) return true;
    if (s.ShapeType() != TopAbs_COMPOUND && s.ShapeType() != TopAbs_COMPSOLID) return false;
    for (TopoDS_Iterator it(s); it.More(); it.Next())
        if (hasSolid(it.Value())) return true;
    return false;
}

bool addOutputs(const TopTools_ListOfShape& list, const SubShapeIndex& out, std::vector<OutputSubShape>& dst) {
    bool any = false;
    for (TopTools_ListIteratorOfListOfShape it(list); it.More(); it.Next()) {
        const TopoDS_Shape& s = it.Value();
        SubShapeKind kind;
        switch (s.ShapeType()) {
        case TopAbs_FACE: kind = SubShapeKind::Face; break;
        case TopAbs_EDGE: kind = SubShapeKind::Edge; break;
        case TopAbs_VERTEX: kind = SubShapeKind::Vertex; break;
        default: continue; // wires/shells/solids are not addressable
        }
        const int idx = out.map(kind).FindIndex(s);
        if (idx == 0) continue; // produced but not part of the final result
        dst.push_back({kind, static_cast<std::uint32_t>(idx - 1)});
        any = true;
    }
    return any;
}

void sortUnique(std::vector<OutputSubShape>& v) {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
}

constexpr SubShapeKind kKinds[] = {SubShapeKind::Face, SubShapeKind::Edge, SubShapeKind::Vertex};

/// Extracts a ShapeHistory from a finished BOP / BRepBuilderAPI_MakeShape for the given operands.
/// Entries are keyed by (operand, kind, index-in-input); output indices are sorted ascending.
ShapeHistory extractHistory(BRepBuilderAPI_MakeShape& algo, const std::vector<const Shape*>& inputs, const SubShapeIndex& out) {
    ShapeHistory h;
    for (std::uint32_t op = 0; op < inputs.size(); ++op) {
        const SubShapeIndex& in = *inputs[op]->impl()->index;
        for (SubShapeKind kind : kKinds) {
            const auto& map = in.map(kind);
            for (int i = 1; i <= map.Extent(); ++i) {
                const TopoDS_Shape& sub = map.FindKey(i);
                const InputSubShape key{op, kind, static_cast<std::uint32_t>(i - 1)};
                bool gen = false, mod = false;
                std::vector<OutputSubShape> g, m;
                gen = addOutputs(algo.Generated(sub), out, g);
                mod = addOutputs(algo.Modified(sub), out, m);
                if (gen) { sortUnique(g); h.generated[key] = std::move(g); }
                if (mod) { sortUnique(m); h.modified[key] = std::move(m); }
                if (mod) continue;
                const int same = out.map(kind).FindIndex(sub);
                if (same != 0) {
                    h.unchanged[key] = {kind, static_cast<std::uint32_t>(same - 1)};
                } else if (!gen || algo.IsDeleted(sub)) {
                    h.deleted.insert(key);
                }
            }
        }
    }
    return h;
}

template <class Op>
Result<ShapeResult> boolean(const Shape& a, const Shape& b, CancelToken cancel, const char* name) {
    if (a.isNull() || b.isNull()) return makeError(ErrorCode::InvalidArgument, std::string(name) + ": null operand");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, std::string(name) + " cancelled");
    try {
        TopTools_ListOfShape args, tools;
        args.Append(occtShape(a));
        tools.Append(occtShape(b));
        Op op;
        op.SetArguments(args);
        op.SetTools(tools);
        op.SetRunParallel(false);      // serial: results feed naming/caching and must be bit-for-bit repeatable
        op.SetNonDestructive(true);    // Shape is immutable; BOP must not touch input tolerances
        op.SetToFillHistory(true);
        op.SetFuzzyValue(0.0);
        Handle(Message_ProgressIndicator) pi = new CancelProgress(cancel);
        op.Build(pi->Start());
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, std::string(name) + " cancelled");
        if (!op.IsDone() || op.HasErrors()) return makeError(ErrorCode::KernelFailure, std::string(name) + ": boolean operation failed");
        TopoDS_Shape res = unwrapSingle(op.Shape());
        if (res.IsNull() || !hasSolid(res)) return makeError(ErrorCode::KernelFailure, std::string(name) + ": result is empty");
        ShapeResult r;
        r.shape = wrapShape(res);
        r.history = extractHistory(op, {&a, &b}, *r.shape.impl()->index);
        return r;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, name));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string(name) + ": " + e.what());
    }
}

} // namespace

Result<ShapeResult> makeBox(const Vec3& origin, double dx, double dy, double dz) {
    if (!finiteVec(origin) || !finitePositive(dx) || !finitePositive(dy) || !finitePositive(dz))
        return makeError(ErrorCode::InvalidArgument, "makeBox: extents must be finite and > 0");
    try {
        BRepPrimAPI_MakeBox mk(gp_Pnt(origin.x, origin.y, origin.z), dx, dy, dz);
        mk.Build();
        if (!mk.IsDone()) return makeError(ErrorCode::KernelFailure, "makeBox: kernel failure");
        return ShapeResult{wrapShape(mk.Shape()), {}};
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "makeBox"));
    }
}

Result<ShapeResult> makeCylinder(const Vec3& origin, const Vec3& axis, double radius, double height) {
    if (!finiteVec(origin) || !finiteVec(axis) || length(axis) < 1e-12 || !finitePositive(radius) || !finitePositive(height))
        return makeError(ErrorCode::InvalidArgument, "makeCylinder: need non-zero axis, radius > 0, height > 0");
    try {
        const gp_Ax2 ax(gp_Pnt(origin.x, origin.y, origin.z), gp_Dir(axis.x, axis.y, axis.z));
        BRepPrimAPI_MakeCylinder mk(ax, radius, height);
        mk.Build();
        if (!mk.IsDone()) return makeError(ErrorCode::KernelFailure, "makeCylinder: kernel failure");
        return ShapeResult{wrapShape(mk.Shape()), {}};
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "makeCylinder"));
    }
}

Result<ShapeResult> makeSphere(const Vec3& center, double radius) {
    if (!finiteVec(center) || !finitePositive(radius))
        return makeError(ErrorCode::InvalidArgument, "makeSphere: need finite centre and radius > 0");
    try {
        const gp_Ax2 ax(gp_Pnt(center.x, center.y, center.z), gp_Dir(0.0, 0.0, 1.0));
        BRepPrimAPI_MakeSphere mk(ax, radius);
        mk.Build();
        if (!mk.IsDone()) return makeError(ErrorCode::KernelFailure, "makeSphere: kernel failure");
        return ShapeResult{wrapShape(mk.Shape()), {}};
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "makeSphere"));
    }
}

Result<ShapeResult> extrudeProfile(const Shape& profile, const Vec3& vector, CancelToken cancel) {
    if (profile.isNull()) return makeError(ErrorCode::InvalidArgument, "extrudeProfile: null profile");
    if (!finiteVec(vector) || length(vector) < 1e-9)
        return makeError(ErrorCode::InvalidArgument, "extrudeProfile: vector must be finite and non-zero");
    if (profile.type() != ShapeType::Face && profile.type() != ShapeType::Wire)
        return makeError(ErrorCode::InvalidArgument, "extrudeProfile: profile must be a planar Face or closed planar Wire");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "extrudeProfile cancelled");
    try {
        TopoDS_Shape in = occtShape(profile);
        TopoDS_Face face;
        if (profile.type() == ShapeType::Face) {
            face = TopoDS::Face(in);
        } else {
            const TopoDS_Wire wire = TopoDS::Wire(in);
            if (!BRep_Tool::IsClosed(wire)) return makeError(ErrorCode::InvalidArgument, "extrudeProfile: wire is not closed");
            BRepBuilderAPI_MakeFace mf(wire, /*OnlyPlane=*/Standard_True);
            if (!mf.IsDone()) return makeError(ErrorCode::InvalidArgument, "extrudeProfile: wire is not planar");
            face = mf.Face();
        }
        BRepPrimAPI_MakePrism prism(face, gp_Vec(vector.x, vector.y, vector.z), /*Copy=*/Standard_False);
        prism.Build();
        if (!prism.IsDone()) return makeError(ErrorCode::KernelFailure, "extrudeProfile: prism failed");
        TopoDS_Shape res = prism.Shape();
        if (res.IsNull() || res.ShapeType() != TopAbs_SOLID)
            return makeError(ErrorCode::KernelFailure, "extrudeProfile: did not produce a solid");
        // Degenerate sweep (vector parallel to the profile plane) yields zero volume.
        GProp_GProps props;
        BRepGProp::VolumeProperties(res, props);
        if (std::abs(props.Mass()) < 1e-12)
            return makeError(ErrorCode::InvalidArgument, "extrudeProfile: vector lies in the profile plane");

        ShapeResult r;
        r.shape = wrapShape(res);
        const SubShapeIndex& out = *r.shape.impl()->index;
        const SubShapeIndex& pin = *profile.impl()->index;
        ShapeHistory& h = r.history;

        // Edges / vertices of the profile: generated side faces / side edges (query the prism with the sub-shapes
        // of the FACE actually swept; they are IsSame to the caller's wire edges, so indices are looked up on `profile`).
        for (SubShapeKind kind : {SubShapeKind::Edge, SubShapeKind::Vertex}) {
            const auto& map = pin.map(kind);
            for (int i = 1; i <= map.Extent(); ++i) {
                const TopoDS_Shape& sub = map.FindKey(i);
                const InputSubShape key{0, kind, static_cast<std::uint32_t>(i - 1)};
                std::vector<OutputSubShape> g;
                if (addOutputs(prism.Generated(sub), out, g)) {
                    sortUnique(g);
                    h.generated[key] = std::move(g);
                }
                const int same = out.map(kind).FindIndex(sub);
                if (same != 0) h.unchanged[key] = {kind, static_cast<std::uint32_t>(same - 1)}; // start-cap boundary
            }
        }
        // Face profile: start cap is the (re-used) profile face -> modified; end cap -> generated.
        if (profile.type() == ShapeType::Face) {
            const InputSubShape key{0, SubShapeKind::Face, 0};
            std::vector<OutputSubShape> first, last;
            TopTools_ListOfShape l1, l2;
            l1.Append(prism.FirstShape());
            l2.Append(prism.LastShape());
            if (addOutputs(l1, out, first)) h.modified[key] = std::move(first);
            if (addOutputs(l2, out, last)) h.generated[key] = std::move(last);
        }
        return r;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "extrudeProfile"));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string("extrudeProfile: ") + e.what());
    }
}

Result<ShapeResult> fuse(const Shape& a, const Shape& b, CancelToken cancel) {
    return boolean<BRepAlgoAPI_Fuse>(a, b, std::move(cancel), "fuse");
}
Result<ShapeResult> cut(const Shape& a, const Shape& b, CancelToken cancel) {
    return boolean<BRepAlgoAPI_Cut>(a, b, std::move(cancel), "cut");
}
Result<ShapeResult> common(const Shape& a, const Shape& b, CancelToken cancel) {
    return boolean<BRepAlgoAPI_Common>(a, b, std::move(cancel), "common");
}

Result<Shape> makePolygonWire(std::span<const Vec3> points) {
    if (points.size() < 3) return makeError(ErrorCode::InvalidArgument, "makePolygonWire: need >= 3 points");
    try {
        BRepBuilderAPI_MakePolygon poly;
        for (const Vec3& p : points) {
            if (!finiteVec(p)) return makeError(ErrorCode::InvalidArgument, "makePolygonWire: non-finite point");
            poly.Add(gp_Pnt(p.x, p.y, p.z));
        }
        poly.Close();
        if (!poly.IsDone()) return makeError(ErrorCode::InvalidArgument, "makePolygonWire: degenerate polygon");
        return wrapShape(poly.Wire());
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "makePolygonWire"));
    }
}

Result<Shape> makePolygonFace(std::span<const Vec3> points) {
    auto w = makePolygonWire(points);
    if (!w) return w;
    try {
        BRepBuilderAPI_MakeFace mf(TopoDS::Wire(occtShape(*w)), Standard_True);
        if (!mf.IsDone()) return makeError(ErrorCode::InvalidArgument, "makePolygonFace: polygon is not planar");
        return wrapShape(mf.Face());
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "makePolygonFace"));
    }
}

Result<MassProperties> massProperties(const Shape& shape) {
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "massProperties: null shape");
    try {
        const TopoDS_Shape s = occtShape(shape);
        GProp_GProps vol, surf;
        BRepGProp::VolumeProperties(s, vol);
        BRepGProp::SurfaceProperties(s, surf);
        MassProperties mp;
        mp.volume = vol.Mass();
        mp.surfaceArea = surf.Mass();
        const gp_Pnt c = vol.CentreOfMass();
        mp.centerOfMass = {c.X(), c.Y(), c.Z()};
        const gp_Mat m = vol.MatrixOfInertia(); // about the centre of mass, unit density
        for (int r = 0; r < 3; ++r)
            for (int col = 0; col < 3; ++col) mp.inertia[static_cast<std::size_t>(r * 3 + col)] = m.Value(r + 1, col + 1);
        return mp;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "massProperties"));
    }
}

} // namespace iw::kernel
