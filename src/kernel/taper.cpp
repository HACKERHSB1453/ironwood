// SPDX-License-Identifier: GPL-3.0-or-later
#include "kernel/taper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <format>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Bnd_Box.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_JoinType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "kernel/internal/shape_impl.hpp"
#include "kernel/ops.hpp"

namespace iw::kernel {
namespace {

constexpr double kPiValue = 3.14159265358979323846;
constexpr double kMaxTaperRadians = 89.0 * kPiValue / 180.0;

bool finiteVec(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

Error occtError(const Standard_Failure& e, const char* what) {
    return Error{ErrorCode::KernelFailure, std::string(what) + ": " + (e.GetMessageString() ? e.GetMessageString() : "OCCT exception")};
}

// The helpers below mirror the validation and history plumbing of kernel/ops.cpp, which keeps them in an anonymous
// namespace. They are repeated here so that ops.cpp stays untouched by the taper work.

void ensureOcctSignals() {
    static std::once_flag once;
    std::call_once(once, [] { OSD::SetSignal(Standard_False); });
}

bool hasSolid(const TopoDS_Shape& s) {
    if (s.IsNull()) return false;
    if (s.ShapeType() == TopAbs_SOLID) return true;
    if (s.ShapeType() != TopAbs_COMPOUND && s.ShapeType() != TopAbs_COMPSOLID) return false;
    for (TopoDS_Iterator it(s); it.More(); it.Next())
        if (hasSolid(it.Value())) return true;
    return false;
}

bool solidsClosed(const TopoDS_Shape& s) {
    for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) {
        TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
        TopExp::MapShapesAndAncestors(ex.Current(), TopAbs_EDGE, TopAbs_FACE, edgeFaces);
        for (int i = 1; i <= edgeFaces.Extent(); ++i) {
            if (BRep_Tool::Degenerated(TopoDS::Edge(edgeFaces.FindKey(i)))) continue;
            if (edgeFaces.FindFromIndex(i).Extent() < 2) return false;
        }
    }
    return true;
}

/// Whole-shape validity: a solid is present, all solids are closed, and BRepCheck_Analyzer passes.
bool structurallyValid(const TopoDS_Shape& s) {
    if (s.IsNull() || !hasSolid(s)) return false;
    if (!solidsClosed(s)) return false;
    return BRepCheck_Analyzer(s).IsValid();
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
        default: continue;
        }
        const int idx = out.map(kind).FindIndex(s);
        if (idx == 0) continue;
        dst.push_back({kind, static_cast<std::uint32_t>(idx - 1)});
        any = true;
    }
    return any;
}

void sortUnique(std::vector<OutputSubShape>& v) {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
}

/// Mitred offset of a planar closed wire by `offset` mm (positive grows the wire, negative shrinks it), in the wire's
/// plane. Empty when the offset collapses or OCCT cannot build it.
std::optional<TopoDS_Wire> offsetWire(const TopoDS_Wire& wire, double offset) {
    BRepOffsetAPI_MakeOffset mk(wire, GeomAbs_Intersection, Standard_False);
    mk.Perform(offset);
    if (!mk.IsDone()) return std::nullopt;
    const TopoDS_Shape s = mk.Shape();
    if (s.IsNull()) return std::nullopt;
    for (TopExp_Explorer ex(s, TopAbs_WIRE); ex.More(); ex.Next()) return TopoDS::Wire(ex.Current());
    return std::nullopt;
}

int edgeCount(const TopoDS_Shape& s) {
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(s, TopAbs_EDGE, edges);
    return edges.Extent();
}

/// Largest inward offset of `wire` (in [0, limit]) that still exists, found by bisection. Used only to report where the
/// profile collapses.
double largestInwardOffset(const TopoDS_Wire& wire, double limit) {
    double lo = 0.0, hi = limit;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (offsetWire(wire, -mid)) lo = mid;
        else hi = mid;
    }
    return lo;
}

} // namespace

Result<ShapeResult> extrudeProfileTapered(const Shape& profile, const Vec3& direction, double taperRadians, const CancelToken& cancel) {
    if (profile.isNull()) return makeError(ErrorCode::InvalidArgument, "extrudeProfileTapered: null profile");
    if (!finiteVec(direction) || length(direction) < 1e-9)
        return makeError(ErrorCode::InvalidArgument, "extrudeProfileTapered: vector must be finite and non-zero");
    if (!std::isfinite(taperRadians) || std::abs(taperRadians) >= kMaxTaperRadians)
        return makeError(ErrorCode::InvalidArgument, "extrudeProfileTapered: taper must be finite and below 89 degrees");
    if (profile.type() != ShapeType::Face && profile.type() != ShapeType::Wire)
        return makeError(ErrorCode::InvalidArgument, "extrudeProfileTapered: profile must be a planar Face or closed planar Wire");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "extrudeProfileTapered cancelled");
    if (std::abs(taperRadians) < 1e-12) return extrudeProfile(profile, direction, cancel);

    ensureOcctSignals();
    try {
        OCC_CATCH_SIGNALS
        const TopoDS_Shape in = occtShape(profile);
        TopoDS_Face face;
        TopoDS_Wire bottom;
        if (profile.type() == ShapeType::Face) {
            face = TopoDS::Face(in);
            int loops = 0;
            for (TopExp_Explorer ex(face, TopAbs_WIRE); ex.More(); ex.Next()) ++loops;
            if (loops != 1)
                return makeError(ErrorCode::InvalidArgument, "extrudeProfileTapered: a tapered profile must have a single boundary (no holes)");
            bottom = BRepTools::OuterWire(face);
        } else {
            bottom = TopoDS::Wire(in);
            if (!BRep_Tool::IsClosed(bottom)) return makeError(ErrorCode::InvalidArgument, "extrudeProfileTapered: wire is not closed");
            BRepBuilderAPI_MakeFace mf(bottom, /*OnlyPlane=*/Standard_True);
            if (!mf.IsDone()) return makeError(ErrorCode::InvalidArgument, "extrudeProfileTapered: wire is not planar");
            face = mf.Face();
        }
        if (bottom.IsNull()) return makeError(ErrorCode::InvalidArgument, "extrudeProfileTapered: profile has no boundary");
        const BRepAdaptor_Surface ad(face, Standard_True);
        if (ad.GetType() != GeomAbs_Plane) return makeError(ErrorCode::InvalidArgument, "extrudeProfileTapered: profile is not planar");
        const gp_Dir nd = ad.Plane().Axis().Direction();
        const double dist = length(direction);
        const double align = (direction.x * nd.X() + direction.y * nd.Y() + direction.z * nd.Z()) / dist;
        if (std::abs(std::abs(align) - 1.0) > 1e-9)
            return makeError(ErrorCode::InvalidArgument,
                             "extrudeProfileTapered: direction must be normal to the profile plane (oblique taper is not supported)");

        // The side of the profile moves by `dist * tan(taper)` over the extrusion (positive taper = inward). The top
        // outline is that mitred offset, moved by the extrusion vector; the sides are ruled between the two outlines.
        const bool inward = taperRadians > 0;
        const double sideOffset = dist * std::tan(std::abs(taperRadians));
        const double signedOffset = inward ? -sideOffset : sideOffset;
        auto top = offsetWire(bottom, signedOffset);
        auto collapseMessage = [&]() {
            const double reach = largestInwardOffset(bottom, sideOffset);
            return makeError(ErrorCode::KernelFailure,
                             std::format("extrudeProfileTapered: taper too large: profile collapses at distance {:g} mm",
                                         reach / std::tan(std::abs(taperRadians))));
        };
        if (!top) {
            if (inward) return collapseMessage();
            return makeError(ErrorCode::KernelFailure, "extrudeProfileTapered: could not offset the profile");
        }
        if (edgeCount(*top) != edgeCount(bottom))
            return makeError(ErrorCode::KernelFailure, "extrudeProfileTapered: taper changes the profile topology (edges vanish)");
        gp_Trsf move;
        move.SetTranslation(gp_Vec(direction.x, direction.y, direction.z));
        const TopoDS_Wire topWire = TopoDS::Wire(top->Moved(TopLoc_Location(move)));

        BRepOffsetAPI_ThruSections loft(/*isSolid=*/Standard_True, /*ruled=*/Standard_True);
        loft.AddWire(bottom);
        loft.AddWire(topWire);
        loft.Build();
        if (!loft.IsDone()) return makeError(ErrorCode::KernelFailure, "extrudeProfileTapered: draft loft failed");
        const TopoDS_Shape res = loft.Shape();
        if (res.IsNull() || res.ShapeType() != TopAbs_SOLID) return makeError(ErrorCode::KernelFailure, "extrudeProfileTapered: draft loft did not produce a solid");

        // Validation: an inward taper cannot enlarge the prism and an outward one cannot shrink it, and the solid must pass
        // the whole-shape checks. A failure here is reported as a collapse only when the offset itself reached the centre.
        GProp_GProps vp, sp;
        BRepGProp::VolumeProperties(res, vp);
        BRepGProp::SurfaceProperties(face, sp);
        const double vol = vp.Mass();
        const double prismVol = sp.Mass() * dist;
        const double slack = 1e-6 * prismVol + 1e-9;
        const bool volumeOk = inward ? (vol > 0 && vol <= prismVol + slack) : (vol >= prismVol - slack);
        if (!volumeOk) return makeError(ErrorCode::KernelFailure, "extrudeProfileTapered: draft solid volume is implausible");
        if (!structurallyValid(res)) return makeError(ErrorCode::KernelFailure, "extrudeProfileTapered: draft produced an invalid solid");

        ShapeResult r;
        r.shape = wrapShape(res);
        const SubShapeIndex& out = *r.shape.impl()->index;
        const SubShapeIndex& pin = *profile.impl()->index;
        ShapeHistory& h = r.history;

        // Profile edges -> generated side faces; profile vertices -> generated side edges.
        for (SubShapeKind kind : {SubShapeKind::Edge, SubShapeKind::Vertex}) {
            const auto& map = pin.map(kind);
            for (int i = 1; i <= map.Extent(); ++i) {
                const TopoDS_Shape& sub = map.FindKey(i);
                const InputSubShape key{0, kind, static_cast<std::uint32_t>(i - 1)};
                std::vector<OutputSubShape> g;
                if (addOutputs(loft.Generated(sub), out, g)) {
                    sortUnique(g);
                    h.generated[key] = std::move(g);
                }
            }
        }
        // Face profile: the start cap is the loft's first cap (modified); the end cap is generated.
        if (profile.type() == ShapeType::Face) {
            const InputSubShape key{0, SubShapeKind::Face, 0};
            std::vector<OutputSubShape> first, last;
            TopTools_ListOfShape l1, l2;
            l1.Append(loft.FirstShape());
            l2.Append(loft.LastShape());
            if (addOutputs(l1, out, first)) h.modified[key] = std::move(first);
            if (addOutputs(l2, out, last)) h.generated[key] = std::move(last);
        }
        return r;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "extrudeProfileTapered"));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string("extrudeProfileTapered: ") + e.what());
    }
}

} // namespace iw::kernel
