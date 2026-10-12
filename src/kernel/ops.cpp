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
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <StdFail_NotDone.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <format>
#include <gp_Ax1.hxx>
#include <gp_Ax3.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>
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
        GProp_GProps vol, surf, lin;
        BRepGProp::VolumeProperties(s, vol);
        BRepGProp::SurfaceProperties(s, surf);
        BRepGProp::LinearProperties(s, lin);
        MassProperties mp;
        mp.volume = vol.Mass();
        mp.surfaceArea = surf.Mass();
        // F16: centre of mass and inertia use the properties of the shape's own dimension: solids -> volume,
        // faces/shells -> surface, wires/edges -> linear. (Volume props of a face or wire are meaningless.)
        const GProp_GProps* chosen = &lin;
        if (TopExp_Explorer(s, TopAbs_SOLID).More()) chosen = &vol;
        else if (TopExp_Explorer(s, TopAbs_FACE).More()) chosen = &surf;
        const gp_Pnt c = chosen->CentreOfMass();
        mp.centerOfMass = {c.X(), c.Y(), c.Z()};
        const gp_Mat m = chosen->MatrixOfInertia(); // about the centre of mass, unit density
        for (int r = 0; r < 3; ++r)
            for (int col = 0; col < 3; ++col) mp.inertia[static_cast<std::size_t>(r * 3 + col)] = m.Value(r + 1, col + 1);
        return mp;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "massProperties"));
    }
}

// ===================================================================================================== P1a (A4)

namespace {

constexpr double kPiValue = 3.14159265358979323846;
constexpr double kTwoPiValue = 2.0 * kPiValue;

const char* kindName(SubShapeKind k) {
    switch (k) {
    case SubShapeKind::Face: return "face";
    case SubShapeKind::Edge: return "edge";
    default: return "vertex";
    }
}

std::string num(double v) { return std::format("{:g}", v); }

/// Sub-shape `idx` of `s` (ordering contract) or InvalidArgument when out of range. `s` must be non-null.
Result<TopoDS_Shape> subShapeAt(const Shape& s, SubShapeKind kind, std::uint32_t idx, const char* what) {
    const std::uint32_t n = s.subShapeCount(kind);
    if (idx >= n)
        return makeError(ErrorCode::InvalidArgument, std::string(what) + ": " + kindName(kind) + " index " +
                                                         std::to_string(idx) + " out of range (shape has " +
                                                         std::to_string(n) + ")");
    return s.impl()->index->map(kind).FindKey(static_cast<int>(idx) + 1);
}

/// A unit vector perpendicular to unit vector `n`, chosen deterministically (world axis least aligned with n).
Vec3 perpendicularTo(const Vec3& n) {
    Vec3 ax{1, 0, 0};
    double best = std::abs(n.x);
    if (std::abs(n.y) < best) { ax = {0, 1, 0}; best = std::abs(n.y); }
    if (std::abs(n.z) < best) { ax = {0, 0, 1}; }
    const Vec3 p = ax - n * dot(ax, n);
    return p * (1.0 / length(p));
}

Vec3 normalised(const Vec3& v) { return v * (1.0 / length(v)); }

/// Validates an edge selection for fillet/chamfer: solid target, non-empty, in-range; returns sorted unique ids.
Result<std::vector<std::uint32_t>> edgeSelection(const Shape& shape, std::span<const std::uint32_t> edgeIndices,
                                                 const char* what) {
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, std::string(what) + ": null shape");
    if (shape.type() != ShapeType::Solid)
        return makeError(ErrorCode::InvalidArgument, std::string(what) + ": target must be a solid");
    if (edgeIndices.empty()) return makeError(ErrorCode::InvalidArgument, std::string(what) + ": no edges selected");
    std::vector<std::uint32_t> ids(edgeIndices.begin(), edgeIndices.end());
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    for (std::uint32_t id : ids) {
        const std::uint32_t n = shape.subShapeCount(SubShapeKind::Edge);
        if (id >= n)
            return makeError(ErrorCode::InvalidArgument, std::string(what) + ": edge index " + std::to_string(id) +
                                                             " out of range (shape has " + std::to_string(n) + ")");
    }
    return ids;
}

/// Runs `build` on the whole selection; if that fails, retries each edge alone so the failure can be attributed to
/// the edge that cannot be processed (named in the message via `describe`). Deterministic: edges are tried in index order.
template <class Build, class Describe>
Result<ShapeResult> diagnoseEdgeOp(const std::vector<std::uint32_t>& ids, CancelToken cancel, Build&& build,
                                   Describe&& describe) {
    auto whole = build(ids);
    if (whole || whole.error().code == ErrorCode::Cancelled) return whole;
    if (ids.size() == 1) return makeError(ErrorCode::KernelFailure, describe(ids[0]));
    for (std::uint32_t id : ids) {
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "edge operation cancelled");
        auto one = build(std::vector<std::uint32_t>{id});
        if (!one) {
            if (one.error().code == ErrorCode::Cancelled) return one;
            return makeError(ErrorCode::KernelFailure, describe(id));
        }
    }
    return makeError(ErrorCode::KernelFailure, whole.error().message);
}

/// Profile (planar Face, or closed planar Wire converted to a face) as an OCCT face.
Result<TopoDS_Face> profileToFace(const Shape& profile, const char* what) {
    const TopoDS_Shape in = occtShape(profile);
    if (profile.type() == ShapeType::Face) return TopoDS::Face(in);
    const TopoDS_Wire wire = TopoDS::Wire(in);
    if (!BRep_Tool::IsClosed(wire))
        return makeError(ErrorCode::InvalidArgument, std::string(what) + ": wire is not closed");
    BRepBuilderAPI_MakeFace mf(wire, Standard_True);
    if (!mf.IsDone()) return makeError(ErrorCode::InvalidArgument, std::string(what) + ": wire is not planar");
    return mf.Face();
}

/// OCCT reports no history for profile edges lying in a plane normal to a FULL-turn revolve axis, although the planar
/// caps they generate are in the result. Recover those links geometrically: such an edge (both ends at the same axial
/// position) maps to the unique planar output face normal to the axis at that position. Only edges without any OCCT
/// history are considered, so partial revolves keep OCCT's answer untouched.
void linkFullTurnCaps(const SubShapeIndex& pin, const SubShapeIndex& out, const Axis& axis, ShapeHistory& h) {
    const Vec3 o = axis.origin;
    const Vec3 d = normalised(axis.direction);
    const auto toVec = [](const gp_Pnt& p) { return Vec3{p.X(), p.Y(), p.Z()}; };
    for (int i = 1; i <= pin.edges.Extent(); ++i) {
        const InputSubShape key{0, SubShapeKind::Edge, static_cast<std::uint32_t>(i - 1)};
        if (h.generated.contains(key) || h.modified.contains(key)) continue;
        const TopoDS_Edge e = TopoDS::Edge(pin.edges.FindKey(i));
        const double za = dot(toVec(BRep_Tool::Pnt(TopExp::FirstVertex(e, Standard_True))) - o, d);
        const double zb = dot(toVec(BRep_Tool::Pnt(TopExp::LastVertex(e, Standard_True))) - o, d);
        if (std::abs(za - zb) > 1e-9) continue; // not perpendicular to the axis
        std::vector<OutputSubShape> candidates;
        for (int f = 1; f <= out.faces.Extent(); ++f) {
            BRepAdaptor_Surface ad(TopoDS::Face(out.faces.FindKey(f)));
            if (ad.GetType() != GeomAbs_Plane) continue;
            const gp_Pln pl = ad.Plane();
            const gp_Dir n = pl.Axis().Direction();
            if (std::abs(std::abs(dot({n.X(), n.Y(), n.Z()}, d)) - 1.0) > 1e-9) continue;
            if (std::abs(dot(toVec(pl.Location()) - o, d) - za) > 1e-7) continue;
            candidates.push_back({SubShapeKind::Face, static_cast<std::uint32_t>(f - 1)});
        }
        if (candidates.size() == 1) h.generated[key] = std::move(candidates);
    }
}

/// History for a sweep (revolve) of `profile`: profile edges/vertices -> generated; face profile -> first cap modified,
/// last cap generated. Mirrors the extrudeProfile conventions.
template <class Sweep>
void sweepHistory(Sweep& sw, const Shape& profile, const SubShapeIndex& out, const Axis& axis, ShapeHistory& h) {
    const SubShapeIndex& pin = *profile.impl()->index;
    for (SubShapeKind kind : {SubShapeKind::Edge, SubShapeKind::Vertex}) {
        const auto& map = pin.map(kind);
        for (int i = 1; i <= map.Extent(); ++i) {
            const TopoDS_Shape& sub = map.FindKey(i);
            const InputSubShape key{0, kind, static_cast<std::uint32_t>(i - 1)};
            std::vector<OutputSubShape> g;
            if (addOutputs(sw.Generated(sub), out, g)) {
                sortUnique(g);
                h.generated[key] = std::move(g);
            }
            const int same = out.map(kind).FindIndex(sub);
            if (same != 0) h.unchanged[key] = {kind, static_cast<std::uint32_t>(same - 1)};
        }
    }
    if (profile.type() == ShapeType::Face) {
        const InputSubShape key{0, SubShapeKind::Face, 0};
        std::vector<OutputSubShape> first, last;
        TopTools_ListOfShape l1, l2;
        l1.Append(sw.FirstShape());
        l2.Append(sw.LastShape());
        if (addOutputs(l1, out, first)) h.modified[key] = std::move(first);
        if (addOutputs(l2, out, last)) h.generated[key] = std::move(last);
    }
    linkFullTurnCaps(*profile.impl()->index, out, axis, h);
}

Result<ShapeResult> buildFillet(const Shape& shape, const std::vector<std::uint32_t>& ids, double radius,
                                CancelToken cancel) {
    try {
        BRepFilletAPI_MakeFillet mk(occtShape(shape));
        for (std::uint32_t id : ids) {
            auto e = subShapeAt(shape, SubShapeKind::Edge, id, "fillet");
            if (!e) return std::unexpected(e.error());
            mk.Add(radius, TopoDS::Edge(*e));
        }
        Handle(Message_ProgressIndicator) pi = new CancelProgress(cancel);
        mk.Build(pi->Start());
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "fillet cancelled");
        if (!mk.IsDone()) return makeError(ErrorCode::KernelFailure, "fillet: kernel could not build the fillet");
        const TopoDS_Shape res = unwrapSingle(mk.Shape());
        if (res.IsNull() || !hasSolid(res)) return makeError(ErrorCode::KernelFailure, "fillet: result is empty");
        ShapeResult r;
        r.shape = wrapShape(res);
        r.history = extractHistory(mk, {&shape}, *r.shape.impl()->index);
        return r;
    } catch (const StdFail_NotDone&) {
        return makeError(ErrorCode::KernelFailure, "fillet: kernel could not build the fillet");
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "fillet"));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string("fillet: ") + e.what());
    }
}

/// Chamfer over `ids`. byAngle: (dist, angle); otherwise (d1, d2). The face carrying `dist` is the adjacent face with
/// the lowest face index, which makes the result independent of OCCT's internal ancestor-list order.
Result<ShapeResult> buildChamfer(const Shape& shape, const std::vector<std::uint32_t>& ids, double dist, double second,
                                 bool byAngle, CancelToken cancel) {
    try {
        const TopoDS_Shape s = occtShape(shape);
        TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
        TopExp::MapShapesAndAncestors(s, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
        const auto& fmap = shape.impl()->index->faces;
        BRepFilletAPI_MakeChamfer mk(s);
        for (std::uint32_t id : ids) {
            auto eRes = subShapeAt(shape, SubShapeKind::Edge, id, "chamfer");
            if (!eRes) return std::unexpected(eRes.error());
            const TopoDS_Edge e = TopoDS::Edge(*eRes);
            if (!edgeFaces.Contains(e))
                return makeError(ErrorCode::KernelFailure, "chamfer: edge " + std::to_string(id) + " has no adjacent face");
            int best = 0;
            TopoDS_Face bestFace;
            for (TopTools_ListIteratorOfListOfShape it(edgeFaces.FindFromKey(e)); it.More(); it.Next()) {
                const int idx = fmap.FindIndex(it.Value());
                if (idx != 0 && (best == 0 || idx < best)) {
                    best = idx;
                    bestFace = TopoDS::Face(it.Value());
                }
            }
            if (best == 0) return makeError(ErrorCode::KernelFailure, "chamfer: edge " + std::to_string(id) + " has no face");
            if (byAngle) mk.AddDA(dist, second, e, bestFace);
            else mk.Add(dist, second, e, bestFace);
        }
        Handle(Message_ProgressIndicator) pi = new CancelProgress(cancel);
        mk.Build(pi->Start());
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "chamfer cancelled");
        if (!mk.IsDone()) return makeError(ErrorCode::KernelFailure, "chamfer: kernel could not build the chamfer");
        const TopoDS_Shape res = unwrapSingle(mk.Shape());
        if (res.IsNull() || !hasSolid(res)) return makeError(ErrorCode::KernelFailure, "chamfer: result is empty");
        ShapeResult r;
        r.shape = wrapShape(res);
        r.history = extractHistory(mk, {&shape}, *r.shape.impl()->index);
        return r;
    } catch (const StdFail_NotDone&) {
        return makeError(ErrorCode::KernelFailure, "chamfer: kernel could not build the chamfer");
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "chamfer"));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string("chamfer: ") + e.what());
    }
}

} // namespace

Result<Plane> planeFromPointNormal(const Vec3& origin, const Vec3& normal) {
    if (!finiteVec(origin) || !finiteVec(normal) || length(normal) < 1e-12)
        return makeError(ErrorCode::InvalidArgument, "planeFromPointNormal: need finite origin and non-zero normal");
    Plane p;
    p.origin = origin;
    p.normal = normalised(normal);
    p.xDir = perpendicularTo(p.normal);
    return p;
}

Result<Plane> planeFromThreePoints(const Vec3& a, const Vec3& b, const Vec3& c) {
    if (!finiteVec(a) || !finiteVec(b) || !finiteVec(c))
        return makeError(ErrorCode::InvalidArgument, "planeFromThreePoints: non-finite point");
    const Vec3 u = b - a, v = c - a;
    const Vec3 n = cross(u, v);
    const double lu = length(u), ln = length(n);
    if (lu < 1e-12 || ln <= 1e-12 * lu * length(v))
        return makeError(ErrorCode::InvalidArgument, "planeFromThreePoints: points are collinear or coincident");
    Plane p;
    p.origin = a;
    p.normal = n * (1.0 / ln);
    p.xDir = u * (1.0 / lu);
    return p;
}

Result<Plane> planeFromFace(const Shape& shape, std::uint32_t faceIndex) {
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "planeFromFace: null shape");
    try {
        auto f = subShapeAt(shape, SubShapeKind::Face, faceIndex, "planeFromFace");
        if (!f) return std::unexpected(f.error());
        const TopoDS_Face face = TopoDS::Face(*f);
        BRepAdaptor_Surface ad(face, Standard_True);
        if (ad.GetType() != GeomAbs_Plane)
            return makeError(ErrorCode::InvalidArgument, "planeFromFace: face " + std::to_string(faceIndex) + " is not planar");
        const gp_Pln pl = ad.Plane();
        const gp_Dir n = pl.Axis().Direction();
        const gp_Dir x = pl.XAxis().Direction();
        Plane p;
        p.origin = {pl.Location().X(), pl.Location().Y(), pl.Location().Z()};
        const double sign = face.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
        p.normal = {sign * n.X(), sign * n.Y(), sign * n.Z()};
        p.xDir = {x.X(), x.Y(), x.Z()};
        return p;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "planeFromFace"));
    }
}

Result<Axis> axisFromPoints(const Vec3& a, const Vec3& b) {
    if (!finiteVec(a) || !finiteVec(b) || length(b - a) < 1e-12)
        return makeError(ErrorCode::InvalidArgument, "axisFromPoints: need two distinct finite points");
    return Axis{a, normalised(b - a)};
}

Result<Axis> axisFromEdge(const Shape& shape, std::uint32_t edgeIndex) {
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "axisFromEdge: null shape");
    try {
        auto e = subShapeAt(shape, SubShapeKind::Edge, edgeIndex, "axisFromEdge");
        if (!e) return std::unexpected(e.error());
        const TopoDS_Edge edge = TopoDS::Edge(*e);
        BRepAdaptor_Curve ac(edge);
        if (ac.GetType() != GeomAbs_Line)
            return makeError(ErrorCode::InvalidArgument, "axisFromEdge: edge " + std::to_string(edgeIndex) + " is not a line");
        const gp_Pnt a = BRep_Tool::Pnt(TopExp::FirstVertex(edge, Standard_True));
        const gp_Pnt b = BRep_Tool::Pnt(TopExp::LastVertex(edge, Standard_True));
        return axisFromPoints({a.X(), a.Y(), a.Z()}, {b.X(), b.Y(), b.Z()});
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "axisFromEdge"));
    }
}

Result<ShapeResult> revolve(const Shape& profile, const Axis& axis, double angle, CancelToken cancel) {
    if (profile.isNull()) return makeError(ErrorCode::InvalidArgument, "revolve: null profile");
    if (!finiteVec(axis.origin) || !finiteVec(axis.direction) || length(axis.direction) < 1e-12)
        return makeError(ErrorCode::InvalidArgument, "revolve: axis needs a finite origin and a non-zero direction");
    if (!std::isfinite(angle) || angle <= 0.0 || angle > kTwoPiValue * (1.0 + 1e-12))
        return makeError(ErrorCode::InvalidArgument, "revolve: angle must be in (0, 2*pi]");
    if (profile.type() != ShapeType::Face && profile.type() != ShapeType::Wire)
        return makeError(ErrorCode::InvalidArgument, "revolve: profile must be a planar Face or closed planar Wire");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "revolve cancelled");
    try {
        auto face = profileToFace(profile, "revolve");
        if (!face) return std::unexpected(face.error());
        const Vec3 d = normalised(axis.direction);
        const gp_Ax1 ax(gp_Pnt(axis.origin.x, axis.origin.y, axis.origin.z), gp_Dir(d.x, d.y, d.z));
        BRepPrimAPI_MakeRevol rv(*face, ax, angle, Standard_False);
        rv.Build();
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "revolve cancelled");
        if (!rv.IsDone()) return makeError(ErrorCode::KernelFailure, "revolve: kernel failure");
        const TopoDS_Shape res = unwrapSingle(rv.Shape());
        if (res.IsNull() || !hasSolid(res)) return makeError(ErrorCode::KernelFailure, "revolve: did not produce a solid");
        GProp_GProps props;
        BRepGProp::VolumeProperties(res, props);
        if (std::abs(props.Mass()) < 1e-12)
            return makeError(ErrorCode::InvalidArgument, "revolve: profile has no area on one side of the axis");
        ShapeResult r;
        r.shape = wrapShape(res);
        sweepHistory(rv, profile, *r.shape.impl()->index, axis, r.history);
        return r;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "revolve"));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string("revolve: ") + e.what());
    }
}

Result<ShapeResult> filletEdges(const Shape& shape, std::span<const std::uint32_t> edgeIndices, double radius,
                                CancelToken cancel) {
    auto ids = edgeSelection(shape, edgeIndices, "fillet");
    if (!ids) return std::unexpected(ids.error());
    if (!finitePositive(radius)) return makeError(ErrorCode::InvalidArgument, "fillet: radius must be finite and > 0");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "fillet cancelled");
    auto build = [&](const std::vector<std::uint32_t>& sel) { return buildFillet(shape, sel, radius, cancel); };
    auto describe = [&](std::uint32_t id) {
        return "fillet radius too large for edge " + std::to_string(id) + " (radius " + num(radius) + " mm)";
    };
    return diagnoseEdgeOp(*ids, cancel, build, describe);
}

Result<ShapeResult> chamferEdges(const Shape& shape, std::span<const std::uint32_t> edgeIndices, double d1, double d2,
                                 CancelToken cancel) {
    auto ids = edgeSelection(shape, edgeIndices, "chamfer");
    if (!ids) return std::unexpected(ids.error());
    if (!finitePositive(d1) || !finitePositive(d2))
        return makeError(ErrorCode::InvalidArgument, "chamfer: distances must be finite and > 0");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "chamfer cancelled");
    auto build = [&](const std::vector<std::uint32_t>& sel) { return buildChamfer(shape, sel, d1, d2, false, cancel); };
    auto describe = [&](std::uint32_t id) {
        return "chamfer distance too large for edge " + std::to_string(id) + " (d1 " + num(d1) + " mm, d2 " + num(d2) +
               " mm)";
    };
    return diagnoseEdgeOp(*ids, cancel, build, describe);
}

Result<ShapeResult> chamferEdgesByAngle(const Shape& shape, std::span<const std::uint32_t> edgeIndices, double distance,
                                        double angle, CancelToken cancel) {
    auto ids = edgeSelection(shape, edgeIndices, "chamfer");
    if (!ids) return std::unexpected(ids.error());
    if (!finitePositive(distance)) return makeError(ErrorCode::InvalidArgument, "chamfer: distance must be finite and > 0");
    if (!std::isfinite(angle) || angle <= 0.0 || angle >= kPiValue)
        return makeError(ErrorCode::InvalidArgument, "chamfer: angle must be in (0, pi)");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "chamfer cancelled");
    auto build = [&](const std::vector<std::uint32_t>& sel) { return buildChamfer(shape, sel, distance, angle, true, cancel); };
    auto describe = [&](std::uint32_t id) {
        return "chamfer distance too large for edge " + std::to_string(id) + " (distance " + num(distance) + " mm, angle " +
               num(angle) + " rad)";
    };
    return diagnoseEdgeOp(*ids, cancel, build, describe);
}

Result<ShapeResult> hole(const Shape& shape, const Axis& placement, const HoleSpec& spec, CancelToken cancel) {
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "hole: null shape");
    if (shape.type() != ShapeType::Solid) return makeError(ErrorCode::InvalidArgument, "hole: target must be a solid");
    if (!finiteVec(placement.origin) || !finiteVec(placement.direction) || length(placement.direction) < 1e-12)
        return makeError(ErrorCode::InvalidArgument, "hole: placement needs a finite origin and a non-zero direction");
    if (!finitePositive(spec.diameter)) return makeError(ErrorCode::InvalidArgument, "hole: diameter must be finite and > 0");
    if (!spec.through && !finitePositive(spec.depth))
        return makeError(ErrorCode::InvalidArgument, "hole: depth must be finite and > 0 (or through)");
    const bool drillOk = spec.drillPointAngle > 0.0 && spec.drillPointAngle <= kPiValue;
    if (!spec.through && !drillOk) return makeError(ErrorCode::InvalidArgument, "hole: drill point angle must be in (0, pi]");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "hole cancelled");

    const Vec3 o = placement.origin;
    const Vec3 d = normalised(placement.direction);
    const double r = spec.diameter / 2.0;
    // The tool starts `margin` outside the entry face so the cut never coincides with the surface.
    const double margin = std::max(1.0, spec.diameter);

    // Farthest extent of the part along the drill direction (for through holes).
    const Box3 bb = shape.boundingBox();
    double tMax = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < 8; ++i) {
        const Vec3 corner{(i & 1) ? bb.max.x : bb.min.x, (i & 2) ? bb.max.y : bb.min.y, (i & 4) ? bb.max.z : bb.min.z};
        tMax = std::max(tMax, dot(corner - o, d));
    }
    if (spec.through && tMax <= 0.0)
        return makeError(ErrorCode::InvalidArgument, "hole: entry point lies beyond the part along the drill direction");
    const double zEnd = spec.through ? tMax + margin : spec.depth;
    // Cone of a drill point with full included angle `a`, height r / tan(a/2); a = pi gives a flat bottom.
    const auto coneHeight = [&] { return spec.through ? 0.0 : r / std::tan(spec.drillPointAngle / 2.0); };
    const double tip = spec.through ? zEnd : spec.depth + coneHeight();

    std::vector<std::pair<double, double>> rz; // profile in (radial, axial) coordinates, closed polygon
    switch (spec.kind) {
    case HoleKind::Simple:
        rz = spec.through ? std::vector<std::pair<double, double>>{{0, -margin}, {r, -margin}, {r, zEnd}, {0, zEnd}}
                          : std::vector<std::pair<double, double>>{{0, -margin}, {r, -margin}, {r, spec.depth}, {0, tip}};
        break;
    case HoleKind::Counterbore: {
        if (!(spec.counterboreDiameter > spec.diameter))
            return makeError(ErrorCode::InvalidArgument, "hole: counterbore diameter must exceed the hole diameter");
        if (!finitePositive(spec.counterboreDepth))
            return makeError(ErrorCode::InvalidArgument, "hole: counterbore depth must be finite and > 0");
        if (!spec.through && !(spec.counterboreDepth < spec.depth))
            return makeError(ErrorCode::InvalidArgument, "hole: counterbore must be shallower than the hole depth");
        const double R = spec.counterboreDiameter / 2.0;
        rz = spec.through
                 ? std::vector<std::pair<double, double>>{{0, -margin}, {R, -margin}, {R, spec.counterboreDepth}, {r, spec.counterboreDepth}, {r, zEnd}, {0, zEnd}}
                 : std::vector<std::pair<double, double>>{{0, -margin}, {R, -margin}, {R, spec.counterboreDepth}, {r, spec.counterboreDepth}, {r, spec.depth}, {0, tip}};
        break;
    }
    case HoleKind::Countersink: {
        if (!(spec.countersinkDiameter > spec.diameter))
            return makeError(ErrorCode::InvalidArgument, "hole: countersink diameter must exceed the hole diameter");
        if (!(spec.countersinkAngle > 0.0 && spec.countersinkAngle < kPiValue))
            return makeError(ErrorCode::InvalidArgument, "hole: countersink angle must be in (0, pi)");
        const double R = spec.countersinkDiameter / 2.0;
        const double t = std::tan(spec.countersinkAngle / 2.0);   // dr/dz of the cone wall
        const double zc = (R - r) / t;                             // depth where the cone meets the hole wall
        const double limit = spec.through ? zEnd : spec.depth;
        if (!(zc < limit)) return makeError(ErrorCode::InvalidArgument, "hole: countersink is deeper than the hole");
        const double rTop = R + margin * t;                        // cone extended above the surface
        rz = spec.through
                 ? std::vector<std::pair<double, double>>{{0, -margin}, {rTop, -margin}, {r, zc}, {r, zEnd}, {0, zEnd}}
                 : std::vector<std::pair<double, double>>{{0, -margin}, {rTop, -margin}, {r, zc}, {r, spec.depth}, {0, tip}};
        break;
    }
    }

    try {
        // Profile lies in the plane spanned by the axis and a perpendicular direction; revolving it gives the tool.
        const Vec3 x = perpendicularTo(d);
        std::vector<Vec3> pts;
        pts.reserve(rz.size());
        for (const auto& [radial, axial] : rz) pts.push_back(o + d * axial + x * radial);
        auto face = makePolygonFace(pts);
        if (!face) return makeError(ErrorCode::Internal, "hole: could not build tool profile: " + face.error().message);
        auto tool = revolve(*face, Axis{o, d}, kTwoPiValue, cancel);
        if (!tool) return std::unexpected(tool.error());
        auto result = cut(shape, tool->shape, cancel);
        if (!result && result.error().code == ErrorCode::KernelFailure)
            return makeError(ErrorCode::KernelFailure, "hole: cut failed (" + result.error().message + ")");
        return result;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "hole"));
    }
}

Result<ShapeResult> transform(const Shape& shape, const Transform& matrix, CancelToken cancel) {
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "transform: null shape");
    const auto& m = matrix.m;
    for (double v : m)
        if (!std::isfinite(v)) return makeError(ErrorCode::InvalidArgument, "transform: non-finite matrix entry");
    constexpr double eps = 1e-9;
    if (std::abs(m[12]) > eps || std::abs(m[13]) > eps || std::abs(m[14]) > eps || std::abs(m[15] - 1.0) > eps)
        return makeError(ErrorCode::InvalidArgument, "transform: last row must be (0, 0, 0, 1)");
    const Vec3 c0{m[0], m[4], m[8]}, c1{m[1], m[5], m[9]}, c2{m[2], m[6], m[10]};
    const bool orthonormal = std::abs(length(c0) - 1) < eps && std::abs(length(c1) - 1) < eps &&
                             std::abs(length(c2) - 1) < eps && std::abs(dot(c0, c1)) < eps &&
                             std::abs(dot(c0, c2)) < eps && std::abs(dot(c1, c2)) < eps;
    if (!orthonormal || std::abs(dot(c0, cross(c1, c2)) - 1.0) > eps)
        return makeError(ErrorCode::InvalidArgument, "transform: must be rigid (rotation + translation; no scale or mirror)");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "transform cancelled");
    try {
        gp_Trsf t;
        t.SetValues(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]);
        BRepBuilderAPI_Transform bt(occtShape(shape), t, Standard_False);
        if (!bt.IsDone()) return makeError(ErrorCode::KernelFailure, "transform: kernel failure");
        const TopoDS_Shape res = bt.Shape();
        ShapeResult r;
        r.shape = wrapShape(res);
        // A rigid move keeps topology and its traversal order: input sub-shape i becomes output sub-shape i.
        for (SubShapeKind kind : {SubShapeKind::Face, SubShapeKind::Edge, SubShapeKind::Vertex}) {
            const std::uint32_t n = shape.subShapeCount(kind);
            if (r.shape.subShapeCount(kind) != n)
                return makeError(ErrorCode::KernelFailure, "transform: topology changed unexpectedly");
            for (std::uint32_t i = 0; i < n; ++i)
                r.history.modified[{0, kind, i}] = {OutputSubShape{kind, i}};
        }
        return r;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "transform"));
    }
}

Result<ShapeResult> intersectWith(const Shape& a, const Shape& b, CancelToken cancel) {
    return common(a, b, std::move(cancel));
}

Result<std::vector<std::uint32_t>> tangentChain(const Shape& shape, std::span<const std::uint32_t> edgeIndices, CancelToken cancel) {
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "tangentChain: null shape");
    if (edgeIndices.empty()) return makeError(ErrorCode::InvalidArgument, "tangentChain: no edges given");
    const std::uint32_t nEdges = shape.subShapeCount(SubShapeKind::Edge);
    for (std::uint32_t e : edgeIndices)
        if (e >= nEdges) return makeError(ErrorCode::InvalidArgument, "tangentChain: edge index " + std::to_string(e) + " out of range");
    try {
        const SubShapeIndex& idx = *shape.impl()->index;
        TopTools_IndexedDataMapOfShapeListOfShape vertexEdges;
        TopExp::MapShapesAndAncestors(occtShape(shape), TopAbs_VERTEX, TopAbs_EDGE, vertexEdges);

        // Outgoing unit tangent of edge `e` at its end `v` (pointing from v into the edge).
        auto outgoing = [&](const TopoDS_Edge& e, const TopoDS_Vertex& v, gp_Vec& out) -> bool {
            if (BRep_Tool::Degenerated(e)) return false;
            const double p = BRep_Tool::Parameter(v, e);
            BRepAdaptor_Curve c(e);
            gp_Pnt pt;
            gp_Vec d;
            c.D1(p, pt, d);
            if (d.Magnitude() < 1e-12) return false;
            const bool atStart = std::abs(p - c.FirstParameter()) <= std::abs(p - c.LastParameter());
            out = atStart ? d.Normalized() : d.Normalized().Reversed();
            return true;
        };

        std::vector<std::uint32_t> result(edgeIndices.begin(), edgeIndices.end());
        std::vector<std::uint32_t> queue(edgeIndices.begin(), edgeIndices.end());
        std::vector<bool> seen(nEdges, false);
        for (std::uint32_t e : queue) seen[e] = true;
        while (!queue.empty()) {
            if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "tangentChain cancelled");
            const std::uint32_t ei = queue.back();
            queue.pop_back();
            const TopoDS_Edge edge = TopoDS::Edge(idx.edges.FindKey(static_cast<int>(ei) + 1));
            TopoDS_Vertex v1, v2;
            TopExp::Vertices(edge, v1, v2);
            for (const TopoDS_Vertex& v : {v1, v2}) {
                if (v.IsNull() || !vertexEdges.Contains(v)) continue;
                gp_Vec tE;
                if (!outgoing(edge, v, tE)) continue;
                for (const TopoDS_Shape& cand : vertexEdges.FindFromKey(v)) {
                    const TopoDS_Edge other = TopoDS::Edge(cand);
                    const int fi = idx.edges.FindIndex(other);
                    if (fi == 0 || seen[static_cast<std::size_t>(fi - 1)]) continue;
                    gp_Vec tF;
                    if (!outgoing(other, v, tF)) continue;
                    // Tangent-continuous: outgoing directions point opposite (cos ~ -1), tolerance 1e-4 rad.
                    if (tE.Dot(tF) <= -std::cos(1e-4)) {
                        const auto fidx = static_cast<std::uint32_t>(fi - 1);
                        seen[fidx] = true;
                        result.push_back(fidx);
                        queue.push_back(fidx);
                    }
                }
            }
        }
        std::sort(result.begin(), result.end());
        result.erase(std::unique(result.begin(), result.end()), result.end());
        return result;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "tangentChain"));
    }
}

namespace {

/// Applies a gp_Trsf that keeps topology (mirror / uniform scale): copies, checks counts, maps i -> i.
Result<ShapeResult> applyTopologyPreserving(const Shape& shape, const gp_Trsf& trsf, const char* what, CancelToken cancel) {
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, std::string(what) + " cancelled");
    try {
        BRepBuilderAPI_Transform bt(occtShape(shape), trsf, Standard_True);
        if (!bt.IsDone()) return makeError(ErrorCode::KernelFailure, std::string(what) + ": kernel failure");
        ShapeResult r;
        r.shape = wrapShape(bt.Shape());
        for (SubShapeKind kind : {SubShapeKind::Face, SubShapeKind::Edge, SubShapeKind::Vertex}) {
            const std::uint32_t n = shape.subShapeCount(kind);
            if (r.shape.subShapeCount(kind) != n)
                return makeError(ErrorCode::KernelFailure, std::string(what) + ": topology changed unexpectedly");
            for (std::uint32_t i = 0; i < n; ++i) r.history.modified[{0, kind, i}] = {OutputSubShape{kind, i}};
        }
        return r;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, what));
    }
}

} // namespace

Result<ShapeResult> mirror(const Shape& shape, const Plane& plane, CancelToken cancel) {
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "mirror: null shape");
    if (!finiteVec(plane.origin) || !finiteVec(plane.normal) || length(plane.normal) < 1e-12)
        return makeError(ErrorCode::InvalidArgument, "mirror: plane needs a finite origin and a non-zero normal");
    const Vec3 n = normalised(plane.normal);
    try {
        gp_Trsf t;
        t.SetMirror(gp_Ax2(gp_Pnt(plane.origin.x, plane.origin.y, plane.origin.z), gp_Dir(n.x, n.y, n.z)));
        return applyTopologyPreserving(shape, t, "mirror", std::move(cancel));
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "mirror"));
    }
}

Result<ShapeResult> scale(const Shape& shape, const Vec3& center, double factor, CancelToken cancel) {
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "scale: null shape");
    if (!finiteVec(center) || !finitePositive(factor))
        return makeError(ErrorCode::InvalidArgument, "scale: centre must be finite and factor finite and > 0");
    try {
        gp_Trsf t;
        t.SetScale(gp_Pnt(center.x, center.y, center.z), factor);
        return applyTopologyPreserving(shape, t, "scale", std::move(cancel));
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(e, "scale"));
    }
}

} // namespace iw::kernel
