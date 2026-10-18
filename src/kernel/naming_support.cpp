// SPDX-License-Identifier: GPL-3.0-or-later
#include "kernel/naming_support.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>

#include "kernel/internal/profile_support.hpp"
#include "kernel/internal/shape_impl.hpp"

namespace iw::kernel {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool finitePositive(double v) { return std::isfinite(v) && v > 0.0; }
bool finiteVec(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

/// Unit vector perpendicular to unit `n` (world axis least aligned with n); same rule as the kernel's.
Vec3 perpendicularTo(const Vec3& n) {
    Vec3 ax{1, 0, 0};
    double best = std::abs(n.x);
    if (std::abs(n.y) < best) { ax = {0, 1, 0}; best = std::abs(n.y); }
    if (std::abs(n.z) < best) { ax = {0, 0, 1}; }
    const Vec3 p = ax - n * dot(ax, n);
    return p * (1.0 / length(p));
}

double rounded(double v) {
    const double q = std::round(v * 1e9) / 1e9;
    return q == 0.0 ? 0.0 : q;
}

} // namespace

std::array<double, 4> stableSubShapeKey(const Shape& shape, SubShapeKind kind, std::size_t index) {
    auto key = shape.subShapeKey(kind, index);
    if (kind != SubShapeKind::Edge || shape.isNull() || key[3] > 1e-9) return key;
    // Zero-length edge: GProp reports no centre of mass (origin). Use where the edge actually is.
    try {
        const auto& map = shape.impl()->index->map(kind);
        if (index >= static_cast<std::size_t>(map.Extent())) return key;
        const TopoDS_Edge e = TopoDS::Edge(map.FindKey(static_cast<int>(index) + 1));
        const TopoDS_Vertex v = TopExp::FirstVertex(e, /*CumOri=*/Standard_False);
        if (v.IsNull()) return key;
        const gp_Pnt p = BRep_Tool::Pnt(v);
        return {rounded(p.X()), rounded(p.Y()), rounded(p.Z()), 0.0};
    } catch (const Standard_Failure&) {
        return key;
    }
}

Result<HoleDetail> holeWithTool(const Shape& shape, const Axis& placement, const HoleSpec& spec, CancelToken cancel) {
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "hole: null shape");
    if (shape.type() != ShapeType::Solid) return makeError(ErrorCode::InvalidArgument, "hole: target must be a solid");
    if (!finiteVec(placement.origin) || !finiteVec(placement.direction) || length(placement.direction) < 1e-12)
        return makeError(ErrorCode::InvalidArgument, "hole: placement needs a finite origin and a non-zero direction");
    if (!finitePositive(spec.diameter)) return makeError(ErrorCode::InvalidArgument, "hole: diameter must be finite and > 0");
    if (!spec.through && !finitePositive(spec.depth))
        return makeError(ErrorCode::InvalidArgument, "hole: depth must be finite and > 0 (or through)");
    const bool drillOk = spec.drillPointAngle > 0.0 && spec.drillPointAngle <= kPi;
    if (!spec.through && !drillOk) return makeError(ErrorCode::InvalidArgument, "hole: drill point angle must be in (0, pi]");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "hole cancelled");

    const Vec3 o = placement.origin;
    const Vec3 d = placement.direction * (1.0 / length(placement.direction));
    const double r = spec.diameter / 2.0;
    const double margin = std::max(1.0, spec.diameter);

    const Box3 bb = shape.boundingBox();
    double tMax = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < 8; ++i) {
        const Vec3 corner{(i & 1) ? bb.max.x : bb.min.x, (i & 2) ? bb.max.y : bb.min.y, (i & 4) ? bb.max.z : bb.min.z};
        tMax = std::max(tMax, dot(corner - o, d));
    }
    if (spec.through && tMax <= 0.0)
        return makeError(ErrorCode::InvalidArgument, "hole: entry point lies beyond the part along the drill direction");
    const double zEnd = spec.through ? tMax + margin : spec.depth;
    const bool flatBottom = spec.drillPointAngle >= kPi - 1e-9;
    const double coneHeight = spec.through ? 0.0 : r / std::tan(spec.drillPointAngle / 2.0);
    const double tip = spec.through ? zEnd : spec.depth + coneHeight;
    const char* bottomRole = spec.through ? "end" : (flatBottom ? "floor" : "drill_cone");

    struct Seg { double radial, axial; const char* role; }; // vertex, and the role of the segment that STARTS at it
    std::vector<Seg> segs;
    const double endAxial = spec.through ? zEnd : spec.depth;
    switch (spec.kind) {
    case HoleKind::Simple:
        segs = {{0, -margin, "top"}, {r, -margin, "wall"}, {r, endAxial, bottomRole}, {0, spec.through ? zEnd : tip, "axis"}};
        break;
    case HoleKind::Counterbore: {
        if (!(spec.counterboreDiameter > spec.diameter))
            return makeError(ErrorCode::InvalidArgument, "hole: counterbore diameter must exceed the hole diameter");
        if (!finitePositive(spec.counterboreDepth))
            return makeError(ErrorCode::InvalidArgument, "hole: counterbore depth must be finite and > 0");
        if (!spec.through && !(spec.counterboreDepth < spec.depth))
            return makeError(ErrorCode::InvalidArgument, "hole: counterbore must be shallower than the hole depth");
        const double R = spec.counterboreDiameter / 2.0;
        segs = {{0, -margin, "top"},          {R, -margin, "cbore_wall"},          {R, spec.counterboreDepth, "cbore_floor"},
                {r, spec.counterboreDepth, "wall"}, {r, endAxial, bottomRole}, {0, spec.through ? zEnd : tip, "axis"}};
        break;
    }
    case HoleKind::Countersink: {
        if (!(spec.countersinkDiameter > spec.diameter))
            return makeError(ErrorCode::InvalidArgument, "hole: countersink diameter must exceed the hole diameter");
        if (!(spec.countersinkAngle > 0.0 && spec.countersinkAngle < kPi))
            return makeError(ErrorCode::InvalidArgument, "hole: countersink angle must be in (0, pi)");
        const double R = spec.countersinkDiameter / 2.0;
        const double t = std::tan(spec.countersinkAngle / 2.0);
        const double zc = (R - r) / t;
        const double limit = spec.through ? zEnd : spec.depth;
        if (!(zc < limit)) return makeError(ErrorCode::InvalidArgument, "hole: countersink is deeper than the hole");
        const double rTop = R + margin * t;
        segs = {{0, -margin, "top"}, {rTop, -margin, "csink_cone"}, {r, zc, "wall"}, {r, endAxial, bottomRole},
                {0, spec.through ? zEnd : tip, "axis"}};
        break;
    }
    }

    HoleDetail out;
    const Vec3 x = perpendicularTo(d);
    for (const Seg& s : segs) {
        out.profile.points.push_back(o + d * s.axial + x * s.radial);
        out.profile.segmentRoles.emplace_back(s.role);
    }
    try {
        auto face = makePolygonFace(out.profile.points);
        if (!face) return makeError(ErrorCode::Internal, "hole: could not build tool profile: " + face.error().message);
        out.profileFace = *face;
        auto tool = revolve(*face, Axis{o, d}, kTwoPi, cancel);
        if (!tool) return std::unexpected(tool.error());
        auto result = cut(shape, tool->shape, cancel);
        if (!result && result.error().code == ErrorCode::KernelFailure)
            return makeError(ErrorCode::KernelFailure, "hole: cut failed (" + result.error().message + ")");
        if (!result) return std::unexpected(result.error());
        out.tool = std::move(*tool);
        out.result = std::move(*result);
        return out;
    } catch (const Standard_Failure& e) {
        return makeError(ErrorCode::KernelFailure, std::string("hole: ") + (e.GetMessageString() ? e.GetMessageString() : "OCCT exception"));
    }
}

} // namespace iw::kernel
