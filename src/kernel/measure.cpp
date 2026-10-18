// SPDX-License-Identifier: GPL-3.0-or-later
#include "kernel/measure.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <GProp_PrincipalProps.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include "kernel/internal/shape_impl.hpp"
#include "kernel/occt_access.hpp"

namespace iw::kernel::measure {

namespace {

constexpr double kRadToDeg = 180.0 / std::numbers::pi;
constexpr double kMmToM = 1e-3;

/// kg per mm^3 for a density given in kg/m^3.
double kgPerMm3(double densityKgPerM3) { return densityKgPerM3 * kMmToM * kMmToM * kMmToM; }

Vec3 toVec(const gp_Pnt& p) { return {p.X(), p.Y(), p.Z()}; }

SubShapeKind kindOf(EntityKind k) {
    switch (k) {
    case EntityKind::Vertex: return SubShapeKind::Vertex;
    case EntityKind::Edge: return SubShapeKind::Edge;
    default: return SubShapeKind::Face;
    }
}

/// OCCT sub-shape addressed by `e` (ordering contract of kernel/shape.hpp). Body entities return the whole shape.
Result<TopoDS_Shape> resolve(const Entity& e) {
    if (e.body.isNull()) return makeError(ErrorCode::InvalidArgument, "measure: null shape");
    if (e.kind == EntityKind::Body) return occt::get(e.body);
    const SubShapeKind k = kindOf(e.kind);
    if (e.index >= e.body.subShapeCount(k))
        return makeError(ErrorCode::NotFound, "measure: sub-shape index " + std::to_string(e.index) + " out of range");
    return e.body.impl()->index->map(k).FindKey(static_cast<int>(e.index) + 1);
}

Result<gp_Dir> planeNormal(const TopoDS_Face& face) {
    BRepAdaptor_Surface ad(face);
    if (ad.GetType() != GeomAbs_Plane)
        return makeError(ErrorCode::InvalidArgument, "measure: angle needs planar faces");
    gp_Dir d = ad.Plane().Axis().Direction();
    if (face.Orientation() == TopAbs_REVERSED) d.Reverse();
    return d;
}

Result<gp_Dir> lineDirection(const TopoDS_Edge& edge) {
    BRepAdaptor_Curve ad(edge);
    if (ad.GetType() != GeomAbs_Line)
        return makeError(ErrorCode::InvalidArgument, "measure: angle needs linear edges");
    return ad.Line().Direction();
}

/// Unoriented angle between two lines, in radians, folded into [0, pi/2].
double lineAngle(const gp_Dir& a, const gp_Dir& b) {
    const double t = a.Angle(b);
    return std::min(t, std::numbers::pi - t);
}

Result<void> requireSolid(const TopoDS_Shape& s) {
    if (s.IsNull()) return makeError(ErrorCode::InvalidArgument, "measure: null shape");
    if (!TopExp_Explorer(s, TopAbs_SOLID).More())
        return makeError(ErrorCode::InvalidArgument, "measure: body has no solid");
    return {};
}

} // namespace

Result<Closest> minimumDistance(const Entity& a, const Entity& b) {
    auto sa = resolve(a);
    if (!sa) return std::unexpected(sa.error());
    auto sb = resolve(b);
    if (!sb) return std::unexpected(sb.error());
    try {
        BRepExtrema_DistShapeShape dist(*sa, *sb);
        if (!dist.IsDone()) return makeError(ErrorCode::KernelFailure, "minimumDistance: OCCT distance failed");
        Closest c;
        c.distance = dist.Value();
        if (dist.NbSolution() >= 1) {
            c.onA = toVec(dist.PointOnShape1(1));
            c.onB = toVec(dist.PointOnShape2(1));
        } else if (c.distance <= 0) {
            // Overlapping shapes: OCCT reports no witness pair. Report distance 0 at A's box centre.
            const Box3 box = a.body.boundingBox();
            c.onA = c.onB = (box.min + box.max) * 0.5;
        } else {
            return makeError(ErrorCode::KernelFailure, "minimumDistance: no closest pair found");
        }
        return c;
    } catch (const Standard_Failure& e) {
        return makeError(ErrorCode::KernelFailure, std::string("minimumDistance: ") + e.GetMessageString());
    }
}

Result<double> angleBetween(const Entity& a, const Entity& b) {
    auto sa = resolve(a);
    if (!sa) return std::unexpected(sa.error());
    auto sb = resolve(b);
    if (!sb) return std::unexpected(sb.error());
    try {
        if (a.kind == EntityKind::Face && b.kind == EntityKind::Face) {
            auto na = planeNormal(TopoDS::Face(*sa));
            if (!na) return std::unexpected(na.error());
            auto nb = planeNormal(TopoDS::Face(*sb));
            if (!nb) return std::unexpected(nb.error());
            return na->Angle(*nb) * kRadToDeg;
        }
        if (a.kind == EntityKind::Edge && b.kind == EntityKind::Edge) {
            auto da = lineDirection(TopoDS::Edge(*sa));
            if (!da) return std::unexpected(da.error());
            auto db = lineDirection(TopoDS::Edge(*sb));
            if (!db) return std::unexpected(db.error());
            return lineAngle(*da, *db) * kRadToDeg;
        }
        const bool aEdge = a.kind == EntityKind::Edge && b.kind == EntityKind::Face;
        const bool bEdge = b.kind == EntityKind::Edge && a.kind == EntityKind::Face;
        if (aEdge || bEdge) {
            const TopoDS_Shape& edgeShape = aEdge ? *sa : *sb;
            const TopoDS_Shape& faceShape = aEdge ? *sb : *sa;
            auto de = lineDirection(TopoDS::Edge(edgeShape));
            if (!de) return std::unexpected(de.error());
            auto nf = planeNormal(TopoDS::Face(faceShape));
            if (!nf) return std::unexpected(nf.error());
            // Angle between a line and a plane: 90 deg minus the angle between the line and the plane normal.
            return (std::numbers::pi / 2.0 - lineAngle(*de, *nf)) * kRadToDeg;
        }
        return makeError(ErrorCode::InvalidArgument, "measure: no angle for this pair of entity kinds");
    } catch (const Standard_Failure& e) {
        return makeError(ErrorCode::KernelFailure, std::string("angleBetween: ") + e.GetMessageString());
    }
}

Result<Vec3> vertexPoint(const Entity& vertex) {
    if (vertex.kind != EntityKind::Vertex) return makeError(ErrorCode::InvalidArgument, "vertexPoint: not a vertex");
    auto s = resolve(vertex);
    if (!s) return std::unexpected(s.error());
    try {
        return toVec(BRep_Tool::Pnt(TopoDS::Vertex(*s)));
    } catch (const Standard_Failure& e) {
        return makeError(ErrorCode::KernelFailure, std::string("vertexPoint: ") + e.GetMessageString());
    }
}

Result<double> edgeLength(const Entity& edge) {
    if (edge.kind != EntityKind::Edge) return makeError(ErrorCode::InvalidArgument, "edgeLength: not an edge");
    auto s = resolve(edge);
    if (!s) return std::unexpected(s.error());
    try {
        GProp_GProps p;
        BRepGProp::LinearProperties(*s, p);
        return p.Mass();
    } catch (const Standard_Failure& e) {
        return makeError(ErrorCode::KernelFailure, std::string("edgeLength: ") + e.GetMessageString());
    }
}

Result<double> faceArea(const Entity& face) {
    if (face.kind != EntityKind::Face) return makeError(ErrorCode::InvalidArgument, "faceArea: not a face");
    auto s = resolve(face);
    if (!s) return std::unexpected(s.error());
    try {
        GProp_GProps p;
        BRepGProp::SurfaceProperties(*s, p);
        return p.Mass();
    } catch (const Standard_Failure& e) {
        return makeError(ErrorCode::KernelFailure, std::string("faceArea: ") + e.GetMessageString());
    }
}

Result<CircleInfo> circleOf(const Entity& edge) {
    if (edge.kind != EntityKind::Edge) return makeError(ErrorCode::InvalidArgument, "circleOf: not an edge");
    auto s = resolve(edge);
    if (!s) return std::unexpected(s.error());
    try {
        BRepAdaptor_Curve ad(TopoDS::Edge(*s));
        if (ad.GetType() != GeomAbs_Circle)
            return makeError(ErrorCode::InvalidArgument, "circleOf: edge is not a circle");
        const gp_Circ c = ad.Circle();
        CircleInfo info;
        info.centre = toVec(c.Location());
        const gp_Dir n = c.Axis().Direction();
        info.normal = {n.X(), n.Y(), n.Z()};
        info.radius = c.Radius();
        info.diameter = 2.0 * c.Radius();
        return info;
    } catch (const Standard_Failure& e) {
        return makeError(ErrorCode::KernelFailure, std::string("circleOf: ") + e.GetMessageString());
    }
}

Result<BodyProperties> bodyProperties(const Shape& body, double densityKgPerM3) {
    if (densityKgPerM3 < 0 || !std::isfinite(densityKgPerM3))
        return makeError(ErrorCode::InvalidArgument, "bodyProperties: density must be finite and >= 0");
    if (body.isNull()) return makeError(ErrorCode::InvalidArgument, "bodyProperties: null shape");
    const TopoDS_Shape s = occt::get(body);
    if (auto ok = requireSolid(s); !ok) return std::unexpected(ok.error());
    try {
        GProp_GProps vol, surf;
        BRepGProp::VolumeProperties(s, vol);
        BRepGProp::SurfaceProperties(s, surf);
        BodyProperties bp;
        bp.volume = vol.Mass();
        bp.surfaceArea = surf.Mass();
        bp.centreOfMass = toVec(vol.CentreOfMass());
        bp.boundingBox = body.boundingBox();
        bp.density = densityKgPerM3;
        bp.mass = bp.volume * kgPerMm3(densityKgPerM3);
        double ix = 0, iy = 0, iz = 0;
        GProp_PrincipalProps pp = vol.PrincipalProperties();
        pp.Moments(ix, iy, iz);
        const double k = kgPerMm3(densityKgPerM3);
        bp.principalMoments = {ix * k, iy * k, iz * k};
        return bp;
    } catch (const Standard_Failure& e) {
        return makeError(ErrorCode::KernelFailure, std::string("bodyProperties: ") + e.GetMessageString());
    }
}

namespace {

void addPoint(std::vector<MeasureRow>& rows, const std::string& prefix, const Vec3& p) {
    rows.push_back({prefix + " X", p.x, "mm"});
    rows.push_back({prefix + " Y", p.y, "mm"});
    rows.push_back({prefix + " Z", p.z, "mm"});
}

} // namespace

Result<std::vector<MeasureRow>> describe(const Entity& e) {
    std::vector<MeasureRow> rows;
    switch (e.kind) {
    case EntityKind::Vertex: {
        auto p = vertexPoint(e);
        if (!p) return std::unexpected(p.error());
        addPoint(rows, "Position", *p);
        return rows;
    }
    case EntityKind::Edge: {
        auto len = edgeLength(e);
        if (!len) return std::unexpected(len.error());
        rows.push_back({"Length", *len, "mm"});
        if (auto c = circleOf(e)) {
            rows.push_back({"Radius", c->radius, "mm"});
            rows.push_back({"Diameter", c->diameter, "mm"});
            addPoint(rows, "Centre", c->centre);
        }
        return rows;
    }
    case EntityKind::Face: {
        auto area = faceArea(e);
        if (!area) return std::unexpected(area.error());
        rows.push_back({"Area", *area, "mm^2"});
        return rows;
    }
    case EntityKind::Body: {
        auto bp = bodyProperties(e.body, 0.0);
        if (!bp) return std::unexpected(bp.error());
        rows.push_back({"Volume", bp->volume, "mm^3"});
        rows.push_back({"Area", bp->surfaceArea, "mm^2"});
        return rows;
    }
    }
    return makeError(ErrorCode::Internal, "describe: unknown entity kind");
}

Result<std::vector<MeasureRow>> between(const Entity& a, const Entity& b) {
    auto c = minimumDistance(a, b);
    if (!c) return std::unexpected(c.error());
    std::vector<MeasureRow> rows;
    rows.push_back({"Distance", c->distance, "mm"});
    addPoint(rows, "Point A", c->onA);
    addPoint(rows, "Point B", c->onB);
    if (auto ang = angleBetween(a, b)) rows.push_back({"Angle", *ang, "deg"});
    return rows;
}

// ---------------------------------------------------------------------------------------------------------------
// Section analysis

namespace section {

Result<void> Model::setPlane(const Vec3& origin, const Vec3& normal) {
    const double len = length(normal);
    if (!std::isfinite(len) || len < 1e-12 || !std::isfinite(origin.x) || !std::isfinite(origin.y) ||
        !std::isfinite(origin.z))
        return makeError(ErrorCode::InvalidArgument, "section: plane normal must be finite and non-zero");
    origin_ = origin;
    normal_ = normal * (1.0 / len);
    return {};
}

Vec3 Model::effectiveNormal() const noexcept { return flipped_ ? normal_ * -1.0 : normal_; }

Vec3 Model::effectivePoint() const noexcept { return origin_ + normal_ * offset_; }

double Model::signedDistance(const Vec3& p) const noexcept {
    return dot(effectiveNormal(), p - effectivePoint());
}

RenderRequest Model::request() const noexcept {
    RenderRequest r;
    r.enabled = enabled_;
    const Vec3 n = effectiveNormal();
    const double d = -dot(n, effectivePoint());
    r.clipPlane = {n.x, n.y, n.z, d};
    r.capColour = cap_;
    return r;
}

} // namespace section

} // namespace iw::kernel::measure
