// SPDX-License-Identifier: GPL-3.0-or-later
// Generators + predictor for features::RevolveFeature (polygon profiles about a construction axis or a body edge) and
// for the construction axis feature the revolves use.
//
// A revolve's profile lies in a plane that contains the axis: plane x direction = axis direction, so a profile point
// (s, r) is `o + d*s + yd*r` with `yd` the in-plane radial direction. Every generated element is then a surface of
// revolution whose type follows from its source profile edge: parallel to the axis -> cylinder (radius r), normal
// to it -> plane perpendicular to the axis, otherwise a cone about the axis. The prediction states exactly that.
#include <algorithm>
#include <cmath>
#include <numbers>

#include "features/construction_features.hpp"
#include "features/revolve_feature.hpp"
#include "fuzz/generator.hpp"
#include "gen_util.hpp"
#include "kernel/ops.hpp"

namespace iw::fuzz {
namespace {

using features::ConstructionAxisFeature;
using features::ConstructionAxisParams;
using features::PolygonSource;
using features::RevolveAxis;
using features::RevolveFeature;
using features::RevolveParams;
using naming::ElementName;
using naming::ElementType;

constexpr double kPi = std::numbers::pi;

struct AxisFrame {
    Vec3 o, d, n, yd; ///< axis origin, unit direction, profile plane normal, in-plane radial direction
};

AxisFrame frameFor(const Vec3& o, const Vec3& dirIn) {
    AxisFrame f;
    f.o = o;
    f.d = unit(dirIn);
    static const Vec3 kAxes[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    int best = 0;
    double bd = 2;
    for (int i = 0; i < 3; ++i) {
        const double v = std::abs(dot(kAxes[i], f.d));
        if (v < bd - 1e-12) { bd = v; best = i; }
    }
    f.n = unit(cross(f.d, kAxes[best]));
    f.yd = cross(f.n, f.d);
    return f;
}

/// Profile points (s, r) from a random shape that stays at r > 0.
std::vector<std::array<double, 2>> randomProfile(Rng& rng, double s0, double w, double r0, double h) {
    switch (rng.below(5)) {
    case 0: // right triangle: a cone and a disc
        return {{s0, r0}, {s0 + w, r0}, {s0, r0 + h}};
    case 1: // trapezoid: a cone between two discs
        return {{s0, r0}, {s0 + w, r0}, {s0 + w, r0 + h * 0.5}, {s0, r0 + h}};
    default: return {{s0, r0}, {s0 + w, r0}, {s0 + w, r0 + h}, {s0, r0 + h}};
    }
}


Vec3 rotatePoint(const Vec3& p, const Vec3& o, const Vec3& dIn, double ang) {
    const Vec3 k = unit(dIn);
    const Vec3 v = p - o;
    const double c = std::cos(ang), sn = std::sin(ang);
    return o + v * c + cross(k, v) * sn + k * (dot(k, v) * (1.0 - c));
}
Vec3 rotateDir(const Vec3& v, const Vec3& dIn, double ang) { return rotatePoint(v, Vec3{}, dIn, ang); }

GeomSig mkPlane(Vec3 centroid, double area, Vec3 normal, Vec3 point) {
    GeomSig s;
    s.type = ElementType::Face;
    s.key = {centroid.x, centroid.y, centroid.z, area};
    s.surface = SurfaceKind::Plane;
    s.normal = unit(normal);
    s.point = point;
    return s;
}
GeomSig mkLine(Vec3 a, Vec3 b) {
    GeomSig s;
    s.type = ElementType::Edge;
    const Vec3 m = (a + b) * 0.5;
    s.key = {m.x, m.y, m.z, length(b - a)};
    s.isLine = true;
    s.point = a;
    s.dir = unit(b - a);
    return s;
}
GeomSig mkPoint(Vec3 c) {
    GeomSig s;
    s.type = ElementType::Vertex;
    s.key = {c.x, c.y, c.z, 0.0};
    return s;
}

bool near(double a, double b, double tol = 1e-4) { return std::abs(a - b) <= tol + 1e-6 * std::max(std::abs(a), std::abs(b)); }

/// Geometry of a revolve in axis coordinates, shared by every prediction.
struct RevolveModel {
    Vec3 o, d;                     ///< axis
    std::vector<Vec3> P;           ///< profile vertices of the swept (start) profile, world, after the start offset
    std::vector<double> sP, rP;    ///< axial coordinate / distance from the axis of each profile vertex
    double theta = 2 * kPi;        ///< total sweep
    double alpha0 = 0;             ///< start offset
    bool full = true;
    Vec3 profileNormal;            ///< normal of the (rotated) start profile plane
    Vec3 planePoint;               ///< a point of the (rotated) start profile plane
    bool coplanar = true;          ///< profile plane contains the axis (an upstream edit can move the axis away)
    double area = 0;
    Vec3 centroid;
};

std::optional<RevolveModel> modelOf(const RevolveParams& p, const PredictContext& ctx) {
    const auto* poly = std::get_if<PolygonSource>(&p.source);
    if (!poly || poly->points.size() < 3) return std::nullopt;
    RevolveModel m;
    switch (p.axis.kind) {
    case RevolveAxis::Kind::Construction: {
        auto it = ctx.in.constructions.find(p.axis.construction);
        if (it == ctx.in.constructions.end()) return std::nullopt;
        m.o = it->second.origin;
        m.d = unit(it->second.direction);
        break;
    }
    case RevolveAxis::Kind::Edge: {
        auto shape = ctx.in.bodies.find(p.body);
        auto idx = ctx.indexOf(p.body, p.axis.edge.name);
        if (shape == ctx.in.bodies.end() || !idx) return std::nullopt;
        auto ax = kernel::axisFromEdge(shape->second, *idx);
        if (!ax) return std::nullopt;
        m.o = ax->origin;
        m.d = unit(ax->direction);
        break;
    }
    case RevolveAxis::Kind::SketchLine: return std::nullopt;
    }
    const double a1 = parseDeg(p.angle).value_or(360.0) * kPi / 180.0;
    const double a2 = parseDeg(p.angle2).value_or(0.0) * kPi / 180.0;
    m.full = p.extent == RevolveParams::Extent::Full;
    m.theta = 2 * kPi;
    m.alpha0 = 0;
    switch (p.extent) {
    case RevolveParams::Extent::Full: break;
    case RevolveParams::Extent::OneSide: m.theta = a1; break;
    case RevolveParams::Extent::Symmetric: m.theta = a1; m.alpha0 = -a1 * 0.5; break;
    case RevolveParams::Extent::TwoSides: m.theta = a1 + a2; m.alpha0 = -a2; break;
    }
    if (m.theta > 2 * kPi) m.theta = 2 * kPi;
    // Plane frame exactly as RevolveFeature builds it.
    const Vec3 n = unit(poly->planeNormal);
    Vec3 xd = poly->planeXDir - n * dot(poly->planeXDir, n);
    xd = unit(xd);
    const Vec3 yd = cross(n, xd);
    double a2s = 0, cx = 0, cy = 0;
    const std::size_t cnt = poly->points.size();
    for (std::size_t j = 0; j < cnt; ++j) {
        const auto& q0 = poly->points[j];
        const auto& q1 = poly->points[(j + 1) % cnt];
        const double cr = q0[0] * q1[1] - q1[0] * q0[1];
        a2s += cr;
        cx += (q0[0] + q1[0]) * cr;
        cy += (q0[1] + q1[1]) * cr;
        m.P.push_back(poly->planeOrigin + xd * q0[0] + yd * q0[1]);
    }
    if (std::abs(a2s) < 1e-12) return std::nullopt;
    m.area = std::abs(a2s) / 2.0;
    Vec3 c0 = poly->planeOrigin + xd * (cx / (3.0 * a2s)) + yd * (cy / (3.0 * a2s));
    for (const Vec3& v : m.P) {
        m.sP.push_back(dot(v - m.o, m.d));
        m.rP.push_back(distToLine(v, m.o, m.d));
    }
    m.coplanar = std::abs(dot(n, m.o - poly->planeOrigin)) < 1e-6 && std::abs(dot(n, m.d)) < 1e-6;
    m.planePoint = rotatePoint(poly->planeOrigin, m.o, m.d, m.alpha0);
    m.profileNormal = rotateDir(n, m.d, m.alpha0);
    m.centroid = rotatePoint(c0, m.o, m.d, m.alpha0);
    for (Vec3& v : m.P) v = rotatePoint(v, m.o, m.d, m.alpha0);
    return m;
}

/// Surface of revolution generated by profile edge j (A -> B), with the exact area for the sweep.
GeomSig sideSig(const RevolveModel& m, std::size_t j) {
    const std::size_t k = (j + 1) % m.P.size();
    const double sA = m.sP[j], sB = m.sP[k], rA = m.rP[j], rB = m.rP[k];
    GeomSig s;
    s.type = ElementType::Face;
    // Axial position of the area centroid (segment parametrised by t, weight r(t) per unit length).
    double num = 0, den = 0;
    for (int i = 0; i < 2000; ++i) { // exact enough: midpoint rule over the linear profile
        const double t = (i + 0.5) / 2000.0;
        const double r = rA + (rB - rA) * t, ss = sA + (sB - sA) * t;
        num += ss * r;
        den += r;
    }
    const double sCen = den > 0 ? num / den : sA;
    s.key[0] = m.o.x + m.d.x * sCen; s.key[1] = m.o.y + m.d.y * sCen; s.key[2] = m.o.z + m.d.z * sCen;
    if (near(rA, rB, 1e-9)) {
        s.surface = SurfaceKind::Cylinder;
        s.normal = m.d; s.point = m.o; s.radius = rA;
        s.key[3] = m.theta * rA * std::abs(sB - sA);
    } else if (near(sA, sB, 1e-9)) {
        s.surface = SurfaceKind::Plane;
        s.normal = m.d; s.point = m.o + m.d * sA;
        s.key[3] = m.theta * 0.5 * std::abs(rB * rB - rA * rA);
    } else {
        s.surface = SurfaceKind::Cone;
        s.normal = m.d; s.point = m.o;
        s.key[3] = m.theta * 0.5 * (rA + rB) * std::hypot(sB - sA, rB - rA);
    }
    return s;
}

class AxisGenerator final : public FeatureGenerator {
public:
    std::string_view kind() const override { return "axis"; }
    int weight() const override { return 1; }

    std::shared_ptr<const recompute::Feature> generate(Rng& rng, FeatureId id, const GenContext&) const override {
        static const Vec3 kDirs[] = {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {1, 0, 1}, {0, 1, 1}};
        const Vec3 d = kDirs[rng.below(6)];
        const double k = rng.intIn(8, 20);
        const Vec3 a{static_cast<double>(rng.intIn(-10, 10)), static_cast<double>(rng.intIn(-10, 10)), static_cast<double>(rng.intIn(-10, 10))};
        const Vec3 b = a + d * k;
        ConstructionAxisParams p;
        p.kind = ConstructionAxisParams::Kind::TwoPoints;
        p.a = {mmText(a.x), mmText(a.y), mmText(a.z)};
        p.b = {mmText(b.x), mmText(b.y), mmText(b.z)};
        return std::make_shared<ConstructionAxisFeature>(id, std::move(p));
    }

    std::shared_ptr<const recompute::Feature> editDimension(Rng& rng, const recompute::Feature& f) const override {
        (void)rng; (void)f; // ConstructionAxisFeature does not expose its parameters: not editable
        return nullptr;
    }
};

class RevolveGenerator final : public FeatureGenerator {
public:
    std::string_view kind() const override { return "revolve"; }
    int weight() const override { return 4; }

    std::shared_ptr<const recompute::Feature> generate(Rng& rng, FeatureId id, const GenContext& ctx) const override {
        RevolveParams p;
        PolygonSource poly;
        const bool haveBodies = !ctx.state.bodies.empty();

        // Axis: a body edge (join / cut around it) or a construction axis (any operation).
        std::vector<FeatureId> axes;
        for (const auto& [fid, g] : ctx.state.constructions)
            if (g.kind == recompute::ConstructionGeometry::Kind::Axis) axes.push_back(fid);

        AxisFrame fr;
        double sLo = -4, sHi = 12, rLo = 1.0;
        const bool useEdge = haveBodies && (axes.empty() || rng.chance(0.65));
        if (useEdge) {
            auto pick = util::pickElement(rng, ctx, ElementType::Edge,
                                          [](BodyId, const kernel::Shape& shape, std::uint32_t idx, const ElementName&) {
                                              return shape.subShapeKey(kernel::SubShapeKind::Edge, idx)[3] >= 6.0 &&
                                                     kernel::axisFromEdge(shape, idx).has_value();
                                          });
            if (!pick) return nullptr;
            auto ax = kernel::axisFromEdge(ctx.state.bodies.at(pick->body), pick->index);
            if (!ax) return nullptr;
            fr = frameFor(ax->origin, ax->direction);
            const double len = ctx.state.bodies.at(pick->body).subShapeKey(kernel::SubShapeKind::Edge, pick->index)[3];
            sLo = -1;
            sHi = len;
            p.axis.kind = RevolveAxis::Kind::Edge;
            p.axis.edge = util::refOf(ctx, *pick);
            p.body = pick->body;
            p.operation = rng.chance(0.75) ? features::common::BodyOp::Join : features::common::BodyOp::Cut;
        } else if (!axes.empty()) {
            const FeatureId aid = axes[rng.below(axes.size())];
            const auto& g = ctx.state.constructions.at(aid);
            fr = frameFor(g.origin, g.direction);
            p.axis.kind = RevolveAxis::Kind::Construction;
            p.axis.construction = aid;
            if (!haveBodies || rng.chance(0.55)) {
                p.operation = features::common::BodyOp::NewBody;
                p.body = BodyId{static_cast<std::uint64_t>(rng.intIn(1, 3))};
            } else {
                std::vector<BodyId> bodies;
                for (const auto& [b, shape] : ctx.state.bodies) bodies.push_back(b);
                p.body = bodies[rng.below(bodies.size())];
                p.operation = rng.chance(0.75) ? features::common::BodyOp::Join : features::common::BodyOp::Cut;
            }
        } else {
            return nullptr;
        }

        const double w = std::max(2.0, std::min(sHi - sLo, static_cast<double>(rng.intIn(3, 12))));
        const double s0 = util::round1(sLo + (sHi - sLo - w) * rng.unit() * 0.5);
        const double r0 = util::round1(rLo + rng.real(0.0, 3.0));
        const double h = rng.intIn(2, 8);
        poly.planeOrigin = fr.o;
        poly.planeNormal = fr.n;
        poly.planeXDir = fr.d;
        poly.points = randomProfile(rng, s0, w, r0, h);
        p.source = poly;

        switch (rng.below(8)) {
        case 0:
        case 1:
        case 2:
        case 3: p.extent = RevolveParams::Extent::Full; break;
        case 4:
        case 5:
            p.extent = RevolveParams::Extent::OneSide;
            p.angle = degText(static_cast<double>(rng.intIn(3, 17) * 15));
            break;
        case 6:
            p.extent = RevolveParams::Extent::Symmetric;
            p.angle = degText(static_cast<double>(rng.intIn(3, 17) * 15));
            break;
        default:
            p.extent = RevolveParams::Extent::TwoSides;
            p.angle = degText(static_cast<double>(rng.intIn(2, 8) * 15));
            p.angle2 = degText(static_cast<double>(rng.intIn(2, 8) * 15));
            break;
        }
        return std::make_shared<RevolveFeature>(id, std::move(p));
    }

    std::shared_ptr<const recompute::Feature> editDimension(Rng& rng, const recompute::Feature& f) const override {
        if (f.kind() != "revolve") return nullptr;
        RevolveParams p = static_cast<const RevolveFeature&>(f).params();
        auto* poly = std::get_if<PolygonSource>(&p.source);
        if (!poly) return nullptr;
        if (p.extent != RevolveParams::Extent::Full && rng.chance(0.5)) {
            p.angle = degText(static_cast<double>(rng.intIn(3, 17) * 15));
        } else if (rng.chance(0.5)) {
            const double k = rng.chance(0.5) ? 0.75 : 1.5; // scale along the axis
            for (auto& q : poly->points) q[0] *= k;
        } else {
            const double k = rng.chance(0.5) ? 0.8 : 1.25; // scale the radius (stays positive)
            for (auto& q : poly->points) q[1] *= k;
        }
        return std::make_shared<RevolveFeature>(f.id(), std::move(p));
    }

        Expected sideExpected(const RevolveModel& m, std::size_t j) const {
        Expected e;
        e.sig = sideSig(m, j);
        e.mask = mask::Measure | (e.sig.surface == SurfaceKind::Plane ? mask::SurfaceFlip : mask::Surface);
        if (m.full) e.mask |= mask::Centroid;
        return e;
    }

    std::optional<std::vector<Expected>> predict(const recompute::Feature& f, const ElementName& name,
                                                 const PredictContext& ctx) const override {
        const RevolveParams& p = static_cast<const RevolveFeature&>(f).params();
        auto mm = modelOf(p, ctx);
        if (!mm) return std::nullopt;
        const RevolveModel m = *mm;
        const std::size_t n = m.P.size();
        const std::string& op = name.op;
        const ElementName* src = name.sources.size() == 1 ? &name.sources[0] : nullptr;
        const bool newBody = p.operation == features::common::BodyOp::NewBody;
        std::vector<Expected> out;

        auto axial = [m](const GeomSig& g) { return dot(g.centroid() - m.o, m.d); };
        auto radial = [m](const GeomSig& g) { return distToLine(g.centroid(), m.o, m.d); };
        // Vertex j of the swept profile (any angle): fixed axial position and radius.
        auto vertexOk = [=](std::size_t j) {
            return [=](const GeomSig& g) { return near(axial(g), m.sP[j]) && near(radial(g), m.rP[j]); };
        };
        // Circle / arc generated by vertex j.
        auto arcOk = [=](std::size_t j, bool relax) {
            return [=](const GeomSig& g) {
                if (!near(axial(g), m.sP[j])) return false;
                if (relax) return radial(g) <= m.rP[j] + 1e-4 && g.measure() <= 2 * kPi * m.rP[j] + 1e-4;
                if (m.full) return near(radial(g), 0.0) && near(g.measure(), 2 * kPi * m.rP[j]);
                return radial(g) <= m.rP[j] + 1e-4 && near(g.measure(), m.theta * m.rP[j]);
            };
        };
        // Copy of profile edge j (any angle): its (axial, radius) lies on the profile segment.
        auto edgeOk = [=](std::size_t j, bool relax) {
            const std::size_t k = (j + 1) % n;
            return [=](const GeomSig& g) {
                const double a = axial(g), r = radial(g);
                if (!relax) {
                    const Vec3 mid = (m.P[j] + m.P[k]) * 0.5;
                    return near(a, dot(mid - m.o, m.d)) && near(r, distToLine(mid, m.o, m.d)) &&
                           near(g.measure(), length(m.P[k] - m.P[j]));
                }
                if (!m.coplanar) return a >= std::min(m.sP[j], m.sP[k]) - 1e-4 && a <= std::max(m.sP[j], m.sP[k]) + 1e-4;
                const double ds = m.sP[k] - m.sP[j], dr = m.rP[k] - m.rP[j];
                const double len = std::hypot(ds, dr);
                if (len < 1e-12) return false;
                if (std::abs((a - m.sP[j]) * dr - (r - m.rP[j]) * ds) / len > 1e-4) return false; // on the segment's line
                const double t = ((a - m.sP[j]) * ds + (r - m.rP[j]) * dr) / (len * len);
                if (t < -1e-6 || t > 1 + 1e-6) return false;
                return relax ? g.measure() <= len + 1e-4 : near(g.measure(), len);
            };
        };
        auto custom = [&](ElementType t, std::function<bool(const GeomSig&)> exact, std::function<bool(const GeomSig&)> relaxedFn) {
            Expected e;
            e.sig.type = t;
            e.note = "revolve element";
            e.extra = std::move(exact);
            e.relaxedExtra = std::move(relaxedFn);
            return e;
        };

        if (op == "profile.face" || op == "revolve.cap") {
            const double ang = op == "profile.face" ? 0.0 : m.theta;
            const Vec3 c = rotatePoint(m.centroid, m.o, m.d, ang);
            Expected e = exactly(mkPlane(c, m.area, rotateDir(m.profileNormal, m.d, ang), rotatePoint(m.planePoint, m.o, m.d, ang)));
            e.mask = (e.mask & ~mask::Surface) | mask::SurfaceFlip;
            out.push_back(e);
        } else if (op == "profile.edge") {
            for (std::size_t j = 0; j < n; ++j) out.push_back(exactly(mkLine(m.P[j], m.P[(j + 1) % n])));
        } else if (op == "profile.vertex") {
            for (std::size_t j = 0; j < n; ++j) out.push_back(exactly(mkPoint(m.P[j])));
        } else if (op == "revolve.side" && src && src->op == "profile.edge") {
            if (!m.coplanar) return std::nullopt; // profile plane no longer contains the axis: general surface of revolution
            for (std::size_t j = 0; j < n; ++j) out.push_back(sideExpected(m, j));
        } else if (op == "revolve.edge" || op == "revolve.new") {
            if (name.type == ElementType::Edge) {
                for (std::size_t j = 0; j < n; ++j) {
                    out.push_back(custom(ElementType::Edge, arcOk(j, false), arcOk(j, true)));
                    out.push_back(custom(ElementType::Edge, edgeOk(j, false), edgeOk(j, true)));
                }
            } else if (name.type == ElementType::Vertex) {
                for (std::size_t j = 0; j < n; ++j) out.push_back(custom(ElementType::Vertex, vertexOk(j), vertexOk(j)));
            } else {
                return std::nullopt;
            }
        } else if (op == "revolve.vertex") {
            for (std::size_t j = 0; j < n; ++j) out.push_back(custom(ElementType::Vertex, vertexOk(j), vertexOk(j)));
        } else {
            return std::nullopt;
        }
        // A boolean trims / splits / merges the tool's elements: keep only what survives (see relaxed()).
        if (!newBody) {
            for (Expected& e : out) {
                e = relaxed(e);
                if (p.operation == features::common::BodyOp::Cut && e.sig.type == ElementType::Face) e.mask = mask::SurfaceFlip;
            }
        }
        return out;
    }

    int volumeRelation(const recompute::Feature& f) const override {
        switch (static_cast<const RevolveFeature&>(f).params().operation) {
        case features::common::BodyOp::Join: return 1;
        case features::common::BodyOp::Cut: return -1;
        default: return 0;
        }
    }

    std::optional<BodyId> inheritsFrom(const recompute::Feature& f) const override {
        const RevolveParams& p = static_cast<const RevolveFeature&>(f).params();
        if (p.operation == features::common::BodyOp::NewBody) return std::nullopt;
        return p.body;
    }
};

} // namespace

std::shared_ptr<const FeatureGenerator> makeRevolveGenerator() { return std::make_shared<RevolveGenerator>(); }
std::shared_ptr<const FeatureGenerator> makeAxisGenerator() { return std::make_shared<AxisGenerator>(); }

} // namespace iw::fuzz
