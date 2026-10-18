// SPDX-License-Identifier: GPL-3.0-or-later
// Generator + independent geometry prediction for features::ExtrudeFeature (polygon profiles).
//
// The prediction re-derives, from the feature's parameters alone, where each named tool element of an extrude must lie
// (it deliberately does not call the kernel). Element names follow the naming contract of naming/naming_map.cpp
// (roleOf): profile.face / profile.edge / profile.vertex on the start cap, and extrude.cap / extrude.side /
// extrude.edge / extrude.vertex for the generated parts. For Join / Cut the same tool names survive in the combined
// body, trimmed / split / merged, so the expectation is relaxed to the supporting surface / line / point.
//
// The "coplanar join" mode reads the planar faces of an existing body and builds a profile ON that face's plane, with
// edges snapped to the face's in-plane extent, so the boolean produces coplanar faces that must merge or split.
#include <algorithm>
#include <cmath>
#include <vector>

#include "features/extrude_feature.hpp"
#include "fuzz/generator.hpp"

namespace iw::fuzz {
namespace {

using features::ExtrudeFeature;
using features::ExtrudeParams;
using features::PolygonSource;
using naming::ElementName;
using naming::ElementType;

/// Plane frame exactly as ExtrudeFeature::compute builds it (orthonormal n, xd, yd).
struct Frame {
    Vec3 origin, n, xd, yd;
};

Frame frameOf(const PolygonSource& p) {
    Frame f;
    f.origin = p.planeOrigin;
    f.n = p.planeNormal * (1.0 / length(p.planeNormal));
    Vec3 xd = p.planeXDir - f.n * dot(p.planeXDir, f.n);
    f.xd = xd * (1.0 / length(xd));
    f.yd = cross(f.n, f.xd);
    return f;
}

Vec3 toWorld(const Frame& f, const std::array<double, 2>& q) { return f.origin + f.xd * q[0] + f.yd * q[1]; }

GeomSig pointSig(Vec3 c) {
    GeomSig s;
    s.type = ElementType::Vertex;
    s.key = {c.x, c.y, c.z, 0.0};
    return s;
}

GeomSig lineSig(Vec3 a, Vec3 b) {
    GeomSig s;
    s.type = ElementType::Edge;
    const Vec3 m = (a + b) * 0.5;
    s.key = {m.x, m.y, m.z, length(b - a)};
    s.isLine = true;
    s.point = a;
    s.dir = unit(b - a);
    return s;
}

GeomSig lineThrough(Vec3 a, Vec3 dir, double len, Vec3 centre) {
    GeomSig s;
    s.type = ElementType::Edge;
    s.key = {centre.x, centre.y, centre.z, len};
    s.isLine = true;
    s.point = a;
    s.dir = unit(dir);
    return s;
}

GeomSig planeSig(Vec3 centroid, double area, Vec3 normal, Vec3 point) {
    GeomSig s;
    s.type = ElementType::Face;
    s.key = {centroid.x, centroid.y, centroid.z, area};
    s.surface = SurfaceKind::Plane;
    s.normal = unit(normal);
    s.point = point;
    return s;
}

const ExtrudeFeature* asExtrude(const recompute::Feature& f) {
    return f.kind() == "extrude" ? static_cast<const ExtrudeFeature*>(&f) : nullptr;
}

/// The P0 polygon source of a feature; the fuzzer only generates polygon profiles.
const PolygonSource* polygonOf(const ExtrudeParams& p) { return std::get_if<PolygonSource>(&p.source); }

/// Deterministic in-plane x direction for a plane normal (the world axis least aligned with it).
Vec3 xDirFor(const Vec3& n) {
    static const Vec3 kAxes[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const Vec3 nn = unit(n);
    int best = 0;
    double bestDot = 2;
    for (int i = 0; i < 3; ++i) {
        const double d = std::abs(dot(kAxes[i], nn));
        if (d < bestDot - 1e-12) {
            bestDot = d;
            best = i;
        }
    }
    return kAxes[best];
}

/// A join / cut that builds its profile on a planar face of an existing body. nullptr if no planar face exists.
std::shared_ptr<const recompute::Feature> coplanarOnBody(Rng& rng, FeatureId id, const GenContext& ctx, bool cut) {
    std::vector<BodyId> candidates;
    for (const auto& [b, shape] : ctx.state.bodies) candidates.push_back(b);
    if (candidates.empty()) return nullptr;
    const BodyId body = candidates[rng.below(candidates.size())];
    const kernel::Shape& shape = ctx.state.bodies.at(body);

    std::vector<std::uint32_t> planar;
    const auto nf = shape.subShapeCount(kernel::SubShapeKind::Face);
    for (std::uint32_t i = 0; i < nf; ++i)
        if (shape.faceGeometry(i).kind == kernel::FaceGeometry::Kind::Plane) planar.push_back(i);
    if (planar.empty()) return nullptr;
    const std::uint32_t fi = planar[rng.below(planar.size())];
    const kernel::FaceGeometry fg = shape.faceGeometry(fi);

    PolygonSource poly;
    poly.planeNormal = unit(fg.normal);
    poly.planeXDir = xDirFor(poly.planeNormal);
    poly.planeOrigin = fg.point;
    const Frame fr = frameOf(poly);

    // In-plane extent of the body's vertices lying on the face's plane.
    double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
    const double d0 = dot(fr.n, fg.point);
    const auto nv = shape.subShapeCount(kernel::SubShapeKind::Vertex);
    for (std::uint32_t i = 0; i < nv; ++i) {
        const auto k = shape.subShapeKey(kernel::SubShapeKind::Vertex, i);
        const Vec3 p{k[0], k[1], k[2]};
        if (std::abs(dot(fr.n, p) - d0) > 1e-6) continue;
        const double u = dot(p - fr.origin, fr.xd), v = dot(p - fr.origin, fr.yd);
        umin = std::min(umin, u); umax = std::max(umax, u);
        vmin = std::min(vmin, v); vmax = std::max(vmax, v);
    }
    if (!(umax - umin > 1.0) || !(vmax - vmin > 1.0)) return nullptr;

    // One axis of the rectangle: aligned to a body edge (merge), spanning the full extent (split), or interior.
    auto range = [&](double lo, double hi, bool interior) -> std::pair<double, double> {
        const double ext = hi - lo;
        const double w = std::round(ext * rng.real(0.3, 0.9) * 2.0) / 2.0;
        if (interior) {
            const double a = std::round((lo + (ext - w) * rng.real(0.2, 0.8)) * 2.0) / 2.0;
            return {a, a + std::max(w, 1.0)};
        }
        switch (rng.below(4)) {
        case 0: return {lo, lo + std::max(w, 1.0)};
        case 1: return {hi - std::max(w, 1.0), hi};
        case 2: return {lo, hi};
        default: { // overhang beyond the body on one side
            const double m = rng.intIn(1, 4);
            return rng.chance(0.5) ? std::pair{lo - m, lo + std::max(w, 1.0)} : std::pair{hi - std::max(w, 1.0), hi + m};
        }
        }
    };
    const auto ur = range(umin, umax, cut);
    const auto vr = range(vmin, vmax, cut);
    poly.points = {{ur.first, vr.first}, {ur.second, vr.first}, {ur.second, vr.second}, {ur.first, vr.second}};

    // Join: mostly stack outwards (the tool's start cap lies on the face); sometimes sink inwards so the tool's end cap
    // is coplanar with the face. Cut: a pocket into the face.
    const double h = rng.intIn(2, 12);
    const bool outward = !cut && rng.chance(0.65);
    poly.vector = fr.n * (outward ? h : -h);

    ExtrudeParams p;
    p.source = poly;
    p.operation = cut ? ExtrudeParams::Operation::Cut : ExtrudeParams::Operation::Join;
    p.body = body;
    return std::make_shared<ExtrudeFeature>(id, std::move(p));
}

class ExtrudeGenerator final : public FeatureGenerator {
public:
    std::string_view kind() const override { return "extrude"; }
    int weight() const override { return 6; }

    std::shared_ptr<const recompute::Feature> generate(Rng& rng, FeatureId id, const GenContext& ctx) const override {
        if (!ctx.state.bodies.empty()) {
            const std::uint64_t roll = rng.below(100);
            if (roll < 55) {
                if (auto f = coplanarOnBody(rng, id, ctx, /*cut=*/false)) return f;
            } else if (roll < 65) {
                if (auto f = coplanarOnBody(rng, id, ctx, /*cut=*/true)) return f;
            }
        }
        ExtrudeParams p;
        PolygonSource poly;

        // Plane: mostly axis-aligned, sometimes tilted (exercises the frame orthogonalisation).
        static const Vec3 kNormals[] = {{0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 1, 1}};
        poly.planeNormal = kNormals[rng.below(6)];
        const Vec3 nn = poly.planeNormal * (1.0 / length(poly.planeNormal));
        static const Vec3 kAxes[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        std::vector<Vec3> xdirs;
        for (const Vec3& a : kAxes)
            if (std::abs(dot(a, nn)) < 0.9) xdirs.push_back(a);
        poly.planeXDir = xdirs[rng.below(xdirs.size())];
        poly.planeOrigin = {rng.intIn(-2, 2) * 10.0, rng.intIn(-2, 2) * 10.0, rng.intIn(-1, 1) * 10.0};

        // Profile: a rectangle or a convex polygon (points on a circle, sorted by angle), in plane coordinates.
        if (rng.chance(0.6)) {
            const double x0 = rng.intIn(-3, 3) * 5.0, y0 = rng.intIn(-3, 3) * 5.0;
            const double w = rng.intIn(2, 8) * 5.0, h = rng.intIn(2, 8) * 5.0;
            poly.points = {{x0, y0}, {x0 + w, y0}, {x0 + w, y0 + h}, {x0, y0 + h}};
        } else {
            const int k = rng.intIn(3, 5);
            const double cx = rng.intIn(-2, 2) * 5.0, cy = rng.intIn(-2, 2) * 5.0;
            const double r = rng.real(8.0, 25.0);
            const double step = 2.0 * 3.14159265358979323846 / k;
            for (int i = 0; i < k; ++i) {
                const double a = i * step + rng.real(0.15, 0.85) * step;
                poly.points.push_back({std::round((cx + r * std::cos(a)) * 10.0) / 10.0,
                                       std::round((cy + r * std::sin(a)) * 10.0) / 10.0});
            }
        }

        // Extrusion: along the normal (usually), sometimes oblique; occasionally negative.
        const double h = rng.intIn(3, 25) * 1.0 * (rng.chance(0.2) ? -1.0 : 1.0);
        poly.vector = poly.planeNormal * (1.0 / length(poly.planeNormal)) * h;
        if (rng.chance(0.25)) poly.vector = poly.vector + frameOf(poly).xd * static_cast<double>(rng.intIn(-4, 4));
        p.source = poly;

        // Operation: a new body (possibly replacing an existing id), or a join onto an existing body.
        if (ctx.bodies.empty() || rng.chance(0.4)) {
            p.operation = ExtrudeParams::Operation::NewBody;
            p.body = BodyId{static_cast<std::uint64_t>(rng.intIn(1, 3))};
        } else {
            p.operation = rng.chance(0.85) ? ExtrudeParams::Operation::Join : ExtrudeParams::Operation::Cut;
            p.body = ctx.bodies[rng.below(ctx.bodies.size())];
        }
        return std::make_shared<ExtrudeFeature>(id, std::move(p));
    }

    std::shared_ptr<const recompute::Feature> editDimension(Rng& rng, const recompute::Feature& f) const override {
        const ExtrudeFeature* e = asExtrude(f);
        if (!e) return nullptr;
        const PolygonSource* src = polygonOf(e->params());
        if (!src) return nullptr;
        ExtrudeParams p = e->params();
        PolygonSource poly = *src;
        switch (rng.below(3)) {
        case 0: { // extrusion length (keeps sign)
            static const double kScale[] = {0.5, 0.75, 1.25, 1.5, 2.0};
            poly.vector = poly.vector * kScale[rng.below(5)];
            break;
        }
        case 1: { // translate the plane within its own plane
            const Frame fr = frameOf(poly);
            poly.planeOrigin = poly.planeOrigin + fr.xd * (rng.intIn(-2, 2) * 5.0) + fr.yd * (rng.intIn(-2, 2) * 5.0);
            break;
        }
        default: { // scale the profile about the plane origin
            static const double kScale[] = {0.5, 0.8, 1.5, 2.0};
            const double s = kScale[rng.below(4)];
            for (auto& q : poly.points) { q[0] *= s; q[1] *= s; }
            break;
        }
        }
        p.source = poly;
        return std::make_shared<ExtrudeFeature>(f.id(), std::move(p));
    }

    int volumeRelation(const recompute::Feature& f) const override {
        const ExtrudeFeature* e = asExtrude(f);
        if (!e) return 0;
        switch (e->params().operation) {
        case ExtrudeParams::Operation::Join: return 1;
        case ExtrudeParams::Operation::Cut: return -1;
        default: return 0;
        }
    }

    std::optional<BodyId> inheritsFrom(const recompute::Feature& f) const override {
        const ExtrudeFeature* e = asExtrude(f);
        if (!e || e->params().operation == ExtrudeParams::Operation::NewBody) return std::nullopt;
        return e->params().body;
    }

    std::optional<std::vector<Expected>> predict(const recompute::Feature& f, const ElementName& name,
                                                 const PredictContext&) const override {
        const ExtrudeFeature* e = asExtrude(f);
        if (!e) return std::nullopt;
        const PolygonSource* pp = polygonOf(e->params());
        if (!pp) return std::nullopt;
        const PolygonSource& p = *pp;
        const Frame fr = frameOf(p);
        const std::size_t n = p.points.size();
        if (n < 3) return std::nullopt;
        std::vector<Vec3> P(n);
        for (std::size_t j = 0; j < n; ++j) P[j] = toWorld(fr, p.points[j]);
        const Vec3 v = p.vector;
        const double along = dot(v, fr.n);
        if (std::abs(along) < 1e-9) return std::nullopt;

        // Profile face: area and area-weighted centroid of the polygon (shoelace), in plane coordinates.
        double a2 = 0, cx = 0, cy = 0;
        for (std::size_t j = 0; j < n; ++j) {
            const auto& q0 = p.points[j];
            const auto& q1 = p.points[(j + 1) % n];
            const double cr = q0[0] * q1[1] - q1[0] * q0[1];
            a2 += cr;
            cx += (q0[0] + q1[0]) * cr;
            cy += (q0[1] + q1[1]) * cr;
        }
        if (std::abs(a2) < 1e-12) return std::nullopt;
        const double area = std::abs(a2) / 2.0;
        const Vec3 centroid = toWorld(fr, {cx / (3.0 * a2), cy / (3.0 * a2)});
        const Vec3 startNormal = fr.n * (along > 0 ? -1.0 : 1.0); // outward normal of the start cap

        std::vector<GeomSig> out;
        const std::string& op = name.op;
        const ElementName* src = name.sources.size() == 1 ? &name.sources[0] : nullptr;
        if (op == "profile.face") {
            out.push_back(planeSig(centroid, area, startNormal, fr.origin));
        } else if (op == "extrude.cap") {
            out.push_back(planeSig(centroid + v, area, startNormal * -1.0, fr.origin + v));
        } else if (op == "profile.edge") {
            for (std::size_t j = 0; j < n; ++j) out.push_back(lineSig(P[j], P[(j + 1) % n]));
        } else if (op == "profile.vertex") {
            for (std::size_t j = 0; j < n; ++j) out.push_back(pointSig(P[j]));
        } else if (op == "extrude.side" && src && src->op == "profile.edge") {
            for (std::size_t j = 0; j < n; ++j) {
                const Vec3 e1 = P[(j + 1) % n] - P[j];
                const Vec3 nrm = cross(e1, v);
                out.push_back(planeSig((P[j] + P[(j + 1) % n]) * 0.5 + v * 0.5, length(nrm), nrm, P[j]));
            }
        } else if (op == "extrude.edge" && src && src->op == "profile.edge") {
            for (std::size_t j = 0; j < n; ++j) out.push_back(lineSig(P[j] + v, P[(j + 1) % n] + v));
        } else if (op == "extrude.new" && name.type == ElementType::Edge) {
            for (std::size_t j = 0; j < n; ++j) out.push_back(lineSig(P[j] + v, P[(j + 1) % n] + v));         // end cap boundary
            for (std::size_t j = 0; j < n; ++j) out.push_back(lineThrough(P[j], v, length(v), P[j] + v * 0.5)); // lateral edges
        } else if (op == "extrude.new" && name.type == ElementType::Vertex) {
            for (std::size_t j = 0; j < n; ++j) out.push_back(pointSig(P[j] + v));
        } else if (op == "extrude.edge" && src && src->op == "profile.vertex") {
            for (std::size_t j = 0; j < n; ++j) out.push_back(lineThrough(P[j], v, length(v), P[j] + v * 0.5));
        } else if (op == "extrude.vertex" && src && src->op == "profile.vertex") {
            for (std::size_t j = 0; j < n; ++j) out.push_back(pointSig(P[j] + v));
        } else {
            return std::nullopt;
        }

        const auto operation = e->params().operation;
        std::vector<Expected> res;
        for (const GeomSig& s : out) {
            Expected x = exactly(s);
            if (op == "extrude.side") x.mask = (x.mask & ~mask::Surface) | mask::SurfaceFlip; // orientation of a side is not asserted
            if (operation != ExtrudeParams::Operation::NewBody) {
                x = relaxed(x);
                if (operation == ExtrudeParams::Operation::Cut && x.sig.type == ElementType::Face) x.mask = mask::SurfaceFlip;
            }
            res.push_back(std::move(x));
        }
        return res;
    }
};

} // namespace

std::shared_ptr<const FeatureGenerator> makeExtrudeGenerator() { return std::make_shared<ExtrudeGenerator>(); }

} // namespace iw::fuzz
