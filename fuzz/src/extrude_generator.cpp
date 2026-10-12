// SPDX-License-Identifier: GPL-3.0-or-later
// Generator + independent geometry prediction for recompute::ExtrudeFeature.
//
// The prediction re-derives, from the feature's parameters alone, where each named element of a NewBody extrude
// must lie. It deliberately does not call the kernel. Element names follow the naming contract of
// naming/naming_map.cpp (roleOf): profile.face / profile.edge / profile.vertex on the start cap, and
// extrude.cap / extrude.side / extrude.edge / extrude.vertex for the generated parts.
#include <cmath>
#include <vector>

#include "fuzz/generator.hpp"
#include "features/extrude_feature.hpp"

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

GeomSig sig(ElementType t, Vec3 c, double measure) { return GeomSig{t, {c.x, c.y, c.z, measure}}; }

const ExtrudeFeature* asExtrude(const recompute::Feature& f) {
    return f.kind() == "extrude" ? static_cast<const ExtrudeFeature*>(&f) : nullptr;
}

/// The P0 polygon source of a feature; the fuzzer only generates polygon profiles.
const PolygonSource* polygonOf(const ExtrudeParams& p) { return std::get_if<PolygonSource>(&p.source); }

class ExtrudeGenerator final : public FeatureGenerator {
public:
    std::string_view kind() const override { return "extrude"; }

    std::shared_ptr<const recompute::Feature> generate(Rng& rng, FeatureId id, const GenContext& ctx) const override {
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
        if (rng.chance(0.5)) {
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
        if (ctx.bodies.empty() || rng.chance(0.5)) {
            p.operation = ExtrudeParams::Operation::NewBody;
            p.body = BodyId{static_cast<std::uint64_t>(rng.intIn(1, 3))};
        } else {
            p.operation = ExtrudeParams::Operation::Join;
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

    std::optional<std::vector<GeomSig>> predict(const recompute::Feature& f, const ElementName& name) const override {
        const ExtrudeFeature* e = asExtrude(f);
        if (!e || e->params().operation != ExtrudeParams::Operation::NewBody) return std::nullopt;
        const PolygonSource* pp = polygonOf(e->params());
        if (!pp) return std::nullopt;
        const PolygonSource& p = *pp;
        const Frame fr = frameOf(p);
        const std::size_t n = p.points.size();
        if (n < 3) return std::nullopt;
        std::vector<Vec3> P(n);
        for (std::size_t j = 0; j < n; ++j) P[j] = toWorld(fr, p.points[j]);
        const Vec3 v = p.vector;

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

        std::vector<GeomSig> out;
        const std::string& op = name.op;
        const ElementName* src = name.sources.size() == 1 ? &name.sources[0] : nullptr;
        if (op == "profile.face") {
            out.push_back(sig(ElementType::Face, centroid, area));
        } else if (op == "extrude.cap") {
            out.push_back(sig(ElementType::Face, centroid + v, area));
        } else if (op == "profile.edge") {
            for (std::size_t j = 0; j < n; ++j) {
                const Vec3 e1 = P[(j + 1) % n] - P[j];
                out.push_back(sig(ElementType::Edge, (P[j] + P[(j + 1) % n]) * 0.5, length(e1)));
            }
        } else if (op == "profile.vertex") {
            for (std::size_t j = 0; j < n; ++j) out.push_back(sig(ElementType::Vertex, P[j], 0.0));
        } else if (op == "extrude.side" && src && src->op == "profile.edge") {
            for (std::size_t j = 0; j < n; ++j) {
                const Vec3 e1 = P[(j + 1) % n] - P[j];
                out.push_back(sig(ElementType::Face, (P[j] + P[(j + 1) % n]) * 0.5 + v * 0.5, length(cross(e1, v))));
            }
        } else if (op == "extrude.edge" && src && src->op == "profile.edge") {
            for (std::size_t j = 0; j < n; ++j) {
                const Vec3 e1 = P[(j + 1) % n] - P[j];
                out.push_back(sig(ElementType::Edge, (P[j] + P[(j + 1) % n]) * 0.5 + v, length(e1)));
            }
        } else if (op == "extrude.edge" && src && src->op == "profile.vertex") {
            for (std::size_t j = 0; j < n; ++j) out.push_back(sig(ElementType::Edge, P[j] + v * 0.5, length(v)));
        } else if (op == "extrude.vertex" && src && src->op == "profile.vertex") {
            for (std::size_t j = 0; j < n; ++j) out.push_back(sig(ElementType::Vertex, P[j] + v, 0.0));
        } else {
            return std::nullopt;
        }
        return out;
    }
};

} // namespace

std::shared_ptr<const FeatureGenerator> makeExtrudeGenerator() { return std::make_shared<ExtrudeGenerator>(); }

} // namespace iw::fuzz
