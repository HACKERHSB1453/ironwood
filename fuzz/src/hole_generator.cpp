// SPDX-License-Identifier: GPL-3.0-or-later
// Generator + predictor for features::HoleFeature (simple / counterbore / countersink, blind or through).
//
// The hole is placed on a named planar face of the model state at the insertion position, around the face centroid.
// The kernel keeps no history for the hole tool (F27), so the hole walls are fresh "hole.new" names whose sibling
// ordinals come from geometry. The predictor re-derives the hole axis from the entry face and the parameters and
// requires every hole element to lie on that axis: wall cylinders of the right radius, a cone for the drill point or
// countersink, planes perpendicular to the axis for floors.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>

#include "features/hole_feature.hpp"
#include "fuzz/generator.hpp"
#include "gen_util.hpp"
#include "kernel/ops.hpp"

namespace iw::fuzz {
namespace {

using features::HoleFeature;
using features::HoleParams;
using naming::ElementName;
using naming::ElementType;

class HoleGenerator final : public FeatureGenerator {
public:
    std::string_view kind() const override { return "hole"; }
    int weight() const override { return 4; }

    std::shared_ptr<const recompute::Feature> generate(Rng& rng, FeatureId id, const GenContext& ctx) const override {
        auto pick = util::pickElement(rng, ctx, ElementType::Face,
                                      [](BodyId, const kernel::Shape& shape, std::uint32_t idx, const ElementName&) {
                                          if (shape.faceGeometry(idx).kind != SurfaceKind::Plane) return false;
                                          return shape.subShapeKey(kernel::SubShapeKind::Face, idx)[3] >= 60.0;
                                      });
        if (!pick) return nullptr;
        const kernel::Shape& shape = ctx.state.bodies.at(pick->body);
        auto plane = kernel::planeFromFace(shape, pick->index);
        if (!plane) return nullptr;
        const auto key = shape.subShapeKey(kernel::SubShapeKind::Face, pick->index);
        const Vec3 c{key[0], key[1], key[2]};
        const Vec3 yDir = cross(plane->normal, plane->xDir);
        const double side = std::sqrt(key[3]);

        HoleParams p;
        p.body = pick->body;
        p.face = util::refOf(ctx, *pick);
        const double wiggle = std::min(2.0, side / 8.0);
        p.u = mmText(util::round1(dot(c - plane->origin, plane->xDir) + rng.real(-wiggle, wiggle)));
        p.v = mmText(util::round1(dot(c - plane->origin, yDir) + rng.real(-wiggle, wiggle)));
        const double dia = util::round1(rng.real(1.5, std::min(5.0, side / 4.0)));
        p.diameter = mmText(dia);
        p.through = rng.chance(0.35);
        p.depth = mmText(static_cast<double>(rng.intIn(4, 14)));
        const std::uint64_t k = rng.below(10);
        if (k < 6) {
            p.kind = kernel::HoleKind::Simple;
        } else if (k < 8) {
            p.kind = kernel::HoleKind::Counterbore;
            p.counterboreDiameter = mmText(util::round1(dia * rng.real(1.5, 2.0)));
            p.counterboreDepth = mmText(util::round1(rng.real(1.0, 3.0)));
        } else {
            p.kind = kernel::HoleKind::Countersink;
            p.countersinkDiameter = mmText(util::round1(dia * rng.real(1.5, 2.0)));
            p.countersinkAngle = degText(90.0);
        }
        // A hole whose axis misses the material (the centroid of a ring-shaped face lies in its bore) only grazes the
        // body and produces unrelated geometry: keep holes that really remove material.
        kernel::HoleSpec spec;
        spec.kind = p.kind;
        spec.diameter = dia;
        spec.through = p.through;
        spec.depth = parseMm(p.depth).value_or(0);
        spec.counterboreDiameter = parseMm(p.counterboreDiameter).value_or(0);
        spec.counterboreDepth = parseMm(p.counterboreDepth).value_or(0);
        spec.countersinkDiameter = parseMm(p.countersinkDiameter).value_or(0);
        spec.countersinkAngle = parseDeg(p.countersinkAngle).value_or(90.0) * std::numbers::pi / 180.0;
        const Vec3 entry = plane->origin + plane->xDir * parseMm(p.u).value() + yDir * parseMm(p.v).value();
        auto cutShape = kernel::hole(shape, kernel::Axis{entry, plane->normal * -1.0}, spec);
        auto v0 = kernel::massProperties(shape);
        if (!cutShape || !v0) return nullptr;
        auto v1 = kernel::massProperties(cutShape->shape);
        if (!v1) return nullptr;
        const double removed = v0->volume - v1->volume;
        if (removed < std::numbers::pi * (dia * 0.5) * (dia * 0.5) * 2.0 * 0.5) return nullptr;
        return std::make_shared<HoleFeature>(id, std::move(p));
    }

    std::shared_ptr<const recompute::Feature> editDimension(Rng& rng, const recompute::Feature& f) const override {
        if (f.kind() != "hole") return nullptr;
        HoleParams p = static_cast<const HoleFeature&>(f).params();
        const double dia = parseMm(p.diameter).value_or(3.0);
        switch (rng.below(4)) {
        case 0: { // diameter (stays below any counterbore / countersink diameter)
            double limit = 1e9;
            if (p.kind == kernel::HoleKind::Counterbore) limit = parseMm(p.counterboreDiameter).value_or(1e9);
            if (p.kind == kernel::HoleKind::Countersink) limit = parseMm(p.countersinkDiameter).value_or(1e9);
            static const double kScale[] = {0.6, 0.8, 1.2, 1.4};
            p.diameter = mmText(util::round1(std::min(dia * kScale[rng.below(4)], limit * 0.9)));
            break;
        }
        case 1: // position
            p.u = mmText(util::round1(parseMm(p.u).value_or(0) + rng.real(-2.0, 2.0)));
            p.v = mmText(util::round1(parseMm(p.v).value_or(0) + rng.real(-2.0, 2.0)));
            break;
        case 2: { // depth (counterbore depth stays smaller)
            const double d = parseMm(p.depth).value_or(6.0);
            const double cb = parseMm(p.counterboreDepth).value_or(0.0);
            p.depth = mmText(std::max(cb + 1.0, std::round(d * rng.real(0.7, 1.5))));
            break;
        }
        default: // toggle through
            p.through = !p.through;
            break;
        }
        return std::make_shared<HoleFeature>(f.id(), std::move(p));
    }

    int volumeRelation(const recompute::Feature&) const override { return -1; }

    std::optional<BodyId> inheritsFrom(const recompute::Feature& f) const override {
        return static_cast<const HoleFeature&>(f).params().body;
    }

    /// F27: the hole tool keeps no history, so its elements are named "hole.new#k" by geometric sibling order. Editing a
    /// dimension of the hole (diameter, position, depth) must not make a name denote a different KIND of element (a wall
    /// that becomes a floor, a circle that becomes a seam line).
    std::optional<std::string> roleChanged(const recompute::Feature& before, const recompute::Feature& after,
                                           const ElementName& name, const GeomSig& pre, const GeomSig& post) const override {
        if (before.kind() != "hole" || after.kind() != "hole") return std::nullopt;
        const HoleParams& a = static_cast<const HoleFeature&>(before).params();
        const HoleParams& b = static_cast<const HoleFeature&>(after).params();
        if (a.kind != b.kind || a.through != b.through || !(a.face == b.face)) return std::nullopt;
        if (name.op.compare(0, 5, "hole.") != 0) return std::nullopt;
        if (name.type == ElementType::Face) {
            if (pre.surface != post.surface && pre.surface != SurfaceKind::Other && post.surface != SurfaceKind::Other)
                return "changed from one surface kind to another after a hole dimension edit";
        } else if (name.type == ElementType::Edge) {
            if (pre.isLine != post.isLine) return "changed between a straight edge and a curve after a hole dimension edit";
        }
        return std::nullopt;
    }

    std::optional<std::vector<Expected>> predict(const recompute::Feature& f, const ElementName& name,
                                                 const PredictContext& ctx) const override {
        const HoleParams& p = static_cast<const HoleFeature&>(f).params();
        const std::string role = name.op.substr(name.op.find('.') + 1);
        if (role == "split" || role == "merge") return std::nullopt; // generic, from the sources
        auto body = ctx.in.bodies.find(p.body);
        auto faceIdx = ctx.indexOf(p.body, p.face.name);
        if (body == ctx.in.bodies.end() || !faceIdx) return std::nullopt;
        auto plane = kernel::planeFromFace(body->second, *faceIdx);
        if (!plane) return std::nullopt;
        const double u = parseMm(p.u).value_or(0), v = parseMm(p.v).value_or(0);
        const Vec3 yDir = cross(plane->normal, plane->xDir);
        const Vec3 entry = plane->origin + plane->xDir * u + yDir * v;
        const Vec3 dir = unit(plane->normal * -1.0);
        const double dia = parseMm(p.diameter).value_or(0);
        if (dia <= 0) return std::nullopt;
        double rmax = dia * 0.5;
        std::vector<double> radii{dia * 0.5};
        if (p.kind == kernel::HoleKind::Counterbore) {
            radii.push_back(parseMm(p.counterboreDiameter).value_or(0) * 0.5);
        } else if (p.kind == kernel::HoleKind::Countersink) {
            const double R = parseMm(p.countersinkDiameter).value_or(0) * 0.5;
            radii.push_back(R);
            // The kernel extends the countersink cone `margin` above the entry plane.
            rmax = std::max(rmax, R + std::max(1.0, dia) * std::tan((parseDeg(p.countersinkAngle).value_or(90.0) * std::numbers::pi / 180.0) / 2.0));
        }
        for (double r : radii) rmax = std::max(rmax, r);
        const double margin = std::max(1.0, dia); // the kernel starts the tool this far outside the entry plane
        const double depth = p.through ? 1e9 : parseMm(p.depth).value_or(0) + 2.0 * rmax;

        Expected x;
        x.sig.type = name.type;
        {
            char buf[200];
            std::snprintf(buf, sizeof buf, "on the hole axis: entry=(%.4g,%.4g,%.4g) dir=(%.3g,%.3g,%.3g) depth=%.4g rmax=%.4g", entry.x,
                          entry.y, entry.z, dir.x, dir.y, dir.z, depth, rmax);
            x.note = buf;
        }
        x.extra = [=](const GeomSig& got) {

            if (got.type == ElementType::Edge && got.measure() < 1e-9) return true; // degenerate edge (cone apex)
            const double axial = dot(got.centroid() - entry, dir);
            if (axial < -margin - 1e-3 || axial > depth + 1e-3) return false;
            if (distToLine(got.centroid(), entry, dir) > rmax + 1e-3) return false;
            if (got.type != ElementType::Face) return true;
            switch (got.surface) {
            case SurfaceKind::Cylinder: {
                const bool radiusOk = std::any_of(radii.begin(), radii.end(), [&](double r) { return std::abs(got.radius - r) < 1e-6 + 1e-6 * r; });
                return radiusOk && length(cross(got.normal, dir)) < 1e-6 && distToLine(got.point, entry, dir) < 1e-5;
            }
            case SurfaceKind::Cone: return length(cross(got.normal, dir)) < 1e-6 && distToLine(got.point, entry, dir) < 1e-5;
            case SurfaceKind::Plane: return length(cross(got.normal, dir)) < 1e-6;
            case SurfaceKind::Other: return true;
            }
            return true;
        };
        return std::vector<Expected>{x};
    }
};

} // namespace

std::shared_ptr<const FeatureGenerator> makeHoleGenerator() { return std::make_shared<HoleGenerator>(); }

} // namespace iw::fuzz
