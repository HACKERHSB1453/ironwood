// SPDX-License-Identifier: GPL-3.0-or-later
// Generator + predictor for features::MoveFeature (move and copy). A move keeps every name, so the prediction is
// the rigid transform of the input element's signature (computed here, not by the feature).
#include <algorithm>
#include <cmath>
#include <numbers>

#include "features/move_feature.hpp"
#include "fuzz/generator.hpp"
#include "gen_util.hpp"

namespace iw::fuzz {
namespace {

using features::MoveFeature;
using features::MoveParams;

/// p' = R (p - o) + o + t, R = rotation of `angle` about unit `axis` (Rodrigues).
Transform rigid(const Vec3& o, const Vec3& axisIn, double angle, const Vec3& t) {
    const Vec3 k = unit(axisIn);
    const double c = std::cos(angle), s = std::sin(angle);
    const double R[3][3] = {
        {c + k.x * k.x * (1 - c), k.x * k.y * (1 - c) - k.z * s, k.x * k.z * (1 - c) + k.y * s},
        {k.y * k.x * (1 - c) + k.z * s, c + k.y * k.y * (1 - c), k.y * k.z * (1 - c) - k.x * s},
        {k.z * k.x * (1 - c) - k.y * s, k.z * k.y * (1 - c) + k.x * s, c + k.z * k.z * (1 - c)}};
    Transform m;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m.m[static_cast<std::size_t>(i * 4 + j)] = R[i][j];
    const double ov[3] = {o.x, o.y, o.z};
    const double tv[3] = {t.x, t.y, t.z};
    for (int i = 0; i < 3; ++i)
        m.m[static_cast<std::size_t>(i * 4 + 3)] = ov[i] - (R[i][0] * o.x + R[i][1] * o.y + R[i][2] * o.z) + tv[i];
    return m;
}

class MoveGenerator final : public FeatureGenerator {
public:
    std::string_view kind() const override { return "move"; }
    int weight() const override { return 2; }

    std::shared_ptr<const recompute::Feature> generate(Rng& rng, FeatureId id, const GenContext& ctx) const override {
        std::vector<BodyId> bodies;
        for (const auto& [b, shape] : ctx.state.bodies) bodies.push_back(b);
        if (bodies.empty()) return nullptr;
        MoveParams p;
        p.body = bodies[rng.below(bodies.size())];
        p.copy = rng.chance(0.35);
        if (p.copy) p.newBody = ctx.freshBody();
        for (auto& t : p.translate) t = rng.chance(0.3) ? "0 mm" : mmText(static_cast<double>(rng.intIn(-15, 15)));
        static const double kAngles[] = {0, 0, 30, 45, 60, 90, 180, 270};
        p.angle = degText(kAngles[rng.below(8)]);
        static const Vec3 kAxes[] = {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {1, 1, 1}};
        p.axisDirection = kAxes[rng.below(5)];
        p.axisOrigin = {static_cast<double>(rng.intIn(-5, 5)), static_cast<double>(rng.intIn(-5, 5)), static_cast<double>(rng.intIn(-5, 5))};
        return std::make_shared<MoveFeature>(id, std::move(p));
    }

    std::shared_ptr<const recompute::Feature> editDimension(Rng& rng, const recompute::Feature& f) const override {
        if (f.kind() != "move") return nullptr;
        MoveParams p = static_cast<const MoveFeature&>(f).params();
        if (rng.chance(0.6)) {
            p.translate[rng.below(3)] = mmText(static_cast<double>(rng.intIn(-20, 20)));
        } else {
            static const double kAngles[] = {0, 15, 45, 90, 135};
            p.angle = degText(kAngles[rng.below(5)]);
        }
        return std::make_shared<MoveFeature>(f.id(), std::move(p));
    }

    int volumeRelation(const recompute::Feature& f) const override { return static_cast<const MoveFeature&>(f).params().copy ? 0 : 2; }

    std::optional<BodyId> inheritsFrom(const recompute::Feature& f) const override {
        return static_cast<const MoveFeature&>(f).params().body;
    }

    Expected inherited(const recompute::Feature& f, const GeomSig& in) const override {
        const MoveParams& p = static_cast<const MoveFeature&>(f).params();
        const double angle = parseDeg(p.angle).value_or(0) * std::numbers::pi / 180.0;
        const Vec3 t{parseMm(p.translate[0]).value_or(0), parseMm(p.translate[1]).value_or(0), parseMm(p.translate[2]).value_or(0)};
        // A rigid motion keeps every name 1:1, so the whole signature must follow the transform.
        return exactly(transformed(in, rigid(p.axisOrigin, p.axisDirection, angle, t)));
    }
};

} // namespace

std::shared_ptr<const FeatureGenerator> makeMoveGenerator() { return std::make_shared<MoveGenerator>(); }

} // namespace iw::fuzz
