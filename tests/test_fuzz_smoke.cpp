// SPDX-License-Identifier: GPL-3.0-or-later
// A6 naming fuzzer: smoke campaign (the P1a gate run in ctest) plus oracle sensitivity tests. The sensitivity tests
// inject deliberately wrong resolvers and assert the oracle reports them; a green smoke run is only meaningful if
// those tests are green too.
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <set>
#include <string>

#include "fuzz/runner.hpp"
#include "fuzz/scenario.hpp"
#include "features/extrude_feature.hpp"

using namespace iw;
using namespace iw::fuzz;

namespace {

std::string failuresText(const ScenarioOutcome& out) {
    std::string s;
    for (const auto& f : out.failures) s += "[" + f.category + "] " + f.detail + "\n";
    return s;
}

/// Resolver that answers Exact with a neighbouring element: a silent wrong pick whenever the neighbour differs.
ResolveFn shiftedPick() {
    return [](const naming::ElementRef& ref, const naming::NamingMap& map) -> Result<naming::Resolution> {
        auto r = naming::resolve(ref, map);
        if (!r) return r;
        const auto n = map.size(ref.type);
        if (n > 1) r->index = (r->index + 1) % n;
        return r;
    };
}

/// Resolver that reports NotFound without a message: an unusable diagnostic.
ResolveFn silentNotFound() {
    return [](const naming::ElementRef&, const naming::NamingMap&) -> Result<naming::Resolution> {
        return makeError(ErrorCode::NotFound, "");
    };
}

} // namespace

TEST(FuzzDeterminism, DeriveSeedIsStableAndSpreads) {
    EXPECT_EQ(deriveSeed(1, 0), deriveSeed(1, 0));
    EXPECT_NE(deriveSeed(1, 0), deriveSeed(1, 1));
    EXPECT_NE(deriveSeed(1, 0), deriveSeed(2, 0));
}

TEST(FuzzDeterminism, SameSeedGivesSameScenarioAndOutcome) {
    const auto reg = GeneratorRegistry::defaults();
    ScenarioConfig cfg;
    for (std::uint64_t seed : {7ULL, 99ULL, 123456ULL}) {
        const Scenario a = generateScenario(seed, reg, cfg);
        const Scenario b = generateScenario(seed, reg, cfg);
        EXPECT_EQ(describe(a), describe(b)) << "seed " << seed;
        const auto oa = runScenario(a, reg);
        const auto ob = runScenario(b, reg);
        EXPECT_EQ(oa.counts.refsChecked, ob.counts.refsChecked) << "seed " << seed;
        EXPECT_EQ(oa.counts.resolvedExact, ob.counts.resolvedExact) << "seed " << seed;
        EXPECT_EQ(oa.failures.size(), ob.failures.size()) << "seed " << seed;
    }
}

TEST(FuzzDeterminism, ScenarioStaysWithinConfiguredBounds) {
    const auto reg = GeneratorRegistry::defaults();
    ScenarioConfig cfg;
    cfg.maxFeatures = 3;
    cfg.maxSteps = 4;
    for (std::uint64_t i = 0; i < 40; ++i) {
        const Scenario sc = generateScenario(deriveSeed(5, i), reg, cfg);
        EXPECT_LE(sc.base.size(), 3u);
        EXPECT_LE(sc.steps.size(), 4u);
        recompute::Timeline tl;
        for (const auto& f : sc.base) tl.append(f);
        for (const auto& st : sc.steps) {
            ASSERT_TRUE(applyStep(tl, st).has_value()) << describe(sc);
            EXPECT_LE(tl.size(), 3u) << describe(sc);
        }
    }
}

TEST(FuzzPredictor, UnedittedBaselineAgreesWithKernelGeometry) {
    // A single rectangular prism: every predicted name (profile.*, extrude.*) must match the kernel's geometry, and
    // the baseline must produce no failure at all.
    features::ExtrudeParams p;
    features::PolygonSource poly;
    poly.planeOrigin = {0, 0, 0};
    poly.planeNormal = {0, 0, 1};
    poly.planeXDir = {1, 0, 0};
    poly.points = {{0, 0}, {20, 0}, {20, 10}, {0, 10}};
    poly.vector = {0, 0, 5};
    p.source = poly;
    p.operation = features::ExtrudeParams::Operation::NewBody;
    p.body = BodyId{1};
    Scenario sc;
    sc.seed = 0;
    sc.base.push_back(std::make_shared<features::ExtrudeFeature>(FeatureId{1}, p));
    const auto reg = GeneratorRegistry::defaults();
    const auto out = runScenario(sc, reg);
    EXPECT_TRUE(out.failures.empty()) << failuresText(out);
    EXPECT_GT(out.counts.predictedChecks, 0u);
    EXPECT_EQ(out.counts.resolvedExact, out.counts.refsChecked);
    EXPECT_EQ(out.counts.refsCaptured, 6u + 12u + 8u); // 6 faces, 12 edges, 8 vertices of a box
}

TEST(FuzzOracle, WrongPickIsReported) {
    // A resolver that picks a neighbouring element must be caught by the geometric check.
    const auto reg = GeneratorRegistry::defaults();
    ScenarioConfig cfg;
    FuzzConfig fc;
    fc.seed = 11;
    fc.scenarios = 40;
    fc.scenario = cfg;
    fc.shrinkRuns = 0;
    const auto summary = runFuzz(fc, reg, shiftedPick());
    EXPECT_GT(summary.failingScenarios, 0u);
    // Both oracle paths must be able to fire: the prediction check and the unaffected-prefix geometry check.
    EXPECT_GT(summary.failureCategories.count("wrong_pick_predicted") ? summary.failureCategories.at("wrong_pick_predicted") : 0u, 0u)
        << "no predicted-geometry failure";
    EXPECT_GT(summary.failureCategories.count("wrong_pick_unaffected") ? summary.failureCategories.at("wrong_pick_unaffected") : 0u, 0u)
        << "no unaffected-geometry failure";
}

TEST(FuzzOracle, MissingDiagnosticIsReported) {
    const auto reg = GeneratorRegistry::defaults();
    FuzzConfig fc;
    fc.seed = 12;
    fc.scenarios = 20;
    fc.shrinkRuns = 0;
    const auto summary = runFuzz(fc, reg, silentNotFound());
    ASSERT_GT(summary.failingScenarios, 0u);
    EXPECT_EQ(summary.failures.front().category, "missing_diagnostic");
    EXPECT_GT(summary.failureCategories.count("missing_diagnostic") ? summary.failureCategories.at("missing_diagnostic") : 0u, 0u);
}

TEST(FuzzShrink, ReproducerStillFailsAndIsNotLarger) {
    const auto reg = GeneratorRegistry::defaults();
    const auto resolve = shiftedPick();
    FuzzConfig fc;
    fc.seed = 11;
    fc.scenarios = 40;
    fc.shrinkRuns = 150;
    const auto summary = runFuzz(fc, reg, resolve);
    ASSERT_FALSE(summary.failures.empty());
    for (const auto& f : summary.failures) {
        EXPECT_LE(f.shrunkSteps, f.originalSteps);
        EXPECT_NE(f.reproducer.find("scenario seed="), std::string::npos);
    }
    // Replaying the first failing seed by itself reproduces the failure.
    FuzzConfig replay;
    replay.replaySingle = true;
    replay.replaySeed = summary.failures.front().scenarioSeed;
    const auto again = runFuzz(replay, reg, resolve);
    EXPECT_EQ(again.scenariosRun, 1u);
    EXPECT_GT(again.failingScenarios, 0u);
}

TEST(FuzzSmoke, CampaignPassesGate) {
    // The P1a gate: 200 scenarios, under 60 s, zero oracle failures with the real resolver.
    const auto reg = GeneratorRegistry::defaults();
    FuzzConfig fc;
    fc.seed = 1;
    fc.scenarios = 200;
    fc.timeBudgetSeconds = 60;
    const auto start = std::chrono::steady_clock::now();
    const auto summary = runFuzz(fc, reg);
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::string report;
    for (const auto& f : summary.failures) report += "\n[" + f.category + "] " + f.detail + "\n" + f.reproducer;
    EXPECT_FALSE(summary.timedOut) << "smoke campaign exceeded its 60 s budget";
    EXPECT_EQ(summary.scenariosRun, 200u);
    EXPECT_LT(wall, 60.0);
    EXPECT_EQ(summary.failingScenarios, 0u) << report;
    // Coverage sanity: the oracle really checked things, not just NotFound everywhere.
    EXPECT_GT(summary.counts.refsChecked, 500u);
    EXPECT_GT(summary.counts.predictedChecks, 0u);
    EXPECT_GT(summary.counts.unaffectedChecks, 0u);
}
