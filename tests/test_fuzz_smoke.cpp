// SPDX-License-Identifier: GPL-3.0-or-later
// A6 / F19 naming fuzzer: smoke campaign (the P1a gate run in ctest), predictor tests for every generated feature kind,
// the F27 probe, and oracle sensitivity tests. The sensitivity tests inject deliberately wrong resolvers and assert the
// oracle reports them; a green smoke run is only meaningful if those tests are green too.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <set>
#include <string>

#include "features/construction_features.hpp"
#include "features/edge_features.hpp"
#include "features/extrude_feature.hpp"
#include "features/hole_feature.hpp"
#include "features/move_feature.hpp"
#include "features/revolve_feature.hpp"
#include "fuzz/geom.hpp"
#include "fuzz/runner.hpp"
#include "fuzz/scenario.hpp"
#include "kernel/ops.hpp"
#include "recompute/evaluator.hpp"

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

/// Resolver that rebinds a History-derived answer without the mandatory warning.
ResolveFn warninglessHistory() {
    return [](const naming::ElementRef& ref, const naming::NamingMap& map) -> Result<naming::Resolution> {
        auto r = naming::resolve(ref, map);
        if (r) r->warnings.clear();
        return r;
    };
}

std::size_t count(const FuzzSummary& s, const std::string& category) {
    auto it = s.failureCategories.find(category);
    return it == s.failureCategories.end() ? 0u : it->second;
}

// ---- timeline builders ------------------------------------------------------------------------------------------

std::shared_ptr<const recompute::Feature> boxFeature(std::uint64_t id, std::uint64_t body, double dx, double dy, double dz,
                                                     Vec3 origin = {0, 0, 0}) {
    features::ExtrudeParams p;
    features::PolygonSource poly;
    poly.planeOrigin = origin;
    poly.planeNormal = {0, 0, 1};
    poly.planeXDir = {1, 0, 0};
    poly.points = {{0, 0}, {dx, 0}, {dx, dy}, {0, dy}};
    poly.vector = {0, 0, dz};
    p.source = poly;
    p.operation = features::ExtrudeParams::Operation::NewBody;
    p.body = BodyId{body};
    return std::make_shared<features::ExtrudeFeature>(FeatureId{id}, p);
}

struct Eval {
    recompute::Evaluator ev;
    recompute::Timeline tl;
    recompute::EvalResult result;
    void run() {
        auto r = ev.evaluate(tl);
        ASSERT_TRUE(r.has_value());
        result = std::move(*r);
    }
};

/// First named element of `type` in `body` whose op starts with `opPrefix` (and satisfies `pred`).
template <class Pred>
std::optional<naming::ElementName> findName(const recompute::EvalResult& r, BodyId body, naming::ElementType type,
                                            const std::string& opPrefix, Pred pred) {
    auto n = r.namings.find(body);
    auto s = r.bodies.find(body);
    if (n == r.namings.end() || s == r.bodies.end()) return std::nullopt;
    for (std::uint32_t i = 0; i < n->second.size(type); ++i) {
        auto nm = n->second.nameOf(type, i);
        if (nm && nm->op.compare(0, opPrefix.size(), opPrefix) == 0 && pred(readSig(s->second, type, i))) return nm;
    }
    return std::nullopt;
}

Scenario scenarioOf(std::vector<std::shared_ptr<const recompute::Feature>> base, std::vector<Step> steps = {}) {
    Scenario sc;
    sc.base = std::move(base);
    sc.steps = std::move(steps);
    return sc;
}

Step editStep(std::shared_ptr<const recompute::Feature> f) {
    Step s;
    s.kind = StepKind::EditDimension;
    s.target = f->id();
    s.feature = std::move(f);
    return s;
}

} // namespace

// ---- determinism -------------------------------------------------------------------------------------------------

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

TEST(FuzzDeterminism, RegistryCoversEveryBatchFeatureKind) {
    const auto reg = GeneratorRegistry::defaults();
    std::set<std::string> kinds;
    for (const auto* g : reg.all()) kinds.insert(std::string(g->kind()));
    for (const char* k : {"extrude", "revolve", "fillet", "chamfer", "hole", "move", "axis"}) EXPECT_TRUE(kinds.count(k)) << k;
}

// ---- signatures ---------------------------------------------------------------------------------------------------

TEST(FuzzSignature, BoxFacesCarryOutwardNormalsAndOffsets) {
    Eval e;
    e.tl.append(boxFeature(1, 1, 20, 10, 5));
    e.run();
    const kernel::Shape& box = e.result.bodies.at(BodyId{1});
    int top = 0, bottom = 0, planes = 0;
    for (std::uint32_t i = 0; i < box.subShapeCount(kernel::SubShapeKind::Face); ++i) {
        const GeomSig s = readSig(box, naming::ElementType::Face, i);
        ASSERT_EQ(s.surface, SurfaceKind::Plane);
        ++planes;
        if (s.normal.z > 0.99) { ++top; EXPECT_NEAR(dot(s.normal, s.point), 5.0, 1e-9); }
        if (s.normal.z < -0.99) { ++bottom; EXPECT_NEAR(dot(s.normal, s.point), 0.0, 1e-9); }
    }
    EXPECT_EQ(planes, 6);
    EXPECT_EQ(top, 1);
    EXPECT_EQ(bottom, 1);
    // Every straight edge of a box has a supporting line.
    for (std::uint32_t i = 0; i < box.subShapeCount(kernel::SubShapeKind::Edge); ++i)
        EXPECT_TRUE(readSig(box, naming::ElementType::Edge, i).isLine);
}

TEST(FuzzSignature, DistinguishesNormalOffsetRadiusAndAxis) {
    GeomSig a;
    a.surface = SurfaceKind::Plane;
    a.normal = {0, 0, 1};
    a.point = {0, 0, 5};
    a.key = {1, 2, 5, 10};
    GeomSig flipped = a;
    flipped.normal = {0, 0, -1};
    GeomSig offset = a;
    offset.point = {0, 0, 5.5};
    EXPECT_TRUE(sameGeometry(a, a));
    EXPECT_FALSE(sameGeometry(a, flipped)) << "normal sign is part of the signature";
    EXPECT_TRUE(sameSurface(a, flipped, /*signedNormal=*/false));
    EXPECT_FALSE(sameGeometry(a, offset)) << "parallel plane at another offset";

    GeomSig c;
    c.surface = SurfaceKind::Cylinder;
    c.normal = {0, 0, 1};
    c.point = {0, 0, 0};
    c.radius = 3;
    c.key = {0, 0, 2, 40};
    GeomSig bigger = c;
    bigger.radius = 3.1;
    GeomSig shifted = c;
    shifted.point = {1, 0, 0};
    GeomSig tilted = c;
    tilted.normal = unit({0.01, 0, 1});
    EXPECT_FALSE(sameSurface(c, bigger));
    EXPECT_FALSE(sameSurface(c, shifted)) << "axis line moved";
    EXPECT_FALSE(sameSurface(c, tilted));
    GeomSig reversedAxis = c;
    reversedAxis.normal = {0, 0, -1};
    EXPECT_TRUE(sameSurface(c, reversedAxis)) << "an axis has no sign";
}

TEST(FuzzSignature, RevolvedFacesAreCylindersAndPlanes) {
    Eval e;
    features::ConstructionAxisParams ap;
    ap.kind = features::ConstructionAxisParams::Kind::TwoPoints;
    ap.a = {"0 mm", "0 mm", "0 mm"};
    ap.b = {"0 mm", "0 mm", "10 mm"};
    e.tl.append(std::make_shared<features::ConstructionAxisFeature>(FeatureId{1}, ap));
    features::RevolveParams rp;
    features::PolygonSource poly;
    poly.planeOrigin = {0, 0, 0};
    poly.planeNormal = {0, 1, 0};
    poly.planeXDir = {0, 0, 1};
    poly.points = {{0, 2}, {10, 2}, {10, 5}, {0, 5}};
    rp.source = poly;
    rp.axis.kind = features::RevolveAxis::Kind::Construction;
    rp.axis.construction = FeatureId{1};
    rp.extent = features::RevolveParams::Extent::Full;
    rp.operation = features::common::BodyOp::NewBody;
    rp.body = BodyId{1};
    e.tl.append(std::make_shared<features::RevolveFeature>(FeatureId{2}, rp));
    e.run();
    ASSERT_EQ(e.result.statuses[1].state, recompute::FeatureStatus::State::Ok);
    const kernel::Shape& ring = e.result.bodies.at(BodyId{1});
    std::set<double> radii;
    int planes = 0;
    for (std::uint32_t i = 0; i < ring.subShapeCount(kernel::SubShapeKind::Face); ++i) {
        const GeomSig s = readSig(ring, naming::ElementType::Face, i);
        if (s.surface == SurfaceKind::Cylinder) {
            radii.insert(std::round(s.radius * 1e6) / 1e6);
            EXPECT_NEAR(std::abs(s.normal.z), 1.0, 1e-9);
        }
        if (s.surface == SurfaceKind::Plane) ++planes;
    }
    EXPECT_EQ(radii, (std::set<double>{2.0, 5.0}));
    EXPECT_EQ(planes, 2);
}

// ---- predictors, one kind at a time ------------------------------------------------------------------------------

TEST(FuzzPredictor, UneditedBaselineAgreesWithKernelGeometry) {
    // A single rectangular prism: every predicted name (profile.*, extrude.*) must match the kernel's geometry, and
    // the baseline must produce no failure at all.
    const auto reg = GeneratorRegistry::defaults();
    const auto out = runScenario(scenarioOf({boxFeature(1, 1, 20, 10, 5)}), reg);
    EXPECT_TRUE(out.failures.empty()) << failuresText(out);
    EXPECT_GT(out.counts.preStateChecks, 0u);
    EXPECT_EQ(out.counts.resolvedExact, out.counts.refsChecked);
    EXPECT_EQ(out.counts.refsCaptured, 6u + 12u + 8u); // 6 faces, 12 edges, 8 vertices of a box
    EXPECT_EQ(out.counts.unverifiedMoved, 0u);
}

TEST(FuzzPredictor, RevolveFullAndPartialAreFullyPredicted) {
    const auto reg = GeneratorRegistry::defaults();
    features::ConstructionAxisParams ap;
    ap.kind = features::ConstructionAxisParams::Kind::TwoPoints;
    ap.a = {"0 mm", "0 mm", "0 mm"};
    ap.b = {"0 mm", "0 mm", "10 mm"};
    auto axis = std::make_shared<features::ConstructionAxisFeature>(FeatureId{1}, ap);
    for (auto extent : {features::RevolveParams::Extent::Full, features::RevolveParams::Extent::OneSide,
                        features::RevolveParams::Extent::Symmetric, features::RevolveParams::Extent::TwoSides}) {
        features::RevolveParams rp;
        features::PolygonSource poly;
        poly.planeOrigin = {0, 0, 0};
        poly.planeNormal = {0, 1, 0};
        poly.planeXDir = {0, 0, 1};
        poly.points = {{0, 2}, {10, 2}, {10, 5}, {0, 5}};
        rp.source = poly;
        rp.axis.kind = features::RevolveAxis::Kind::Construction;
        rp.axis.construction = FeatureId{1};
        rp.extent = extent;
        rp.angle = "120 deg";
        rp.angle2 = "45 deg";
        rp.operation = features::common::BodyOp::NewBody;
        rp.body = BodyId{1};
        auto rev = std::make_shared<features::RevolveFeature>(FeatureId{2}, rp);
        auto edited = rp;
        std::get<features::PolygonSource>(edited.source).points = {{0, 2}, {14, 2}, {14, 6}, {0, 6}};
        const auto out = runScenario(scenarioOf({axis, rev}, {editStep(std::make_shared<features::RevolveFeature>(FeatureId{2}, edited))}), reg);
        EXPECT_TRUE(out.failures.empty()) << failuresText(out);
        EXPECT_GT(out.counts.predictedChecks, 10u);
        EXPECT_EQ(out.counts.unverifiedMoved, 0u) << "extent " << static_cast<int>(extent);
    }
}

TEST(FuzzPredictor, FilletChamferHoleMoveAreCheckedAfterEdits) {
    const auto reg = GeneratorRegistry::defaults();
    Eval e;
    e.tl.append(boxFeature(1, 1, 30, 20, 10));
    e.run();
    auto edge = findName(e.result, BodyId{1}, naming::ElementType::Edge, "profile.edge", [](const GeomSig&) { return true; });
    auto cap = findName(e.result, BodyId{1}, naming::ElementType::Face, "extrude.cap", [](const GeomSig&) { return true; });
    ASSERT_TRUE(edge && cap);

    features::FilletParams fp;
    fp.body = BodyId{1};
    fp.edges = {naming::ElementRef{FeatureId{1}, naming::ElementType::Edge, *edge}};
    fp.radius = "1 mm";
    auto fillet = std::make_shared<features::FilletFeature>(FeatureId{2}, fp);
    auto fillet2 = fp;
    fillet2.radius = "1.5 mm";

    features::ChamferParams cp;
    cp.body = BodyId{1};
    cp.edges = {naming::ElementRef{FeatureId{1}, naming::ElementType::Edge, *edge}};
    cp.distance = "1 mm";
    auto chamfer = std::make_shared<features::ChamferFeature>(FeatureId{2}, cp);

    features::MoveParams mp;
    mp.body = BodyId{1};
    mp.translate = {"5 mm", "0 mm", "0 mm"};
    mp.angle = "30 deg";
    mp.axisDirection = {0, 0, 1};
    auto move = std::make_shared<features::MoveFeature>(FeatureId{2}, mp);
    auto mp2 = mp;
    mp2.angle = "60 deg";
    mp2.translate = {"2 mm", "3 mm", "0 mm"};

    // Upstream edit for every case: the box becomes taller.
    auto taller = boxFeature(1, 1, 30, 20, 14);
    for (const auto& feature : std::vector<std::shared_ptr<const recompute::Feature>>{
             fillet, chamfer, move}) {
        const auto out = runScenario(scenarioOf({boxFeature(1, 1, 30, 20, 10), feature}, {editStep(taller)}), reg);
        EXPECT_TRUE(out.failures.empty()) << feature->kind() << "\n" << failuresText(out);
        EXPECT_GT(out.counts.predictedChecks, 20u) << feature->kind();
        EXPECT_EQ(out.counts.unverifiedMoved, 0u) << feature->kind();
        EXPECT_GT(out.counts.perKind.at(std::string(feature->kind())).predicted, 0u) << feature->kind();
    }
    // The feature's own dimension edited.
    {
        const auto out = runScenario(
            scenarioOf({boxFeature(1, 1, 30, 20, 10), fillet}, {editStep(std::make_shared<features::FilletFeature>(FeatureId{2}, fillet2))}), reg);
        EXPECT_TRUE(out.failures.empty()) << failuresText(out);
        EXPECT_EQ(out.counts.unverifiedMoved, 0u);
    }
    {
        const auto out = runScenario(
            scenarioOf({boxFeature(1, 1, 30, 20, 10), move}, {editStep(std::make_shared<features::MoveFeature>(FeatureId{2}, mp2))}), reg);
        EXPECT_TRUE(out.failures.empty()) << failuresText(out);
    }
}

TEST(FuzzPredictor, HoleWallsLieOnTheHoleAxis) {
    const auto reg = GeneratorRegistry::defaults();
    Eval e;
    e.tl.append(boxFeature(1, 1, 30, 20, 10));
    e.run();
    auto cap = findName(e.result, BodyId{1}, naming::ElementType::Face, "extrude.cap", [](const GeomSig&) { return true; });
    ASSERT_TRUE(cap);
    auto top = e.result.bodies.at(BodyId{1});
    const auto plane = kernel::planeFromFace(top, e.result.namings.at(BodyId{1}).indicesOf(*cap).front());
    ASSERT_TRUE(plane.has_value());
    features::HoleParams hp;
    hp.body = BodyId{1};
    hp.face = naming::ElementRef{FeatureId{1}, naming::ElementType::Face, *cap};
    hp.u = "10 mm";
    hp.v = "8 mm";
    hp.diameter = "5 mm";
    hp.depth = "6 mm";
    for (auto kind : {kernel::HoleKind::Simple, kernel::HoleKind::Counterbore, kernel::HoleKind::Countersink}) {
        hp.kind = kind;
        hp.counterboreDiameter = "8 mm";
        hp.counterboreDepth = "2 mm";
        hp.countersinkDiameter = "8 mm";
        auto hole = std::make_shared<features::HoleFeature>(FeatureId{2}, hp);
        const auto out = runScenario(scenarioOf({boxFeature(1, 1, 30, 20, 10), hole}), reg);
        EXPECT_TRUE(out.failures.empty()) << failuresText(out);
        EXPECT_GT(out.counts.perKind.at("hole").predicted, 10u);
        EXPECT_EQ(out.counts.perKind.at("hole").unverified, 0u);
    }
}

TEST(FuzzPredictor, CoplanarJoinsProduceHistoryDerivedResolutions) {
    // The F19 coverage target in miniature: scenarios that insert coplanar joins merge / split faces, so existing names
    // resolve through history. Over 60 seeded scenarios there must be some, all of them checked.
    const auto reg = GeneratorRegistry::defaults();
    FuzzConfig fc;
    fc.seed = 42;
    fc.scenarios = 60;
    fc.shrinkRuns = 0;
    const auto summary = runFuzz(fc, reg);
    EXPECT_GE(summary.counts.resolvedHistory, 5u) << "coplanar-join generator is not producing split/merge resolutions";
    EXPECT_GT(summary.counts.perKind.at("extrude").history + summary.counts.perKind.at("revolve").history, 0u);
}

// ---- F27 probe ----------------------------------------------------------------------------------------------------

namespace {

/// Box with a blind hole in its top face and a fillet on the hole's rim. Returns the rim edge name (in the hole's output).
struct HoleRimModel {
    std::shared_ptr<const recompute::Feature> box, hole, fillet;
    naming::ElementName rim;
    double diameter = 6, top = 10;
};

std::optional<HoleRimModel> buildHoleRim(double sizeX = 30, double sizeY = 20, double height = 10, double diameter = 6,
                                         double u = 15, double v = 10, double depth = 6) {
    Eval e;
    HoleRimModel m;
    m.box = boxFeature(1, 1, sizeX, sizeY, height);
    e.tl.append(m.box);
    e.run();
    auto cap = findName(e.result, BodyId{1}, naming::ElementType::Face, "extrude.cap", [](const GeomSig&) { return true; });
    if (!cap) return std::nullopt;
    features::HoleParams hp;
    hp.body = BodyId{1};
    hp.face = naming::ElementRef{FeatureId{1}, naming::ElementType::Face, *cap};
    auto top = e.result.bodies.at(BodyId{1});
    auto plane = kernel::planeFromFace(top, e.result.namings.at(BodyId{1}).indicesOf(*cap).front());
    if (!plane) return std::nullopt;
    // u, v are relative to the plane origin: place the hole at world (u, v) of the top face.
    const Vec3 yDir = cross(plane->normal, plane->xDir);
    hp.u = mmText(dot(Vec3{u, v, height} - plane->origin, plane->xDir));
    hp.v = mmText(dot(Vec3{u, v, height} - plane->origin, yDir));
    hp.diameter = mmText(diameter);
    hp.depth = mmText(depth);
    m.hole = std::make_shared<features::HoleFeature>(FeatureId{2}, hp);
    e.tl.append(m.hole);
    e.run();
    if (e.result.statuses[1].state != recompute::FeatureStatus::State::Ok) return std::nullopt;
    // The rim: a full circle on the top plane.
    auto rim = findName(e.result, BodyId{1}, naming::ElementType::Edge, "", [&](const GeomSig& s) {
        return !s.isLine && std::abs(s.measure() - std::numbers::pi * diameter) < 1e-6 && std::abs(s.key[2] - height) < 1e-6;
    });
    if (!rim) return std::nullopt;
    m.rim = *rim;
    m.diameter = diameter;
    m.top = height;
    features::FilletParams fp;
    fp.body = BodyId{1};
    fp.edges = {naming::ElementRef{FeatureId{2}, naming::ElementType::Edge, m.rim}};
    fp.radius = "0.5 mm";
    m.fillet = std::make_shared<features::FilletFeature>(FeatureId{3}, fp);
    return m;
}

/// After `editedFeature` replaces its namesake in the base model: does the fillet still bind to the hole's rim? The rim
/// fillet is a blend face generated from that edge: it must exist, sit at the top plane `top` and encircle the hole axis
/// through (cx, cy).
::testing::AssertionResult filletKeepsRim(const HoleRimModel& m, const std::string& what, std::shared_ptr<const recompute::Feature> editedFeature,
                                          double cx, double cy, double top) {
    recompute::Evaluator ev;
    recompute::Timeline tl;
    tl.append(m.box);
    tl.append(m.hole);
    tl.append(m.fillet);
    if (!tl.replace(std::move(editedFeature))) return ::testing::AssertionFailure() << what << ": edit target missing";
    auto r = ev.evaluate(tl);
    if (!r) return ::testing::AssertionFailure() << what << ": evaluation failed: " << r.error().message;
    const auto& st = r->statuses[2];
    if (st.state != recompute::FeatureStatus::State::Ok)
        return ::testing::AssertionFailure() << what << ": the fillet of the hole rim failed after the edit: " << st.error.message;
    const auto& shape = r->bodies.at(BodyId{1});
    const auto& names = r->namings.at(BodyId{1});
    std::size_t blend = 0;
    for (std::uint32_t i = 0; i < names.size(naming::ElementType::Face); ++i) {
        auto nm = names.nameOf(naming::ElementType::Face, i);
        if (!nm || nm->feature != FeatureId{3} || nm->op != "fillet.side") continue;
        const GeomSig s = readSig(shape, naming::ElementType::Face, i);
        const double radial = std::hypot(s.key[0] - cx, s.key[1] - cy);
        if (std::abs(s.key[2] - top) < 0.6 && radial < m.diameter + 2.0) ++blend;
    }
    if (blend == 0)
        return ::testing::AssertionFailure() << what << ": no fillet face at the top plane z=" << top << " around (" << cx << ", " << cy
                                             << ") -- the fillet did not land on the hole rim";
    return ::testing::AssertionSuccess();
}

} // namespace

TEST(FuzzF27, FilletOfAHoleRimSurvivesHoleAndUpstreamEdits) {
    const auto base = buildHoleRim();
    ASSERT_TRUE(base.has_value());
    // The unedited model binds the fillet to the rim.
    EXPECT_TRUE(filletKeepsRim(*base, "unedited", base->hole, 15, 10, 10));

    const auto holeParams = static_cast<const features::HoleFeature&>(*base->hole).params();
    // 1. Hole diameter.
    {
        auto p = holeParams;
        p.diameter = "8 mm";
        EXPECT_TRUE(filletKeepsRim(*base, "hole diameter 6 -> 8", std::make_shared<features::HoleFeature>(FeatureId{2}, p), 15, 10, 10));
    }
    // 2. Hole position.
    {
        auto p = holeParams;
        p.u = mmText(parseMm(p.u).value() + 3.0);
        p.v = mmText(parseMm(p.v).value() - 2.0);
        EXPECT_TRUE(filletKeepsRim(*base, "hole position", std::make_shared<features::HoleFeature>(FeatureId{2}, p), 18, 8, 10));
    }
    // 3. Hole depth.
    {
        auto p = holeParams;
        p.depth = "8 mm";
        EXPECT_TRUE(filletKeepsRim(*base, "hole depth 6 -> 8", std::make_shared<features::HoleFeature>(FeatureId{2}, p), 15, 10, 10));
    }
    // 4. Upstream: a taller box (the top face moves), a wider box, a deeper box. The hole keeps its (u, v) on the face plane.
    EXPECT_TRUE(filletKeepsRim(*base, "box height 10 -> 14", boxFeature(1, 1, 30, 20, 14), 15, 10, 14));
    EXPECT_TRUE(filletKeepsRim(*base, "box width 30 -> 36", boxFeature(1, 1, 36, 20, 10), 15, 10, 10));
    EXPECT_TRUE(filletKeepsRim(*base, "box depth 20 -> 26", boxFeature(1, 1, 30, 26, 10), 15, 10, 10));
}

TEST(FuzzF27, HoleDerivedReferencesAreNotLostByHoleOrUpstreamEdits) {
    // F27 campaign: body -> hole -> fillet of the hole's edges, then random edits. A feature that references an element the
    // hole created must not lose that reference while the hole itself is still alive and fine.
    const auto reg = GeneratorRegistry::defaults();
    FuzzConfig fc;
    fc.seed = 7;
    fc.scenarios = 60;
    fc.shrinkRuns = 0;
    fc.isolate = true;
    fc.scenario.holeFocus = true;
    const auto summary = runFuzz(fc, reg);
    EXPECT_GT(summary.counts.holeRefFeatures, 30u) << "the F27 population is too small to mean anything";
    std::string notes;
    for (const auto& n : summary.notes) notes += n + "\n";
    EXPECT_EQ(summary.counts.holeRefDetached, 0u) << "detached hole references (F27):\n" << notes;
}

TEST(FuzzF27, LegitLossesAreClassifiedAndNotCountedAsFailures) {
    // Detachments the edit forces, pinned by seed. Each must be classified legit_loss with its cause and never counted as
    // an F27 failure. (1) overwritten_body: a new_body extrude written over the hole's body before the fillet on its rim.
    // (2) consumed_element: an earlier fillet consumes the same hole edge. (3) removed_by_owner_edit: a hole edited from
    // through to blind removes the exit rim a fillet references (general campaign seed 42). (4) removed_by_owner_edit: a
    // fillet that later features reference is removed by the edit (its names are gone, no same-geometry element survives).
    const auto reg = GeneratorRegistry::defaults();
    struct Case {
        std::uint64_t seed;
        bool holeFocus;
        std::size_t legit;
        const char* cause;
    };
    const Case cases[] = {
        {10748153307738816448ull, true, 1, "overwritten_body"},
        {15590467392696145363ull, true, 1, "consumed_element"},
        {18184143690146808462ull, false, 1, "removed_by_owner_edit"},
        {11078703195767084418ull, true, 2, "removed_by_owner_edit"},
    };
    for (const Case& c : cases) {
        ScenarioConfig cfg;
        cfg.holeFocus = c.holeFocus;
        const auto out = runScenario(generateScenario(c.seed, reg, cfg), reg);
        EXPECT_TRUE(out.failures.empty()) << "seed " << c.seed << " has oracle failures";
        EXPECT_EQ(out.counts.holeRefDetached, 0u) << "seed " << c.seed << " counted as an F27 failure";
        EXPECT_EQ(out.counts.legitLoss, c.legit) << "seed " << c.seed;
        EXPECT_EQ(out.counts.legitLossByCause.count(c.cause), 1u) << "seed " << c.seed << " not classified as " << c.cause;
        EXPECT_EQ(out.counts.legitLossByCause.at(c.cause), c.legit) << "seed " << c.seed;
    }
}

// ---- oracle sensitivity ------------------------------------------------------------------------------------------

TEST(FuzzOracle, WrongPickIsReported) {
    // A resolver that picks a neighbouring element must be caught by the geometric checks.
    const auto reg = GeneratorRegistry::defaults();
    FuzzConfig fc;
    fc.seed = 11;
    fc.scenarios = 25;
    fc.shrinkRuns = 0;
    const auto summary = runFuzz(fc, reg, shiftedPick());
    EXPECT_GT(summary.failingScenarios, 0u);
    EXPECT_GT(count(summary, "wrong_pick_predicted"), 0u) << "no predicted-geometry failure";
    EXPECT_GT(count(summary, "wrong_pick_unaffected"), 0u) << "no unaffected-geometry failure";
}

TEST(FuzzOracle, EveryFeatureKindsPredictionCatchesAWrongPick) {
    // Per-kind sensitivity: with the shifted resolver, failures must be attributed to each generated feature kind, i.e.
    // the predictors of revolve, fillet, chamfer, hole and move are not vacuous.
    const auto reg = GeneratorRegistry::defaults();
    FuzzConfig fc;
    fc.seed = 21;
    fc.scenarios = 60;
    fc.shrinkRuns = 0;
    const auto summary = runFuzz(fc, reg, shiftedPick());
    for (const char* kind : {"extrude", "revolve", "fillet", "chamfer", "hole", "move"}) {
        auto it = summary.counts.perKind.find(kind);
        ASSERT_NE(it, summary.counts.perKind.end()) << kind;
        EXPECT_GT(it->second.failures, 0u) << kind << " predictions never fired against a wrong resolver";
    }
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
    EXPECT_GT(count(summary, "missing_diagnostic"), 0u);
}

TEST(FuzzOracle, HistoryResolutionWithoutWarningIsReported) {
    const auto reg = GeneratorRegistry::defaults();
    FuzzConfig fc;
    fc.seed = 42;
    fc.scenarios = 60;
    fc.shrinkRuns = 0;
    const auto summary = runFuzz(fc, reg, warninglessHistory());
    EXPECT_GT(count(summary, "missing_warning"), 0u);
}

TEST(FuzzShrink, ReproducerStillFailsAndIsNotLarger) {
    const auto reg = GeneratorRegistry::defaults();
    const auto resolve = shiftedPick();
    FuzzConfig fc;
    fc.seed = 11;
    fc.scenarios = 12;
    fc.shrinkRuns = 60;
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

// ---- the gate ------------------------------------------------------------------------------------------------------

TEST(FuzzSmoke, CampaignPassesGate) {
    // The P1a gate: 200 scenarios, under 60 s, zero oracle failures with the real resolver. Scenarios run in forked
    // workers so a crash of the geometry kernel is reported (as a failure) rather than killing the test binary.
    const auto reg = GeneratorRegistry::defaults();
    FuzzConfig fc;
    fc.seed = 1;
    fc.scenarios = 200;
    fc.timeBudgetSeconds = 60;
    fc.isolate = true;
    const auto start = std::chrono::steady_clock::now();
    const auto summary = runFuzz(fc, reg);
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::string report;
    for (const auto& f : summary.failures)
        report += "\n[" + f.category + "] " + f.detail + "\n" + f.reproducer + "  replay: ironwood-fuzz-naming --replay-seed " +
                  std::to_string(f.scenarioSeed) + "\n";
    EXPECT_FALSE(summary.timedOut) << "smoke campaign exceeded its 60 s budget";
    EXPECT_EQ(summary.scenariosRun, 200u);
    EXPECT_LT(wall, 60.0);
    EXPECT_EQ(summary.failingScenarios, 0u) << report;
    // Coverage sanity: the oracle really checked things, not just NotFound everywhere.
    const auto& c = summary.counts;
    EXPECT_GT(c.refsChecked, 5000u);
    EXPECT_GT(c.predictedChecks, 5000u);
    EXPECT_GT(c.resolvedHistory, 10u);
    EXPECT_LT(static_cast<double>(c.unverifiedMoved), 0.05 * static_cast<double>(c.resolutions()));
    for (const char* kind : {"extrude", "revolve", "fillet", "chamfer", "hole", "move"})
        EXPECT_GT(c.perKind.count(kind) ? c.perKind.at(kind).predicted : 0u, 0u) << kind << " produced no checked resolutions";
}
