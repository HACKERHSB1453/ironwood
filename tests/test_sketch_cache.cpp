// SPDX-License-Identifier: GPL-3.0-or-later
// F30: solved sketches travel on the recompute sketch channel (FeatureResult::sketch -> EvalContext::sketch) instead of a
// Document pre-pass, and SketchFeature memoises its solves keyed by (sketch content, resolved plane, used parameter
// values). The solver-run counter (sketchSolverRuns) shows when the constraint solver actually runs.
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "features/construction_features.hpp"
#include "features/document.hpp"
#include "features/extrude_feature.hpp"
#include "features/hole_feature.hpp"
#include "features/sketch_feature.hpp"
#include "kernel/ops.hpp"
#include "recompute/evaluator.hpp"
#include "sketch/profile.hpp"
#include "sketch/sketch.hpp"

using namespace iw;
using namespace iw::features;
using recompute::FeatureStatus;
using sketch::PointRole;

namespace {

template <class T> T must(Result<T> r) {
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message);
    return r.has_value() ? std::move(*r) : T{};
}
void mustVoid(Result<void> r) { EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message); }

double volumeOf(const kernel::Shape& s) {
    auto m = kernel::massProperties(s);
    EXPECT_TRUE(m.has_value());
    return m ? m->volume : -1;
}

/// Fully constrained w x h rectangle (expressions) with a corner at the sketch origin.
sketch::Sketch rect(const sketch::SketchPlane& plane, const std::string& w, const std::string& h) {
    sketch::Sketch sk(plane);
    auto b = must(sk.addLine({0, 0}, {40, 0}));
    auto r = must(sk.addLine({40, 0}, {40, 20}));
    auto t = must(sk.addLine({40, 20}, {0, 20}));
    auto l = must(sk.addLine({0, 20}, {0, 0}));
    must(sk.addConstraint(sketch::ConstraintKind::Coincident, {{b, PointRole::End}, {r, PointRole::Start}}));
    must(sk.addConstraint(sketch::ConstraintKind::Coincident, {{r, PointRole::End}, {t, PointRole::Start}}));
    must(sk.addConstraint(sketch::ConstraintKind::Coincident, {{t, PointRole::End}, {l, PointRole::Start}}));
    must(sk.addConstraint(sketch::ConstraintKind::Coincident, {{l, PointRole::End}, {b, PointRole::Start}}));
    must(sk.addConstraint(sketch::ConstraintKind::Horizontal, {{b, std::nullopt}}));
    must(sk.addConstraint(sketch::ConstraintKind::Horizontal, {{t, std::nullopt}}));
    must(sk.addConstraint(sketch::ConstraintKind::Vertical, {{r, std::nullopt}}));
    must(sk.addConstraint(sketch::ConstraintKind::Vertical, {{l, std::nullopt}}));
    must(sk.addDimension(sketch::DimensionKind::DistanceX, {{b, PointRole::Start}, {b, PointRole::End}}, w));
    must(sk.addDimension(sketch::DimensionKind::DistanceY, {{r, PointRole::Start}, {r, PointRole::End}}, h));
    return sk;
}

sketch::SketchPlane xy(double z = 0) { return *sketch::SketchPlane::make({0, 0, z}, {0, 0, 1}, {1, 0, 0}); }

std::string onlyProfile(const sketch::Sketch& sk) {
    auto profiles = sketch::detectProfiles(sk);
    EXPECT_TRUE(profiles.has_value() && profiles->size() == 1);
    return (profiles && !profiles->empty()) ? profiles->front().name : std::string{};
}

std::shared_ptr<const recompute::Feature> refExtrude(FeatureId id, BodyId body, FeatureId sk, const std::string& profile,
                                                     const std::string& distance) {
    ExtrudeParams p;
    p.source = SketchReferenceSource{sk, profile};
    p.distance = distance;
    p.body = body;
    return std::make_shared<ExtrudeFeature>(id, p);
}

/// Extrude of an embedded (unparameterised) square on another body, height `distance`: a feature BEFORE the sketch whose
/// parameter does not concern the sketch.
std::shared_ptr<const recompute::Feature> unrelatedExtrude(FeatureId id, BodyId body, const std::string& distance) {
    sketch::Sketch sq(xy(100));
    must(sq.addLine({0, 0}, {5, 0}));
    must(sq.addLine({5, 0}, {5, 5}));
    must(sq.addLine({5, 5}, {0, 5}));
    must(sq.addLine({0, 5}, {0, 0}));
    ExtrudeParams p;
    p.source = SketchSource{sq, onlyProfile(sq)};
    p.distance = distance;
    p.body = body;
    return std::make_shared<ExtrudeFeature>(id, p);
}

const FeatureStatus* statusOf(const DocumentEvaluation& ev, FeatureId id) {
    for (const auto& s : ev.statuses)
        if (s.id == id) return &s;
    return nullptr;
}

DocumentEvaluation evalOk(Document& d) {
    auto ev = d.evaluate();
    EXPECT_TRUE(ev.has_value()) << (ev ? "" : ev.error().message);
    if (!ev) return {};
    for (const auto& s : ev->statuses) EXPECT_EQ(s.state, FeatureStatus::State::Ok) << s.error.message;
    return std::move(*ev);
}

/// Parameters w, h (sketch), thick (extrude after the sketch), d0 (a feature before the sketch), spare (unused).
/// Timeline: unrelated extrude (d0) | sketch rect(w, h) | extrude of the sketch profile (thick).
struct Model {
    Document d;
    BodyId body, other;
    FeatureId pre, sk, ext;

    Model() {
        mustVoid(d.addParameter("w", "40 mm"));
        mustVoid(d.addParameter("h", "20 mm"));
        mustVoid(d.addParameter("thick", "10 mm"));
        mustVoid(d.addParameter("d0", "5 mm"));
        mustVoid(d.addParameter("spare", "1 mm"));
        other = d.allocateBodyId();
        pre = d.allocateFeatureId();
        mustVoid(d.addFeature(unrelatedExtrude(pre, other, "d0")));
        body = d.allocateBodyId();
        sk = d.allocateFeatureId();
        sketch::Sketch r = rect(xy(), "w", "h");
        const std::string profile = onlyProfile(r);
        mustVoid(d.addFeature(std::make_shared<SketchFeature>(sk, SketchFeatureParams{std::move(r), std::nullopt})));
        ext = d.allocateFeatureId();
        mustVoid(d.addFeature(refExtrude(ext, body, sk, profile, "thick")));
    }
};

} // namespace

TEST(SketchCache, EvaluateTwiceSolvesOnce) {
    Model s;
    const std::uint64_t r0 = sketchSolverRuns();
    auto first = evalOk(s.d);
    EXPECT_EQ(sketchSolverRuns() - r0, 1u);
    EXPECT_NEAR(volumeOf(first.bodies.at(s.body)), 40.0 * 20 * 10, 1e-6);
    ASSERT_TRUE(first.sketches.contains(s.sk));
    EXPECT_EQ(first.sketches.at(s.sk).profiles.size(), 1u);

    auto second = evalOk(s.d);
    EXPECT_EQ(sketchSolverRuns() - r0, 1u) << "a second evaluate must not run the solver";
    EXPECT_EQ(second.computedCount, 0u);
    EXPECT_EQ(second.reusedCount, 3u);
    EXPECT_TRUE(statusOf(second, s.sk)->reused);
    ASSERT_TRUE(second.sketches.contains(s.sk)); // still reported from the cached status
    EXPECT_NEAR(volumeOf(second.bodies.at(s.body)), 40.0 * 20 * 10, 1e-6);
}

TEST(SketchCache, UsedParameterEditResolves) {
    Model s;
    evalOk(s.d);
    const std::uint64_t r0 = sketchSolverRuns();
    mustVoid(s.d.setParameterExpression("w", "50 mm"));
    auto ev = evalOk(s.d);
    EXPECT_EQ(sketchSolverRuns() - r0, 1u);
    EXPECT_FALSE(statusOf(ev, s.sk)->reused);
    EXPECT_FALSE(statusOf(ev, s.ext)->reused);
    EXPECT_NEAR(volumeOf(ev.bodies.at(s.body)), 50.0 * 20 * 10, 1e-6);
    EXPECT_NEAR(volumeOf(ev.bodies.at(s.other)), 5.0 * 5 * 5, 1e-6);
}

TEST(SketchCache, UnrelatedParameterEditsDoNotResolve) {
    Model s;
    evalOk(s.d);
    const std::uint64_t r0 = sketchSolverRuns();

    // Unused parameter: nothing recomputes.
    mustVoid(s.d.setParameterExpression("spare", "2 mm"));
    auto a = evalOk(s.d);
    EXPECT_EQ(a.computedCount, 0u);

    // Parameter of a feature AFTER the sketch: the sketch is a reused prefix.
    mustVoid(s.d.setParameterExpression("thick", "12 mm"));
    auto b = evalOk(s.d);
    EXPECT_TRUE(statusOf(b, s.sk)->reused);
    EXPECT_NEAR(volumeOf(b.bodies.at(s.body)), 40.0 * 20 * 12, 1e-6);

    // Parameter of a feature BEFORE the sketch: the evaluator recomputes the sketch (its prefix changed), but its plane and
    // used parameters are unchanged, so the memoised solve is used.
    mustVoid(s.d.setParameterExpression("d0", "7 mm"));
    auto c = evalOk(s.d);
    EXPECT_FALSE(statusOf(c, s.sk)->reused);
    EXPECT_NEAR(volumeOf(c.bodies.at(s.other)), 5.0 * 5 * 7, 1e-6);
    EXPECT_NEAR(volumeOf(c.bodies.at(s.body)), 40.0 * 20 * 12, 1e-6);

    EXPECT_EQ(sketchSolverRuns() - r0, 0u) << "no edit above concerns the sketch";
}

TEST(SketchCache, UndoRedoOfSketchEditReusesSolves) {
    Model s;
    evalOk(s.d);
    const std::uint64_t r0 = sketchSolverRuns();
    mustVoid(s.d.replaceSketch(s.sk, rect(xy(), "w", "30 mm")));
    auto edited = evalOk(s.d);
    EXPECT_EQ(sketchSolverRuns() - r0, 1u);
    EXPECT_NEAR(volumeOf(edited.bodies.at(s.body)), 40.0 * 30 * 10, 1e-6);

    mustVoid(s.d.undo()); // the previous SketchFeature instance comes back with its memoised solve
    auto undone = evalOk(s.d);
    EXPECT_NEAR(volumeOf(undone.bodies.at(s.body)), 40.0 * 20 * 10, 1e-6);
    mustVoid(s.d.redo());
    auto redone = evalOk(s.d);
    EXPECT_NEAR(volumeOf(redone.bodies.at(s.body)), 40.0 * 30 * 10, 1e-6);
    EXPECT_EQ(sketchSolverRuns() - r0, 1u);
}

TEST(SketchCache, AttachedSketchResolvesOnlyWhenItsPlaneMoves) {
    Document d;
    mustVoid(d.addParameter("zp", "10 mm"));
    mustVoid(d.addParameter("d0", "5 mm"));
    const BodyId other = d.allocateBodyId();
    mustVoid(d.addFeature(unrelatedExtrude(d.allocateFeatureId(), other, "d0")));
    ConstructionPlaneParams pp;
    pp.kind = ConstructionPlaneParams::Kind::ThreePoints;
    pp.points = {{{"0 mm", "0 mm", "zp"}, {"10 mm", "0 mm", "zp"}, {"0 mm", "10 mm", "zp"}}};
    const FeatureId plane = d.allocateFeatureId();
    mustVoid(d.addFeature(std::make_shared<ConstructionPlaneFeature>(plane, pp)));
    const FeatureId sk = d.allocateFeatureId();
    sketch::Sketch r = rect(xy(), "40 mm", "20 mm");
    const std::string profile = onlyProfile(r);
    PlaneRef at;
    at.isFace = false;
    at.construction = plane;
    mustVoid(d.addFeature(std::make_shared<SketchFeature>(sk, SketchFeatureParams{std::move(r), at})));
    const BodyId body = d.allocateBodyId();
    mustVoid(d.addFeature(refExtrude(d.allocateFeatureId(), body, sk, profile, "10 mm")));

    auto zMin = [&](const DocumentEvaluation& ev) { return ev.bodies.at(body).boundingBox().min.z; };
    auto first = evalOk(d);
    EXPECT_NEAR(zMin(first), 10.0, 1e-6);
    const std::uint64_t r0 = sketchSolverRuns();

    mustVoid(d.setParameterExpression("d0", "6 mm")); // upstream edit, plane unchanged
    auto a = evalOk(d);
    EXPECT_FALSE(statusOf(a, sk)->reused);
    EXPECT_EQ(sketchSolverRuns() - r0, 0u);

    mustVoid(d.setParameterExpression("zp", "25 mm")); // the attachment plane moves: a new solve on the new plane
    auto b = evalOk(d);
    EXPECT_EQ(sketchSolverRuns() - r0, 1u);
    EXPECT_NEAR(zMin(b), 25.0, 1e-6);
    EXPECT_NEAR(b.sketches.at(sk).sketch.plane().origin.z, 25.0, 1e-9);
}

TEST(SketchCache, ChannelDeliversSketchToPlainEvaluator) {
    // No Document pre-pass: a plain recompute::Evaluator over the stored timeline resolves sketch references through the
    // channel (extrude and a hole placed from sketch points).
    ParameterTable params;
    mustVoid(params.add("w", "40 mm"));
    recompute::Timeline tl;
    const FeatureId sk{1};
    sketch::Sketch r = rect(xy(), "w", "20 mm");
    const std::string profile = onlyProfile(r);
    const auto pt = must(r.addPoint({10, 10}));
    tl.append(std::make_shared<SketchFeature>(sk, SketchFeatureParams{std::move(r), std::nullopt}));
    const BodyId body{2};
    tl.append(refExtrude(FeatureId{3}, body, sk, profile, "10 mm"));
    HoleParams hp;
    hp.body = body;
    hp.diameter = "4 mm";
    hp.through = true;
    hp.sketchPoints = HoleSketchPlacement{sk, {pt}, false, std::nullopt};
    tl.append(std::make_shared<HoleFeature>(FeatureId{4}, hp));

    recompute::Evaluator ev;
    auto res = ev.evaluate(tl, &params);
    ASSERT_TRUE(res.has_value());
    for (const auto& s : res->statuses) EXPECT_EQ(s.state, FeatureStatus::State::Ok) << s.error.message;
    ASSERT_TRUE(res->statuses.front().sketch != nullptr);
    EXPECT_NEAR(volumeOf(res->bodies.at(body)), 40.0 * 20 * 10 - 3.14159265358979 * 4 * 10, 1e-3);
}

TEST(SketchCache, SuppressedOrLaterSketchIsNotVisible) {
    Model s;
    mustVoid(s.d.setSuppressed(s.sk, true));
    auto ev = s.d.evaluate();
    ASSERT_TRUE(ev.has_value());
    const FeatureStatus* e = statusOf(*ev, s.ext);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->state, FeatureStatus::State::Failed);
    EXPECT_EQ(e->error.code, ErrorCode::NotFound);
    EXPECT_FALSE(ev->sketches.contains(s.sk));

    // A reference to a sketch later on the timeline sees nothing either.
    recompute::Timeline tl;
    sketch::Sketch r = rect(xy(), "40 mm", "20 mm");
    const std::string profile = onlyProfile(r);
    tl.append(refExtrude(FeatureId{10}, BodyId{11}, FeatureId{12}, profile, "10 mm"));
    tl.append(std::make_shared<SketchFeature>(FeatureId{12}, SketchFeatureParams{std::move(r), std::nullopt}));
    recompute::Evaluator evaluator;
    auto res = evaluator.evaluate(tl);
    ASSERT_TRUE(res.has_value());
    EXPECT_EQ(res->statuses[0].state, FeatureStatus::State::Failed);
    EXPECT_EQ(res->statuses[1].state, FeatureStatus::State::Ok);
}
