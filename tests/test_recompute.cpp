// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <atomic>
#include <filesystem>
#include <random>

#include "io/document_io.hpp"
#include "kernel/ops.hpp"
#include "recompute/evaluator.hpp"
#include "recompute/extrude_feature.hpp"

using namespace iw;
using namespace iw::recompute;

namespace {

constexpr std::uint64_t kStampMask = ~0ULL;

struct DummyFeature : Feature {
    explicit DummyFeature(std::uint64_t i) : id_(i) {}
    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "dummy"; }
    std::vector<BodyId> writesBodies() const override { return {}; }
    std::uint64_t paramHash() const override { return 0; }
    std::string paramsJson() const override { return "{}"; }
    Result<FeatureResult> compute(const EvalContext&) const override { return FeatureResult{}; }
    FeatureId id_;
};

/// Calls cancel.cancel() from inside compute and reports Cancelled (a feature that observed cancellation).
struct CancellingFeature : Feature {
    CancellingFeature(std::uint64_t i, CancelSource src) : id_(i), src_(std::move(src)) {}
    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "cancelling"; }
    std::vector<BodyId> writesBodies() const override { return {}; }
    std::uint64_t paramHash() const override { return 7; }
    std::string paramsJson() const override { return "{}"; }
    Result<FeatureResult> compute(const EvalContext&) const override {
        src_.cancel();
        return makeError(ErrorCode::Cancelled, "cancelled");
    }
    FeatureId id_;
    mutable CancelSource src_;
};

ExtrudeParams rect(double x0, double y0, double w, double h, double dz, BodyId body,
                   ExtrudeParams::Operation op = ExtrudeParams::Operation::NewBody, Vec3 origin = {}) {
    ExtrudeParams p;
    p.planeOrigin = origin;
    p.points = {{x0, y0}, {x0 + w, y0}, {x0 + w, y0 + h}, {x0, y0 + h}};
    p.vector = {0, 0, dz};
    p.operation = op;
    p.body = body;
    return p;
}

std::shared_ptr<ExtrudeFeature> ext(std::uint64_t id, ExtrudeParams p) {
    return std::make_shared<ExtrudeFeature>(FeatureId{id}, std::move(p));
}

constexpr BodyId kB1{1}, kB2{2};

/// F1: box 10x10x10 -> B1; F2: far box -> B2; F3: join a 10x10x10 box shifted by 5 in x into B1 (overlap 5x10x10).
Timeline makeTimeline(double f3Height = 10) {
    Timeline t;
    t.append(ext(1, rect(0, 0, 10, 10, 10, kB1)));
    t.append(ext(2, rect(0, 0, 4, 4, 4, kB2, ExtrudeParams::Operation::NewBody, {100, 0, 0})));
    t.append(ext(3, rect(5, 0, 10, 10, f3Height, kB1, ExtrudeParams::Operation::Join)));
    return t;
}

double volumeOf(const kernel::Shape& s) {
    auto m = kernel::massProperties(s);
    EXPECT_TRUE(m.has_value());
    return m ? m->volume : -1;
}

std::uint32_t faces(const kernel::Shape& s) { return s.subShapeCount(kernel::SubShapeKind::Face); }

struct Fingerprint {
    std::map<BodyId, std::uint64_t> stamps;
    std::map<BodyId, std::array<std::uint32_t, 3>> counts;
    bool operator==(const Fingerprint&) const = default;
};
Fingerprint fingerprint(const EvalResult& r) {
    Fingerprint f;
    for (const auto& [id, s] : r.bodies) {
        f.stamps[id] = s.stamp().value & kStampMask;
        f.counts[id] = {s.subShapeCount(kernel::SubShapeKind::Face), s.subShapeCount(kernel::SubShapeKind::Edge),
                        s.subShapeCount(kernel::SubShapeKind::Vertex)};
    }
    return f;
}

} // namespace

TEST(Timeline, InsertRemoveMarkerSuppress) {
    Timeline t;
    t.append(std::make_shared<DummyFeature>(1));
    t.append(std::make_shared<DummyFeature>(2));
    t.append(std::make_shared<DummyFeature>(3));
    EXPECT_EQ(t.rollbackMarker(), 3u);
    t.setRollbackMarker(2);
    t.insert(0, std::make_shared<DummyFeature>(9)); // before marker -> marker shifts
    EXPECT_EQ(t.rollbackMarker(), 3u);
    EXPECT_EQ(t.indexOf(FeatureId{2}), 2u);
    ASSERT_TRUE(t.remove(FeatureId{9}));
    EXPECT_EQ(t.rollbackMarker(), 2u);
    EXPECT_FALSE(t.remove(FeatureId{42}));
    t.setSuppressed(FeatureId{1}, true);
    EXPECT_TRUE(t.isSuppressed(FeatureId{1}));
}

TEST(Registry, UnknownKind) {
    FeatureRegistry r;
    EXPECT_EQ(r.create("nope", FeatureId{1}, "{}").error().code, ErrorCode::NotFound);
}

TEST(Registry, ExtrudeRegisteredAndRejectsBadJson) {
    FeatureRegistry r;
    registerExtrudeFeature(r);
    EXPECT_EQ(r.create("extrude", FeatureId{1}, "not json").error().code, ErrorCode::ParseError);
    EXPECT_EQ(r.create("extrude", FeatureId{1}, "{}").error().code, ErrorCode::ParseError);
    auto f = ext(1, rect(0, 0, 1, 1, 1, kB1));
    auto back = r.create("extrude", FeatureId{1}, f->paramsJson());
    ASSERT_TRUE(back.has_value());
    EXPECT_EQ((*back)->paramHash(), f->paramHash());
    EXPECT_EQ((*back)->paramsJson(), f->paramsJson());
}

TEST(Evaluator, EmptyTimeline) {
    Evaluator e;
    Timeline t;
    auto r = e.evaluate(t);
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(r->bodies.empty());
    EXPECT_EQ(r->computedCount, 0u);
}

TEST(Evaluator, ThreeFeatureTimelineEvaluates) {
    Evaluator e;
    Timeline t = makeTimeline();
    auto r = e.evaluate(t);
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(r->statuses.size(), 3u);
    for (const auto& s : r->statuses) {
        EXPECT_EQ(s.state, FeatureStatus::State::Ok);
        EXPECT_FALSE(s.reused);
    }
    EXPECT_EQ(r->computedCount, 3u);
    EXPECT_EQ(r->reusedCount, 0u);
    ASSERT_EQ(r->bodies.size(), 2u);
    EXPECT_NEAR(volumeOf(r->bodies.at(kB1)), 1500.0, 1e-6); // 1000 + 1000 - 500 overlap
    EXPECT_NEAR(volumeOf(r->bodies.at(kB2)), 64.0, 1e-6);
    for (const auto& [id, shape] : r->bodies) {
        EXPECT_NE(shape.stamp().value, 0u);
        const auto& nm = r->namings.at(id);
        EXPECT_EQ(nm.size(naming::ElementType::Face), faces(shape));
        for (std::uint32_t i = 0; i < faces(shape); ++i) EXPECT_TRUE(nm.nameOf(naming::ElementType::Face, i).has_value());
    }
    // Join result is named by feature 3 / earlier features, never left unnamed; box B2 has 6 distinct face names.
    std::set<std::string> names;
    for (std::uint32_t i = 0; i < 6; ++i) names.insert(r->namings.at(kB2).nameOf(naming::ElementType::Face, i)->toString());
    EXPECT_EQ(names.size(), 6u);
}

TEST(Evaluator, EditFeature3ReusesFirstTwo) {
    Evaluator e;
    Timeline t = makeTimeline();
    auto a = e.evaluate(t);
    ASSERT_TRUE(a.has_value());

    ASSERT_TRUE(t.replace(ext(3, rect(5, 0, 10, 10, 20, kB1, ExtrudeParams::Operation::Join))));
    auto b = e.evaluate(t);
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(b->reusedCount, 2u);
    EXPECT_EQ(b->computedCount, 1u);
    EXPECT_TRUE(b->statuses[0].reused);
    EXPECT_TRUE(b->statuses[1].reused);
    EXPECT_FALSE(b->statuses[2].reused);
    EXPECT_EQ(b->bodies.at(kB2).stamp(), a->bodies.at(kB2).stamp());       // untouched by the edit
    EXPECT_NE(b->bodies.at(kB1).stamp(), a->bodies.at(kB1).stamp());
    EXPECT_NEAR(volumeOf(b->bodies.at(kB1)), 2500, 1e-6); // 10*10*20 box, overlap 5*10*10
}

TEST(Evaluator, ImplicitBodyDependencyIsHonoured) {
    // Feature 3 holds no explicit reference to feature 1; its dependency on body B1 is only implicit.
    Evaluator e;
    Timeline t = makeTimeline();
    auto a = e.evaluate(t);
    ASSERT_TRUE(a.has_value());
    EXPECT_EQ(t.at(2)->readsBodies(), std::vector<BodyId>{kB1});
    EXPECT_TRUE(t.at(2)->explicitRefs().empty());

    // Editing feature 1 (feature 3 unchanged) must recompute feature 3 against the new B1.
    ASSERT_TRUE(t.replace(ext(1, rect(0, 0, 10, 10, 30, kB1))));
    auto b = e.evaluate(t);
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(b->reusedCount, 0u);
    EXPECT_EQ(b->computedCount, 3u);
    EXPECT_NEAR(volumeOf(b->bodies.at(kB1)), 3000 + 1000 - 500, 1e-6);
    EXPECT_NE(b->bodies.at(kB1).stamp(), a->bodies.at(kB1).stamp());
}

TEST(Evaluator, RollbackMarkerMovesAreCacheHits) {
    Evaluator e;
    Timeline t = makeTimeline();
    auto full = e.evaluate(t);
    ASSERT_TRUE(full.has_value());

    t.setRollbackMarker(1);
    auto r1 = e.evaluate(t);
    ASSERT_TRUE(r1.has_value());
    EXPECT_EQ(r1->computedCount, 0u);
    EXPECT_EQ(r1->reusedCount, 1u);
    EXPECT_EQ(r1->statuses.size(), 1u);
    ASSERT_EQ(r1->bodies.size(), 1u);
    EXPECT_NEAR(volumeOf(r1->bodies.at(kB1)), 1000.0, 1e-6);

    t.setRollbackMarker(2);
    auto r2 = e.evaluate(t);
    ASSERT_TRUE(r2.has_value());
    EXPECT_EQ(r2->computedCount, 0u);
    EXPECT_EQ(r2->reusedCount, 2u);

    t.setRollbackMarker(0);
    auto r0 = e.evaluate(t);
    ASSERT_TRUE(r0.has_value());
    EXPECT_TRUE(r0->bodies.empty());
    EXPECT_EQ(r0->computedCount, 0u);

    t.clearRollbackMarker();
    auto again = e.evaluate(t);
    ASSERT_TRUE(again.has_value());
    EXPECT_EQ(again->computedCount, 0u);
    EXPECT_EQ(again->reusedCount, 3u);
    EXPECT_EQ(fingerprint(*again), fingerprint(*full));
}

TEST(Evaluator, SuppressAndUnsuppress) {
    Evaluator e;
    Timeline t = makeTimeline();
    auto base = e.evaluate(t);
    ASSERT_TRUE(base.has_value());

    t.setSuppressed(FeatureId{1}, true);
    auto s = e.evaluate(t);
    ASSERT_TRUE(s.has_value());
    EXPECT_EQ(s->statuses[0].state, FeatureStatus::State::Suppressed);
    EXPECT_EQ(s->statuses[1].state, FeatureStatus::State::Ok);
    // Join into a body that no longer exists fails (and is not silently turned into something else).
    EXPECT_EQ(s->statuses[2].state, FeatureStatus::State::Failed);
    EXPECT_EQ(s->statuses[2].error.code, ErrorCode::NotFound);
    EXPECT_FALSE(s->bodies.contains(kB1));
    EXPECT_TRUE(s->bodies.contains(kB2));
    EXPECT_EQ(s->computedCount, 3u);

    t.setSuppressed(FeatureId{1}, false);
    auto u = e.evaluate(t);
    ASSERT_TRUE(u.has_value());
    EXPECT_EQ(fingerprint(*u), fingerprint(*base));
    for (const auto& st : u->statuses) EXPECT_EQ(st.state, FeatureStatus::State::Ok);
}

TEST(Evaluator, FailingFeatureLaterFeaturesSeePreviousState) {
    Evaluator e;
    Timeline t = makeTimeline();
    // Feature 2 fails (extrusion vector lies in the profile plane); feature 3 (independent of B2) still runs.
    ExtrudeParams bad = rect(0, 0, 4, 4, 0, kB2, ExtrudeParams::Operation::NewBody, {100, 0, 0});
    bad.vector = {1, 0, 0};
    ASSERT_TRUE(t.replace(ext(2, bad)));
    auto r = e.evaluate(t);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->statuses[0].state, FeatureStatus::State::Ok);
    EXPECT_EQ(r->statuses[1].state, FeatureStatus::State::Failed);
    EXPECT_NE(r->statuses[1].error.code, ErrorCode::NotImplemented);
    EXPECT_FALSE(r->statuses[1].diagnostics.empty());
    EXPECT_EQ(r->statuses[2].state, FeatureStatus::State::Ok);
    EXPECT_FALSE(r->bodies.contains(kB2));
    EXPECT_NEAR(volumeOf(r->bodies.at(kB1)), 1500.0, 1e-6);
}

TEST(Evaluator, FailureSkipsDependentsAndPropagates) {
    Evaluator e;
    Timeline t = makeTimeline();
    ExtrudeParams bad = rect(0, 0, 10, 10, 0, kB1);
    bad.vector = {1, 0, 0};
    ASSERT_TRUE(t.replace(ext(1, bad)));
    t.append(ext(4, rect(0, 0, 1, 1, 1, kB1, ExtrudeParams::Operation::Join))); // reads B1 transitively
    auto r = e.evaluate(t);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->statuses[0].state, FeatureStatus::State::Failed);
    EXPECT_EQ(r->statuses[1].state, FeatureStatus::State::Ok);
    EXPECT_EQ(r->statuses[2].state, FeatureStatus::State::Skipped);
    EXPECT_EQ(r->statuses[3].state, FeatureStatus::State::Skipped);
    EXPECT_TRUE(r->bodies.contains(kB2));
    EXPECT_FALSE(r->bodies.contains(kB1));

    // Fixing feature 1 recomputes everything and heals the dependents.
    ASSERT_TRUE(t.replace(ext(1, rect(0, 0, 10, 10, 10, kB1))));
    auto fixed = e.evaluate(t);
    ASSERT_TRUE(fixed.has_value());
    for (const auto& st : fixed->statuses) EXPECT_EQ(st.state, FeatureStatus::State::Ok);
    EXPECT_TRUE(fixed->bodies.contains(kB1));
}

TEST(Evaluator, CancelledBeforeStartKeepsCacheConsistent) {
    Evaluator e;
    Timeline t = makeTimeline();
    auto base = e.evaluate(t);
    ASSERT_TRUE(base.has_value());
    ASSERT_TRUE(t.replace(ext(2, rect(0, 0, 8, 8, 8, kB2, ExtrudeParams::Operation::NewBody, {100, 0, 0}))));

    CancelSource src;
    src.cancel();
    auto c = e.evaluate(t, src.token());
    ASSERT_FALSE(c.has_value());
    EXPECT_EQ(c.error().code, ErrorCode::Cancelled);

    auto after = e.evaluate(t);
    ASSERT_TRUE(after.has_value());
    EXPECT_EQ(after->reusedCount, 1u); // feature 1 survived; 2 and 3 were recomputed
    EXPECT_EQ(after->computedCount, 2u);
    EXPECT_NEAR(volumeOf(after->bodies.at(kB2)), 512.0, 1e-6);

    Evaluator fresh;
    auto ref = fresh.evaluate(t);
    ASSERT_TRUE(ref.has_value());
    EXPECT_EQ(fingerprint(*after), fingerprint(*ref));
}

TEST(Evaluator, CancelledMidwayKeepsCompletedPrefix) {
    Evaluator e;
    CancelSource src;
    Timeline t = makeTimeline();
    t.insert(2, std::make_shared<CancellingFeature>(50, src));
    auto c = e.evaluate(t, src.token());
    ASSERT_FALSE(c.has_value());
    EXPECT_EQ(c.error().code, ErrorCode::Cancelled);

    ASSERT_TRUE(t.remove(FeatureId{50}));
    auto r = e.evaluate(t);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->reusedCount, 2u); // features 1 and 2 completed before the cancellation
    EXPECT_EQ(r->computedCount, 1u);
    EXPECT_NEAR(volumeOf(r->bodies.at(kB1)), 1500.0, 1e-6);
}

TEST(Evaluator, RepeatedEvaluationIdentical) {
    Timeline t = makeTimeline();
    Evaluator a, b;
    auto r1 = a.evaluate(t);
    auto r2 = a.evaluate(t);        // fully cached
    auto r3 = b.evaluate(t);        // independent evaluator
    a.clearCache();
    auto r4 = a.evaluate(t);        // recomputed from scratch
    ASSERT_TRUE(r1 && r2 && r3 && r4);
    EXPECT_EQ(r2->reusedCount, 3u);
    EXPECT_EQ(r4->computedCount, 3u);
    EXPECT_EQ(fingerprint(*r1), fingerprint(*r2));
    EXPECT_EQ(fingerprint(*r1), fingerprint(*r3));
    EXPECT_EQ(fingerprint(*r1), fingerprint(*r4));
    EXPECT_EQ(faces(r1->bodies.at(kB2)), 6u);
    for (const auto& [id, nm] : r1->namings)
        for (std::uint32_t i = 0; i < faces(r1->bodies.at(id)); ++i)
            EXPECT_EQ(nm.nameOf(naming::ElementType::Face, i), r4->namings.at(id).nameOf(naming::ElementType::Face, i));
}

TEST(Evaluator, JoinOfDisjointBodyFailsCleanlyOrFuses) {
    // Join of a disjoint solid yields a compound or fails; either way the evaluator must not crash and must report.
    Evaluator e;
    Timeline t;
    t.append(ext(1, rect(0, 0, 10, 10, 10, kB1)));
    t.append(ext(2, rect(0, 0, 2, 2, 2, kB1, ExtrudeParams::Operation::Join, {500, 0, 0})));
    auto r = e.evaluate(t);
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(r->statuses[1].state == FeatureStatus::State::Ok || r->statuses[1].state == FeatureStatus::State::Failed);
    EXPECT_TRUE(r->bodies.contains(kB1));
}

TEST(IoRoundTrip, ExtrudeTimelineReevaluatesIdentically) {
    FeatureRegistry reg;
    registerExtrudeFeature(reg);

    io::Document doc;
    doc.name = "extrudes";
    doc.nextId = 100;
    doc.timeline = makeTimeline();
    doc.timeline.setSuppressed(FeatureId{2}, true);

    std::random_device rd;
    const auto dir = std::filesystem::temp_directory_path() / ("iw_recompute_rt_" + std::to_string(rd()) + ".iwd");
    std::filesystem::remove_all(dir);
    ASSERT_TRUE(io::saveDocument(dir, doc).has_value());
    auto loaded = io::loadDocument(dir, reg);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    std::filesystem::remove_all(dir);

    ASSERT_EQ(loaded->timeline.size(), doc.timeline.size());
    for (std::size_t i = 0; i < doc.timeline.size(); ++i) {
        EXPECT_EQ(loaded->timeline.at(i)->kind(), "extrude");
        EXPECT_EQ(loaded->timeline.at(i)->paramHash(), doc.timeline.at(i)->paramHash());
    }
    EXPECT_TRUE(loaded->timeline.isSuppressed(FeatureId{2}));

    Evaluator e1, e2;
    auto a = e1.evaluate(doc.timeline);
    auto b = e2.evaluate(loaded->timeline);
    ASSERT_TRUE(a && b);
    EXPECT_EQ(fingerprint(*a), fingerprint(*b));
    ASSERT_TRUE(a->bodies.contains(kB1));
    EXPECT_NEAR(volumeOf(b->bodies.at(kB1)), 1500.0, 1e-6);
    EXPECT_FALSE(b->bodies.contains(kB2)); // suppressed
}
