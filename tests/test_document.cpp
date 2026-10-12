// SPDX-License-Identifier: GPL-3.0-or-later
// B1c: Qt-free document model: undoable commands (add / edit / suppress / delete / rollback marker / parameters),
// change listeners, evaluation statuses, and save -> load -> evaluate reproducing identical stamps.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <unistd.h>
#include <string>
#include <vector>

#include "features/document.hpp"
#include "features/extrude_feature.hpp"
#include "features/sketch_feature.hpp"
#include "kernel/ops.hpp"
#include "sketch/profile.hpp"
#include "sketch/solver/bridge.hpp"

using namespace iw;
using namespace iw::features;
using recompute::FeatureStatus;
using sketch::PointRole;

namespace {

template <class T> T must(Result<T> r) {
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message);
    return r.has_value() ? *r : T{};
}
void mustVoid(Result<void> r) { EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message); }

double volumeOf(const kernel::Shape& s) {
    auto m = kernel::massProperties(s);
    EXPECT_TRUE(m.has_value());
    return m ? m->volume : -1;
}

/// Polygon (P0) extrude of a w x h footprint at the origin, height 10, NewBody onto `body`.
std::shared_ptr<const recompute::Feature> box(FeatureId id, BodyId body, double w, double h) {
    ExtrudeParams p;
    PolygonSource src;
    src.points = {{0, 0}, {w, 0}, {w, h}, {0, h}};
    src.vector = {0, 0, 10};
    p.source = src;
    p.body = body;
    return std::make_shared<ExtrudeFeature>(id, p);
}

DocumentEvaluation evalOk(Document& d) {
    auto ev = d.evaluate();
    EXPECT_TRUE(ev.has_value()) << (ev ? "" : ev.error().message);
    if (!ev) return {};
    for (const auto& s : ev->statuses) EXPECT_EQ(s.state, FeatureStatus::State::Ok) << s.error.message;
    return *ev;
}

std::filesystem::path scratchDir(const std::string& leaf) {
    auto dir = std::filesystem::temp_directory_path() / ("iw_b1c_" + leaf + "_" + std::to_string(::getpid()));
    std::filesystem::remove_all(dir);
    return dir;
}

} // namespace

TEST(Document, AddSuppressEditDeleteMarkerAndParametersUndoAndRedo) {
    Document d;
    const BodyId body = d.allocateBodyId();
    mustVoid(d.addParameter("d1", "10 mm"));
    const FeatureId f1 = d.allocateFeatureId();
    mustVoid(d.addFeature(box(f1, body, 10, 10)));
    EXPECT_NEAR(volumeOf(evalOk(d).bodies.at(body)), 1000.0, 1e-6);
    EXPECT_EQ(d.undoText(), "Add Feature");

    // Suppress: no body; undo restores it; redo suppresses again.
    mustVoid(d.setSuppressed(f1, true));
    EXPECT_EQ(d.evaluate()->bodies.count(body), 0u);
    mustVoid(d.undo());
    EXPECT_NEAR(volumeOf(evalOk(d).bodies.at(body)), 1000.0, 1e-6);
    mustVoid(d.redo());
    EXPECT_EQ(d.evaluate()->bodies.count(body), 0u);
    mustVoid(d.undo());

    // Edit: 20 x 10 x 10; undo returns to 10 x 10 x 10.
    mustVoid(d.editFeature(box(f1, body, 20, 10)));
    EXPECT_NEAR(volumeOf(evalOk(d).bodies.at(body)), 2000.0, 1e-6);
    mustVoid(d.undo());
    EXPECT_NEAR(volumeOf(evalOk(d).bodies.at(body)), 1000.0, 1e-6);

    // Parameter edit and its undo.
    mustVoid(d.setParameterExpression("d1", "20 mm"));
    EXPECT_EQ(d.parameters().find("d1")->expression.source(), "20 mm");
    mustVoid(d.undo());
    EXPECT_EQ(d.parameters().find("d1")->expression.source(), "10 mm");

    // Rollback marker before the feature: no body; undo restores the end marker.
    mustVoid(d.setRollbackMarker(0));
    EXPECT_EQ(d.evaluate()->bodies.count(body), 0u);
    mustVoid(d.undo());
    EXPECT_NEAR(volumeOf(evalOk(d).bodies.at(body)), 1000.0, 1e-6);

    // Delete: gone; undo brings it back at the same position.
    mustVoid(d.deleteFeature(f1));
    EXPECT_EQ(d.timeline().size(), 0u);
    mustVoid(d.undo());
    EXPECT_EQ(d.timeline().size(), 1u);
    EXPECT_NEAR(volumeOf(evalOk(d).bodies.at(body)), 1000.0, 1e-6);
}

TEST(Document, FailedCommandsLeaveTheDocumentUnchanged) {
    Document d;
    const BodyId body = d.allocateBodyId();
    const FeatureId f1 = d.allocateFeatureId();
    mustVoid(d.addFeature(box(f1, body, 10, 10)));

    EXPECT_FALSE(d.addFeature(box(f1, body, 5, 5)).has_value()); // duplicate id
    EXPECT_FALSE(d.setSuppressed(FeatureId{999}, true).has_value());
    EXPECT_FALSE(d.deleteFeature(FeatureId{999}).has_value());
    EXPECT_FALSE(d.setRollbackMarker(7).has_value());
    EXPECT_FALSE(d.setParameterExpression("nope", "1 mm").has_value());
    EXPECT_EQ(d.timeline().size(), 1u);
    EXPECT_NEAR(volumeOf(evalOk(d).bodies.at(body)), 100.0 * 10.0, 1e-6);
    // Only the successful add is undoable.
    mustVoid(d.undo());
    EXPECT_FALSE(d.canUndo());
}

TEST(Document, ListenersFireOncePerSuccessfulChange) {
    Document d;
    int calls = 0;
    const auto id = d.addListener([&] { ++calls; });
    const BodyId body = d.allocateBodyId();
    const FeatureId f1 = d.allocateFeatureId();
    mustVoid(d.addFeature(box(f1, body, 10, 10)));
    EXPECT_EQ(calls, 1);
    EXPECT_FALSE(d.setSuppressed(FeatureId{42}, true).has_value());
    EXPECT_EQ(calls, 1); // failure: no notification
    mustVoid(d.setSuppressed(f1, true));
    EXPECT_EQ(calls, 2);
    mustVoid(d.undo());
    EXPECT_EQ(calls, 3);
    d.removeListener(id);
    mustVoid(d.redo());
    EXPECT_EQ(calls, 3);
}

TEST(Document, RollbackMarkerHidesLaterFeaturesWithSkippedStatus) {
    Document d;
    const BodyId body = d.allocateBodyId();
    const FeatureId f1 = d.allocateFeatureId();
    const FeatureId f2 = d.allocateFeatureId();
    mustVoid(d.addFeature(box(f1, body, 10, 10)));
    mustVoid(d.addFeature(box(f2, d.allocateBodyId(), 4, 4)));
    mustVoid(d.setRollbackMarker(1));
    auto ev = d.evaluate();
    ASSERT_TRUE(ev.has_value());
    ASSERT_EQ(ev->statuses.size(), 2u);
    EXPECT_EQ(ev->statuses[1].state, FeatureStatus::State::Skipped);
    EXPECT_EQ(ev->bodies.size(), 1u);
}

TEST(Document, SaveLoadEvaluateGivesIdenticalStamps) {
    const auto dir = scratchDir("roundtrip");
    DocumentEvaluation before;
    std::vector<std::string> names;
    {
        Document d;
        mustVoid(d.addParameter("w", "40 mm"));
        mustVoid(d.addParameter("h", "20 mm"));
        // Sketch rectangle (constraints + dimensions) extruded by reference, then a polygon block on a second body.
        sketch::Sketch sk(*sketch::SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0}));
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
        must(sk.addDimension(sketch::DimensionKind::DistanceX, {{b, PointRole::Start}, {b, PointRole::End}}, "w"));
        must(sk.addDimension(sketch::DimensionKind::DistanceY, {{r, PointRole::Start}, {r, PointRole::End}}, "h"));
        auto profiles = sketch::detectProfiles(sk);
        ASSERT_TRUE(profiles.has_value());
        const FeatureId skId = d.allocateFeatureId();
        mustVoid(d.addFeature(std::make_shared<SketchFeature>(skId, SketchFeatureParams{sk, std::nullopt})));
        ExtrudeParams p;
        p.source = SketchReferenceSource{skId, profiles->front().name};
        p.distance = "10 mm";
        p.body = d.allocateBodyId();
        mustVoid(d.addFeature(std::make_shared<ExtrudeFeature>(d.allocateFeatureId(), p)));
        before = evalOk(d);
        mustVoid(d.save(dir));
        EXPECT_TRUE(d.isClean());
    }

    Document loaded;
    mustVoid(loaded.load(dir));
    EXPECT_FALSE(loaded.canUndo());
    DocumentEvaluation after = evalOk(loaded);
    ASSERT_EQ(after.bodies.size(), before.bodies.size());
    for (const auto& [id, shape] : before.bodies) {
        ASSERT_TRUE(after.bodies.count(id));
        EXPECT_EQ(after.bodies.at(id).stamp().value, shape.stamp().value) << "body " << id.value();
        EXPECT_NEAR(volumeOf(after.bodies.at(id)), volumeOf(shape), 1e-9);
    }
    ASSERT_EQ(after.statuses.size(), before.statuses.size());
    for (std::size_t i = 0; i < before.statuses.size(); ++i) {
        EXPECT_EQ(after.statuses[i].id, before.statuses[i].id);
        EXPECT_EQ(after.statuses[i].state, before.statuses[i].state);
    }
    EXPECT_NEAR(volumeOf(after.bodies.begin()->second), 8000.0, 1e-6);
    std::filesystem::remove_all(dir);
}
