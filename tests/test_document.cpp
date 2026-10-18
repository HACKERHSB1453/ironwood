// SPDX-License-Identifier: GPL-3.0-or-later
// B1c: Qt-free document model: undoable commands (add / edit / suppress / delete / rollback marker / parameters),
// change listeners, evaluation statuses, and save -> load -> evaluate reproducing identical stamps.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
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

// ---- W2a: parameter API, feature names, construction edits, sketchForEdit / replaceSketch -------------------------

namespace {

/// Sketch rectangle `w` x `h` (both expressions) with its own profile name, for the parameter-rename tests.
sketch::Sketch exprRect(const std::string& w, const std::string& h) {
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
    must(sk.addDimension(sketch::DimensionKind::DistanceX, {{b, PointRole::Start}, {b, PointRole::End}}, w));
    must(sk.addDimension(sketch::DimensionKind::DistanceY, {{r, PointRole::Start}, {r, PointRole::End}}, h));
    return sk;
}

/// Named planar face of `body` at +z height `z` (the top of a box), if any.
std::optional<naming::ElementRef> topFaceAt(const DocumentEvaluation& ev, BodyId body, double z) {
    const kernel::Shape& s = ev.bodies.at(body);
    const naming::NamingMap& m = ev.namings.at(body);
    for (std::uint32_t i = 0; i < s.subShapeCount(kernel::SubShapeKind::Face); ++i) {
        const kernel::FaceGeometry g = s.faceGeometry(i);
        if (g.kind != kernel::FaceGeometry::Kind::Plane) continue;
        if (std::abs(g.normal.z - 1) > 1e-9 || std::abs(g.point.z - z) > 1e-9) continue;
        if (auto nm = m.nameOf(naming::ElementType::Face, i)) return naming::ElementRef{nm->feature, naming::ElementType::Face, *nm};
    }
    return std::nullopt;
}

std::shared_ptr<const recompute::Feature> sketchFeat(FeatureId id, sketch::Sketch sk) {
    return std::make_shared<SketchFeature>(id, SketchFeatureParams{std::move(sk), std::nullopt});
}

std::string firstProfile(const sketch::Sketch& sk) {
    auto profiles = sketch::detectProfiles(sk);
    EXPECT_TRUE(profiles.has_value() && profiles->size() == 1);
    return (profiles && !profiles->empty()) ? profiles->front().name : std::string{};
}

} // namespace

TEST(DocumentParams, RenameUpdatesSketchDimensionExtrudeDistanceAndOtherParameters) {
    Document d;
    mustVoid(d.addParameter("w", "40 mm"));
    mustVoid(d.addParameter("thick", "10 mm"));
    mustVoid(d.addParameter("double_w", "w * 2"));
    const FeatureId sk = d.allocateFeatureId();
    const BodyId body = d.allocateBodyId();
    sketch::Sketch rect = exprRect("w", "20 mm");
    const std::string profile = firstProfile(rect);
    mustVoid(d.addFeature(sketchFeat(sk, rect)));
    ExtrudeParams p;
    p.source = SketchReferenceSource{sk, profile};
    p.distance = "thick";
    p.body = body;
    mustVoid(d.addFeature(std::make_shared<ExtrudeFeature>(d.allocateFeatureId(), p)));
    const double before = volumeOf(evalOk(d).bodies.at(body));
    EXPECT_NEAR(before, 40 * 20 * 10, 1e-6);

    mustVoid(d.renameParameter("w", "width"));
    mustVoid(d.renameParameter("thick", "height"));

    // Parameter expressions, the sketch dimension and the extrude distance were all rewritten.
    EXPECT_EQ(d.parameters().find("double_w")->expression.source(), "width * 2");
    const auto& feats = d.timeline();
    const auto* sf = dynamic_cast<const SketchFeature*>(feats.at(0).get());
    ASSERT_NE(sf, nullptr);
    bool sawWidth = false;
    for (const auto& [id, dim] : sf->params().sketch.dimensions()) sawWidth = sawWidth || dim.expression == "width";
    EXPECT_TRUE(sawWidth);
    const auto* ex = dynamic_cast<const ExtrudeFeature*>(feats.at(1).get());
    ASSERT_NE(ex, nullptr);
    EXPECT_EQ(ex->params().distance, "height");

    // Evaluation is unchanged.
    EXPECT_NEAR(volumeOf(evalOk(d).bodies.at(body)), before, 1e-6);

    // Undo the two renames, then redo them: the timeline follows the table each time.
    mustVoid(d.undo());
    EXPECT_EQ(dynamic_cast<const ExtrudeFeature*>(d.timeline().at(1).get())->params().distance, "thick");
    mustVoid(d.undo());
    EXPECT_EQ(d.parameters().find("double_w")->expression.source(), "w * 2");
    mustVoid(d.redo());
    mustVoid(d.redo());
    EXPECT_EQ(d.parameters().find("double_w")->expression.source(), "width * 2");
    EXPECT_NEAR(volumeOf(evalOk(d).bodies.at(body)), before, 1e-6);
}

TEST(DocumentParams, RenameRejectsTakenNameAndUnknownSourceWithoutChange) {
    Document d;
    mustVoid(d.addParameter("a", "1 mm"));
    mustVoid(d.addParameter("b", "2 mm"));
    EXPECT_FALSE(d.renameParameter("a", "b").has_value());
    EXPECT_FALSE(d.renameParameter("nope", "c").has_value());
    EXPECT_NE(d.parameters().find("a"), nullptr);
    EXPECT_FALSE(d.canUndo() && d.undoText() == "Rename Parameter");
}

TEST(DocumentParams, RemoveRefusedWhileReferencedListsUsers) {
    Document d;
    mustVoid(d.addParameter("w", "40 mm"));
    mustVoid(d.addParameter("dup", "w * 2"));
    mustVoid(d.addParameter("unused", "3 mm"));
    const FeatureId sk = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchFeat(sk, exprRect("w", "20 mm"))));

    auto refused = d.removeParameter("w");
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(refused.error().message.find("dup"), std::string::npos) << refused.error().message;
    EXPECT_NE(refused.error().message.find("Sketch 1"), std::string::npos) << refused.error().message;
    EXPECT_NE(d.parameters().find("w"), nullptr);

    mustVoid(d.removeParameter("unused"));
    EXPECT_EQ(d.parameters().find("unused"), nullptr);
    mustVoid(d.undo());
    EXPECT_NE(d.parameters().find("unused"), nullptr);
}

TEST(DocumentParams, UnitCommentFavouriteAndReplaceAllAreUndoable) {
    Document d;
    mustVoid(d.addParameter("p", "5"));
    mustVoid(d.setParameterUnit("p", "mm"));
    mustVoid(d.setParameterComment("p", "wall"));
    mustVoid(d.setParameterFavourite("p", true));
    EXPECT_EQ(d.parameters().find("p")->unit, "mm");
    EXPECT_EQ(d.parameters().find("p")->comment, "wall");
    EXPECT_TRUE(d.parameters().find("p")->favourite);
    mustVoid(d.undo());
    EXPECT_FALSE(d.parameters().find("p")->favourite);
    mustVoid(d.undo());
    mustVoid(d.undo());
    EXPECT_EQ(d.parameters().find("p")->unit, "");

    ParameterTable table;
    ASSERT_TRUE(table.add("x", "7 mm"));
    mustVoid(d.replaceAllParameters(table));
    EXPECT_EQ(d.parameters().size(), 1u);
    EXPECT_NE(d.parameters().find("x"), nullptr);
    mustVoid(d.undo());
    EXPECT_NE(d.parameters().find("p"), nullptr);
    EXPECT_EQ(d.parameters().find("x"), nullptr);
    mustVoid(d.redo());
    EXPECT_NE(d.parameters().find("x"), nullptr);
}

TEST(DocumentNames, DefaultNamesArePerKindAndRenameIsUndoable) {
    Document d;
    const FeatureId e1 = d.allocateFeatureId();
    const FeatureId e2 = d.allocateFeatureId();
    mustVoid(d.addFeature(box(e1, d.allocateBodyId(), 10, 10)));
    mustVoid(d.addFeature(box(e2, d.allocateBodyId(), 5, 5)));
    mustVoid(d.addFeature(sketchFeat(d.allocateFeatureId(), exprRect("40 mm", "20 mm"))));
    EXPECT_EQ(d.timeline().at(0)->name(), "Extrude 1");
    EXPECT_EQ(d.timeline().at(1)->name(), "Extrude 2");
    EXPECT_EQ(d.timeline().at(2)->name(), "Sketch 1");

    mustVoid(d.renameFeature(e2, "Base"));
    EXPECT_EQ(d.timeline().at(1)->name(), "Base");
    EXPECT_FALSE(d.renameFeature(e2, "   ").has_value());
    mustVoid(d.undo());
    EXPECT_EQ(d.timeline().at(1)->name(), "Extrude 2");
    mustVoid(d.redo());
    EXPECT_EQ(d.timeline().at(1)->name(), "Base");

    // Numbers are never handed out again after a delete.
    mustVoid(d.deleteFeature(e2));
    mustVoid(d.addFeature(box(d.allocateFeatureId(), d.allocateBodyId(), 1, 1)));
    EXPECT_EQ(d.timeline().at(d.timeline().size() - 1)->name(), "Extrude 3");
}

TEST(DocumentNames, NamesRoundTripThroughSaveAndLoad) {
    const auto dir = scratchDir("names");
    {
        Document d;
        mustVoid(d.addFeature(box(d.allocateFeatureId(), d.allocateBodyId(), 10, 10)));
        mustVoid(d.renameFeature(d.timeline().at(0)->id(), "Base block"));
        mustVoid(d.addFeature(box(d.allocateFeatureId(), d.allocateBodyId(), 2, 2)));
        mustVoid(d.save(dir));
    }
    Document loaded;
    mustVoid(loaded.load(dir));
    ASSERT_EQ(loaded.timeline().size(), 2u);
    EXPECT_EQ(loaded.timeline().at(0)->name(), "Base block");
    EXPECT_EQ(loaded.timeline().at(1)->name(), "Extrude 2");
    mustVoid(loaded.addFeature(box(loaded.allocateFeatureId(), loaded.allocateBodyId(), 3, 3)));
    EXPECT_EQ(loaded.timeline().at(2)->name(), "Extrude 3");
    std::filesystem::remove_all(dir);
}

TEST(DocumentConstruction, PlaneOffsetEditMovesSketchOnIt) {
    Document d;
    const FeatureId baseId = d.allocateFeatureId();
    const BodyId b1 = d.allocateBodyId();
    mustVoid(d.addFeature(box(baseId, b1, 40, 20)));
    DocumentEvaluation baseEv = evalOk(d);
    auto top = topFaceAt(baseEv, b1, 10.0);
    ASSERT_TRUE(top.has_value());

    // Offset plane 5 mm above the top face; a circle sketch on it, extruded 2 mm.
    ConstructionPlaneParams cp;
    cp.kind = ConstructionPlaneParams::Kind::Offset;
    cp.base.isFace = true;
    cp.base.body = b1;
    cp.base.face = *top;
    cp.distance = "5 mm";
    const FeatureId planeId = d.allocateFeatureId();
    mustVoid(d.addFeature(std::make_shared<ConstructionPlaneFeature>(planeId, cp)));

    sketch::Sketch disc(*sketch::SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0}));
    must(disc.addCircle({0, 0}, 2.0));
    PlaneRef attach;
    attach.isFace = false;
    attach.construction = planeId;
    const FeatureId discSk = d.allocateFeatureId();
    const BodyId b2 = d.allocateBodyId();
    mustVoid(d.addFeature(std::make_shared<SketchFeature>(discSk, SketchFeatureParams{disc, attach})));
    ExtrudeParams ep;
    ep.source = SketchReferenceSource{discSk, firstProfile(disc)};
    ep.distance = "2 mm";
    ep.body = b2;
    mustVoid(d.addFeature(std::make_shared<ExtrudeFeature>(d.allocateFeatureId(), ep)));

    EXPECT_NEAR(evalOk(d).sketches.at(discSk).sketch.plane().origin.z, 15.0, 1e-6);

    // Edit the offset through the typed command: the sketch on the plane moves with it.
    ASSERT_NE(dynamic_cast<const ConstructionPlaneFeature*>(d.timeline().at(1).get()), nullptr);
    EXPECT_EQ(dynamic_cast<const ConstructionPlaneFeature*>(d.timeline().at(1).get())->params().distance, "5 mm");
    cp.distance = "12 mm";
    mustVoid(d.editConstructionPlane(planeId, cp));
    EXPECT_NEAR(evalOk(d).sketches.at(discSk).sketch.plane().origin.z, 22.0, 1e-6);
    EXPECT_EQ(d.timeline().at(1)->name(), "Plane 1");

    mustVoid(d.undo());
    EXPECT_NEAR(evalOk(d).sketches.at(discSk).sketch.plane().origin.z, 15.0, 1e-6);

    // Wrong kind is refused and changes nothing.
    EXPECT_FALSE(d.editConstructionAxis(planeId, ConstructionAxisParams{}).has_value());
}

TEST(DocumentSketch, SketchForEditAndReplaceSketchAreUndoable) {
    Document d;
    const FeatureId skId = d.allocateFeatureId();
    sketch::Sketch one(*sketch::SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0}));
    must(one.addCircle({0, 0}, 2.0));
    mustVoid(d.addFeature(sketchFeat(skId, one)));
    EXPECT_EQ(evalOk(d).sketches.at(skId).profiles.size(), 1u);

    // Re-entering sketch mode: a copy of the stored definition.
    auto copy = d.sketchForEdit(skId);
    ASSERT_TRUE(copy.has_value());
    EXPECT_EQ(copy->entities().size(), 1u);
    must(copy->addCircle({10, 0}, 1.0));
    EXPECT_EQ(d.sketchForEdit(skId)->entities().size(), 1u) << "the returned copy is independent";
    EXPECT_FALSE(d.sketchForEdit(d.allocateFeatureId()).has_value());

    mustVoid(d.replaceSketch(skId, *copy));
    EXPECT_EQ(evalOk(d).sketches.at(skId).profiles.size(), 2u);
    EXPECT_EQ(d.timeline().at(0)->name(), "Sketch 1");
    mustVoid(d.undo());
    EXPECT_EQ(evalOk(d).sketches.at(skId).profiles.size(), 1u);
    mustVoid(d.redo());
    EXPECT_EQ(evalOk(d).sketches.at(skId).profiles.size(), 2u);
    EXPECT_FALSE(d.replaceSketch(d.allocateFeatureId(), *copy).has_value());
}
