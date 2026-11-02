// SPDX-License-Identifier: GPL-3.0-or-later
// F26: Hole placed from sketch points (Fusion: Hole -> From Sketch, multiple holes). Evaluated through features::Document,
// which embeds the solved sketch into the hole before evaluation.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <string>

#include "features/document.hpp"
#include "features/extrude_feature.hpp"
#include "features/hole_feature.hpp"
#include "features/sketch_feature.hpp"
#include "kernel/ops.hpp"
#include "sketch/sketch.hpp"

using namespace iw;
using namespace iw::features;
using recompute::Diagnostic;
using recompute::FeatureStatus;
using sketch::SketchEntityId;

namespace {

constexpr double kPi = std::numbers::pi;

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

/// Material of `s` inside the 2 x 2 column around (x, y) between z0 and z1 (a 4 mm hole there removes all of it).
double materialAt(const kernel::Shape& s, double x, double y, double z0, double z1) {
    auto probe = kernel::makeBox({x - 1, y - 1, z0}, 2, 2, z1 - z0);
    EXPECT_TRUE(probe.has_value());
    if (!probe) return -1;
    auto c = kernel::common(s, probe->shape);
    if (!c) return 0.0; // empty common
    auto m = kernel::massProperties(c->shape);
    return m ? m->volume : 0.0;
}

/// Block 40 x 20 x 10 from the origin (polygon extrude), body `body`.
std::shared_ptr<const recompute::Feature> block(FeatureId id, BodyId body) {
    ExtrudeParams p;
    PolygonSource src;
    src.points = {{0, 0}, {40, 0}, {40, 20}, {0, 20}};
    src.vector = {0, 0, 10};
    p.source = src;
    p.body = body;
    return std::make_shared<ExtrudeFeature>(id, p);
}

sketch::Sketch sketchAtZ(double z) { return sketch::Sketch(*sketch::SketchPlane::make({0, 0, z}, {0, 0, 1}, {1, 0, 0})); }

sketch::Sketch editable(const Document& d, FeatureId id) {
    auto r = d.sketchForEdit(id);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message);
    return r ? std::move(*r) : sketchAtZ(0);
}

std::shared_ptr<const recompute::Feature> sketchFeature(FeatureId id, sketch::Sketch sk) {
    return std::make_shared<SketchFeature>(id, SketchFeatureParams{std::move(sk), std::nullopt});
}

std::shared_ptr<HoleFeature> sketchHole(FeatureId id, BodyId body, FeatureId sk, std::vector<SketchEntityId> pts, bool through,
                                        std::string depth = "10 mm", bool flip = false) {
    HoleParams p;
    p.body = body;
    p.diameter = "4 mm";
    p.through = through;
    p.depth = std::move(depth);
    p.drillAngle = "180 deg"; // flat bottom: blind volumes are plain cylinders
    p.sketchPoints = HoleSketchPlacement{sk, std::move(pts), flip, std::nullopt};
    return std::make_shared<HoleFeature>(id, p);
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

struct Setup {
    Document d;
    BodyId body;
    FeatureId sk, hole;
};

constexpr double kBlock = 40.0 * 20.0 * 10.0;
const double kThroughHole = kPi * 4.0 * 10.0; // r = 2, h = 10

} // namespace

TEST(HoleFromSketch, MultipleHolesFromPointsConstructionPointsAndCircleCentres) {
    Document d;
    const BodyId body = d.allocateBodyId();
    mustVoid(d.addFeature(block(d.allocateFeatureId(), body)));
    sketch::Sketch sk = sketchAtZ(10);
    const SketchEntityId p1 = must(sk.addPoint({8, 10}));
    const SketchEntityId p2 = must(sk.addPoint({20, 10}, /*construction=*/true));
    const SketchEntityId c = must(sk.addCircle({32, 10}, 3.0));
    const FeatureId skId = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchFeature(skId, sk)));
    const FeatureId holeId = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchHole(holeId, body, skId, {p1, p2, c}, true)));

    DocumentEvaluation ev = evalOk(d);
    ASSERT_TRUE(ev.bodies.count(body));
    const kernel::Shape& s = ev.bodies.at(body);
    EXPECT_NEAR(volumeOf(s), kBlock - 3.0 * kThroughHole, 1e-6 * kBlock);
    for (double x : {8.0, 20.0, 32.0}) EXPECT_NEAR(materialAt(s, x, 10, 0, 10), 0.0, 1e-6) << x;
    EXPECT_NEAR(materialAt(s, 14, 10, 0, 10), 40.0, 1e-6);
    EXPECT_EQ(statusOf(ev, holeId)->attributes.at("holes"), "3");
    // Every element of the body is named; the hole faces carry the sketch point id in their op.
    const naming::NamingMap& m = ev.namings.at(body);
    int perPoint[3] = {0, 0, 0};
    const SketchEntityId ids[3] = {p1, p2, c};
    for (std::uint32_t i = 0; i < m.size(naming::ElementType::Face); ++i) {
        auto nm = m.nameOf(naming::ElementType::Face, i);
        ASSERT_TRUE(nm.has_value());
        for (int k = 0; k < 3; ++k)
            if (nm->op.find(".e" + std::to_string(ids[k].value()) + ".") != std::string::npos) ++perPoint[k];
    }
    for (int k = 0; k < 3; ++k) EXPECT_GE(perPoint[k], 1) << k;
}

TEST(HoleFromSketch, EditingTheSketchMovesTheHoles) {
    Document d;
    const BodyId body = d.allocateBodyId();
    mustVoid(d.addFeature(block(d.allocateFeatureId(), body)));
    sketch::Sketch sk = sketchAtZ(10);
    const SketchEntityId p1 = must(sk.addPoint({10, 10}));
    const SketchEntityId p2 = must(sk.addPoint({30, 10}));
    const FeatureId skId = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchFeature(skId, sk)));
    mustVoid(d.addFeature(sketchHole(d.allocateFeatureId(), body, skId, {p1, p2}, true)));
    DocumentEvaluation before = evalOk(d);
    EXPECT_NEAR(materialAt(before.bodies.at(body), 10, 10, 0, 10), 0.0, 1e-6);
    const std::size_t facesBefore = before.namings.at(body).size(naming::ElementType::Face);

    sketch::Sketch edited = editable(d, skId);
    mustVoid(edited.setGeometry(p1, sketch::PointGeom{{10, 5}}));
    mustVoid(d.replaceSketch(skId, std::move(edited)));
    DocumentEvaluation after = evalOk(d);
    const kernel::Shape& s = after.bodies.at(body);
    EXPECT_NEAR(volumeOf(s), kBlock - 2.0 * kThroughHole, 1e-6 * kBlock);
    EXPECT_NEAR(materialAt(s, 10, 5, 0, 10), 0.0, 1e-6);   // moved hole
    EXPECT_NEAR(materialAt(s, 10, 13, 0, 10), 40.0, 1e-6); // old location (y 12..14) is solid again
    EXPECT_NEAR(materialAt(s, 30, 10, 0, 10), 0.0, 1e-6);  // untouched hole
    EXPECT_EQ(after.namings.at(body).size(naming::ElementType::Face), facesBefore);

    // Undo restores the original position.
    mustVoid(d.undo());
    DocumentEvaluation undone = evalOk(d);
    EXPECT_NEAR(materialAt(undone.bodies.at(body), 10, 10, 0, 10), 0.0, 1e-6);
}

TEST(HoleFromSketch, DeletedPointIsSkippedWithWarningAndAllDeletedFails) {
    Document d;
    const BodyId body = d.allocateBodyId();
    mustVoid(d.addFeature(block(d.allocateFeatureId(), body)));
    sketch::Sketch sk = sketchAtZ(10);
    const SketchEntityId p1 = must(sk.addPoint({10, 10}));
    const SketchEntityId p2 = must(sk.addPoint({30, 10}));
    const FeatureId skId = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchFeature(skId, sk)));
    const FeatureId holeId = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchHole(holeId, body, skId, {p1, p2}, true)));
    evalOk(d);

    sketch::Sketch edited = editable(d, skId);
    mustVoid(edited.removeEntity(p1));
    mustVoid(d.replaceSketch(skId, edited));
    DocumentEvaluation ev = evalOk(d);
    EXPECT_NEAR(volumeOf(ev.bodies.at(body)), kBlock - kThroughHole, 1e-6 * kBlock);
    const FeatureStatus* st = statusOf(ev, holeId);
    ASSERT_NE(st, nullptr);
    bool warned = false;
    for (const Diagnostic& diag : st->diagnostics)
        if (diag.severity == Diagnostic::Severity::Warning &&
            diag.message.find("e" + std::to_string(p1.value()) + " no longer exists") != std::string::npos)
            warned = true;
    EXPECT_TRUE(warned);
    EXPECT_EQ(st->attributes.at("holes"), "1");

    mustVoid(edited.removeEntity(p2));
    mustVoid(d.replaceSketch(skId, edited));
    auto ev2 = d.evaluate();
    ASSERT_TRUE(ev2.has_value());
    const FeatureStatus* st2 = statusOf(*ev2, holeId);
    ASSERT_NE(st2, nullptr);
    EXPECT_EQ(st2->state, FeatureStatus::State::Failed);
    EXPECT_EQ(st2->error.code, ErrorCode::NotFound);
    EXPECT_NE(st2->error.message.find("none of the selected sketch points"), std::string::npos) << st2->error.message;
}

TEST(HoleFromSketch, PointOffThePartWarnsThatTheHoleMissesTheBody) {
    Document d;
    const BodyId body = d.allocateBodyId();
    mustVoid(d.addFeature(block(d.allocateFeatureId(), body)));
    sketch::Sketch sk = sketchAtZ(10);
    const SketchEntityId on = must(sk.addPoint({10, 10}));
    const SketchEntityId off = must(sk.addPoint({100, 10})); // outside the 40 x 20 block
    const FeatureId skId = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchFeature(skId, sk)));
    const FeatureId holeId = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchHole(holeId, body, skId, {on, off}, true)));
    DocumentEvaluation ev = evalOk(d);
    EXPECT_NEAR(volumeOf(ev.bodies.at(body)), kBlock - kThroughHole, 1e-6 * kBlock);
    const FeatureStatus* st = statusOf(ev, holeId);
    ASSERT_NE(st, nullptr);
    int missWarnings = 0;
    for (const Diagnostic& diag : st->diagnostics)
        if (diag.severity == Diagnostic::Severity::Warning && diag.message.find("does not intersect the body") != std::string::npos) {
            ++missWarnings;
            EXPECT_NE(diag.message.find("e" + std::to_string(off.value())), std::string::npos) << diag.message;
        }
    EXPECT_EQ(missWarnings, 1);
}

TEST(HoleFromSketch, DirectionFollowsTheBodySideOfTheSketchPlaneAndFlip) {
    // Sketch on z = 0 (normal +z) under the block: the body is on the +n side, so the hole drills upwards.
    {
        Document d;
        const BodyId body = d.allocateBodyId();
        mustVoid(d.addFeature(block(d.allocateFeatureId(), body)));
        sketch::Sketch sk = sketchAtZ(0);
        const SketchEntityId p = must(sk.addPoint({10, 10}));
        const FeatureId skId = d.allocateFeatureId();
        mustVoid(d.addFeature(sketchFeature(skId, sk)));
        mustVoid(d.addFeature(sketchHole(d.allocateFeatureId(), body, skId, {p}, false, "4 mm")));
        DocumentEvaluation ev = evalOk(d);
        const kernel::Shape& s = ev.bodies.at(body);
        EXPECT_NEAR(volumeOf(s), kBlock - kPi * 4.0 * 4.0, 1e-6 * kBlock);
        EXPECT_NEAR(materialAt(s, 10, 10, 0, 4), 0.0, 1e-6);
        EXPECT_NEAR(materialAt(s, 10, 10, 6, 10), 16.0, 1e-6);
    }
    // Sketch plane through the middle of the block: drills along -n (down); flip drills up. The kernel tool also clears
    // max(1 mm, diameter) = 4 mm on the entry side (it assumes the entry lies on a surface), so with the plane inside the
    // body the removed column is depth + 4 mm: down z in [2, 5] plus clearance [5, 9]; flip [5, 8] plus clearance [1, 5].
    for (bool flip : {false, true}) {
        Document d;
        const BodyId body = d.allocateBodyId();
        mustVoid(d.addFeature(block(d.allocateFeatureId(), body)));
        sketch::Sketch sk = sketchAtZ(5);
        const SketchEntityId p = must(sk.addPoint({10, 10}));
        const FeatureId skId = d.allocateFeatureId();
        mustVoid(d.addFeature(sketchFeature(skId, sk)));
        mustVoid(d.addFeature(sketchHole(d.allocateFeatureId(), body, skId, {p}, false, "3 mm", flip)));
        DocumentEvaluation ev = evalOk(d);
        const kernel::Shape& s = ev.bodies.at(body);
        EXPECT_NEAR(volumeOf(s), kBlock - kPi * 4.0 * (3.0 + 4.0), 1e-6 * kBlock) << flip;
        const double lo = materialAt(s, 10, 10, 1.2, 1.8), hi = materialAt(s, 10, 10, 8.2, 8.8);
        EXPECT_NEAR(flip ? lo : hi, 0.0, 1e-6) << flip;
        EXPECT_NEAR(flip ? hi : lo, 2.4, 1e-6) << flip;
    }
}

TEST(HoleFromSketch, LineEntityIsRejectedAndMissingSketchFails) {
    Document d;
    const BodyId body = d.allocateBodyId();
    mustVoid(d.addFeature(block(d.allocateFeatureId(), body)));
    sketch::Sketch sk = sketchAtZ(10);
    const SketchEntityId line = must(sk.addLine({5, 5}, {15, 5}));
    const FeatureId skId = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchFeature(skId, sk)));
    const FeatureId holeId = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchHole(holeId, body, skId, {line}, true)));
    auto ev = d.evaluate();
    ASSERT_TRUE(ev.has_value());
    EXPECT_EQ(statusOf(*ev, holeId)->state, FeatureStatus::State::Failed);
    EXPECT_EQ(statusOf(*ev, holeId)->error.code, ErrorCode::InvalidArgument);

    // Evaluated without the Document's sketch resolution (no embedded sketch): NotFound.
    recompute::Timeline tl;
    tl.append(block(FeatureId{1}, body));
    tl.append(sketchHole(FeatureId{2}, body, FeatureId{99}, {SketchEntityId{1}}, true));
    recompute::Evaluator evaluator;
    auto r = evaluator.evaluate(tl);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->statuses[1].state, FeatureStatus::State::Failed);
    EXPECT_EQ(r->statuses[1].error.code, ErrorCode::NotFound);
}

TEST(HoleFromSketch, JsonRoundTripAndFacePlacementLayoutUnchanged) {
    auto h = sketchHole(FeatureId{7}, BodyId{3}, FeatureId{5}, {SketchEntityId{11}, SketchEntityId{4}}, false, "6 mm", true);
    const std::string text = h->paramsJson();
    auto back = HoleFeature::fromJson(FeatureId{7}, text);
    ASSERT_TRUE(back.has_value()) << back.error().message;
    const auto* hb = dynamic_cast<const HoleFeature*>(back->get());
    ASSERT_NE(hb, nullptr);
    ASSERT_TRUE(hb->params().sketchPoints.has_value());
    EXPECT_EQ(hb->params().sketchPoints->sketch, FeatureId{5});
    ASSERT_EQ(hb->params().sketchPoints->points.size(), 2u);
    EXPECT_EQ(hb->params().sketchPoints->points[0], SketchEntityId{11});
    EXPECT_EQ(hb->params().sketchPoints->points[1], SketchEntityId{4});
    EXPECT_TRUE(hb->params().sketchPoints->flip);
    EXPECT_EQ(hb->paramsJson(), text);
    EXPECT_EQ(hb->paramHash(), h->paramHash());
    EXPECT_TRUE(hb->explicitRefs().empty());

    // Through the registry (the path the document loader takes).
    recompute::FeatureRegistry reg;
    registerHoleFeature(reg);
    auto viaReg = reg.create("hole", FeatureId{7}, text);
    ASSERT_TRUE(viaReg.has_value()) << viaReg.error().message;
    EXPECT_EQ((*viaReg)->paramsJson(), text);

    // Face placement keeps its layout: no "placement" key, the face reference is present.
    HoleParams fp;
    fp.body = BodyId{3};
    fp.face = naming::ElementRef{FeatureId{1}, naming::ElementType::Face, naming::ElementName{}};
    const std::string faceText = HoleFeature(FeatureId{8}, fp).paramsJson();
    EXPECT_EQ(faceText.find("placement"), std::string::npos);
    EXPECT_NE(faceText.find("\"face\""), std::string::npos);

    // Malformed sketch placements.
    EXPECT_FALSE(HoleFeature::fromJson(FeatureId{7}, R"({"body":1,"placement":"sketch_points","sketch_feature":5,"points":[]})"));
    EXPECT_FALSE(HoleFeature::fromJson(FeatureId{7}, R"({"body":1,"placement":"sketch_points","points":[1]})"));
    EXPECT_FALSE(HoleFeature::fromJson(FeatureId{7}, R"({"body":1,"placement":"somewhere"})"));
}
