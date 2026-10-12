// SPDX-License-Identifier: GPL-3.0-or-later
// B1c: SketchFeature (parametric sketch solved on the timeline) and by-reference sketch sources of Extrude / Revolve.
// Evaluated through features::Document, which resolves sketch references before evaluation.
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <optional>
#include <set>
#include <string>

#include "features/document.hpp"
#include "features/edge_features.hpp"
#include "features/extrude_feature.hpp"
#include "features/sketch_feature.hpp"
#include "kernel/ops.hpp"
#include "sketch/profile.hpp"
#include "sketch/solver/bridge.hpp"

using namespace iw;
using namespace iw::features;
using naming::ElementName;
using naming::ElementRef;
using naming::ElementType;
using recompute::FeatureStatus;
using sketch::PointRole;
using sketch::SketchEntityId;

namespace {

constexpr double kPi = std::numbers::pi;

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

/// Rectangle sketch on the XY plane (4 coincident, H bottom/top, V left/right) with width and height dimensions given as
/// expressions. Entities: bottom, right, top, left.
struct Rect {
    sketch::Sketch sk{*sketch::SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0})};
    SketchEntityId bottom, right, top, left;
};

Rect makeRect(const std::string& w, const std::string& h, double W = 40, double H = 20) {
    Rect r;
    r.bottom = must(r.sk.addLine({0, 0}, {W, 0}));
    r.right = must(r.sk.addLine({W, 0}, {W, H}));
    r.top = must(r.sk.addLine({W, H}, {0, H}));
    r.left = must(r.sk.addLine({0, H}, {0, 0}));
    must(r.sk.addConstraint(sketch::ConstraintKind::Coincident, {{r.bottom, PointRole::End}, {r.right, PointRole::Start}}));
    must(r.sk.addConstraint(sketch::ConstraintKind::Coincident, {{r.right, PointRole::End}, {r.top, PointRole::Start}}));
    must(r.sk.addConstraint(sketch::ConstraintKind::Coincident, {{r.top, PointRole::End}, {r.left, PointRole::Start}}));
    must(r.sk.addConstraint(sketch::ConstraintKind::Coincident, {{r.left, PointRole::End}, {r.bottom, PointRole::Start}}));
    must(r.sk.addConstraint(sketch::ConstraintKind::Horizontal, {{r.bottom, std::nullopt}}));
    must(r.sk.addConstraint(sketch::ConstraintKind::Horizontal, {{r.top, std::nullopt}}));
    must(r.sk.addConstraint(sketch::ConstraintKind::Vertical, {{r.right, std::nullopt}}));
    must(r.sk.addConstraint(sketch::ConstraintKind::Vertical, {{r.left, std::nullopt}}));
    must(r.sk.addDimension(sketch::DimensionKind::DistanceX, {{r.bottom, PointRole::Start}, {r.bottom, PointRole::End}}, w));
    must(r.sk.addDimension(sketch::DimensionKind::DistanceY, {{r.right, PointRole::Start}, {r.right, PointRole::End}}, h));
    return r;
}

std::string onlyProfile(const sketch::Sketch& sk) {
    auto profiles = sketch::detectProfiles(sk);
    EXPECT_TRUE(profiles.has_value());
    if (!profiles || profiles->size() != 1) {
        ADD_FAILURE() << "expected exactly one profile";
        return {};
    }
    return profiles->front().name;
}

std::shared_ptr<const recompute::Feature> sketchFeature(FeatureId id, sketch::Sketch sk, std::optional<PlaneRef> attach = std::nullopt) {
    return std::make_shared<SketchFeature>(id, SketchFeatureParams{std::move(sk), std::move(attach)});
}

std::shared_ptr<const recompute::Feature> extrudeRef(FeatureId id, FeatureId sketchId, const std::string& profile,
                                                    const std::string& distance, BodyId body,
                                                    ExtrudeParams::Operation op = ExtrudeParams::Operation::NewBody) {
    ExtrudeParams p;
    p.source = SketchReferenceSource{sketchId, profile};
    p.distance = distance;
    p.operation = op;
    p.body = body;
    return std::make_shared<ExtrudeFeature>(id, p);
}

/// Evaluates the document, requiring every status Ok.
DocumentEvaluation evalOk(Document& d) {
    auto ev = d.evaluate();
    EXPECT_TRUE(ev.has_value()) << (ev ? "" : ev.error().message);
    if (!ev) return {};
    for (const auto& s : ev->statuses) EXPECT_EQ(s.state, FeatureStatus::State::Ok) << s.error.message;
    return *ev;
}

const FeatureStatus* statusOf(const DocumentEvaluation& ev, FeatureId id) {
    for (const auto& s : ev.statuses)
        if (s.id == id) return &s;
    return nullptr;
}

std::set<std::string> faceNames(const DocumentEvaluation& ev, BodyId body) {
    std::set<std::string> out;
    const naming::NamingMap& m = ev.namings.at(body);
    for (std::uint32_t i = 0; i < m.size(ElementType::Face); ++i) {
        auto n = m.nameOf(ElementType::Face, i);
        out.insert(n ? n->toString() : std::string("<unnamed>"));
    }
    return out;
}

/// Reference to the element of `body` whose kernel key satisfies `pred`.
std::optional<ElementRef> refWhere(const DocumentEvaluation& ev, BodyId body, ElementType type,
                                   bool (*pred)(const std::array<double, 4>&)) {
    const kernel::Shape& s = ev.bodies.at(body);
    const naming::NamingMap& m = ev.namings.at(body);
    const auto kind = naming::toSubShapeKind(type);
    for (std::uint32_t i = 0; i < s.subShapeCount(kind); ++i) {
        if (!pred(s.subShapeKey(kind, i))) continue;
        auto nm = m.nameOf(type, i);
        if (nm) return ElementRef{nm->feature, type, *nm};
    }
    return std::nullopt;
}

bool near(double a, double b, double tol = 1e-6) { return std::abs(a - b) <= tol; }

/// Vertical edge at (40, 0) of the 40 x 20 x 10 block: centroid (40, 0, 5), length 10.
bool isCornerEdgeAt40x0(const std::array<double, 4>& k) { return near(k[0], 40) && near(k[1], 0) && near(k[3], 10); }

/// Planar face with outward normal +Z lying on the plane z = `z`.
std::optional<ElementRef> topFaceAt(const DocumentEvaluation& ev, BodyId body, double z) {
    const kernel::Shape& s = ev.bodies.at(body);
    const naming::NamingMap& m = ev.namings.at(body);
    for (std::uint32_t i = 0; i < s.subShapeCount(kernel::SubShapeKind::Face); ++i) {
        const kernel::FaceGeometry g = s.faceGeometry(i);
        if (g.kind != kernel::FaceGeometry::Kind::Plane) continue;
        if (!near(g.normal.z, 1) || !near(g.point.z, z)) continue;
        auto nm = m.nameOf(ElementType::Face, i);
        if (nm) return ElementRef{nm->feature, ElementType::Face, *nm};
    }
    return std::nullopt;
}

double zMax(const kernel::Shape& s) { return s.boundingBox().max.z; }

} // namespace

TEST(SketchFeature, RectangleSketchExtrudedByReferenceGivesVolume) {
    Document d;
    mustVoid(d.addParameter("w", "40 mm"));
    mustVoid(d.addParameter("h", "20 mm"));
    const Rect r = makeRect("w", "h");
    const FeatureId sk = d.allocateFeatureId();
    const BodyId body = d.allocateBodyId();
    mustVoid(d.addFeature(sketchFeature(sk, r.sk)));
    mustVoid(d.addFeature(extrudeRef(d.allocateFeatureId(), sk, onlyProfile(r.sk), "10 mm", body)));

    DocumentEvaluation ev = evalOk(d);
    ASSERT_TRUE(ev.bodies.count(body));
    EXPECT_NEAR(volumeOf(ev.bodies.at(body)), 8000.0, 1e-6);
    ASSERT_TRUE(ev.sketches.count(sk));
    EXPECT_EQ(ev.sketches.at(sk).profiles.size(), 1u);
}

TEST(SketchFeature, EditingWidthChangesVolumeAndKeepsSideFaceNames) {
    Document d;
    mustVoid(d.addParameter("w", "40 mm"));
    mustVoid(d.addParameter("h", "20 mm"));
    const Rect r = makeRect("w", "h");
    const FeatureId sk = d.allocateFeatureId();
    const BodyId body = d.allocateBodyId();
    mustVoid(d.addFeature(sketchFeature(sk, r.sk)));
    mustVoid(d.addFeature(extrudeRef(d.allocateFeatureId(), sk, onlyProfile(r.sk), "10 mm", body)));
    const std::set<std::string> before = faceNames(evalOk(d), body);

    mustVoid(d.setParameterExpression("w", "50 mm"));
    DocumentEvaluation after = evalOk(d);
    EXPECT_NEAR(volumeOf(after.bodies.at(body)), 10000.0, 1e-6);
    EXPECT_EQ(faceNames(after, body), before);
}

TEST(SketchFeature, FilletOnExtrudeEdgeSurvivesWidthEdit) {
    Document d;
    mustVoid(d.addParameter("w", "40 mm"));
    mustVoid(d.addParameter("h", "20 mm"));
    const Rect r = makeRect("w", "h");
    const FeatureId sk = d.allocateFeatureId();
    const BodyId body = d.allocateBodyId();
    mustVoid(d.addFeature(sketchFeature(sk, r.sk)));
    mustVoid(d.addFeature(extrudeRef(d.allocateFeatureId(), sk, onlyProfile(r.sk), "10 mm", body)));
    DocumentEvaluation base = evalOk(d);
    auto edge = refWhere(base, body, ElementType::Edge, isCornerEdgeAt40x0);
    ASSERT_TRUE(edge.has_value());

    FilletParams fp;
    fp.body = body;
    fp.edges = {*edge};
    fp.radius = "1 mm";
    fp.tangentChain = false;
    const FeatureId fid = d.allocateFeatureId();
    mustVoid(d.addFeature(std::make_shared<FilletFeature>(fid, fp)));
    DocumentEvaluation filleted = evalOk(d);
    EXPECT_NEAR(volumeOf(filleted.bodies.at(body)), 8000.0 - (1.0 - kPi / 4.0) * 10.0, 1e-6);

    mustVoid(d.setParameterExpression("w", "50 mm"));
    DocumentEvaluation edited = evalOk(d);
    const FeatureStatus* fs = statusOf(edited, fid);
    ASSERT_NE(fs, nullptr);
    EXPECT_EQ(fs->state, FeatureStatus::State::Ok);
    EXPECT_TRUE(fs->diagnostics.empty() || fs->diagnostics.front().severity != recompute::Diagnostic::Severity::Error);
    EXPECT_NEAR(volumeOf(edited.bodies.at(body)), 10000.0 - (1.0 - kPi / 4.0) * 10.0, 1e-6);
}

TEST(SketchFeature, OverConstrainedSketchFailsListingConflictingIds) {
    Document d;
    mustVoid(d.addParameter("w", "40 mm"));
    mustVoid(d.addParameter("h", "20 mm"));
    Rect r = makeRect("w", "h");
    // A second width on the top edge that contradicts the bottom width.
    must(r.sk.addDimension(sketch::DimensionKind::DistanceX, {{r.top, PointRole::Start}, {r.top, PointRole::End}}, "30 mm"));
    ParameterTable table;
    ASSERT_TRUE(table.add("w", "40 mm").has_value());
    ASSERT_TRUE(table.add("h", "20 mm").has_value());
    const sketch::SketchSolveReport rep = sketch::diagnoseSketch(r.sk, &table);
    ASSERT_FALSE(rep.conflictingDimensions.empty() && rep.conflictingConstraints.empty());

    const FeatureId sk = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchFeature(sk, r.sk)));
    auto ev = d.evaluate();
    ASSERT_TRUE(ev.has_value());
    const FeatureStatus* st = statusOf(*ev, sk);
    ASSERT_NE(st, nullptr);
    EXPECT_EQ(st->state, FeatureStatus::State::Failed);
    EXPECT_NE(st->error.message.find("over-constrained"), std::string::npos) << st->error.message;
    for (auto c : rep.conflictingConstraints)
        EXPECT_NE(st->error.message.find(std::to_string(c.value())), std::string::npos) << st->error.message;
    for (auto dim : rep.conflictingDimensions)
        EXPECT_NE(st->error.message.find(std::to_string(dim.value())), std::string::npos) << st->error.message;
}

TEST(SketchFeature, DependentExtrudeFailsWhenSketchFailed) {
    Document d;
    Rect r = makeRect("40 mm", "20 mm");
    must(r.sk.addDimension(sketch::DimensionKind::DistanceX, {{r.top, PointRole::Start}, {r.top, PointRole::End}}, "30 mm"));
    const FeatureId sk = d.allocateFeatureId();
    const FeatureId ex = d.allocateFeatureId();
    mustVoid(d.addFeature(sketchFeature(sk, r.sk)));
    mustVoid(d.addFeature(extrudeRef(ex, sk, "e1.0", "10 mm", d.allocateBodyId())));
    auto ev = d.evaluate();
    ASSERT_TRUE(ev.has_value());
    EXPECT_EQ(statusOf(*ev, sk)->state, FeatureStatus::State::Failed);
    EXPECT_EQ(statusOf(*ev, ex)->state, FeatureStatus::State::Failed);
    EXPECT_NE(statusOf(*ev, ex)->error.message.find("no solved profile"), std::string::npos);
}

TEST(SketchFeature, SketchAttachedToFaceMovesWhenBodyDimensionChanges) {
    Document d;
    mustVoid(d.addParameter("h", "10 mm"));
    // Base block 20 x 20, height h (sketch rectangle extruded by reference).
    Rect base = makeRect("20 mm", "20 mm");
    const FeatureId baseSk = d.allocateFeatureId();
    const BodyId b1 = d.allocateBodyId();
    mustVoid(d.addFeature(sketchFeature(baseSk, base.sk)));
    mustVoid(d.addFeature(extrudeRef(d.allocateFeatureId(), baseSk, onlyProfile(base.sk), "h", b1)));
    DocumentEvaluation baseEv = evalOk(d);
    auto top = topFaceAt(baseEv, b1, 10.0);
    ASSERT_TRUE(top.has_value());

    // Circle sketch attached to the top face of the block.
    sketch::Sketch disc(*sketch::SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0}));
    must(disc.addCircle({0, 0}, 2.0));
    PlaneRef attach;
    attach.isFace = true;
    attach.body = b1;
    attach.face = *top;
    const FeatureId discSk = d.allocateFeatureId();
    const BodyId b2 = d.allocateBodyId();
    mustVoid(d.addFeature(sketchFeature(discSk, disc, attach)));
    mustVoid(d.addFeature(extrudeRef(d.allocateFeatureId(), discSk, onlyProfile(disc), "5 mm", b2)));

    DocumentEvaluation ev = evalOk(d);
    EXPECT_NEAR(ev.sketches.at(discSk).sketch.plane().origin.z, 10.0, 1e-6);
    EXPECT_NEAR(zMax(ev.bodies.at(b2)), 15.0, 1e-6);

    mustVoid(d.setParameterExpression("h", "15 mm"));
    DocumentEvaluation moved = evalOk(d);
    EXPECT_NEAR(moved.sketches.at(discSk).sketch.plane().origin.z, 15.0, 1e-6);
    EXPECT_NEAR(zMax(moved.bodies.at(b2)), 20.0, 1e-6);
}

TEST(SketchFeature, ParametersRoundTripThroughJson) {
    const Rect r = makeRect("w", "h");
    SketchFeatureParams p{r.sk, std::nullopt};
    SketchFeature f(FeatureId{7}, p);
    auto again = SketchFeature::fromJson(FeatureId{7}, f.paramsJson());
    ASSERT_TRUE(again.has_value()) << again.error().message;
    EXPECT_EQ((*again)->paramsJson(), f.paramsJson());
    EXPECT_EQ((*again)->parameterReferences(), (std::vector<std::string>{"h", "w"}));
}
