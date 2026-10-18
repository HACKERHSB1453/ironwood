// SPDX-License-Identifier: GPL-3.0-or-later
// B1a: ExtrudeFeature from sketch profiles (one side / two sides / symmetric; NewBody, Join, Cut, Intersect),
// expression-driven parameters with stamp invalidation, naming stability across sketch dimension edits, the P0
// polygon format, the registry and io round trips.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "core/params.hpp"
#include "features/extrude_feature.hpp"
#include "features/sketch_json.hpp"
#include "io/document_io.hpp"
#include "kernel/ops.hpp"
#include "recompute/evaluator.hpp"
#include "sketch/profile.hpp"
#include "sketch/sketch.hpp"

using namespace iw;
using namespace iw::features;
using namespace iw::recompute;
using sketch::Point2;
using sketch::SketchEntityId;

namespace {

constexpr BodyId kB1{1}, kB2{2};

/// Sketch on the XY plane with a W x H rectangle made of four lines e1 (bottom), e2 (right), e3 (top), e4 (left).
struct RectSketch {
    sketch::Sketch sk;
    SketchEntityId e1, e2, e3, e4;
};

RectSketch makeRect(double w, double h, Vec3 origin = {}) {
    auto plane = sketch::SketchPlane::make(origin, {0, 0, 1}, {1, 0, 0});
    EXPECT_TRUE(plane.has_value());
    RectSketch r{sketch::Sketch(*plane), {}, {}, {}, {}};
    r.e1 = *r.sk.addLine({0, 0}, {w, 0});
    r.e2 = *r.sk.addLine({w, 0}, {w, h});
    r.e3 = *r.sk.addLine({w, h}, {0, h});
    r.e4 = *r.sk.addLine({0, h}, {0, 0});
    return r;
}

/// Moves the rectangle's width to W, keeping every entity id.
void setWidth(RectSketch& r, double w, double h) {
    ASSERT_TRUE(r.sk.setGeometry(r.e1, sketch::LineGeom{{0, 0}, {w, 0}}).has_value());
    ASSERT_TRUE(r.sk.setGeometry(r.e2, sketch::LineGeom{{w, 0}, {w, h}}).has_value());
    ASSERT_TRUE(r.sk.setGeometry(r.e3, sketch::LineGeom{{w, h}, {0, h}}).has_value());
}

/// The single profile name of a sketch with exactly one region.
std::string onlyProfile(const sketch::Sketch& sk) {
    auto profiles = sketch::detectProfiles(sk);
    EXPECT_TRUE(profiles.has_value());
    if (!profiles) return {};
    EXPECT_EQ(profiles->size(), 1u);
    return profiles->empty() ? std::string{} : profiles->front().name;
}

ExtrudeParams sketchParams(const sketch::Sketch& sk, const std::string& profile, std::string distance,
                           ExtrudeParams::Operation op = ExtrudeParams::Operation::NewBody, BodyId body = kB1,
                           ExtrudeParams::Extent extent = ExtrudeParams::Extent::OneSide, std::string distance2 = "0 mm") {
    ExtrudeParams p;
    p.source = SketchSource{sk, profile};
    p.extent = extent;
    p.distance = std::move(distance);
    p.distance2 = std::move(distance2);
    p.operation = op;
    p.body = body;
    return p;
}

/// Polygon (P0) extrude of an axis-aligned box: origin + (x0,y0), w x h footprint, dz along +Z.
ExtrudeParams boxParams(double x0, double y0, double w, double h, double dz, BodyId body,
                        ExtrudeParams::Operation op = ExtrudeParams::Operation::NewBody, Vec3 origin = {}) {
    ExtrudeParams p;
    PolygonSource src;
    src.planeOrigin = origin;
    src.points = {{x0, y0}, {x0 + w, y0}, {x0 + w, y0 + h}, {x0, y0 + h}};
    src.vector = {0, 0, dz};
    p.source = src;
    p.operation = op;
    p.body = body;
    return p;
}

std::shared_ptr<Feature> feat(std::uint64_t id, ExtrudeParams p) {
    return std::make_shared<ExtrudeFeature>(FeatureId{id}, std::move(p));
}

double volumeOf(const kernel::Shape& s) {
    auto m = kernel::massProperties(s);
    EXPECT_TRUE(m.has_value());
    return m ? m->volume : -1;
}

/// Bounding box z-range of a shape.
std::pair<double, double> zRange(const kernel::Shape& s) {
    const Box3 bb = s.boundingBox();
    return {bb.min.z, bb.max.z};
}

std::set<std::string> faceNames(const naming::NamingMap& m) {
    std::set<std::string> out;
    for (std::uint32_t i = 0; i < m.size(naming::ElementType::Face); ++i) {
        auto n = m.nameOf(naming::ElementType::Face, i);
        out.insert(n ? n->toString() : std::string("<unnamed>"));
    }
    return out;
}

/// Area of the face with the given name (0 if absent).
double areaOfFace(const kernel::Shape& s, const naming::NamingMap& m, const std::string& name) {
    for (std::uint32_t i = 0; i < m.size(naming::ElementType::Face); ++i) {
        auto n = m.nameOf(naming::ElementType::Face, i);
        if (n && n->toString() == name) return s.subShapeKey(kernel::SubShapeKind::Face, i)[3];
    }
    return 0;
}

std::string faceNameOfSource(const naming::NamingMap& m, const std::string& sourceOp) {
    for (std::uint32_t i = 0; i < m.size(naming::ElementType::Face); ++i) {
        auto n = m.nameOf(naming::ElementType::Face, i);
        if (n && n->op == "extrude.side" && n->sources.size() == 1 && n->sources[0].op == sourceOp) return n->toString();
    }
    return {};
}

} // namespace

// ---- expressions and parameters -------------------------------------------------------------------------------

TEST(FeaturesParams, EditingOneParameterRecomputesOnlyItsUsersAndDownstream) {
    // Timeline: [F1 unrelated polygon on B2] [F2 sketch extrude using d1 on B1]. Editing d1 must reuse F1.
    ParameterTable table;
    ASSERT_TRUE(table.add("d1", "10 mm"));
    ASSERT_TRUE(table.add("d2", "3 mm"));
    const RectSketch r = makeRect(10, 10);
    Timeline t;
    t.append(feat(1, boxParams(0, 0, 4, 4, 4, kB2)));
    t.append(feat(2, sketchParams(r.sk, onlyProfile(r.sk), "d1")));

    Evaluator e;
    auto first = e.evaluate(t, &table);
    ASSERT_TRUE(first.has_value()) << first.error().message;
    EXPECT_EQ(first->computedCount, 2u);
    EXPECT_NEAR(volumeOf(first->bodies.at(kB1)), 1000.0, 1e-6);

    // Unchanged table: everything is served from the cache.
    auto same = e.evaluate(t, &table);
    ASSERT_TRUE(same.has_value());
    EXPECT_EQ(same->computedCount, 0u);
    EXPECT_EQ(same->reusedCount, 2u);

    // Editing an unrelated parameter invalidates nothing.
    ParameterTable other = table;
    ASSERT_TRUE(other.setExpression("d2", "7 mm"));
    auto unrelated = e.evaluate(t, &other);
    ASSERT_TRUE(unrelated.has_value());
    EXPECT_EQ(unrelated->computedCount, 0u);

    // Editing d1: F1 (before the user) is reused; F2 (the user) is recomputed with the new value.
    ParameterTable edited = table;
    ASSERT_TRUE(edited.setExpression("d1", "20 mm"));
    auto after = e.evaluate(t, &edited);
    ASSERT_TRUE(after.has_value()) << after.error().message;
    EXPECT_EQ(after->statuses[0].reused, true);
    EXPECT_EQ(after->statuses[1].reused, false);
    EXPECT_EQ(after->computedCount, 1u);
    EXPECT_NEAR(volumeOf(after->bodies.at(kB1)), 2000.0, 1e-6);
}

TEST(FeaturesParams, FeatureAfterTheUserIsRecomputedAsDownstream) {
    // Timeline: [F1 sketch extrude using d1] [F2 unrelated polygon]. Editing d1 recomputes F1 and, because the stamp
    // chain is a prefix chain, also F2 (its downstream). Unrelated features before the user are never touched.
    ParameterTable table;
    ASSERT_TRUE(table.add("d1", "10 mm"));
    const RectSketch r = makeRect(10, 10);
    Timeline t;
    t.append(feat(1, sketchParams(r.sk, onlyProfile(r.sk), "d1")));
    t.append(feat(2, boxParams(0, 0, 4, 4, 4, kB2)));
    Evaluator e;
    ASSERT_TRUE(e.evaluate(t, &table).has_value());
    ParameterTable edited = table;
    ASSERT_TRUE(edited.setExpression("d1", "15 mm"));
    auto after = e.evaluate(t, &edited);
    ASSERT_TRUE(after.has_value());
    EXPECT_EQ(after->computedCount, 2u);
    EXPECT_NEAR(volumeOf(after->bodies.at(kB1)), 1500.0, 1e-6);
}

TEST(FeaturesParams, ReferencesAndExpressionValuesAreEvaluatedAtComputeTime) {
    const RectSketch r = makeRect(10, 10);
    ExtrudeParams p = sketchParams(r.sk, onlyProfile(r.sk), "2 in + 5 mm");
    ExtrudeFeature f(FeatureId{1}, p);
    EXPECT_EQ(f.parameterReferences(), std::vector<std::string>{});

    ExtrudeParams q = sketchParams(r.sk, onlyProfile(r.sk), "d1 * 2");
    ExtrudeFeature g(FeatureId{1}, q);
    EXPECT_EQ(g.parameterReferences(), std::vector<std::string>{"d1"});

    Evaluator e;
    Timeline t;
    t.append(std::make_shared<ExtrudeFeature>(FeatureId{1}, p));
    auto r1 = e.evaluate(t); // no table: literal expression still evaluates
    ASSERT_TRUE(r1.has_value());
    EXPECT_EQ(r1->statuses[0].state, FeatureStatus::State::Ok);
    EXPECT_NEAR(volumeOf(r1->bodies.at(kB1)), 100.0 * 55.8, 1e-6); // 2 in + 5 mm = 55.8 mm
}

TEST(FeaturesParams, ExpressionErrorsFailTheFeatureCleanly) {
    const RectSketch r = makeRect(10, 10);
    const std::string prof = onlyProfile(r.sk);
    ParameterTable table;
    ASSERT_TRUE(table.add("d1", "10 mm"));

    auto run = [&](ExtrudeParams p, ErrorCode expect) {
        Timeline t;
        t.append(feat(1, std::move(p)));
        auto res = Evaluator{}.evaluate(t, &table);
        ASSERT_TRUE(res.has_value());
        EXPECT_EQ(res->statuses[0].state, FeatureStatus::State::Failed);
        EXPECT_EQ(res->statuses[0].error.code, expect) << res->statuses[0].error.message;
    };
    run(sketchParams(r.sk, prof, "missing_param"), ErrorCode::NotFound);
    run(sketchParams(r.sk, prof, "10 deg"), ErrorCode::InvalidArgument);      // angle where a length is needed
    run(sketchParams(r.sk, prof, "10"), ErrorCode::InvalidArgument);          // unitless where a length is needed
    run(sketchParams(r.sk, "e9.9", "d1"), ErrorCode::NotFound);               // no such profile
    ExtrudeParams tapered = sketchParams(r.sk, prof, "d1");
    tapered.taper = "5 deg";
    run(tapered, ErrorCode::NotImplemented);
    ExtrudeParams nonzeroTwo = sketchParams(r.sk, prof, "d1", ExtrudeParams::Operation::NewBody, kB1,
                                            ExtrudeParams::Extent::TwoSides, "-d1");
    run(nonzeroTwo, ErrorCode::InvalidArgument); // d + d2 = 0 gives a zero extrusion vector
}

TEST(FeaturesParams, MalformedExpressionIsRejectedAtLoad) {
    FeatureRegistry reg;
    registerFeatures(reg);
    const RectSketch r = makeRect(10, 10);
    ExtrudeFeature f(FeatureId{1}, sketchParams(r.sk, onlyProfile(r.sk), "d1 +"));
    auto back = reg.create("extrude", FeatureId{1}, f.paramsJson());
    EXPECT_EQ(back.error().code, ErrorCode::ParseError);
}

// ---- sketch profiles ------------------------------------------------------------------------------------------

TEST(FeaturesSketch, OneSideExtrudeVolumeAndExtent) {
    const RectSketch r = makeRect(10, 10);
    Timeline t;
    t.append(feat(1, sketchParams(r.sk, onlyProfile(r.sk), "12 mm")));
    auto res = Evaluator{}.evaluate(t);
    ASSERT_TRUE(res.has_value());
    ASSERT_EQ(res->statuses[0].state, FeatureStatus::State::Ok) << res->statuses[0].error.message;
    const auto& body = res->bodies.at(kB1);
    EXPECT_NEAR(volumeOf(body), 1200.0, 1e-6);
    const auto [z0, z1] = zRange(body);
    EXPECT_NEAR(z0, 0.0, 1e-7);
    EXPECT_NEAR(z1, 12.0, 1e-7);
}

TEST(FeaturesSketch, TwoSidesAndSymmetricExtents) {
    const RectSketch r = makeRect(10, 10);
    const std::string prof = onlyProfile(r.sk);
    Timeline two;
    two.append(feat(1, sketchParams(r.sk, prof, "6 mm", ExtrudeParams::Operation::NewBody, kB1,
                                    ExtrudeParams::Extent::TwoSides, "4 mm")));
    auto a = Evaluator{}.evaluate(two);
    ASSERT_TRUE(a.has_value());
    ASSERT_EQ(a->statuses[0].state, FeatureStatus::State::Ok) << a->statuses[0].error.message;
    EXPECT_NEAR(volumeOf(a->bodies.at(kB1)), 1000.0, 1e-6);
    EXPECT_NEAR(zRange(a->bodies.at(kB1)).first, -4.0, 1e-7);
    EXPECT_NEAR(zRange(a->bodies.at(kB1)).second, 6.0, 1e-7);

    Timeline sym;
    sym.append(feat(1, sketchParams(r.sk, prof, "10 mm", ExtrudeParams::Operation::NewBody, kB1,
                                    ExtrudeParams::Extent::Symmetric)));
    auto b = Evaluator{}.evaluate(sym);
    ASSERT_TRUE(b.has_value());
    ASSERT_EQ(b->statuses[0].state, FeatureStatus::State::Ok) << b->statuses[0].error.message;
    EXPECT_NEAR(volumeOf(b->bodies.at(kB1)), 1000.0, 1e-6);
    EXPECT_NEAR(zRange(b->bodies.at(kB1)).first, -5.0, 1e-7);
    EXPECT_NEAR(zRange(b->bodies.at(kB1)).second, 5.0, 1e-7);
}

TEST(FeaturesSketch, NegativeDistanceExtrudesBackwards) {
    const RectSketch r = makeRect(10, 10);
    Timeline t;
    t.append(feat(1, sketchParams(r.sk, onlyProfile(r.sk), "-5 mm")));
    auto res = Evaluator{}.evaluate(t);
    ASSERT_TRUE(res.has_value());
    ASSERT_EQ(res->statuses[0].state, FeatureStatus::State::Ok);
    EXPECT_NEAR(zRange(res->bodies.at(kB1)).second, 0.0, 1e-7);
    EXPECT_NEAR(zRange(res->bodies.at(kB1)).first, -5.0, 1e-7);
}

TEST(FeaturesSketch, JoinCutAndIntersectOntoExistingBody) {
    // Base: 10 x 10 x 10 box on B1.
    const RectSketch base = makeRect(10, 10);
    auto baseRun = [&](Timeline& t) { t.append(feat(1, sketchParams(base.sk, onlyProfile(base.sk), "10 mm"))); };

    // Join: 10 x 10 slab 5 mm above the box.
    {
        const RectSketch top = makeRect(10, 10, {0, 0, 10});
        Timeline t;
        baseRun(t);
        t.append(feat(2, sketchParams(top.sk, onlyProfile(top.sk), "5 mm", ExtrudeParams::Operation::Join)));
        auto res = Evaluator{}.evaluate(t);
        ASSERT_TRUE(res.has_value());
        ASSERT_EQ(res->statuses[1].state, FeatureStatus::State::Ok) << res->statuses[1].error.message;
        EXPECT_NEAR(volumeOf(res->bodies.at(kB1)), 1500.0, 1e-5);
    }
    // Cut: a radius-2 circle through the box (through-hole of volume pi r^2 h).
    {
        auto plane = sketch::SketchPlane::make({0, 0, 10}, {0, 0, 1}, {1, 0, 0});
        ASSERT_TRUE(plane.has_value());
        sketch::Sketch circ(*plane);
        ASSERT_TRUE(circ.addCircle({5, 5}, 2.0).has_value());
        Timeline t;
        baseRun(t);
        t.append(feat(2, sketchParams(circ, onlyProfile(circ), "-10 mm", ExtrudeParams::Operation::Cut)));
        auto res = Evaluator{}.evaluate(t);
        ASSERT_TRUE(res.has_value());
        ASSERT_EQ(res->statuses[1].state, FeatureStatus::State::Ok) << res->statuses[1].error.message;
        EXPECT_NEAR(volumeOf(res->bodies.at(kB1)), 1000.0 - 3.14159265358979323846 * 4.0 * 10.0, 1e-3);
    }
    // Intersect: a 5 x 5 column (extruded 20 mm through the box) keeps 5 x 5 x 10 = 250 mm^3.
    {
        const RectSketch col = makeRect(5, 5);
        Timeline t;
        baseRun(t);
        t.append(feat(2, sketchParams(col.sk, onlyProfile(col.sk), "20 mm", ExtrudeParams::Operation::Intersect)));
        auto res = Evaluator{}.evaluate(t);
        ASSERT_TRUE(res.has_value());
        ASSERT_EQ(res->statuses[1].state, FeatureStatus::State::Ok) << res->statuses[1].error.message;
        EXPECT_NEAR(volumeOf(res->bodies.at(kB1)), 250.0, 1e-5);
    }
}

TEST(FeaturesSketch, HoleProfileGivesRingVolume) {
    // Rectangle 10x10 with a centred circle of radius 2 as a second loop: the profile is a ring-with-hole region.
    auto plane = sketch::SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0});
    ASSERT_TRUE(plane.has_value());
    sketch::Sketch sk(*plane);
    ASSERT_TRUE(sk.addLine({0, 0}, {10, 0}).has_value());
    ASSERT_TRUE(sk.addLine({10, 0}, {10, 10}).has_value());
    ASSERT_TRUE(sk.addLine({10, 10}, {0, 10}).has_value());
    ASSERT_TRUE(sk.addLine({0, 10}, {0, 0}).has_value());
    ASSERT_TRUE(sk.addCircle({5, 5}, 2.0).has_value());
    auto profiles = sketch::detectProfiles(sk);
    ASSERT_TRUE(profiles.has_value());
    std::string ringName;
    for (const auto& pr : *profiles)
        if (!pr.holes.empty()) ringName = pr.name;
    ASSERT_FALSE(ringName.empty());
    Timeline t;
    t.append(feat(1, sketchParams(sk, ringName, "4 mm")));
    auto res = Evaluator{}.evaluate(t);
    ASSERT_TRUE(res.has_value());
    ASSERT_EQ(res->statuses[0].state, FeatureStatus::State::Ok) << res->statuses[0].error.message;
    EXPECT_NEAR(volumeOf(res->bodies.at(kB1)), (100.0 - 3.14159265358979323846 * 4.0) * 4.0, 1e-3);
}

// ---- naming ---------------------------------------------------------------------------------------------------

TEST(FeaturesNaming, SideFaceNamesSurviveADimensionEditThatKeepsEntityIds) {
    RectSketch r = makeRect(10, 10);
    const std::string prof = onlyProfile(r.sk);
    auto evalNamed = [&](const sketch::Sketch& sk) {
        Timeline t;
        t.append(feat(7, sketchParams(sk, prof, "5 mm")));
        auto res = Evaluator{}.evaluate(t);
        EXPECT_TRUE(res.has_value());
        return *res;
    };
    const EvalResult before = evalNamed(r.sk);
    const naming::NamingMap& mb = before.namings.at(kB1);
    const std::set<std::string> namesBefore = faceNames(mb);
    const double bottomBefore = areaOfFace(before.bodies.at(kB1), mb, faceNameOfSource(mb, "sketch.e1.0"));
    EXPECT_NEAR(bottomBefore, 10.0 * 5.0, 1e-6);

    // Widen the rectangle to 20: same entity ids, new geometry.
    setWidth(r, 20, 10);
    const EvalResult after = evalNamed(r.sk);
    const naming::NamingMap& ma = after.namings.at(kB1);
    EXPECT_EQ(faceNames(ma), namesBefore) << "a dimension edit must not rename faces";
    const double bottomAfter = areaOfFace(after.bodies.at(kB1), ma, faceNameOfSource(ma, "sketch.e1.0"));
    EXPECT_NEAR(bottomAfter, 20.0 * 5.0, 1e-6); // same name, new geometry
    // The right side (from e2, x = W) keeps its size: H x h = 10 x 5.
    EXPECT_NEAR(areaOfFace(after.bodies.at(kB1), ma, faceNameOfSource(ma, "sketch.e2.0")), 10.0 * 5.0, 1e-6);
}

TEST(FeaturesNaming, ProfileFaceAndVertexNamesAreFromSketchEntities) {
    const RectSketch r = makeRect(10, 10);
    Timeline t;
    t.append(feat(3, sketchParams(r.sk, onlyProfile(r.sk), "5 mm")));
    auto res = Evaluator{}.evaluate(t);
    ASSERT_TRUE(res.has_value());
    const naming::NamingMap& m = res->namings.at(kB1);
    // Start cap: the profile face itself (modified, keeps "sketch.region"). End cap: generated from the profile face.
    // Sides: one per profile edge, named from the sketch piece they came from.
    std::size_t startCaps = 0, endCaps = 0, sides = 0;
    for (std::uint32_t i = 0; i < m.size(naming::ElementType::Face); ++i) {
        auto n = m.nameOf(naming::ElementType::Face, i);
        ASSERT_TRUE(n.has_value());
        if (n->op == "sketch.region") {
            ++startCaps;
        } else if (n->op == "extrude.cap") {
            ++endCaps;
            ASSERT_EQ(n->sources.size(), 1u);
            EXPECT_EQ(n->sources[0].op, "sketch.region");
        } else if (n->op == "extrude.side") {
            ++sides;
        } else {
            ADD_FAILURE() << "unexpected face name " << n->toString();
        }
    }
    EXPECT_EQ(startCaps, 1u);
    EXPECT_EQ(endCaps, 1u);
    EXPECT_EQ(sides, 4u);
    // Every side is named by one of the four rectangle lines.
    for (const char* line : {"sketch.e1.0", "sketch.e2.0", "sketch.e3.0", "sketch.e4.0"})
        EXPECT_FALSE(faceNameOfSource(m, line).empty()) << line;
    // No element fell back to a positional name.
    for (std::uint32_t i = 0; i < m.size(naming::ElementType::Vertex); ++i) {
        auto n = m.nameOf(naming::ElementType::Vertex, i);
        ASSERT_TRUE(n.has_value());
        EXPECT_NE(n->op, "sketch.unmatched");
    }
}

TEST(FeaturesNaming, SketchNamesSurviveSerialisationWithTheSameIds) {
    const RectSketch r = makeRect(10, 10);
    ExtrudeFeature f(FeatureId{4}, sketchParams(r.sk, onlyProfile(r.sk), "5 mm"));
    FeatureRegistry reg;
    registerFeatures(reg);
    auto g = reg.create("extrude", FeatureId{4}, f.paramsJson());
    ASSERT_TRUE(g.has_value()) << g.error().message;
    EXPECT_EQ((*g)->paramsJson(), f.paramsJson());
    EXPECT_EQ((*g)->paramHash(), f.paramHash());

    Timeline a, b;
    a.append(std::make_shared<ExtrudeFeature>(FeatureId{4}, f.params()));
    b.append(*g);
    auto ra = Evaluator{}.evaluate(a);
    auto rb = Evaluator{}.evaluate(b);
    ASSERT_TRUE(ra && rb);
    EXPECT_EQ(faceNames(ra->namings.at(kB1)), faceNames(rb->namings.at(kB1)));
}

TEST(FeaturesSketch, SerialisationKeepsIdsAndNextIdAcrossRemovals) {
    // Entities 1..4 are added, the first is removed, and entity 5 added: a reload must keep 2,3,4,5 and never reuse 1.
    auto plane = sketch::SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0});
    ASSERT_TRUE(plane.has_value());
    sketch::Sketch sk(*plane);
    auto a = sk.addPoint({0, 0});
    auto b = sk.addLine({0, 0}, {5, 0});
    auto c = sk.addLine({5, 0}, {5, 5});
    auto d = sk.addCircle({1, 1}, 0.5, true);
    ASSERT_TRUE(a && b && c && d);
    ASSERT_TRUE(sk.removeEntity(*a).has_value());
    auto j = features::sketchToJson(sk);
    ASSERT_TRUE(j.has_value());
    auto loaded = features::sketchFromJson(*j);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    EXPECT_EQ(loaded->entities().size(), 3u);
    EXPECT_TRUE(loaded->findEntity(*b) != nullptr);
    EXPECT_TRUE(loaded->findEntity(*c) != nullptr);
    EXPECT_TRUE(loaded->findEntity(*d) != nullptr);
    EXPECT_TRUE(loaded->entities().at(*d).construction);
    // The next allocated id continues after the original counter, never re-using the removed id.
    sketch::Sketch copy = *loaded;
    auto next = copy.addPoint({9, 9});
    auto orig = sk;
    auto origNext = orig.addPoint({9, 9});
    ASSERT_TRUE(next && origNext);
    EXPECT_EQ(next->value(), origNext->value());
    EXPECT_GT(next->value(), a->value());
}

// ---- P0 compatibility, registry and io ------------------------------------------------------------------------

TEST(FeaturesCompat, P0ManifestLoadsAndRebuilds) {
    // A document as written by P0 (no "source" key; numeric extrusion vector; "join" operation).
    const std::string p0Manifest = R"({
  "format": "ironwood-document",
  "version": 0,
  "name": "p0",
  "displayUnit": "mm",
  "nextId": 4,
  "timeline": {
    "rollbackMarker": null,
    "suppressed": [],
    "features": [
      { "id": 1, "kind": "extrude", "params": {
          "origin": [0, 0, 0], "normal": [0, 0, 1], "xdir": [1, 0, 0],
          "points": [[0, 0], [10, 0], [10, 10], [0, 10]], "vector": [0, 0, 10],
          "operation": "new_body", "body": 1 } },
      { "id": 2, "kind": "extrude", "params": {
          "origin": [100, 0, 0], "normal": [0, 0, 1], "xdir": [1, 0, 0],
          "points": [[0, 0], [4, 0], [4, 4], [0, 4]], "vector": [0, 0, 4],
          "operation": "new_body", "body": 2 } },
      { "id": 3, "kind": "extrude", "params": {
          "origin": [0, 0, 0], "normal": [0, 0, 1], "xdir": [1, 0, 0],
          "points": [[5, 0], [15, 0], [15, 10], [5, 10]], "vector": [0, 0, 10],
          "operation": "join", "body": 1 } }
    ]
  }
})";
    const auto dir = std::filesystem::temp_directory_path() / ("iw_p0_manifest_" + std::to_string(std::random_device{}()) + ".iwd");
    std::filesystem::create_directories(dir);
    {
        std::ofstream out(dir / "manifest.json");
        out << p0Manifest;
    }
    FeatureRegistry reg;
    registerFeatures(reg);
    auto doc = io::loadDocument(dir, reg);
    std::filesystem::remove_all(dir);
    ASSERT_TRUE(doc.has_value()) << doc.error().message;
    ASSERT_EQ(doc->timeline.size(), 3u);
    auto res = Evaluator{}.evaluate(doc->timeline);
    ASSERT_TRUE(res.has_value());
    for (const auto& s : res->statuses) EXPECT_EQ(s.state, FeatureStatus::State::Ok) << s.error.message;
    // Body 1: box 10x10x10 joined with a 10x10x10 box shifted by 5 in x -> overlap 5x10x10 -> union 1500.
    EXPECT_NEAR(volumeOf(res->bodies.at(kB1)), 1500.0, 1e-5);
    // The re-serialised P0 polygon form keeps the P0 key set (no "source").
    EXPECT_EQ(doc->timeline.at(0)->paramsJson().find("\"source\""), std::string::npos);
}

TEST(FeaturesCompat, SketchExtrudeRoundTripsThroughIo) {
    const RectSketch r = makeRect(10, 10);
    io::Document doc;
    doc.name = "sketch-extrude";
    doc.nextId = 10;
    ASSERT_TRUE(doc.parameters.add("d1", "2 in + 5 mm"));
    doc.timeline.append(feat(1, boxParams(0, 0, 4, 4, 4, kB2)));
    doc.timeline.append(feat(2, sketchParams(r.sk, onlyProfile(r.sk), "d1", ExtrudeParams::Operation::NewBody, kB1,
                                             ExtrudeParams::Extent::TwoSides, "1 mm")));
    doc.timeline.append(feat(3, sketchParams(r.sk, onlyProfile(r.sk), "d1", ExtrudeParams::Operation::NewBody, kB1,
                                             ExtrudeParams::Extent::OneSide)));

    FeatureRegistry reg;
    registerFeatures(reg);
    const auto dir = std::filesystem::temp_directory_path() / ("iw_features_rt_" + std::to_string(std::random_device{}()) + ".iwd");
    std::filesystem::remove_all(dir);
    ASSERT_TRUE(io::saveDocument(dir, doc).has_value());
    auto loaded = io::loadDocument(dir, reg);
    std::filesystem::remove_all(dir);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    ASSERT_EQ(loaded->timeline.size(), 3u);
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(loaded->timeline.at(i)->paramsJson(), doc.timeline.at(i)->paramsJson());
        EXPECT_EQ(loaded->timeline.at(i)->paramHash(), doc.timeline.at(i)->paramHash());
    }
    Evaluator e1, e2;
    auto a = e1.evaluate(doc.timeline, &doc.parameters);
    auto b = e2.evaluate(loaded->timeline, &loaded->parameters);
    ASSERT_TRUE(a && b);
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(a->statuses[i].state, b->statuses[i].state);
        EXPECT_EQ(a->statuses[i].state, FeatureStatus::State::Ok) << a->statuses[i].error.message;
    }
    EXPECT_NEAR(volumeOf(a->bodies.at(kB1)), volumeOf(b->bodies.at(kB1)), 1e-9);
    // Feature 3 is a NewBody onto B1 again, so it replaces the two-sided body: 10 x 10 x (2 in + 5 mm) = 5580.
    EXPECT_NEAR(volumeOf(b->bodies.at(kB1)), 100.0 * 55.8, 1e-3);
}

// ---- B1b: revolve, fillet, chamfer, hole, move, construction features ---------------------------------------------
#include "features/construction_features.hpp"
#include "features/edge_features.hpp"
#include "features/hole_feature.hpp"
#include "features/move_feature.hpp"
#include "features/revolve_feature.hpp"
#include <numbers>
#include <cstdio>
#include <functional>

namespace {

using common::BodyOp;
using naming::ElementName;
using naming::ElementRef;
using naming::ElementType;
using recompute::ConstructionGeometry;

constexpr double kPi = std::numbers::pi;

std::string mm(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.12f mm", v);
    return buf;
}

/// Evaluates a timeline and requires every feature to be Ok.
EvalResult evalAllOk(const Timeline& tl, const ParameterTable* params = nullptr) {
    Evaluator ev;
    auto r = ev.evaluate(tl, params);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message);
    if (!r) return {};
    for (const auto& s : r->statuses) EXPECT_EQ(s.state, FeatureStatus::State::Ok) << s.error.message;
    return *r;
}

/// Name of the element of `type` whose kernel key satisfies `pred`, as a reference owned by `owner`.
std::optional<ElementRef> refWhere(const EvalResult& r, BodyId body, ElementType type,
                                   const std::function<bool(const std::array<double, 4>&)>& pred) {
    const kernel::Shape& s = r.bodies.at(body);
    const naming::NamingMap& m = r.namings.at(body);
    const auto kind = naming::toSubShapeKind(type);
    for (std::uint32_t i = 0; i < s.subShapeCount(kind); ++i) {
        if (!pred(s.subShapeKey(kind, i))) continue;
        auto nm = m.nameOf(type, i);
        if (!nm) continue;
        return ElementRef{nm->feature, type, *nm};
    }
    return std::nullopt;
}

bool near(double a, double b, double tol = 1e-6) { return std::abs(a - b) <= tol; }

/// 10 x 10 x 10 box from the origin (NewBody onto kB1), feature id 1.
std::shared_ptr<Feature> box10(std::uint64_t id = 1, BodyId body = kB1) { return feat(id, boxParams(0, 0, 10, 10, 10, body)); }

/// Vertical box edge at (10,10): centroid (10,10,5), length 10.
bool isCornerEdge(const std::array<double, 4>& k) { return near(k[0], 10) && near(k[1], 10) && near(k[3], 10); }

/// Planar face of the box with outward normal `n` and location on the plane z = `z` (box faces have z = 0 or 10).
bool isBoxFace(const kernel::Shape& s, std::uint32_t i, Vec3 n, double z) {
    const kernel::FaceGeometry g = s.faceGeometry(i);
    return g.kind == kernel::FaceGeometry::Kind::Plane && near(g.normal.x, n.x) && near(g.normal.y, n.y) && near(g.normal.z, n.z) &&
           near(g.point.z, z);
}

std::optional<ElementRef> boxFace(const EvalResult& r, BodyId body, Vec3 n, double z) {
    const kernel::Shape& s = r.bodies.at(body);
    const naming::NamingMap& m = r.namings.at(body);
    for (std::uint32_t i = 0; i < s.subShapeCount(kernel::SubShapeKind::Face); ++i) {
        if (!isBoxFace(s, i, n, z)) continue;
        auto nm = m.nameOf(ElementType::Face, i);
        if (nm) return ElementRef{nm->feature, ElementType::Face, *nm};
    }
    return std::nullopt;
}

/// Plane-coordinate offsets (u, v) of world point `p` on the planar face `ref` (for hole placement).
std::pair<double, double> faceUV(const kernel::Shape& s, std::uint32_t faceIdx, Vec3 p) {
    auto pl = kernel::planeFromFace(s, faceIdx);
    EXPECT_TRUE(pl.has_value());
    const Vec3 yd = cross(pl->normal, pl->xDir);
    const Vec3 d = p - pl->origin;
    return {dot(d, pl->xDir), dot(d, yd)};
}

std::uint32_t faceIndexOf(const EvalResult& r, BodyId body, Vec3 n, double z) {
    const kernel::Shape& s = r.bodies.at(body);
    for (std::uint32_t i = 0; i < s.subShapeCount(kernel::SubShapeKind::Face); ++i)
        if (isBoxFace(s, i, n, z)) return i;
    ADD_FAILURE() << "no box face found";
    return 0;
}

std::shared_ptr<Feature> axisTwoPoints(std::uint64_t id, Vec3 a, Vec3 b) {
    ConstructionAxisParams p;
    p.kind = ConstructionAxisParams::Kind::TwoPoints;
    p.a = {mm(a.x), mm(a.y), mm(a.z)};
    p.b = {mm(b.x), mm(b.y), mm(b.z)};
    return std::make_shared<ConstructionAxisFeature>(FeatureId{id}, p);
}

/// Rectangle x in [x0, x1], y in [0, h] in the XY plane, revolved (profile) about a construction axis feature.
std::shared_ptr<Feature> revolveRect(std::uint64_t id, double x0, double x1, double h, std::uint64_t axisFeature,
                                     RevolveParams::Extent ext, std::string angle, std::string angle2 = "0 deg",
                                     BodyId body = kB1, BodyOp op = BodyOp::NewBody) {
    RevolveParams p;
    PolygonSource src;
    src.planeNormal = {0, 0, 1};
    src.planeXDir = {1, 0, 0};
    src.points = {{x0, 0}, {x1, 0}, {x1, h}, {x0, h}};
    p.source = src;
    p.axis.kind = RevolveAxis::Kind::Construction;
    p.axis.construction = FeatureId{axisFeature};
    p.extent = ext;
    p.angle = std::move(angle);
    p.angle2 = std::move(angle2);
    p.body = body;
    p.operation = op;
    return std::make_shared<RevolveFeature>(FeatureId{id}, p);
}

std::shared_ptr<Feature> fillet(std::uint64_t id, ElementRef edge, std::string radius, bool chain = false, BodyId body = kB1) {
    FilletParams p;
    p.body = body;
    p.edges = {edge};
    p.radius = std::move(radius);
    p.tangentChain = chain;
    return std::make_shared<FilletFeature>(FeatureId{id}, p);
}

std::shared_ptr<Feature> chamfer(std::uint64_t id, ElementRef edge, ChamferParams::Mode mode, std::string d1, std::string d2 = "1 mm",
                                 std::string angle = "45 deg", BodyId body = kB1) {
    ChamferParams p;
    p.body = body;
    p.edges = {edge};
    p.mode = mode;
    p.distance = std::move(d1);
    p.distance2 = std::move(d2);
    p.angle = std::move(angle);
    p.tangentChain = false;
    return std::make_shared<ChamferFeature>(FeatureId{id}, p);
}

ElementRef cornerEdgeOfBox(const EvalResult& r, BodyId body = kB1) {
    auto ref = refWhere(r, body, ElementType::Edge, isCornerEdge);
    EXPECT_TRUE(ref.has_value());
    return ref.value_or(ElementRef{});
}

} // namespace

TEST(FeaturesRevolve, FullRevolveOfRectangleIsATube) {
    Timeline tl;
    tl.append(axisTwoPoints(1, {0, 0, 0}, {0, 1, 0}));
    tl.append(revolveRect(2, 2, 5, 10, 1, RevolveParams::Extent::Full, "360 deg"));
    EvalResult r = evalAllOk(tl);
    ASSERT_TRUE(r.bodies.count(kB1));
    // Rectangle [2,5] x [0,10] about the Y axis: pi * (5^2 - 2^2) * 10.
    EXPECT_NEAR(volumeOf(r.bodies.at(kB1)), 210.0 * kPi, 1e-6 * 210.0 * kPi);
}

TEST(FeaturesRevolve, OneSideTwoSidesAndSymmetricAngles) {
    struct Case { RevolveParams::Extent ext; std::string a, a2; double sweepDeg; };
    const Case cases[] = {
        {RevolveParams::Extent::OneSide, "180 deg", "0 deg", 180},
        {RevolveParams::Extent::Symmetric, "90 deg", "0 deg", 90},
        {RevolveParams::Extent::TwoSides, "60 deg", "30 deg", 90},
    };
    for (const Case& c : cases) {
        Timeline tl;
        tl.append(axisTwoPoints(1, {0, 0, 0}, {0, 1, 0}));
        tl.append(revolveRect(2, 2, 5, 10, 1, c.ext, c.a, c.a2));
        EvalResult r = evalAllOk(tl);
        EXPECT_NEAR(volumeOf(r.bodies.at(kB1)), 210.0 * kPi * c.sweepDeg / 360.0, 1e-6 * 210.0 * kPi) << c.a << " " << c.a2;
    }
}

TEST(FeaturesRevolve, SketchLineAxisRevolvesARectangleIntoACylinder) {
    // Rectangle of width 3 and height 10; its left side (entity e4) is the revolve axis: cylinder r = 3, h = 10.
    const RectSketch r = makeRect(3, 10);
    RevolveParams p;
    p.source = SketchSource{r.sk, onlyProfile(r.sk)};
    p.axis.kind = RevolveAxis::Kind::SketchLine;
    p.axis.line = r.e4;
    p.extent = RevolveParams::Extent::Full;
    p.body = kB1;
    Timeline tl;
    tl.append(std::make_shared<RevolveFeature>(FeatureId{1}, p));
    EvalResult res = evalAllOk(tl);
    EXPECT_NEAR(volumeOf(res.bodies.at(kB1)), 90.0 * kPi, 1e-6 * 90.0 * kPi);
}

TEST(FeaturesRevolve, ExpressionAngleFromParameterTable) {
    ParameterTable params;
    ASSERT_TRUE(params.add("sweep", "180 deg"));
    Timeline tl;
    tl.append(axisTwoPoints(1, {0, 0, 0}, {0, 1, 0}));
    tl.append(revolveRect(2, 2, 5, 10, 1, RevolveParams::Extent::OneSide, "sweep"));
    EvalResult r = evalAllOk(tl, &params);
    EXPECT_NEAR(volumeOf(r.bodies.at(kB1)), 105.0 * kPi, 1e-6 * 105.0 * kPi);
}

TEST(FeaturesFillet, ConstantFilletRemovesQuarterCylinder) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    tl.append(fillet(2, cornerEdgeOfBox(base), "1 mm"));
    EvalResult r = evalAllOk(tl);
    // Removed: (1 - pi/4) * r^2 * 10 with r = 1.
    EXPECT_NEAR(volumeOf(r.bodies.at(kB1)), 1000.0 - (1.0 - kPi / 4.0) * 10.0, 1e-6 * 1000.0);
}

TEST(FeaturesFillet, RadiusFromParameterRecomputesOnEdit) {
    ParameterTable params;
    ASSERT_TRUE(params.add("r", "1 mm"));
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl, &params);
    tl.append(fillet(2, cornerEdgeOfBox(base), "r"));
    EvalResult a = evalAllOk(tl, &params);
    EXPECT_NEAR(volumeOf(a.bodies.at(kB1)), 1000.0 - (1.0 - kPi / 4.0) * 10.0, 1e-6 * 1000.0);
    ASSERT_TRUE(params.setExpression("r", "2 mm"));
    EvalResult b = evalAllOk(tl, &params);
    EXPECT_NEAR(volumeOf(b.bodies.at(kB1)), 1000.0 - (1.0 - kPi / 4.0) * 4.0 * 10.0, 1e-6 * 1000.0);
}

TEST(FeaturesFillet, UnresolvableEdgeReferenceFailsTheFeatureWithAMessage) {
    Timeline tl;
    tl.append(box10());
    ElementRef bogus{FeatureId{1}, ElementType::Edge, ElementName{FeatureId{1}, "extrude.edge", ElementType::Edge, 999, {}}};
    tl.append(fillet(2, bogus, "1 mm"));
    Evaluator ev;
    auto r = ev.evaluate(tl);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->statuses[0].state, FeatureStatus::State::Ok);
    EXPECT_EQ(r->statuses[1].state, FeatureStatus::State::Failed);
    EXPECT_NE(r->statuses[1].error.message.find("could not be resolved"), std::string::npos) << r->statuses[1].error.message;
}

TEST(FeaturesFillet, NegativeRadiusIsAFailedFeatureNotACrash) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    tl.append(fillet(2, cornerEdgeOfBox(base), "-1 mm"));
    Evaluator ev;
    auto r = ev.evaluate(tl);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->statuses[1].state, FeatureStatus::State::Failed);
}

TEST(FeaturesChamfer, EqualTwoDistancesAndDistanceAngle) {
    struct Case { ChamferParams::Mode mode; std::string d1, d2, angle; double removed; };
    const Case cases[] = {
        {ChamferParams::Mode::Equal, "2 mm", "1 mm", "45 deg", 0.5 * 2 * 2 * 10},
        {ChamferParams::Mode::TwoDistances, "1 mm", "3 mm", "45 deg", 0.5 * 1 * 3 * 10},
        {ChamferParams::Mode::DistanceAngle, "2 mm", "1 mm", "45 deg", 0.5 * 2 * 2 * 10},
    };
    for (const Case& c : cases) {
        Timeline tl;
        tl.append(box10());
        EvalResult base = evalAllOk(tl);
        tl.append(chamfer(2, cornerEdgeOfBox(base), c.mode, c.d1, c.d2, c.angle));
        EvalResult r = evalAllOk(tl);
        EXPECT_NEAR(volumeOf(r.bodies.at(kB1)), 1000.0 - c.removed, 1e-6 * 1000.0) << static_cast<int>(c.mode);
    }
}

TEST(FeaturesChamfer, TangentChainOfABoxEdgeIsJustThatEdge) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    const ElementRef e = cornerEdgeOfBox(base);
    // Box corners are 90 degrees: no tangent continuation, so the chained feature equals the single-edge fillet.
    FilletParams p;
    p.body = kB1;
    p.edges = {e};
    p.radius = "1 mm";
    p.tangentChain = true;
    tl.append(std::make_shared<FilletFeature>(FeatureId{2}, p));
    EvalResult r = evalAllOk(tl);
    EXPECT_NEAR(volumeOf(r.bodies.at(kB1)), 1000.0 - (1.0 - kPi / 4.0) * 10.0, 1e-6 * 1000.0);
}

TEST(FeaturesHole, ThroughSimpleHoleRemovesCylinderAndCarriesThreadAttribute) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    const std::uint32_t top = faceIndexOf(base, kB1, {0, 0, 1}, 10);
    HoleParams p;
    p.body = kB1;
    p.face = *boxFace(base, kB1, {0, 0, 1}, 10);
    auto [u, v] = faceUV(base.bodies.at(kB1), top, {5, 5, 10});
    p.u = mm(u);
    p.v = mm(v);
    p.diameter = "4 mm";
    p.through = true;
    p.thread = "M4x0.7";
    tl.append(std::make_shared<HoleFeature>(FeatureId{2}, p));
    EvalResult r = evalAllOk(tl);
    EXPECT_NEAR(volumeOf(r.bodies.at(kB1)), 1000.0 - kPi * 4.0 * 10.0, 1e-6 * 1000.0);
    EXPECT_EQ(r.statuses[1].attributes.at("thread"), "M4x0.7");
    // The thread is cosmetic: geometry is identical to the same hole without it.
    p.thread.clear();
    Timeline tl2;
    tl2.append(box10());
    tl2.append(std::make_shared<HoleFeature>(FeatureId{2}, p));
    EvalResult r2 = evalAllOk(tl2);
    EXPECT_NEAR(volumeOf(r2.bodies.at(kB1)), volumeOf(r.bodies.at(kB1)), 1e-9);
    EXPECT_EQ(r2.statuses[1].attributes.count("thread"), 0u);
}

TEST(FeaturesHole, CounterboreRemovesBothSteps) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    const std::uint32_t top = faceIndexOf(base, kB1, {0, 0, 1}, 10);
    HoleParams p;
    p.body = kB1;
    p.face = *boxFace(base, kB1, {0, 0, 1}, 10);
    auto [u, v] = faceUV(base.bodies.at(kB1), top, {5, 5, 10});
    p.u = mm(u);
    p.v = mm(v);
    p.kind = kernel::HoleKind::Counterbore;
    p.diameter = "4 mm";
    p.through = true;
    p.counterboreDiameter = "6 mm";
    p.counterboreDepth = "2 mm";
    tl.append(std::make_shared<HoleFeature>(FeatureId{2}, p));
    EvalResult r = evalAllOk(tl);
    // Counterbore: pi * 3^2 * 2; through hole below it: pi * 2^2 * 8.
    EXPECT_NEAR(volumeOf(r.bodies.at(kB1)), 1000.0 - kPi * (9.0 * 2.0 + 4.0 * 8.0), 1e-6 * 1000.0);
}

TEST(FeaturesHole, BadThreadDesignationFails) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    HoleParams p;
    p.body = kB1;
    p.face = *boxFace(base, kB1, {0, 0, 1}, 10);
    p.thread = "six-millimetre";
    tl.append(std::make_shared<HoleFeature>(FeatureId{2}, p));
    Evaluator ev;
    auto r = ev.evaluate(tl);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->statuses[1].state, FeatureStatus::State::Failed);
}

namespace {

/// 20 x 20 x `height` box (feature 1) with a counterbore hole in its top face (feature 2).
struct CbHole {
    std::shared_ptr<Feature> box;
    HoleParams hp;
};

CbHole makeCbHole(double height = 10, double u = 10, double v = 10) {
    CbHole c;
    c.box = feat(1, boxParams(0, 0, 20, 20, height, kB1));
    Timeline tl;
    tl.append(c.box);
    EvalResult base = evalAllOk(tl);
    const std::uint32_t top = faceIndexOf(base, kB1, {0, 0, 1}, height);
    c.hp.body = kB1;
    c.hp.face = *boxFace(base, kB1, {0, 0, 1}, height);
    auto [pu, pv] = faceUV(base.bodies.at(kB1), top, {u, v, height});
    c.hp.u = mm(pu);
    c.hp.v = mm(pv);
    c.hp.kind = kernel::HoleKind::Counterbore;
    c.hp.diameter = "4 mm";
    c.hp.depth = "8 mm";
    c.hp.counterboreDiameter = "8 mm";
    c.hp.counterboreDepth = "3 mm";
    return c;
}

double parseFloat(const std::string& text) { return std::stod(text); }

/// Every name owned by feature `f` in body 1's naming map, as text.
std::set<std::string> namesOwnedBy(const EvalResult& r, std::uint64_t f) {
    std::set<std::string> out;
    const auto& m = r.namings.at(kB1);
    for (auto t : {ElementType::Face, ElementType::Edge, ElementType::Vertex})
        for (std::uint32_t i = 0; i < m.size(t); ++i)
            if (auto n = m.nameOf(t, i); n && n->feature.value() == f) out.insert(n->toString());
    return out;
}

/// The rim edge of the counterbore on the top face: a circle of radius 4 at z = `top`.
std::optional<ElementRef> counterboreRim(const EvalResult& r, double top) {
    return refWhere(r, kB1, ElementType::Edge, [&](const std::array<double, 4>& k) {
        return near(k[2], top) && near(k[3], 2.0 * kPi * 4.0, 1e-6);
    });
}

EvalResult evalCbHole(const CbHole& c, const HoleParams& hp, const std::shared_ptr<Feature>& box = nullptr) {
    Timeline tl;
    tl.append(box ? box : c.box);
    tl.append(std::make_shared<HoleFeature>(FeatureId{2}, hp));
    return evalAllOk(tl);
}

} // namespace

TEST(FeaturesHole, ElementsAreNamedByRoleAndEveryRoleIsPresent) {
    const CbHole c = makeCbHole();
    const EvalResult r = evalCbHole(c, c.hp);
    const auto names = namesOwnedBy(r, 2);
    auto has = [&](const std::string& part) {
        return std::any_of(names.begin(), names.end(), [&](const std::string& n) { return n.find(part) != std::string::npos; });
    };
    for (const char* role : {"holetool.wall", "holetool.cbore_wall", "holetool.cbore_floor", "holetool.drill_cone"})
        EXPECT_TRUE(has(role)) << role;
    // The names carry no geometric ordinals: every hole face / edge of this simple hole has ordinal 0.
    const auto& m = r.namings.at(kB1);
    for (auto t : {ElementType::Face, ElementType::Edge})
        for (std::uint32_t i = 0; i < m.size(t); ++i) {
            auto n = m.nameOf(t, i);
            if (n && n->feature.value() == 2 && n->op.rfind("holetool", 0) == 0) {
                EXPECT_EQ(n->ordinal, 0u) << n->toString();
            }
        }
}

TEST(FeaturesHole, NamesAreStableUnderDiameterPositionDepthAndUpstreamEdits) {
    const CbHole c = makeCbHole();
    const EvalResult base = evalCbHole(c, c.hp);
    const auto baseNames = namesOwnedBy(base, 2);
    ASSERT_GT(baseNames.size(), 8u);

    HoleParams dia = c.hp;
    dia.diameter = "5 mm";
    EXPECT_EQ(namesOwnedBy(evalCbHole(c, dia), 2), baseNames) << "diameter edit";

    HoleParams pos = c.hp;
    pos.u = mm(parseFloat(pos.u) + 2.0);
    pos.v = mm(parseFloat(pos.v) - 3.0);
    EXPECT_EQ(namesOwnedBy(evalCbHole(c, pos), 2), baseNames) << "position edit";

    HoleParams depth = c.hp;
    depth.depth = "8.5 mm";
    EXPECT_EQ(namesOwnedBy(evalCbHole(c, depth), 2), baseNames) << "depth edit";

    HoleParams cb = c.hp;
    cb.counterboreDiameter = "9 mm";
    cb.counterboreDepth = "2 mm";
    EXPECT_EQ(namesOwnedBy(evalCbHole(c, cb), 2), baseNames) << "counterbore edit";

    // Upstream: a taller box moves the top face; the hole keeps its (u, v) on it.
    const CbHole tall = makeCbHole(14);
    EXPECT_EQ(namesOwnedBy(evalCbHole(c, c.hp, feat(1, boxParams(0, 0, 20, 20, 14, kB1))), 2), baseNames) << "box height";
    (void)tall;
}

TEST(FeaturesHole, FilletOfTheRimSurvivesHoleAndUpstreamEdits) {
    const CbHole c = makeCbHole();
    const EvalResult base = evalCbHole(c, c.hp);
    auto rim = counterboreRim(base, 10);
    ASSERT_TRUE(rim.has_value());
    EXPECT_EQ(rim->name.op, "hole.edge");

    auto withFillet = [&](const HoleParams& hp, double height) {
        Timeline tl;
        tl.append(feat(1, boxParams(0, 0, 20, 20, height, kB1)));
        tl.append(std::make_shared<HoleFeature>(FeatureId{2}, hp));
        tl.append(fillet(3, *rim, "0.5 mm"));
        Evaluator ev;
        auto r = ev.evaluate(tl);
        EXPECT_TRUE(r.has_value());
        return r ? r->statuses[2] : FeatureStatus{};
    };
    EXPECT_EQ(withFillet(c.hp, 10).state, FeatureStatus::State::Ok);

    HoleParams dia = c.hp;
    dia.diameter = "5 mm";
    dia.counterboreDiameter = "9 mm"; // the rim is the counterbore circle: it must still exist under the same name
    EXPECT_EQ(withFillet(dia, 10).state, FeatureStatus::State::Ok) << withFillet(dia, 10).error.message;

    HoleParams pos = c.hp;
    pos.u = mm(parseFloat(pos.u) + 2.0);
    pos.v = mm(parseFloat(pos.v) - 3.0);
    EXPECT_EQ(withFillet(pos, 10).state, FeatureStatus::State::Ok) << withFillet(pos, 10).error.message;

    HoleParams depth = c.hp;
    depth.depth = "8.5 mm";
    EXPECT_EQ(withFillet(depth, 10).state, FeatureStatus::State::Ok) << withFillet(depth, 10).error.message;

    // A taller box: the hole is placed on the (moved) top face with the same u, v.
    HoleParams tallHole = c.hp;
    EXPECT_EQ(withFillet(tallHole, 14).state, FeatureStatus::State::Ok) << withFillet(tallHole, 14).error.message;
}

TEST(FeaturesMove, MoveTranslatesAndRotatesTheBody) {
    Timeline tl;
    tl.append(box10());
    MoveParams m;
    m.body = kB1;
    m.translate = {mm(0), mm(0), mm(3)};
    tl.append(std::make_shared<MoveFeature>(FeatureId{2}, m));
    EvalResult r = evalAllOk(tl);
    const auto z = zRange(r.bodies.at(kB1));
    EXPECT_NEAR(z.first, 3.0, 1e-9);
    EXPECT_NEAR(z.second, 13.0, 1e-9);
    EXPECT_NEAR(volumeOf(r.bodies.at(kB1)), 1000.0, 1e-6);
}

TEST(FeaturesMove, CopyCreatesANewBodyAndKeepsTheOriginal) {
    Timeline tl;
    tl.append(box10());
    MoveParams m;
    m.body = kB1;
    m.copy = true;
    m.newBody = kB2;
    m.translate = {mm(20), mm(0), mm(0)};
    tl.append(std::make_shared<MoveFeature>(FeatureId{2}, m));
    EvalResult r = evalAllOk(tl);
    ASSERT_TRUE(r.bodies.count(kB1));
    ASSERT_TRUE(r.bodies.count(kB2));
    EXPECT_NEAR(zRange(r.bodies.at(kB1)).first, 0.0, 1e-9);
    EXPECT_NEAR(volumeOf(r.bodies.at(kB2)), 1000.0, 1e-6);
    EXPECT_NEAR(r.bodies.at(kB2).boundingBox().min.x, 20.0, 1e-9);
}

TEST(FeaturesConstruction, OffsetPlaneFromFaceAndMidplane) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    ConstructionPlaneParams off;
    off.kind = ConstructionPlaneParams::Kind::Offset;
    off.base.isFace = true;
    off.base.body = kB1;
    off.base.face = *boxFace(base, kB1, {0, 0, 1}, 10);
    off.distance = "3 mm";
    tl.append(std::make_shared<ConstructionPlaneFeature>(FeatureId{2}, off));
    ConstructionPlaneParams mid;
    mid.kind = ConstructionPlaneParams::Kind::MidPlane;
    mid.base.isFace = true;
    mid.base.body = kB1;
    mid.base.face = *boxFace(base, kB1, {0, 0, -1}, 0);
    mid.other.isFace = true;
    mid.other.body = kB1;
    mid.other.face = *boxFace(base, kB1, {0, 0, 1}, 10);
    tl.append(std::make_shared<ConstructionPlaneFeature>(FeatureId{3}, mid));
    EvalResult r = evalAllOk(tl);
    ASSERT_TRUE(r.statuses[1].construction.has_value());
    EXPECT_NEAR(r.statuses[1].construction->origin.z, 13.0, 1e-9);
    EXPECT_NEAR(r.statuses[1].construction->normal.z, 1.0, 1e-9);
    ASSERT_TRUE(r.statuses[2].construction.has_value());
    EXPECT_NEAR(r.statuses[2].construction->origin.z, 5.0, 1e-9);
    EXPECT_NEAR(std::abs(r.statuses[2].construction->normal.z), 1.0, 1e-9);
    // Construction features produce no bodies.
    EXPECT_EQ(r.bodies.size(), 1u);
}

TEST(FeaturesConstruction, ThreePointPlaneAxisFromEdgeAndPointFromVertex) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    ConstructionPlaneParams three;
    three.kind = ConstructionPlaneParams::Kind::ThreePoints;
    three.points = {{{mm(0), mm(0), mm(4)}, {mm(10), mm(0), mm(4)}, {mm(0), mm(10), mm(4)}}};
    tl.append(std::make_shared<ConstructionPlaneFeature>(FeatureId{2}, three));

    ConstructionAxisParams ax;
    ax.kind = ConstructionAxisParams::Kind::Edge;
    ax.body = kB1;
    ax.ref = cornerEdgeOfBox(base);
    tl.append(std::make_shared<ConstructionAxisFeature>(FeatureId{3}, ax));

    ConstructionPointParams pt;
    pt.kind = ConstructionPointParams::Kind::Vertex;
    pt.body = kB1;
    auto vref = refWhere(base, kB1, ElementType::Vertex, [](const std::array<double, 4>& k) {
        return near(k[0], 10) && near(k[1], 10) && near(k[2], 10);
    });
    ASSERT_TRUE(vref.has_value());
    pt.ref = *vref;
    tl.append(std::make_shared<ConstructionPointFeature>(FeatureId{4}, pt));

    EvalResult r = evalAllOk(tl);
    // statuses[0] is the box; [1] plane, [2] axis, [3] point.
    ASSERT_TRUE(r.statuses[1].construction.has_value());
    EXPECT_NEAR(std::abs(r.statuses[1].construction->normal.z), 1.0, 1e-9);
    ASSERT_TRUE(r.statuses[2].construction.has_value());
    EXPECT_NEAR(std::abs(r.statuses[2].construction->direction.z), 1.0, 1e-9);
    ASSERT_TRUE(r.statuses[3].construction.has_value());
    EXPECT_NEAR(r.statuses[3].construction->origin.x, 10.0, 1e-9);
    EXPECT_NEAR(r.statuses[3].construction->origin.y, 10.0, 1e-9);
    EXPECT_NEAR(r.statuses[3].construction->origin.z, 10.0, 1e-9);
}

TEST(FeaturesConstruction, RevolveAboutAConstructionAxisFeature) {
    // Axis feature built from a cylinder face is covered by the kernel; here the axis drives a revolve through
    // the construction geometry channel (no body is produced by the axis itself).
    Timeline tl;
    tl.append(axisTwoPoints(1, {0, 0, 0}, {0, 1, 0}));
    tl.append(revolveRect(2, 1, 2, 3, 1, RevolveParams::Extent::Full, "360 deg"));
    EvalResult r = evalAllOk(tl);
    EXPECT_NEAR(volumeOf(r.bodies.at(kB1)), kPi * (4.0 - 1.0) * 3.0, 1e-6 * 10.0);
    EXPECT_FALSE(r.statuses[0].construction->origin.y != 0.0);
}

TEST(FeaturesRegistry, EveryNewKindRoundTripsThroughRegistryAndIo) {
    Timeline tl;
    tl.append(box10(1, kB1));
    EvalResult base = evalAllOk(tl);
    tl.append(axisTwoPoints(2, {0, 0, 0}, {0, 1, 0}));
    tl.append(revolveRect(3, 20, 25, 10, 2, RevolveParams::Extent::TwoSides, "30 deg", "15 deg", kB2, BodyOp::NewBody));
    tl.append(fillet(4, cornerEdgeOfBox(base), "1 mm"));
    ChamferParams cp;
    cp.body = kB1;
    cp.edges = {cornerEdgeOfBox(base)};
    cp.mode = ChamferParams::Mode::DistanceAngle;
    cp.distance = "1 mm";
    cp.angle = "30 deg";
    tl.append(std::make_shared<ChamferFeature>(FeatureId{5}, cp));
    HoleParams hp;
    hp.body = kB1;
    hp.face = *boxFace(base, kB1, {0, 0, 1}, 10);
    hp.diameter = "3 mm";
    hp.depth = "5 mm";
    hp.thread = "M3";
    tl.append(std::make_shared<HoleFeature>(FeatureId{6}, hp));
    MoveParams mp;
    mp.body = kB1;
    mp.copy = true;
    mp.newBody = BodyId{3};
    mp.translate = {mm(30), mm(0), mm(0)};
    tl.append(std::make_shared<MoveFeature>(FeatureId{7}, mp));
    ConstructionPlaneParams cpl;
    cpl.kind = ConstructionPlaneParams::Kind::AngleAxis;
    cpl.base.isFace = true;
    cpl.base.body = kB1;
    cpl.base.face = *boxFace(base, kB1, {0, 0, 1}, 10);
    cpl.axis = FeatureId{2};
    cpl.angle = "10 deg";
    tl.append(std::make_shared<ConstructionPlaneFeature>(FeatureId{8}, cpl));
    ConstructionPointParams ptp;
    ptp.body = kB1;
    ptp.ref = cornerEdgeOfBox(base);
    ptp.kind = ConstructionPointParams::Kind::CircleCentre;
    tl.append(std::make_shared<ConstructionPointFeature>(FeatureId{9}, ptp));
    ConstructionAxisParams axp;
    axp.kind = ConstructionAxisParams::Kind::Cylinder;
    axp.body = kB1;
    axp.ref = *boxFace(base, kB1, {0, 0, 1}, 10);
    tl.append(std::make_shared<ConstructionAxisFeature>(FeatureId{10}, axp));

    FeatureRegistry reg;
    registerFeatures(reg);
    for (std::size_t i = 0; i < tl.size(); ++i) {
        auto again = reg.create(tl.at(i)->kind(), tl.at(i)->id(), tl.at(i)->paramsJson());
        ASSERT_TRUE(again.has_value()) << tl.at(i)->kind() << ": " << again.error().message;
        EXPECT_EQ((*again)->paramsJson(), tl.at(i)->paramsJson()) << tl.at(i)->kind();
        EXPECT_EQ((*again)->paramHash(), tl.at(i)->paramHash()) << tl.at(i)->kind();
    }

    io::Document doc;
    doc.name = "b1b-roundtrip";
    doc.nextId = 20;
    for (std::size_t i = 0; i < tl.size(); ++i) doc.timeline.append(tl.at(i));
    const auto dir = std::filesystem::temp_directory_path() / ("iw_b1b_rt_" + std::to_string(std::random_device{}()) + ".iwd");
    std::filesystem::remove_all(dir);
    ASSERT_TRUE(io::saveDocument(dir, doc).has_value());
    auto loaded = io::loadDocument(dir, reg);
    std::filesystem::remove_all(dir);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    ASSERT_EQ(loaded->timeline.size(), tl.size());
    for (std::size_t i = 0; i < tl.size(); ++i)
        EXPECT_EQ(loaded->timeline.at(i)->paramsJson(), tl.at(i)->paramsJson()) << tl.at(i)->kind();
    Evaluator e1, e2;
    auto a = e1.evaluate(tl);
    auto b = e2.evaluate(loaded->timeline);
    ASSERT_TRUE(a && b);
    for (std::size_t i = 0; i < tl.size(); ++i) EXPECT_EQ(a->statuses[i].state, b->statuses[i].state) << tl.at(i)->kind();
    EXPECT_NEAR(volumeOf(a->bodies.at(kB1)), volumeOf(b->bodies.at(kB1)), 1e-9);
}
