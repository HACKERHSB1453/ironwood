// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "kernel/ops.hpp"
#include "kernel/shape.hpp"
#include "sketch/profile.hpp"
#include "sketch/sketch.hpp"

using namespace iw;
using namespace iw::sketch;

namespace {

constexpr double kPi = std::numbers::pi;

Sketch xySketch() {
    auto plane = SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0});
    EXPECT_TRUE(plane.has_value());
    return Sketch(*plane);
}

void addRect(Sketch& s, double x, double y, double w, double h, bool construction = false) {
    const Point2 a{x, y}, b{x + w, y}, c{x + w, y + h}, d{x, y + h};
    ASSERT_TRUE(s.addLine(a, b, construction).has_value());
    ASSERT_TRUE(s.addLine(b, c, construction).has_value());
    ASSERT_TRUE(s.addLine(c, d, construction).has_value());
    ASSERT_TRUE(s.addLine(d, a, construction).has_value());
}

SketchEntityId addCircle(Sketch& s, double x, double y, double r) {
    auto id = s.addCircle({x, y}, r);
    EXPECT_TRUE(id.has_value());
    return id.value_or(SketchEntityId{});
}

std::vector<SketchProfile> detect(const Sketch& s) {
    auto r = detectProfiles(s);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message);
    return r.value_or(std::vector<SketchProfile>{});
}

void expectRel(double actual, double expected, double rel = 1e-9) {
    EXPECT_NEAR(actual, expected, rel * std::max(1.0, std::abs(expected)));
}

const SketchProfile* findByName(const std::vector<SketchProfile>& ps, const std::string& name) {
    for (const auto& p : ps)
        if (p.name == name) return &p;
    return nullptr;
}

std::vector<double> sortedAreas(const std::vector<SketchProfile>& ps) {
    std::vector<double> a;
    for (const auto& p : ps) a.push_back(p.area);
    std::sort(a.begin(), a.end());
    return a;
}

} // namespace

// ---- plane ---------------------------------------------------------------------------------------------------

TEST(SketchPlane, MakeValidatesAndOrthonormalises) {
    EXPECT_FALSE(SketchPlane::make({0, 0, 0}, {0, 0, 0}, {1, 0, 0}).has_value());
    EXPECT_FALSE(SketchPlane::make({0, 0, 0}, {0, 0, 1}, {0, 0, 2}).has_value()); // xDir parallel to normal
    auto p = SketchPlane::make({1, 2, 3}, {0, 0, 4}, {1, 0, 1}); // xDir not perpendicular, normal not unit
    ASSERT_TRUE(p.has_value());
    EXPECT_NEAR(length(p->normal), 1.0, 1e-12);
    EXPECT_NEAR(length(p->xDir), 1.0, 1e-12);
    EXPECT_NEAR(dot(p->normal, p->xDir), 0.0, 1e-12);
}

TEST(SketchPlane, RoundTripBetween2DAnd3D) {
    auto p = SketchPlane::make({10, -4, 2}, {1, 1, 0}, {0, 0, 1});
    ASSERT_TRUE(p.has_value());
    for (const Point2 q : {Point2{0, 0}, Point2{3.5, -2.25}, Point2{-100, 42}}) {
        const Vec3 w = p->toWorld(q);
        const Point2 back = p->toLocal(w);
        EXPECT_NEAR(back.x, q.x, 1e-9);
        EXPECT_NEAR(back.y, q.y, 1e-9);
        EXPECT_NEAR(dot(w - p->origin, p->normal), 0.0, 1e-9); // lies in the plane
    }
}

// ---- entities and records --------------------------------------------------------------------------------------

TEST(SketchEntities, InvalidGeometryIsRejected) {
    Sketch s = xySketch();
    EXPECT_EQ(s.addLine({0, 0}, {0, 0}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(s.addCircle({0, 0}, 0).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(s.addCircle({0, 0}, -1).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(s.addArc({0, 0}, 1, 0, 0).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(s.addArc({0, 0}, 1, 0, 2 * kPi).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(s.addPoint({std::nan(""), 0}).error().code, ErrorCode::InvalidArgument);
    EXPECT_TRUE(s.entities().empty()); // failed calls change nothing
}

TEST(SketchEntities, IdsAreNeverReused) {
    Sketch s = xySketch();
    auto a = s.addPoint({0, 0});
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(s.removeEntity(*a).has_value());
    auto b = s.addPoint({1, 1});
    ASSERT_TRUE(b.has_value());
    EXPECT_NE(a->value(), b->value());
    EXPECT_GT(b->value(), a->value());
}

TEST(SketchEntities, RemoveEntityDropsReferencingRecords) {
    Sketch s = xySketch();
    auto l1 = s.addLine({0, 0}, {10, 0});
    auto l2 = s.addLine({0, 5}, {10, 5});
    ASSERT_TRUE(l1 && l2);
    ASSERT_TRUE(s.addConstraint(ConstraintKind::Parallel, {SketchRef{*l1, {}}, SketchRef{*l2, {}}}).has_value());
    ASSERT_TRUE(s.addDimension(DimensionKind::Distance, {SketchRef{*l1, PointRole::Start}}, "5 mm").has_value());
    ASSERT_TRUE(s.addDimension(DimensionKind::Distance, {SketchRef{*l2, {}}}, "3 mm").has_value());
    ASSERT_TRUE(s.removeEntity(*l1).has_value());
    EXPECT_EQ(s.constraints().size(), 0u);
    ASSERT_EQ(s.dimensions().size(), 1u);
    EXPECT_EQ(s.dimensions().begin()->second.expression, "3 mm");
    EXPECT_EQ(s.removeEntity(*l1).error().code, ErrorCode::NotFound);
}

TEST(SketchRecords, RefsAreValidatedAgainstEntityKind) {
    Sketch s = xySketch();
    auto line = s.addLine({0, 0}, {10, 0});
    auto circle = s.addCircle({0, 0}, 2);
    auto arc = s.addArc({0, 0}, 3, 0, kPi / 2);
    ASSERT_TRUE(line && circle && arc);
    EXPECT_TRUE(s.addConstraint(ConstraintKind::Horizontal, {SketchRef{*line, {}}}).has_value());
    EXPECT_TRUE(s.addConstraint(ConstraintKind::Concentric, {SketchRef{*circle, PointRole::Center}}).has_value());
    EXPECT_TRUE(s.addDimension(DimensionKind::Angle, {SketchRef{*arc, PointRole::Mid}}, "45 deg").has_value());
    // Center is not a line sub-point; Start is not a circle sub-point.
    EXPECT_EQ(s.addConstraint(ConstraintKind::Fix, {SketchRef{*line, PointRole::Center}}).error().code,
              ErrorCode::InvalidArgument);
    EXPECT_EQ(s.addDimension(DimensionKind::Radius, {SketchRef{*circle, PointRole::Start}}, "1 mm").error().code,
              ErrorCode::InvalidArgument);
    EXPECT_EQ(s.addConstraint(ConstraintKind::Coincident, {SketchRef{SketchEntityId{999}, {}}}).error().code,
              ErrorCode::NotFound);
    EXPECT_EQ(s.constraints().size(), 2u); // invalid records were not stored
    EXPECT_EQ(s.dimensions().size(), 1u);
}

TEST(SketchRecords, PointOfResolvesSubPoints) {
    Sketch s = xySketch();
    auto arc = s.addArc({0, 0}, 2, 0, kPi / 2);
    auto line = s.addLine({0, 0}, {4, 2});
    ASSERT_TRUE(arc && line);
    auto end = s.pointOf(SketchRef{*arc, PointRole::End});
    ASSERT_TRUE(end.has_value());
    EXPECT_NEAR(end->x, 0.0, 1e-12);
    EXPECT_NEAR(end->y, 2.0, 1e-12);
    auto mid = s.pointOf(SketchRef{*line, PointRole::Mid});
    ASSERT_TRUE(mid.has_value());
    EXPECT_EQ(mid->x, 2.0);
    EXPECT_EQ(mid->y, 1.0);
}

TEST(SketchEntities, SetGeometryKeepsIdAndKind) {
    Sketch s = xySketch();
    auto c = s.addCircle({0, 0}, 1);
    ASSERT_TRUE(c.has_value());
    EXPECT_TRUE(s.setGeometry(*c, CircleGeom{{5, 5}, 2}).has_value());
    EXPECT_EQ(s.setGeometry(*c, LineGeom{{0, 0}, {1, 1}}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(s.setGeometry(SketchEntityId{77}, CircleGeom{{0, 0}, 1}).error().code, ErrorCode::NotFound);
    EXPECT_EQ(std::get<CircleGeom>(s.findEntity(*c)->geometry).radius, 2.0);
}

// ---- profile detection -----------------------------------------------------------------------------------------

TEST(Profiles, RectangleIsOneProfile) {
    Sketch s = xySketch();
    addRect(s, 0, 0, 20, 10);
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 1u);
    EXPECT_TRUE(ps[0].holes.empty());
    expectRel(ps[0].area, 200.0);
    EXPECT_NEAR(ps[0].centroid.x, 10.0, 1e-9);
    EXPECT_NEAR(ps[0].centroid.y, 5.0, 1e-9);
    EXPECT_EQ(ps[0].outer.edges.size(), 4u);
    EXPECT_GT(ps[0].outer.area, 0.0); // outer loops are CCW
}

TEST(Profiles, RectangleWithInnerCircleIsRingAndDisc) {
    Sketch s = xySketch();
    addRect(s, 0, 0, 20, 10);
    const SketchEntityId c = addCircle(s, 10, 5, 3);
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 2u);
    const double disc = kPi * 9.0;
    const double ring = 200.0 - disc;
    EXPECT_NEAR(sortedAreas(ps)[0], disc, 1e-9 * disc);
    EXPECT_NEAR(sortedAreas(ps)[1], ring, 1e-9 * ring);
    const SketchProfile* discP = findByName(ps, "e" + std::to_string(c.value()) + ".0");
    ASSERT_NE(discP, nullptr) << "disc must be named by its circle piece";
    EXPECT_TRUE(discP->holes.empty());
    const SketchProfile* ringP = nullptr;
    for (const auto& p : ps)
        if (&p != discP) ringP = &p;
    ASSERT_NE(ringP, nullptr);
    ASSERT_EQ(ringP->holes.size(), 1u);
    EXPECT_LT(ringP->holes[0].area, 0.0); // holes are CW
    EXPECT_NEAR(ringP->centroid.x, 10.0, 1e-9);
    EXPECT_NEAR(ringP->centroid.y, 5.0, 1e-9);
}

TEST(Profiles, TwoOverlappingCirclesGiveThreeRegions) {
    Sketch s = xySketch();
    addCircle(s, 0, 0, 1);
    addCircle(s, 1, 0, 1);
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 3u);
    // Lens area for equal radii r and centre distance d.
    const double r = 1, d = 1;
    const double lens = 2 * r * r * std::acos(d / (2 * r)) - (d / 2) * std::sqrt(4 * r * r - d * d);
    const auto areas = sortedAreas(ps);
    EXPECT_NEAR(areas[0], lens, 1e-9);
    EXPECT_NEAR(areas[1], kPi - lens, 1e-9);
    EXPECT_NEAR(areas[2], kPi - lens, 1e-9);
    for (const auto& p : ps) EXPECT_TRUE(p.holes.empty());
}

TEST(Profiles, OpenChainHasNoProfiles) {
    Sketch s = xySketch();
    ASSERT_TRUE(s.addLine({0, 0}, {10, 0}).has_value());
    ASSERT_TRUE(s.addLine({10, 0}, {10, 10}).has_value());
    ASSERT_TRUE(s.addLine({10, 10}, {0, 10}).has_value());
    EXPECT_TRUE(detect(s).empty());
}

TEST(Profiles, EmptySketchHasNoProfiles) {
    Sketch s = xySketch();
    EXPECT_TRUE(detect(s).empty());
}

TEST(Profiles, ConstructionGeometryIsIgnored) {
    Sketch s = xySketch();
    addRect(s, 0, 0, 10, 10);
    ASSERT_TRUE(s.addLine({0, 0}, {10, 10}, /*construction=*/true).has_value()); // would split the square
    ASSERT_TRUE(s.addCircle({5, 5}, 4, /*construction=*/true).has_value());      // would cut a hole / disc
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 1u);
    expectRel(ps[0].area, 100.0);
    EXPECT_TRUE(ps[0].holes.empty());
}

TEST(Profiles, DanglingStubIsIgnored) {
    Sketch s = xySketch();
    addRect(s, 0, 0, 10, 10);
    ASSERT_TRUE(s.addLine({5, 5}, {7, 8}).has_value());     // dangling spur inside the square
    ASSERT_TRUE(s.addLine({10, 5}, {14, 5}).has_value());   // dangling spur outside, touching an edge at one end
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 1u);
    expectRel(ps[0].area, 100.0);
    EXPECT_TRUE(ps[0].holes.empty());
}

TEST(Profiles, LineTJunctionSplitsRectangle) {
    Sketch s = xySketch();
    addRect(s, 0, 0, 10, 10);
    ASSERT_TRUE(s.addLine({0, 5}, {10, 5}).has_value()); // ends lie on the left and right edges
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 2u);
    expectRel(ps[0].area, 50.0);
    expectRel(ps[1].area, 50.0);
}

TEST(Profiles, LineThroughCircleSplitsBothCurves) {
    Sketch s = xySketch();
    addRect(s, 0, 0, 10, 10);
    addCircle(s, 5, 5, 2);
    ASSERT_TRUE(s.addLine({0, 5}, {10, 5}).has_value()); // line/circle intersections at (3,5) and (7,5)
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 4u); // top / bottom rectangle halves, upper / lower half-discs
    const double halfDisc = 2 * kPi;
    const auto areas = sortedAreas(ps);
    EXPECT_NEAR(areas[0], halfDisc, 1e-9);
    EXPECT_NEAR(areas[1], halfDisc, 1e-9);
    EXPECT_NEAR(areas[2], 50.0 - halfDisc, 1e-9);
    EXPECT_NEAR(areas[3], 50.0 - halfDisc, 1e-9);
}

TEST(Profiles, ArcAndLineFormHalfDiscAndAreSplitAtChord) {
    Sketch s = xySketch();
    auto arc = s.addArc({0, 0}, 1, 0, kPi); // upper semicircle from (1,0) to (-1,0)
    ASSERT_TRUE(arc.has_value());
    ASSERT_TRUE(s.addLine({-1, 0}, {1, 0}).has_value());
    auto ps = detect(s);
    ASSERT_EQ(ps.size(), 1u);
    expectRel(ps[0].area, kPi / 2);

    // A vertical radius from the centre to the arc splits the half-disc into two quarter-discs (T-junction at
    // the centre and endpoint-on-arc at the top).
    ASSERT_TRUE(s.addLine({0, 0}, {0, 1}).has_value());
    ps = detect(s);
    ASSERT_EQ(ps.size(), 2u);
    expectRel(ps[0].area, kPi / 4);
    expectRel(ps[1].area, kPi / 4);
}

TEST(Profiles, NestedLoopsUseEvenOddNesting) {
    Sketch s = xySketch();
    addRect(s, 0, 0, 10, 10);
    const SketchEntityId big = addCircle(s, 5, 5, 3);
    const SketchEntityId small = addCircle(s, 5, 5, 1);
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 3u);
    // rect minus big disc; big disc minus small disc (has a hole); small disc.
    const auto areas = sortedAreas(ps);
    EXPECT_NEAR(areas[0], kPi, 1e-9);
    EXPECT_NEAR(areas[1], 8 * kPi, 1e-9);
    EXPECT_NEAR(areas[2], 100.0 - 9 * kPi, 1e-9);
    // The annulus is outer = big circle (positive), hole = small circle; its name carries both pieces.
    const std::string bigTok = "e" + std::to_string(big.value()) + ".0";
    const std::string smallTok = "e" + std::to_string(small.value()) + ".0";
    const SketchProfile* annulus = findByName(ps, bigTok + "+" + smallTok);
    ASSERT_NE(annulus, nullptr);
    EXPECT_EQ(annulus->holes.size(), 1u);
    EXPECT_NE(findByName(ps, smallTok), nullptr);
}

TEST(Profiles, NamesAreStableWhenUnrelatedEntityIsAdded) {
    Sketch s = xySketch();
    addRect(s, 0, 0, 20, 10);
    addCircle(s, 10, 5, 3);
    const auto before = detect(s);
    std::vector<std::string> namesBefore;
    for (const auto& p : before) namesBefore.push_back(p.name);

    addRect(s, 100, 100, 5, 5); // unrelated square far away
    const auto after = detect(s);
    ASSERT_EQ(after.size(), 3u);
    for (const std::string& n : namesBefore) {
        const SketchProfile* p = findByName(after, n);
        ASSERT_NE(p, nullptr) << "name lost: " << n;
    }
}

TEST(Profiles, NamesAreDeterministicAcrossRuns) {
    Sketch a = xySketch();
    Sketch b = xySketch();
    for (Sketch* s : {&a, &b}) {
        addRect(*s, 0, 0, 20, 10);
        addCircle(*s, 10, 5, 3);
        addCircle(*s, 3, 3, 1);
    }
    const auto pa = detect(a);
    const auto pb = detect(b);
    ASSERT_EQ(pa.size(), pb.size());
    for (std::size_t i = 0; i < pa.size(); ++i) {
        EXPECT_EQ(pa[i].name, pb[i].name);
        EXPECT_EQ(pa[i].area, pb[i].area);
    }
}

TEST(Profiles, GridOfOverlappingSquaresGivesOneRegionPerCell) {
    // Adjacent squares share edges exactly (duplicate geometry): tests dedupe and collinear overlaps.
    Sketch s = xySketch();
    constexpr int N = 10;
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) addRect(s, i, j, 1, 1);
    const auto t0 = std::chrono::steady_clock::now();
    const auto ps = detect(s);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
    ASSERT_EQ(ps.size(), static_cast<std::size_t>(N * N));
    double total = 0;
    for (const auto& p : ps) {
        expectRel(p.area, 1.0);
        total += p.area;
    }
    expectRel(total, 100.0);
    EXPECT_LT(ms.count(), 5000) << "profile detection on 400 lines should be well under 5 s";
}

// ---- region -> kernel face ---------------------------------------------------------------------------------------

TEST(ProfileFace, RingFaceHasHoleAndMatchesArea) {
    Sketch s = xySketch();
    addRect(s, 0, 0, 20, 10);
    addCircle(s, 10, 5, 3);
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 2u);
    for (const auto& p : ps) {
        auto face = profileFace(s.plane(), p);
        ASSERT_TRUE(face.has_value()) << face.error().message;
        EXPECT_EQ(face->type(), kernel::ShapeType::Face);
        auto mp = kernel::massProperties(*face);
        ASSERT_TRUE(mp.has_value());
        expectRel(mp->surfaceArea, p.area, 1e-6);
        const Box3 box = face->boundingBox();
        EXPECT_NEAR(box.min.z, 0.0, 1e-6); // lies on the XY plane
        EXPECT_NEAR(box.max.z, 0.0, 1e-6);
    }
}

TEST(ProfileFace, RespectsPlaneOffsetAndOrientation) {
    auto plane = SketchPlane::make({0, 0, 7}, {1, 0, 0}, {0, 1, 0});
    ASSERT_TRUE(plane.has_value());
    Sketch s(*plane);
    addRect(s, 0, 0, 4, 3);
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 1u);
    auto face = profileFace(s.plane(), ps[0]);
    ASSERT_TRUE(face.has_value()) << face.error().message;
    auto mp = kernel::massProperties(*face);
    ASSERT_TRUE(mp.has_value());
    expectRel(mp->surfaceArea, 12.0, 1e-6);
    // u (sketch x) maps to world y, v (sketch y = normal x xDir = +z) maps to world z, offset by origin z = 7.

}

TEST(Profiles, TwoArcsFormLens) {
    // Lens of circles (0,0) r=1 and (1,0) r=1: the part of each circle inside the other is an arc.
    Sketch s = xySketch();
    ASSERT_TRUE(s.addArc({0, 0}, 1, -kPi / 3, 2 * kPi / 3).has_value());     // circle A, inside B
    ASSERT_TRUE(s.addArc({1, 0}, 1, 2 * kPi / 3, 2 * kPi / 3).has_value());  // circle B, inside A
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 1u);
    const double lens = 2 * kPi / 3 - std::sqrt(3.0) / 2.0;
    expectRel(ps[0].area, lens, 1e-9);
}

TEST(Profiles, InternallyTangentCirclesShareOneVertex) {
    Sketch s = xySketch();
    addCircle(s, 0, 0, 2);
    addCircle(s, 1, 0, 1); // touches the big circle at (2, 0)
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 2u);
    const auto areas = sortedAreas(ps);
    EXPECT_NEAR(areas[0], kPi, 1e-9);
    EXPECT_NEAR(areas[1], 3 * kPi, 1e-9);
}

TEST(Profiles, ExternallyTangentCirclesGiveTwoDiscs) {
    Sketch s = xySketch();
    addCircle(s, 0, 0, 1);
    addCircle(s, 2, 0, 1); // touches at (1, 0)
    const auto ps = detect(s);
    ASSERT_EQ(ps.size(), 2u);
    for (const auto& p : ps) expectRel(p.area, kPi, 1e-9);
}

// ---- persistence restore (ids kept, counter restored) ----------------------------------------------------------

TEST(SketchRestore, EntitiesKeepTheirOriginalIds) {
    Sketch sk(SketchPlane{});
    auto a = sk.addPoint({1, 1});
    auto b = sk.addLine({0, 0}, {5, 0});
    ASSERT_TRUE(a && b);
    ASSERT_TRUE(sk.removeEntity(*a).has_value());

    Sketch r(SketchPlane{});
    ASSERT_TRUE(r.restoreEntity(*b, LineGeom{{0, 0}, {5, 0}}).has_value());
    ASSERT_NE(r.findEntity(*b), nullptr);
    EXPECT_EQ(r.findEntity(*b)->id, *b);
    EXPECT_GE(r.nextId(), b->value() + 1);
}

TEST(SketchRestore, RestoreNextIdPreventsReuseOfRetiredIds) {
    // Ids 1..4 used, the last two removed: a reload must not hand out 3 or 4 again.
    Sketch sk(SketchPlane{});
    auto a = sk.addPoint({0, 0});
    auto b = sk.addPoint({1, 0});
    auto c = sk.addPoint({2, 0});
    auto d = sk.addPoint({3, 0});
    ASSERT_TRUE(a && b && c && d);
    ASSERT_TRUE(sk.removeEntity(*c).has_value());
    ASSERT_TRUE(sk.removeEntity(*d).has_value());
    const std::uint64_t counter = sk.nextId();

    Sketch r(SketchPlane{});
    ASSERT_TRUE(r.restoreEntity(*a, PointGeom{{0, 0}}).has_value());
    ASSERT_TRUE(r.restoreEntity(*b, PointGeom{{1, 0}}).has_value());
    ASSERT_TRUE(r.restoreNextId(counter).has_value());
    EXPECT_EQ(r.nextId(), counter);
    auto fresh = r.addPoint({9, 9});
    ASSERT_TRUE(fresh.has_value());
    EXPECT_EQ(fresh->value(), counter);
    EXPECT_GT(fresh->value(), d->value());
}

TEST(SketchRestore, ConstraintsAndDimensionsRestoreWithIds) {
    Sketch sk(SketchPlane{});
    auto l = sk.addLine({0, 0}, {5, 0});
    ASSERT_TRUE(l);
    auto h = sk.addConstraint(ConstraintKind::Horizontal, {SketchRef{*l, std::nullopt}});
    auto d = sk.addDimension(DimensionKind::Distance, {SketchRef{*l, PointRole::Start}, SketchRef{*l, PointRole::End}},
                             "d1 * 2");
    ASSERT_TRUE(h && d);

    Sketch r(SketchPlane{});
    ASSERT_TRUE(r.restoreEntity(*l, LineGeom{{0, 0}, {5, 0}}).has_value());
    ASSERT_TRUE(r.restoreConstraint(*h, ConstraintKind::Horizontal, {SketchRef{*l, std::nullopt}}).has_value());
    ASSERT_TRUE(r.restoreDimension(*d, DimensionKind::Distance,
                                   {SketchRef{*l, PointRole::Start}, SketchRef{*l, PointRole::End}}, "d1 * 2")
                    .has_value());
    ASSERT_TRUE(r.restoreNextId(sk.nextId()).has_value());
    ASSERT_TRUE(r.constraints().contains(*h));
    EXPECT_EQ(r.dimensions().at(*d).expression, "d1 * 2");
    EXPECT_EQ(r.nextId(), sk.nextId());
}

TEST(SketchRestore, RejectsBadInput) {
    Sketch r(SketchPlane{});
    ASSERT_TRUE(r.restoreEntity(SketchEntityId{5}, PointGeom{{0, 0}}).has_value());
    // Duplicate id (also across record kinds, since they share one counter).
    EXPECT_EQ(r.restoreEntity(SketchEntityId{5}, PointGeom{{1, 1}}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(r.restoreConstraint(SketchConstraintId{5}, ConstraintKind::Fix, {}).error().code, ErrorCode::InvalidArgument);
    // Zero id.
    EXPECT_EQ(r.restoreEntity(SketchEntityId{0}, PointGeom{{0, 0}}).error().code, ErrorCode::InvalidArgument);
    // Same geometry rules as creation.
    EXPECT_EQ(r.restoreEntity(SketchEntityId{6}, LineGeom{{0, 0}, {0, 0}}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(r.restoreEntity(SketchEntityId{7}, CircleGeom{{0, 0}, -1}).error().code, ErrorCode::InvalidArgument);
    // Refs must name live entities with a valid role.
    EXPECT_EQ(r.restoreConstraint(SketchConstraintId{8}, ConstraintKind::Fix, {SketchRef{SketchEntityId{99}, std::nullopt}})
                  .error()
                  .code,
              ErrorCode::NotFound);
    // Counter: must exceed every live id and must not go down.
    EXPECT_EQ(r.restoreNextId(5).error().code, ErrorCode::InvalidArgument);
    ASSERT_TRUE(r.restoreNextId(20).has_value());
    EXPECT_EQ(r.restoreNextId(10).error().code, ErrorCode::InvalidArgument);
}
