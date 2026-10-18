// SPDX-License-Identifier: GPL-3.0-or-later
// Sketch <-> solver bridge (src/sketch/solver/bridge.*): every constraint and dimension kind, parameter-driven
// dimensions, conflict/redundancy diagnostics mapped to record ids, and write-back of solved geometry.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>

#include "core/params.hpp"
#include "sketch/sketch.hpp"
#include "sketch/solver/bridge.hpp"

using namespace iw;
using namespace iw::sketch;

namespace {

constexpr double kTol = 1e-6;

Sketch makeSketch() {
    auto plane = SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0});
    EXPECT_TRUE(plane.has_value());
    return Sketch(*plane);
}

template <class T> requires(!std::is_void_v<T>) T must(Result<T> r) {
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message);
    return r.has_value() ? *r : T{};
}

const LineGeom& lineOf(const Sketch& s, SketchEntityId id) {
    return std::get<LineGeom>(s.findEntity(id)->geometry);
}

const ArcGeom& arcOf(const Sketch& s, SketchEntityId id) {
    return std::get<ArcGeom>(s.findEntity(id)->geometry);
}

const CircleGeom& circleOf(const Sketch& s, SketchEntityId id) {
    return std::get<CircleGeom>(s.findEntity(id)->geometry);
}

bool near(Point2 a, Point2 b, double tol = kTol) {
    return std::abs(a.x - b.x) < tol && std::abs(a.y - b.y) < tol;
}

struct Rect {
    SketchEntityId bottom, right, top, left;
};

// Rectangle with rough initial coordinates; constraints: 4 coincident, H bottom/top, V left/right.
Rect addRoughRect(Sketch& s) {
    Rect r;
    r.bottom = must(s.addLine({0.4, -0.2}, {41.0, 0.3}));
    r.right = must(s.addLine({41.0, 0.3}, {39.5, 21.2}));
    r.top = must(s.addLine({39.5, 21.2}, {-0.7, 19.6}));
    r.left = must(s.addLine({-0.7, 19.6}, {0.4, -0.2}));
    must(s.addConstraint(ConstraintKind::Coincident, {{r.bottom, PointRole::End}, {r.right, PointRole::Start}}));
    must(s.addConstraint(ConstraintKind::Coincident, {{r.right, PointRole::End}, {r.top, PointRole::Start}}));
    must(s.addConstraint(ConstraintKind::Coincident, {{r.top, PointRole::End}, {r.left, PointRole::Start}}));
    must(s.addConstraint(ConstraintKind::Coincident, {{r.left, PointRole::End}, {r.bottom, PointRole::Start}}));
    must(s.addConstraint(ConstraintKind::Horizontal, {{r.bottom, std::nullopt}}));
    must(s.addConstraint(ConstraintKind::Horizontal, {{r.top, std::nullopt}}));
    must(s.addConstraint(ConstraintKind::Vertical, {{r.right, std::nullopt}}));
    must(s.addConstraint(ConstraintKind::Vertical, {{r.left, std::nullopt}}));
    return r;
}

} // namespace

TEST(SolverBridge, RectangleWithLiteralDimensionsSolvesAndWritesBack) {
    Sketch s = makeSketch();
    Rect r = addRoughRect(s);
    // Width 40 mm (bottom, left to right) and height 20 mm (right, bottom to top).
    must(s.addDimension(DimensionKind::DistanceX, {{r.bottom, PointRole::Start}, {r.bottom, PointRole::End}}, "40 mm"));
    auto hd = must(s.addDimension(DimensionKind::DistanceY, {{r.right, PointRole::Start}, {r.right, PointRole::End}},
                                  "20"));
    (void)hd;

    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    EXPECT_TRUE(rep.ok());

    const LineGeom& b = lineOf(s, r.bottom);
    const LineGeom& rt = lineOf(s, r.right);
    const LineGeom& t = lineOf(s, r.top);
    const LineGeom& l = lineOf(s, r.left);
    EXPECT_NEAR(b.end.x - b.start.x, 40.0, kTol);
    EXPECT_NEAR(rt.end.y - rt.start.y, 20.0, kTol);
    // Corners are shared after the solve (coincident applied to the written geometry).
    EXPECT_TRUE(near(b.end, rt.start));
    EXPECT_TRUE(near(rt.end, t.start));
    EXPECT_TRUE(near(t.end, l.start));
    EXPECT_TRUE(near(l.end, b.start));
    // H/V held.
    EXPECT_NEAR(b.start.y, b.end.y, kTol);
    EXPECT_NEAR(rt.start.x, rt.end.x, kTol);
    // Two free position DOFs remain (no anchor in this fixture).
    EXPECT_EQ(rep.dof, 2);
}

TEST(SolverBridge, DiagnoseDoesNotModifySketch) {
    Sketch s = makeSketch();
    Rect r = addRoughRect(s);
    must(s.addDimension(DimensionKind::DistanceX, {{r.bottom, PointRole::Start}, {r.bottom, PointRole::End}}, "40"));
    const LineGeom before = lineOf(s, r.bottom);
    SketchSolveReport rep = diagnoseSketch(s);
    EXPECT_TRUE(rep.ok());
    EXPECT_EQ(lineOf(s, r.bottom).end, before.end);
    EXPECT_NEAR(lineOf(s, r.bottom).end.x - lineOf(s, r.bottom).start.x, before.end.x - before.start.x, kTol);
    // The rough geometry was not solved: the bottom line still has its original length.
    EXPECT_NEAR(before.end.x - before.start.x, 40.6, 0.2);
}

TEST(SolverBridge, ParameterDimensionUsesTable) {
    ParameterTable params;
    ASSERT_TRUE(params.add("d1", "10 mm").has_value());

    Sketch s = makeSketch();
    auto a = must(s.addPoint({0, 0}));
    auto b = must(s.addPoint({7, 0}));
    must(s.addDimension(DimensionKind::Distance, {{a, std::nullopt}, {b, std::nullopt}}, "d1 * 2"));
    // A point is referenced via its entity id; Start is the point's position.
    SketchSolveReport rep = solveSketch(s, &params);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    const Point2 pa = must(s.pointOf({a, std::nullopt}));
    const Point2 pb = must(s.pointOf({b, std::nullopt}));
    EXPECT_NEAR(distance(pa, pb), 20.0, kTol);
}

TEST(SolverBridge, ParameterReferenceWithoutTableIsInvalidInput) {
    Sketch s = makeSketch();
    auto a = must(s.addPoint({0, 0}));
    auto b = must(s.addPoint({7, 0}));
    must(s.addDimension(DimensionKind::Distance, {{a, std::nullopt}, {b, std::nullopt}}, "d1"));
    SketchSolveReport rep = solveSketch(s);
    EXPECT_EQ(rep.status, SolverStatus::InvalidInput);
    EXPECT_FALSE(rep.message.empty());
    // The sketch is untouched.
    EXPECT_EQ(must(s.pointOf({b, std::nullopt})).x, 7.0);
}

TEST(SolverBridge, DimensionDimensionMismatchIsInvalidInput) {
    Sketch s = makeSketch();
    auto a = must(s.addPoint({0, 0}));
    auto b = must(s.addPoint({7, 0}));
    must(s.addDimension(DimensionKind::Distance, {{a, std::nullopt}, {b, std::nullopt}}, "10 deg"));
    SketchSolveReport rep = solveSketch(s);
    EXPECT_EQ(rep.status, SolverStatus::InvalidInput);
    EXPECT_NE(rep.message.find("expected length"), std::string::npos) << rep.message;
}

TEST(SolverBridge, AngleDimensionBetweenLines) {
    Sketch s = makeSketch();
    auto l1 = must(s.addLine({0, 0}, {10, 0}));
    auto l2 = must(s.addLine({0, 0}, {7, 7}));
    must(s.addConstraint(ConstraintKind::Coincident, {{l1, PointRole::Start}, {l2, PointRole::Start}}));
    must(s.addDimension(DimensionKind::Angle, {{l1, std::nullopt}, {l2, std::nullopt}}, "30 deg"));
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    const LineGeom& a = lineOf(s, l1);
    const LineGeom& b = lineOf(s, l2);
    const double ang1 = std::atan2(a.end.y - a.start.y, a.end.x - a.start.x);
    const double ang2 = std::atan2(b.end.y - b.start.y, b.end.x - b.start.x);
    EXPECT_NEAR(std::abs(ang2 - ang1) * 180.0 / std::numbers::pi, 30.0, 1e-4);
}

TEST(SolverBridge, RadiusDimensionWritesBackArc) {
    Sketch s = makeSketch();
    auto arc = must(s.addArc({0, 0}, 9.0, 0.0, std::numbers::pi / 2));
    must(s.addDimension(DimensionKind::Radius, {{arc, std::nullopt}}, "4 mm"));
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    const ArcGeom& a = arcOf(s, arc);
    EXPECT_NEAR(a.radius, 4.0, kTol);
    // The end point is free (no constraint pins the sweep): it must still lie on the solved circle.
    const Point2 end = arcEnd(a);
    EXPECT_NEAR(distance(end, a.center), 4.0, kTol);
}

TEST(SolverBridge, DiameterDimensionOnCircle) {
    Sketch s = makeSketch();
    auto c = must(s.addCircle({1, 1}, 3.0));
    must(s.addDimension(DimensionKind::Diameter, {{c, std::nullopt}}, "10 mm"));
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    EXPECT_NEAR(circleOf(s, c).radius, 5.0, kTol);
}

TEST(SolverBridge, ConcentricAndEqualCircles) {
    Sketch s = makeSketch();
    auto c1 = must(s.addCircle({0, 0}, 5.0));
    auto c2 = must(s.addCircle({3, 2}, 8.0));
    must(s.addConstraint(ConstraintKind::Concentric, {{c1, std::nullopt}, {c2, std::nullopt}}));
    must(s.addConstraint(ConstraintKind::Equal, {{c1, std::nullopt}, {c2, std::nullopt}}));
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    EXPECT_TRUE(near(circleOf(s, c1).center, circleOf(s, c2).center));
    EXPECT_NEAR(circleOf(s, c1).radius, circleOf(s, c2).radius, kTol);
}

TEST(SolverBridge, MidpointAndFixAndDistanceAreKept) {
    Sketch s = makeSketch();
    auto line = must(s.addLine({0, 0}, {10, 0}));
    auto p = must(s.addPoint({4, 3}));
    must(s.addConstraint(ConstraintKind::Midpoint, {{p, std::nullopt}, {line, std::nullopt}}));
    must(s.addConstraint(ConstraintKind::Fix, {{line, std::nullopt}}));
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    // Line is fixed at (0,0)-(10,0): its midpoint (5,0) pulls the point onto it.
    const Point2 pp = must(s.pointOf({p, std::nullopt}));
    EXPECT_NEAR(pp.x, 5.0, kTol);
    EXPECT_NEAR(pp.y, 0.0, kTol);
    EXPECT_TRUE(near(lineOf(s, line).start, {0, 0}));
    EXPECT_TRUE(near(lineOf(s, line).end, {10, 0}));
}

TEST(SolverBridge, ConflictingDimensionsReportDimensionRecords) {
    Sketch s = makeSketch();
    Rect r = addRoughRect(s);
    auto d1 = must(s.addDimension(DimensionKind::DistanceX, {{r.bottom, PointRole::Start}, {r.bottom, PointRole::End}}, "40"));
    auto d2 = must(s.addDimension(DimensionKind::DistanceX, {{r.top, PointRole::Start}, {r.top, PointRole::End}}, "50"));
    const LineGeom before = lineOf(s, r.bottom);

    SketchSolveReport rep = solveSketch(s);
    EXPECT_EQ(rep.status, SolverStatus::Failed);
    EXPECT_FALSE(rep.ok());
    EXPECT_FALSE(rep.conflictingDimensions.empty());
    // The two X-extent dimensions are in the conflict; the rectangle's H/V/coincident records are part of the
    // minimal explanation too (the verticals tie both extents together), so they may be listed as well.
    EXPECT_TRUE(std::find(rep.conflictingDimensions.begin(), rep.conflictingDimensions.end(), d1) !=
                rep.conflictingDimensions.end());
    EXPECT_TRUE(std::find(rep.conflictingDimensions.begin(), rep.conflictingDimensions.end(), d2) !=
                rep.conflictingDimensions.end());
    EXPECT_FALSE(rep.conflictingConstraints.empty());
    // Failed solve: geometry untouched.
    EXPECT_EQ(lineOf(s, r.bottom).end, before.end);
}

TEST(SolverBridge, RedundantHorizontalReportsConstraintRecord) {
    Sketch s = makeSketch();
    auto line = must(s.addLine({0, 0}, {10, 0}));
    auto h1 = must(s.addConstraint(ConstraintKind::Horizontal, {{line, std::nullopt}}));
    auto h2 = must(s.addConstraint(ConstraintKind::Horizontal, {{line, std::nullopt}}));
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    const bool reported = std::find(rep.redundantConstraints.begin(), rep.redundantConstraints.end(), h2) !=
                              rep.redundantConstraints.end() ||
                          std::find(rep.redundantConstraints.begin(), rep.redundantConstraints.end(), h1) !=
                              rep.redundantConstraints.end();
    EXPECT_TRUE(reported);
}

TEST(SolverBridge, ExpressionErrorIsReportedNotWritten) {
    Sketch s = makeSketch();
    auto a = must(s.addPoint({0, 0}));
    auto b = must(s.addPoint({7, 0}));
    must(s.addDimension(DimensionKind::Distance, {{a, std::nullopt}, {b, std::nullopt}}, "3 + * 2"));
    SketchSolveReport rep = solveSketch(s);
    EXPECT_EQ(rep.status, SolverStatus::InvalidInput);
    EXPECT_NE(rep.message.find("dimension"), std::string::npos) << rep.message;
}

// F20-sym: symmetric about a line with a fixed point and a fixed axis is consistent and fully determined.
TEST(SolverBridge, SymmetricAboutAFixedLineMirrorsAndHasNoConflict) {
    Sketch s = makeSketch();
    auto axis = must(s.addLine({0, -10}, {0, 10}));
    auto a = must(s.addPoint({3, 4}));
    auto b = must(s.addPoint({-2, 6}));
    must(s.addConstraint(ConstraintKind::Fix, {{axis, std::nullopt}}));
    must(s.addConstraint(ConstraintKind::Fix, {{a, std::nullopt}}));
    must(s.addConstraint(ConstraintKind::Symmetric, {{a, std::nullopt}, {b, std::nullopt}, {axis, std::nullopt}}));
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    EXPECT_EQ(rep.dof, 0);
    EXPECT_TRUE(rep.conflictingConstraints.empty());
    const auto q = must(s.pointOf({b, std::nullopt}));
    EXPECT_NEAR(q.x, -3.0, kTol);
    EXPECT_NEAR(q.y, 4.0, kTol);
}

// ---- F47: Fix on a line endpoint pins that endpoint only -------------------------------------------------------

// Regression: Fix on the bottom-left corner (bottom line, role Start) + H/V + two dimensions was reported redundant and
// then conflicting, because the adapter pinned BOTH endpoints of the line (the role was ignored). Pinning the sibling
// endpoint made the horizontal constraint redundant and the width dimension conflict with its pinned coordinate.
TEST(SolverBridge, FixOnLineEndpointPinsOnlyThatEndpoint) {
    Sketch s = makeSketch();
    Rect r = addRoughRect(s);
    must(s.addConstraint(ConstraintKind::Fix, {{r.bottom, PointRole::Start}}));
    auto dw = must(s.addDimension(DimensionKind::DistanceX, {{r.bottom, PointRole::Start}, {r.bottom, PointRole::End}}, "40"));
    auto dh = must(s.addDimension(DimensionKind::DistanceY, {{r.left, PointRole::Start}, {r.left, PointRole::End}}, "20"));
    (void)dw;
    (void)dh;
    const Point2 corner0 = lineOf(s, r.bottom).start;

    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    EXPECT_TRUE(rep.redundantConstraints.empty());
    EXPECT_TRUE(rep.redundantDimensions.empty());
    EXPECT_TRUE(rep.conflictingConstraints.empty());
    EXPECT_TRUE(rep.conflictingDimensions.empty());
    EXPECT_EQ(rep.dof, 0);
    EXPECT_TRUE(near(lineOf(s, r.bottom).start, corner0));  // the pinned endpoint did not move
    EXPECT_NEAR(distance(lineOf(s, r.bottom).start, lineOf(s, r.bottom).end), 40.0, kTol);

    // Editing a dimension must not conflict with the pinned corner: the width changes, the corner stays.
    ASSERT_TRUE(s.setDimensionExpression(dw, "60").has_value());
    SketchSolveReport edited = solveSketch(s);
    ASSERT_EQ(edited.status, SolverStatus::Solved) << edited.message;
    EXPECT_TRUE(near(lineOf(s, r.bottom).start, corner0));
    EXPECT_NEAR(lineOf(s, r.bottom).end.x - corner0.x, 60.0, kTol);
}

// Same constraints with the fix on a sub-point of a point entity: the existing working form, kept as a control.
TEST(SolverBridge, FixOnPointEntityAndLineEndpointAgree) {
    Sketch s = makeSketch();
    auto p = must(s.addPoint({0.4, -0.2}));
    Rect r = addRoughRect(s);
    must(s.addConstraint(ConstraintKind::Coincident, {{p, std::nullopt}, {r.bottom, PointRole::Start}}));
    must(s.addConstraint(ConstraintKind::Fix, {{p, std::nullopt}}));
    must(s.addDimension(DimensionKind::DistanceX, {{r.bottom, PointRole::Start}, {r.bottom, PointRole::End}}, "40"));
    must(s.addDimension(DimensionKind::DistanceY, {{r.left, PointRole::Start}, {r.left, PointRole::End}}, "20"));
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    EXPECT_EQ(rep.dof, 0);
    EXPECT_TRUE(rep.redundantConstraints.empty());
}

// Fix on a whole line (no sub-point) still pins both endpoints, as before.
TEST(SolverBridge, FixOnWholeLinePinsBothEndpoints) {
    Sketch s = makeSketch();
    auto line = must(s.addLine({0, 0}, {10, 0.5}));
    must(s.addConstraint(ConstraintKind::Fix, {{line, std::nullopt}}));
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    EXPECT_EQ(rep.dof, 0);
    EXPECT_TRUE(near(lineOf(s, line).start, {0, 0}));
    EXPECT_TRUE(near(lineOf(s, line).end, {10, 0.5}));
}

// ---- F46: reference geometry and minimal-movement solves --------------------------------------------------------

namespace {

// Corner centroid of the rough rectangle (its four line endpoints, averaged).
Point2 centroidOf(const Sketch& s, const Rect& r) {
    const Point2 pts[] = {lineOf(s, r.bottom).start, lineOf(s, r.right).start, lineOf(s, r.top).start,
                          lineOf(s, r.left).start};
    Point2 c{};
    for (const Point2& p : pts) c = c + p;
    return c * 0.25;
}

// Rectangle that is dimensioned but not anchored: H/V and a width and height dimension, no position constraint.
Rect dimensionedRect(Sketch& s, SketchDimensionId& width) {
    Rect r = addRoughRect(s);
    width = must(s.addDimension(DimensionKind::DistanceX, {{r.bottom, PointRole::Start}, {r.bottom, PointRole::End}}, "40"));
    must(s.addDimension(DimensionKind::DistanceY, {{r.left, PointRole::Start}, {r.left, PointRole::End}}, "20"));
    return r;
}

}  // namespace

// Regression (drift): a parameter edit on a dimension-only rectangle must not translate it. Anchored to the origin,
// the bottom-left corner stays on (0, 0) and only the far edge moves.
TEST(SolverBridge, DimensionOnlyRectangleAnchoredToOriginDoesNotDrift) {
    Sketch s = makeSketch();
    SketchDimensionId width{};
    Rect r = dimensionedRect(s, width);
    must(s.addConstraint(ConstraintKind::Coincident, {{r.bottom, PointRole::Start}, {kOriginPointId, std::nullopt}}));
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    EXPECT_EQ(rep.dof, 0);
    EXPECT_TRUE(near(lineOf(s, r.bottom).start, {0, 0}));

    ASSERT_TRUE(s.setDimensionExpression(width, "60").has_value());
    SketchSolveReport edited = solveSketch(s);
    ASSERT_EQ(edited.status, SolverStatus::Solved) << edited.message;
    EXPECT_TRUE(near(lineOf(s, r.bottom).start, {0, 0}, kTol));
    EXPECT_NEAR(lineOf(s, r.bottom).end.x, 60.0, kTol);
    EXPECT_NEAR(lineOf(s, r.right).start.x, 60.0, kTol);
}

// Regression (drift): an UNanchored dimension-only rectangle has free translation. A parameter edit must change its
// size in place: the centroid stays put (minimal change from the current positions), it does not jump to a new place.
TEST(SolverBridge, UnanchoredDimensionOnlyRectangleStaysPutOnParameterEdit) {
    Sketch s = makeSketch();
    SketchDimensionId width{};
    Rect r = dimensionedRect(s, width);
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    EXPECT_GT(rep.dof, 0);  // under-constrained: translation is free
    const Point2 c0 = centroidOf(s, r);

    ASSERT_TRUE(s.setDimensionExpression(width, "60").has_value());
    SketchSolveReport edited = solveSketch(s);
    ASSERT_EQ(edited.status, SolverStatus::Solved) << edited.message;
    EXPECT_NEAR(distance(lineOf(s, r.bottom).start, lineOf(s, r.bottom).end), 60.0, kTol);
    const Point2 c1 = centroidOf(s, r);
    EXPECT_NEAR(c1.x, c0.x, kTol);
    EXPECT_NEAR(c1.y, c0.y, kTol);
}

// Fusion parity: the origin is a fixed construction point and the axes are fixed construction lines, so a sketch can
// snap to them. Coincident with the origin pins a corner and removes the translation DOF.
TEST(SolverBridge, OriginAndAxesAreFixedAndSnappable) {
    Sketch s = makeSketch();
    SketchDimensionId width{};
    Rect r = dimensionedRect(s, width);
    must(s.addConstraint(ConstraintKind::PointOnObject, {{r.bottom, PointRole::Start}, {kXAxisId, std::nullopt}}));
    must(s.addConstraint(ConstraintKind::PointOnObject, {{r.bottom, PointRole::Start}, {kYAxisId, std::nullopt}}));
    SketchSolveReport rep = solveSketch(s);
    ASSERT_EQ(rep.status, SolverStatus::Solved) << rep.message;
    EXPECT_EQ(rep.dof, 0);
    EXPECT_TRUE(near(lineOf(s, r.bottom).start, {0, 0}));
    // Reference geometry itself never moves.
    EXPECT_TRUE(near(std::get<PointGeom>(s.findEntity(kOriginPointId)->geometry).p, {0, 0}));
    EXPECT_NEAR(lineOf(s, kXAxisId).start.x, -kReferenceAxisHalfLength, kTol);
}

// Under-constrained: DOF > 0 is reported (the feature layer turns it into a warning).
TEST(SolverBridge, UnanchoredRectangleReportsPositiveDof) {
    Sketch s = makeSketch();
    SketchDimensionId width{};
    dimensionedRect(s, width);
    SketchSolveReport rep = diagnoseSketch(s);
    EXPECT_GT(rep.dof, 0);
}
