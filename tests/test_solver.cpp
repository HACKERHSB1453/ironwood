// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include "sketch/solver/sketch_solver.hpp"

namespace iw::sketch {
namespace {

// Entity and constraint ids used by the fixtures below.
constexpr SolverId kP0 = 1, kP1 = 2, kP2 = 3, kP3 = 4;
constexpr SolverId kL0 = 11, kL1 = 12, kL2 = 13, kL3 = 14;  // bottom, right, top, left

SolverRef ref(SolverId entity, SolverRole role = SolverRole::Start)
{
    return SolverRef{entity, role};
}

SolverConstraint con(SolverId id, SolverConstraintKind kind, std::vector<SolverRef> refs, double value = 0.0)
{
    SolverConstraint c;
    c.id = id;
    c.kind = kind;
    c.refs = std::move(refs);
    c.value = value;
    return c;
}

// Axis-aligned rectangle with deliberately rough initial coordinates. Corners: p0 bottom-left,
// p1 bottom-right, p2 top-right, p3 top-left. Lines: l0 bottom, l1 right, l2 top, l3 left.
SolverSketch rectangle()
{
    SolverSketch s;
    s.points = {{kP0, 0.5, -0.3}, {kP1, 41.0, 0.4}, {kP2, 39.2, 21.1}, {kP3, -0.6, 19.4}};
    s.lines = {{kL0, kP0, kP1}, {kL1, kP1, kP2}, {kL2, kP2, kP3}, {kL3, kP3, kP0}};
    return s;
}

const SolverPoint& pointById(const SolverSketch& s, SolverId id)
{
    return *std::find_if(s.points.begin(), s.points.end(), [id](const SolverPoint& p) { return p.id == id; });
}

constexpr double kTol = 1e-6;

}  // namespace

TEST(SketchSolver, FullyConstrainedRectangleHasZeroDof)
{
    SolverSketch s = rectangle();
    s.points[0] = {kP0, 0.0, 0.0};  // Fix pins the initial position, so start the anchor at the origin
    s.constraints = {
        con(100, SolverConstraintKind::Horizontal, {ref(kL0)}),
        con(101, SolverConstraintKind::Vertical, {ref(kL1)}),
        con(102, SolverConstraintKind::Horizontal, {ref(kL2)}),
        con(103, SolverConstraintKind::Vertical, {ref(kL3)}),
        con(104, SolverConstraintKind::Fix, {ref(kP0)}),
        con(105, SolverConstraintKind::DistanceX, {ref(kP0), ref(kP1)}, 40.0),
        con(106, SolverConstraintKind::DistanceY, {ref(kP0), ref(kP3)}, 20.0),
    };
    SketchSolver solver(s);

    EXPECT_EQ(solver.dof(), 0);
    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    EXPECT_EQ(r.diagnostics.dof, 0);
    EXPECT_TRUE(r.diagnostics.conflicting.empty());
    EXPECT_TRUE(r.diagnostics.redundant.empty());

    const SolverSketch& out = solver.sketch();
    EXPECT_NEAR(pointById(out, kP0).x, 0.0, kTol);
    EXPECT_NEAR(pointById(out, kP0).y, 0.0, kTol);
    EXPECT_NEAR(pointById(out, kP1).x, 40.0, kTol);
    EXPECT_NEAR(pointById(out, kP1).y, 0.0, kTol);
    EXPECT_NEAR(pointById(out, kP2).x, 40.0, kTol);
    EXPECT_NEAR(pointById(out, kP2).y, 20.0, kTol);
    EXPECT_NEAR(pointById(out, kP3).x, 0.0, kTol);
    EXPECT_NEAR(pointById(out, kP3).y, 20.0, kTol);
}

TEST(SketchSolver, UnconstrainedRectangleReportsDof)
{
    SolverSketch s = rectangle();
    s.constraints = {
        con(100, SolverConstraintKind::Horizontal, {ref(kL0)}),
        con(101, SolverConstraintKind::Vertical, {ref(kL1)}),
        con(102, SolverConstraintKind::Horizontal, {ref(kL2)}),
        con(103, SolverConstraintKind::Vertical, {ref(kL3)}),
    };
    SketchSolver solver(s);
    // 4 points = 8 coordinates, 4 independent H/V equations.
    EXPECT_EQ(solver.dof(), 4);
    EXPECT_EQ(solver.solve().status, SolverStatus::Solved);
}

TEST(SketchSolver, DragCornerKeepsHorizontalAndVertical)
{
    SolverSketch s = rectangle();
    s.constraints = {
        con(100, SolverConstraintKind::Horizontal, {ref(kL0)}),
        con(101, SolverConstraintKind::Vertical, {ref(kL1)}),
        con(102, SolverConstraintKind::Horizontal, {ref(kL2)}),
        con(103, SolverConstraintKind::Vertical, {ref(kL3)}),
    };
    SketchSolver solver(s);

    const SolverResult r = solver.dragPoint(ref(kP2), 50.0, 30.0);
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;

    const SolverSketch& out = solver.sketch();
    const SolverPoint& p0 = pointById(out, kP0);
    const SolverPoint& p1 = pointById(out, kP1);
    const SolverPoint& p2 = pointById(out, kP2);
    const SolverPoint& p3 = pointById(out, kP3);
    EXPECT_NEAR(p2.x, 50.0, kTol);
    EXPECT_NEAR(p2.y, 30.0, kTol);
    EXPECT_NEAR(p0.y, p1.y, kTol);  // bottom stays horizontal
    EXPECT_NEAR(p2.y, p3.y, kTol);  // top stays horizontal
    EXPECT_NEAR(p1.x, p2.x, kTol);  // right side stays vertical
    EXPECT_NEAR(p0.x, p3.x, kTol);  // left side stays vertical
}

TEST(SketchSolver, DragBeyondConstraintsStopsAtNearestConsistentPosition)
{
    SolverSketch s = rectangle();
    s.points[0] = {kP0, 0.0, 0.0};
    s.constraints = {
        con(100, SolverConstraintKind::Horizontal, {ref(kL0)}),
        con(101, SolverConstraintKind::Vertical, {ref(kL1)}),
        con(102, SolverConstraintKind::Horizontal, {ref(kL2)}),
        con(103, SolverConstraintKind::Vertical, {ref(kL3)}),
        con(104, SolverConstraintKind::Fix, {ref(kP0)}),
        con(105, SolverConstraintKind::DistanceX, {ref(kP0), ref(kP1)}, 40.0),
        con(106, SolverConstraintKind::DistanceY, {ref(kP0), ref(kP3)}, 20.0),
    };
    SketchSolver solver(s);

    // The fully constrained rectangle cannot move, so the drag ends where the constraints allow.
    const SolverResult r = solver.dragPoint(ref(kP2), 60.0, 60.0);
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    const SolverSketch& out = solver.sketch();
    EXPECT_NEAR(pointById(out, kP2).x, 40.0, kTol);
    EXPECT_NEAR(pointById(out, kP2).y, 20.0, kTol);
    EXPECT_NEAR(pointById(out, kP1).x - pointById(out, kP0).x, 40.0, kTol);
    EXPECT_NEAR(pointById(out, kP3).y - pointById(out, kP0).y, 20.0, kTol);
}

TEST(SketchSolver, ConflictingDimensionsReportExactlyTheConflictingSet)
{
    SolverSketch s = rectangle();
    s.constraints = {
        con(100, SolverConstraintKind::Horizontal, {ref(kL0)}),
        con(101, SolverConstraintKind::Vertical, {ref(kL1)}),
        con(102, SolverConstraintKind::Horizontal, {ref(kL2)}),
        con(103, SolverConstraintKind::Vertical, {ref(kL3)}),
        con(104, SolverConstraintKind::Fix, {ref(kP0)}),
        con(200, SolverConstraintKind::DistanceX, {ref(kP0), ref(kP1)}, 40.0),
        con(201, SolverConstraintKind::DistanceX, {ref(kP0), ref(kP1)}, 55.0),  // conflicts with 200
    };
    SketchSolver solver(s);

    const SolverResult r = solver.solve();
    EXPECT_EQ(r.status, SolverStatus::Failed);
    EXPECT_EQ(r.diagnostics.conflicting, (std::vector<SolverId>{200, 201}));
    EXPECT_TRUE(r.diagnostics.redundant.empty());
}

TEST(SketchSolver, RedundantHorizontalOnHorizontalLineIsReported)
{
    SolverSketch s = rectangle();
    s.constraints = {
        con(100, SolverConstraintKind::Horizontal, {ref(kL0)}),
        con(101, SolverConstraintKind::Vertical, {ref(kL1)}),
        con(102, SolverConstraintKind::Horizontal, {ref(kL2)}),
        con(103, SolverConstraintKind::Vertical, {ref(kL3)}),
        con(110, SolverConstraintKind::Horizontal, {ref(kL0)}),  // already implied by 100
    };
    SketchSolver solver(s);

    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    EXPECT_TRUE(r.diagnostics.conflicting.empty());
    EXPECT_EQ(r.diagnostics.redundant, (std::vector<SolverId>{110}));  // the later duplicate
}

TEST(SketchSolver, TangentLineToCircleWithRadiusAndCentreConstraints)
{
    // Circle centre fixed at (0, 0) with radius 10; a line that must be tangent to it and whose
    // start point is pinned to (-10, 15). Its end point is free.
    constexpr SolverId kC = 21, kCenter = 5, kA = 6, kB = 7, kLine = 22;
    SolverSketch s;
    s.points = {{kCenter, 0.0, 0.0}, {kA, -10.0, 15.0}, {kB, 12.0, -4.0}};
    s.circles = {{kC, kCenter, 9.0}};  // initial radius deliberately off
    s.lines = {{kLine, kA, kB}};
    s.constraints = {
        con(300, SolverConstraintKind::Fix, {ref(kCenter)}),
        con(301, SolverConstraintKind::Radius, {ref(kC)}, 10.0),
        con(302, SolverConstraintKind::Tangent, {ref(kLine), ref(kC)}),
        con(303, SolverConstraintKind::Fix, {ref(kA)}),
    };
    SketchSolver solver(s);

    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    EXPECT_NEAR(solver.sketch().circles[0].radius, 10.0, kTol);
    // Distance from the centre to the line equals the radius.
    const SolverSketch& out = solver.sketch();
    const SolverPoint& a = pointById(out, kA);
    const SolverPoint& b = pointById(out, kB);
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dist = std::abs(dx * (a.y - 0.0) - dy * (a.x - 0.0)) / std::hypot(dx, dy);
    EXPECT_NEAR(dist, 10.0, 1e-5);
}

TEST(SketchSolver, MidpointPerpendicularAndDistance)
{
    // X is fixed from (-5,0) to (5,0). A is the midpoint of X, so A = (0,0). Line L runs from A to
    // B, is perpendicular to X and has length 5, which puts B at (0, +-5). The start guess picks +5.
    constexpr SolverId kA = 31, kB = 32, kX0 = 33, kX1 = 34, kX = 35, kL = 36;
    SolverSketch s;
    s.points = {{kA, 1.0, 1.0}, {kB, 3.1, 4.4}, {kX0, -5.0, 0.0}, {kX1, 5.0, 0.0}};
    s.lines = {{kX, kX0, kX1}, {kL, kA, kB}};
    s.constraints = {
        con(400, SolverConstraintKind::Midpoint, {ref(kA), ref(kX)}),
        con(401, SolverConstraintKind::Perpendicular, {ref(kX), ref(kL)}),
        con(402, SolverConstraintKind::Distance, {ref(kA), ref(kB)}, 5.0),
        con(403, SolverConstraintKind::Fix, {ref(kX0)}),
        con(404, SolverConstraintKind::Fix, {ref(kX1)}),
    };
    SketchSolver solver(s);
    EXPECT_EQ(solver.dof(), 0);

    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    const SolverSketch& out = solver.sketch();
    const SolverPoint& a = pointById(out, kA);
    const SolverPoint& b = pointById(out, kB);
    EXPECT_NEAR(a.x, 0.0, kTol);
    EXPECT_NEAR(a.y, 0.0, kTol);
    EXPECT_NEAR(b.x, 0.0, kTol);
    EXPECT_NEAR(std::abs(b.y), 5.0, kTol);
}

TEST(SketchSolver, InvalidInputIsReportedNotThrown)
{
    SolverSketch s = rectangle();
    s.constraints = {con(1, SolverConstraintKind::Horizontal, {ref(9999)})};
    SketchSolver solver(s);
    const SolverResult r = solver.solve();
    EXPECT_EQ(r.status, SolverStatus::InvalidInput);
    EXPECT_FALSE(r.message.empty());
    EXPECT_EQ(solver.dof(), -1);

    SolverSketch wrongArity = rectangle();
    wrongArity.constraints = {con(2, SolverConstraintKind::Distance, {ref(kP0)}, 3.0)};
    EXPECT_EQ(SketchSolver(wrongArity).solve().status, SolverStatus::InvalidInput);
}

// 21 x 21 point grid with horizontal and vertical edges: 441 points + 840 lines = 1281 entities.
// Fully constrained: a fixed origin, 40 spacing dimensions on row 0 and column 0.
TEST(SketchSolver, LargeGridSolvesAfterSingleDimensionEdit)
{
    constexpr int kN = 21;
    SolverSketch s;
    auto pid = [](int i, int j) { return static_cast<SolverId>(1000 + i * kN + j); };
    auto lid = [](int k) { return static_cast<SolverId>(100000 + k); };
    for (int i = 0; i < kN; ++i) {
        for (int j = 0; j < kN; ++j) {
            // Deliberately perturbed initial positions.
            const double jitter = 0.05 * std::sin(3.0 * i + 7.0 * j);
            s.points.push_back({pid(i, j), 10.0 * j + jitter, 10.0 * i - jitter});
        }
    }
    SolverId cid = 1;
    int lineCount = 0;
    for (int i = 0; i < kN; ++i) {
        for (int j = 0; j < kN; ++j) {
            if (j + 1 < kN) {
                s.lines.push_back({lid(lineCount++), pid(i, j), pid(i, j + 1)});
                s.constraints.push_back(con(cid++, SolverConstraintKind::Horizontal, {ref(lid(lineCount - 1))}));
            }
            if (i + 1 < kN) {
                s.lines.push_back({lid(lineCount++), pid(i, j), pid(i + 1, j)});
                s.constraints.push_back(con(cid++, SolverConstraintKind::Vertical, {ref(lid(lineCount - 1))}));
            }
        }
    }
    s.constraints.push_back(con(cid++, SolverConstraintKind::Fix, {ref(pid(0, 0))}));
    std::vector<SolverId> dimIds;
    for (int j = 0; j + 1 < kN; ++j) {
        dimIds.push_back(cid);
        s.constraints.push_back(con(cid++, SolverConstraintKind::DistanceX, {ref(pid(0, j)), ref(pid(0, j + 1))}, 10.0));
    }
    for (int i = 0; i + 1 < kN; ++i) {
        s.constraints.push_back(con(cid++, SolverConstraintKind::DistanceY, {ref(pid(i, 0)), ref(pid(i + 1, 0))}, 10.0));
    }
    ASSERT_EQ(lineCount, 840);

    SketchSolver solver(s);
    ASSERT_EQ(solver.solve().status, SolverStatus::Solved);
    EXPECT_EQ(solver.dof(), 0);

    // Single-dimension edit: widen the first column spacing; then time the re-solve.
    SolverSketch& model = solver.sketch();
    for (SolverConstraint& c : model.constraints) {
        if (c.id == dimIds[0]) {
            c.value = 12.0;
        }
    }
    const auto t0 = std::chrono::steady_clock::now();
    const SolverResult r = solver.solve();
    const auto t1 = std::chrono::steady_clock::now();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;

    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    RecordProperty("resolve_ms", std::to_string(ms));
    // Acceptance budget is for release builds only; the debug build still checks the result.
#ifdef NDEBUG
    EXPECT_LT(ms, 50.0);
#endif
    EXPECT_NEAR(pointById(solver.sketch(), pid(0, 1)).x, 12.0, 1e-5);
    EXPECT_NEAR(pointById(solver.sketch(), pid(0, kN - 1)).x, 12.0 + 10.0 * (kN - 2), 1e-4);
}

// ---- F20: solver-level tests for the constraint kinds the UI exercises, plus PointOnObject (F29) ----

TEST(SketchSolver, CoincidentMovesSecondPointOntoFirst)
{
    constexpr SolverId kA = 501, kB = 502;
    SolverSketch s;
    s.points = {{kA, 5.0, 5.0}, {kB, 1.0, 2.0}};
    s.constraints = {
        con(10, SolverConstraintKind::Fix, {ref(kA)}),
        con(11, SolverConstraintKind::Coincident, {ref(kA), ref(kB)}),
    };
    SketchSolver solver(s);
    EXPECT_EQ(solver.dof(), 0);
    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    EXPECT_NEAR(pointById(solver.sketch(), kB).x, 5.0, kTol);
    EXPECT_NEAR(pointById(solver.sketch(), kB).y, 5.0, kTol);
}

TEST(SketchSolver, ParallelMakesSecondLineMatchFirstDirection)
{
    constexpr SolverId kA = 511, kB = 512, kC = 513, kD = 514, kL1 = 515, kL2 = 516;
    SolverSketch s;
    s.points = {{kA, 0.0, 0.0}, {kB, 10.0, 0.0}, {kC, 1.0, 5.0}, {kD, 8.0, 9.0}};
    s.lines = {{kL1, kA, kB}, {kL2, kC, kD}};
    s.constraints = {
        con(20, SolverConstraintKind::Fix, {ref(kA)}),
        con(21, SolverConstraintKind::Fix, {ref(kB)}),
        con(22, SolverConstraintKind::Fix, {ref(kC)}),
        con(23, SolverConstraintKind::Parallel, {ref(kL1), ref(kL2)}),
    };
    SketchSolver solver(s);
    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    // Line 2 keeps its start; its end moves onto the horizontal direction of line 1.
    EXPECT_NEAR(pointById(solver.sketch(), kD).y, 5.0, kTol);
}

TEST(SketchSolver, EqualLengthLinesAndEqualRadiiCircles)
{
    constexpr SolverId kA = 521, kB = 522, kC = 523, kD = 524, kL1 = 525, kL2 = 526;
    constexpr SolverId kCenter1 = 527, kCenter2 = 528, kCirc1 = 529, kCirc2 = 530;
    SolverSketch s;
    s.points = {{kA, 0.0, 0.0}, {kB, 10.0, 0.0}, {kC, 0.0, 5.0}, {kD, 3.0, 5.5},
                {kCenter1, 0.0, 0.0}, {kCenter2, 20.0, 0.0}};
    s.lines = {{kL1, kA, kB}, {kL2, kC, kD}};
    s.circles = {{kCirc1, kCenter1, 4.0}, {kCirc2, kCenter2, 7.0}};
    s.constraints = {
        con(30, SolverConstraintKind::Fix, {ref(kA)}),
        con(31, SolverConstraintKind::Fix, {ref(kB)}),
        con(32, SolverConstraintKind::Fix, {ref(kC)}),
        con(33, SolverConstraintKind::Equal, {ref(kL1), ref(kL2)}),
        con(34, SolverConstraintKind::Fix, {ref(kCenter1)}),
        con(35, SolverConstraintKind::Fix, {ref(kCenter2)}),
        con(36, SolverConstraintKind::Equal, {ref(kCirc1), ref(kCirc2)}),
        con(37, SolverConstraintKind::Radius, {ref(kCirc1)}, 4.0),
    };
    SketchSolver solver(s);
    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    const SolverSketch& out = solver.sketch();
    const SolverPoint& c = pointById(out, kC);
    const SolverPoint& d = pointById(out, kD);
    EXPECT_NEAR(std::hypot(d.x - c.x, d.y - c.y), 10.0, 1e-5);
    EXPECT_NEAR(out.circles[1].radius, 4.0, 1e-5);
}

TEST(SketchSolver, ConcentricCirclesShareCentre)
{
    constexpr SolverId kCen1 = 543, kCen2 = 544, kCirc1 = 545, kCirc2 = 546;
    SolverSketch s;
    s.points = {{kCen1, 0.0, 0.0}, {kCen2, 2.0, 3.0}};
    s.circles = {{kCirc1, kCen1, 5.0}, {kCirc2, kCen2, 2.0}};
    s.constraints = {
        con(40, SolverConstraintKind::Fix, {ref(kCen1)}),
        con(41, SolverConstraintKind::Concentric, {ref(kCirc1), ref(kCirc2)}),
        con(42, SolverConstraintKind::Radius, {ref(kCirc2)}, 2.0),
        con(43, SolverConstraintKind::Radius, {ref(kCirc1)}, 5.0),
    };
    SketchSolver solver(s);
    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    EXPECT_NEAR(pointById(solver.sketch(), kCen2).x, 0.0, kTol);
    EXPECT_NEAR(pointById(solver.sketch(), kCen2).y, 0.0, kTol);
}

TEST(SketchSolver, CollinearPullsBothEndsOntoReferenceLine)
{
    constexpr SolverId kA = 551, kB = 552, kC = 553, kD = 554, kL1 = 555, kL2 = 556;
    SolverSketch s;
    s.points = {{kA, 0.0, 0.0}, {kB, 10.0, 0.0}, {kC, 2.0, 1.0}, {kD, 7.0, 4.0}};
    s.lines = {{kL1, kA, kB}, {kL2, kC, kD}};
    s.constraints = {
        con(50, SolverConstraintKind::Fix, {ref(kA)}),
        con(51, SolverConstraintKind::Fix, {ref(kB)}),
        con(52, SolverConstraintKind::Collinear, {ref(kL2), ref(kL1)}),
        con(53, SolverConstraintKind::DistanceX, {ref(kC), ref(kD)}, 5.0),
    };
    SketchSolver solver(s);
    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    EXPECT_NEAR(pointById(solver.sketch(), kC).y, 0.0, kTol);
    EXPECT_NEAR(pointById(solver.sketch(), kD).y, 0.0, kTol);
    EXPECT_NEAR(pointById(solver.sketch(), kD).x - pointById(solver.sketch(), kC).x, 5.0, kTol);
}

TEST(SketchSolver, SymmetricAboutLineAndAboutPoint)
{
    constexpr SolverId kA = 561, kB = 562, kAx0 = 563, kAx1 = 564, kAxis = 565;
    constexpr SolverId kM = 566, kP = 567, kQ = 568;
    SolverSketch s;
    s.points = {{kA, 3.0, 4.0}, {kB, -2.0, 6.0}, {kAx0, 0.0, -10.0}, {kAx1, 0.0, 10.0},
                {kM, 1.0, 1.0}, {kP, 0.5, -0.5}, {kQ, 9.0, 9.0}};
    s.lines = {{kAxis, kAx0, kAx1}};
    s.constraints = {
        con(60, SolverConstraintKind::Fix, {ref(kA)}),
        con(61, SolverConstraintKind::Fix, {ref(kAx0)}),
        con(62, SolverConstraintKind::Fix, {ref(kAx1)}),
        con(63, SolverConstraintKind::Symmetric, {ref(kA), ref(kB), ref(kAxis)}),
    };
    SketchSolver solver(s);
    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    EXPECT_TRUE(r.diagnostics.conflicting.empty());
    EXPECT_EQ(r.diagnostics.dof, 6);   // only the three loose points M, P, Q remain free
    const SolverSketch& out = solver.sketch();
    EXPECT_NEAR(pointById(out, kB).x, -3.0, kTol);  // mirror of (3, 4) across the y axis
    EXPECT_NEAR(pointById(out, kB).y, 4.0, kTol);
}

TEST(SketchSolver, SymmetricAboutPoint)
{
    constexpr SolverId kM = 566, kP = 567, kQ = 568;
    SolverSketch s;
    s.points = {{kM, 1.0, 1.0}, {kP, 0.5, -0.5}, {kQ, 9.0, 9.0}};
    s.constraints = {
        con(64, SolverConstraintKind::Fix, {ref(kP)}),
        con(65, SolverConstraintKind::Fix, {ref(kM)}),
        con(66, SolverConstraintKind::Symmetric, {ref(kP), ref(kQ), ref(kM)}),
    };
    SketchSolver solver(s);
    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    // Q is the reflection of P through M.
    EXPECT_NEAR(pointById(solver.sketch(), kQ).x, 1.5, kTol);
    EXPECT_NEAR(pointById(solver.sketch(), kQ).y, 2.5, kTol);
}

TEST(SketchSolver, AngleBetweenLinesIsEnforced)
{
    constexpr SolverId kA = 571, kB = 572, kC = 573, kD = 574, kL1 = 575, kL2 = 576;
    SolverSketch s;
    s.points = {{kA, 0.0, 0.0}, {kB, 10.0, 0.0}, {kC, 0.0, 0.0}, {kD, 3.0, 3.0}};
    s.lines = {{kL1, kA, kB}, {kL2, kC, kD}};
    s.constraints = {
        con(70, SolverConstraintKind::Fix, {ref(kA)}),
        con(71, SolverConstraintKind::Fix, {ref(kB)}),
        con(72, SolverConstraintKind::Fix, {ref(kC)}),
        con(73, SolverConstraintKind::Distance, {ref(kC), ref(kD)}, 5.0),
        con(74, SolverConstraintKind::Angle, {ref(kL1), ref(kL2)}, 30.0),
    };
    SketchSolver solver(s);
    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    const SolverPoint& d = pointById(solver.sketch(), kD);
    EXPECT_NEAR(std::atan2(d.y, d.x) * 180.0 / std::numbers::pi, 30.0, 1e-5);
    EXPECT_NEAR(std::hypot(d.x, d.y), 5.0, 1e-5);
}

TEST(SketchSolver, DiameterSetsCircleRadiusHalfOfValue)
{
    constexpr SolverId kCen = 581, kCirc = 582;
    SolverSketch s;
    s.points = {{kCen, 1.0, 1.0}};
    s.circles = {{kCirc, kCen, 3.0}};
    s.constraints = {
        con(80, SolverConstraintKind::Fix, {ref(kCen)}),
        con(81, SolverConstraintKind::Diameter, {ref(kCirc)}, 12.0),
    };
    SketchSolver solver(s);
    EXPECT_EQ(solver.dof(), 0);
    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    EXPECT_NEAR(solver.sketch().circles[0].radius, 6.0, kTol);
}

TEST(SketchSolver, PointOnLineCircleAndArc)
{
    constexpr SolverId kLA = 591, kLB = 592, kLine = 593;
    constexpr SolverId kP1 = 594, kP2 = 595, kP3 = 596;
    constexpr SolverId kCen = 597, kCirc = 598, kArcCen = 599, kArc = 600;
    constexpr SolverId kArcStart = 601, kArcEnd = 602;
    SolverSketch s;
    s.points = {{kLA, 0.0, 0.0},     {kLB, 10.0, 0.0},    {kP1, 3.0, 2.0},     {kP2, 3.0, 4.0},
                {kP3, 6.0, 1.0},     {kCen, 0.0, 0.0},    {kArcCen, 20.0, 0.0}, {kArcStart, 22.0, 0.5},
                {kArcEnd, 19.5, 2.2}};
    s.lines = {{kLine, kLA, kLB}};
    s.circles = {{kCirc, kCen, 5.0}};
    s.arcs = {{kArc, kArcCen, kArcStart, kArcEnd}};
    s.constraints = {
        con(90, SolverConstraintKind::Fix, {ref(kLA)}),
        con(91, SolverConstraintKind::Fix, {ref(kLB)}),
        con(92, SolverConstraintKind::Fix, {ref(kCen)}),
        con(93, SolverConstraintKind::Radius, {ref(kCirc)}, 5.0),
        con(94, SolverConstraintKind::PointOnObject, {ref(kP1), ref(kLine)}),
        con(95, SolverConstraintKind::PointOnObject, {ref(kP2), ref(kCirc)}),
        con(96, SolverConstraintKind::PointOnObject, {ref(kP3), ref(kArc)}),
        con(97, SolverConstraintKind::Fix, {ref(kArcCen)}),
        con(98, SolverConstraintKind::Radius, {ref(kArc)}, 2.0),
    };
    SketchSolver solver(s);
    const SolverResult r = solver.solve();
    ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
    const SolverSketch& out = solver.sketch();
    EXPECT_NEAR(pointById(out, kP1).y, 0.0, kTol);  // on the line (the x axis)
    EXPECT_NEAR(std::hypot(pointById(out, kP2).x, pointById(out, kP2).y), 5.0, 1e-5);  // on the circle
    const SolverPoint& p3 = pointById(out, kP3);
    EXPECT_NEAR(std::hypot(p3.x - 20.0, p3.y), 2.0, 1e-5);  // on the arc's circle
}

TEST(SketchSolver, PointOnObjectRejectsPointAsCurve)
{
    SolverSketch s = rectangle();
    s.constraints = {con(1, SolverConstraintKind::PointOnObject, {ref(kP0), ref(kP1)})};
    EXPECT_EQ(SketchSolver(s).solve().status, SolverStatus::InvalidInput);
}

// Drag timing (F29 / B3b): a 200-entity sketch (100 points + 100 lines) on a 10x10 grid with horizontal rows,
// vertical first-column edges, one diagonal line, and a fixed origin. Each drag step must stay under 16 ms in
// release builds. The first drag step builds the drag model and is not part of the budget.
TEST(SketchSolver, DragStepUnder16msFor200EntitySketch)
{
    constexpr int kN = 10;
    SolverSketch s;
    auto pid = [](int i, int j) { return static_cast<SolverId>(1000 + i * kN + j); };
    auto lid = [](int k) { return static_cast<SolverId>(100000 + k); };
    for (int i = 0; i < kN; ++i) {
        for (int j = 0; j < kN; ++j) {
            const double jitter = 0.03 * std::sin(2.0 * i + 5.0 * j);
            s.points.push_back({pid(i, j), 10.0 * j + jitter, 10.0 * i - jitter});
        }
    }
    SolverId cid = 1;
    int lineCount = 0;
    for (int i = 0; i < kN; ++i) {  // 90 horizontal edges
        for (int j = 0; j + 1 < kN; ++j) {
            s.lines.push_back({lid(lineCount), pid(i, j), pid(i, j + 1)});
            s.constraints.push_back(con(cid++, SolverConstraintKind::Horizontal, {ref(lid(lineCount))}));
            ++lineCount;
        }
    }
    for (int i = 0; i + 1 < kN; ++i) {  // 9 vertical edges in the first column
        s.lines.push_back({lid(lineCount), pid(i, 0), pid(i + 1, 0)});
        s.constraints.push_back(con(cid++, SolverConstraintKind::Vertical, {ref(lid(lineCount))}));
        ++lineCount;
    }
    // One diagonal line (unconstrained) brings the entity count to 200.
    s.lines.push_back({lid(lineCount), pid(0, 0), pid(1, 1)});
    ++lineCount;
    s.constraints.push_back(con(cid++, SolverConstraintKind::Fix, {ref(pid(0, 0))}));
    ASSERT_EQ(s.points.size() + s.lines.size(), 200u);

    SketchSolver solver(s);
    ASSERT_EQ(solver.solve().status, SolverStatus::Solved);

    const SolverRef corner = ref(pid(kN - 1, kN - 1));
    ASSERT_EQ(solver.dragPoint(corner, 95.0, 92.0).status, SolverStatus::Solved);
    double worstMs = 0.0;
    for (int step = 1; step <= 60; ++step) {
        const double t = step / 60.0;
        const double x = 95.0 + 5.0 * std::sin(6.0 * t);
        const double y = 92.0 + 3.0 * t;
        const auto t0 = std::chrono::steady_clock::now();
        const SolverResult r = solver.dragPoint(corner, x, y);
        const auto t1 = std::chrono::steady_clock::now();
        ASSERT_EQ(r.status, SolverStatus::Solved) << r.message;
        worstMs = std::max(worstMs, std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    RecordProperty("drag_worst_ms", std::to_string(worstMs));
    // Acceptance budget is for release builds only; the debug build still checks the result.
#ifdef NDEBUG
    EXPECT_LT(worstMs, 16.0);
#endif
    // Horizontal rows stay horizontal after the drag.
    EXPECT_NEAR(pointById(solver.sketch(), pid(kN - 1, 0)).y, pointById(solver.sketch(), pid(kN - 1, 9)).y, 1e-6);
}

}  // namespace iw::sketch
