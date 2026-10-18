// SPDX-License-Identifier: GPL-3.0-or-later
// B3a part 1: the Qt-free SketchSession (rectangle/circle/point/polyline tools, constructions, undo) and sketch mode
// in the shell (Create Sketch -> plane chooser -> look-at -> mouse/keyboard tools -> Finish). Offscreen only.
#include <gtest/gtest.h>

#include <QApplication>
#include <QCheckBox>
#include <QColor>
#include <QCoreApplication>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QImage>
#include <QKeyEvent>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QSettings>
#include <QToolButton>
#include <QTest>

#include <cmath>
#include <cstdlib>
#include <numbers>
#include <memory>
#include <string>
#include <vector>

#include "core/math.hpp"
#include "core/params.hpp"
#include "core/undo.hpp"
#include "render/camera.hpp"
#include "render/viewport.hpp"
#include "sketch/profile.hpp"
#include "sketch/sketch.hpp"
#include "ui/command_actions.hpp"
#include "ui/command_dialog.hpp"
#include "ui/main_window.hpp"
#include "ui/sketch/plane_chooser.hpp"
#include "ui/sketch/session.hpp"
#include "ui/sketch/sketch_mode.hpp"
#include "ui/sketch/sketch_overlay.hpp"
#include "ui/sketch/view_math.hpp"

using namespace iw;
using namespace iw::ui;
using sketch::ConstraintKind;
using sketch::LineGeom;
using sketch::PointRole;
using sketch::Point2;

namespace {

/// One QApplication for the binary. Settings go to a test-only organisation, cleared before every run.
class SketchEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
        app_ = new QApplication(argc_, argv_);
        QCoreApplication::setOrganizationName("IronwoodTest");
        QCoreApplication::setApplicationName("ironwood-ui-sketch-test");
        QSettings().clear();
    }
    void TearDown() override {
        QSettings().clear();
        delete app_;
    }

private:
    int argc_ = 1;
    char name_[16] = "test_ui_sketch";
    char* argv_[2] = {name_, nullptr};
    QApplication* app_ = nullptr;
};
const auto* const kEnv = ::testing::AddGlobalTestEnvironment(new SketchEnv);

constexpr double kTol = 1e-6;

int countKind(const sketch::Sketch& s, sketch::EntityKind k) {
    int n = 0;
    for (const auto& [id, e] : s.entities())
        if (e.kind() == k) ++n;
    return n;
}

int countConstraints(const sketch::Sketch& s, ConstraintKind k) {
    int n = 0;
    for (const auto& [id, c] : s.constraints()) n += c.kind == k;
    return n;
}

void processFor(int ms) {
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 10);
}

/// Pumps events until the viewport finished its camera animation (bounded).
void settle(render::Viewport* vp) {
    QElapsedTimer t;
    t.start();
    while (vp->isAnimating() && t.elapsed() < 3000) QApplication::processEvents(QEventLoop::AllEvents, 10);
    processFor(20);
}

QPoint screenOf(MainWindow& win, Point2 p) {
    render::Viewport* vp = win.viewport();
    const auto s = planeToScreen(vp->camera(), vp->width(), vp->height(), win.sketchMode()->activeSession()->plane(), p);
    EXPECT_TRUE(s.has_value());
    return s ? QPoint(static_cast<int>(std::lround(s->x)), static_cast<int>(std::lround(s->y))) : QPoint();
}

/// Sketch point under a click at `p` (inverse of screenOf).
Point2 sketchAt(MainWindow& win, QPoint p) {
    render::Viewport* vp = win.viewport();
    const auto s = screenToPlane(vp->camera(), vp->width(), vp->height(), win.sketchMode()->activeSession()->plane(),
                                 ScreenPoint{double(p.x()), double(p.y())});
    EXPECT_TRUE(s.has_value());
    return s ? *s : Point2{};
}

// ===================================================================================================================
// SketchSession: no widgets.
// ===================================================================================================================

SketchSession makeSession() { return SketchSession(*sketch::SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0})); }

TEST(SketchSession, RectangleIsFourLinesEightConstraintsDofFourOneProfile) {
    SketchSession s = makeSession();
    auto r = s.addRect2pt({0, 0}, {40, 20});
    ASSERT_TRUE(r) << r.error().message;
    EXPECT_EQ(countKind(s.sketch(), sketch::EntityKind::Line), 4);
    EXPECT_EQ(s.sketch().entities().size(), 4u);
    EXPECT_EQ(s.sketch().constraints().size(), 8u);
    int coincident = 0, horizontal = 0, vertical = 0;
    for (const auto& [id, c] : s.sketch().constraints()) {
        coincident += c.kind == ConstraintKind::Coincident;
        horizontal += c.kind == ConstraintKind::Horizontal;
        vertical += c.kind == ConstraintKind::Vertical;
    }
    EXPECT_EQ(coincident, 4);
    EXPECT_EQ(horizontal, 2);
    EXPECT_EQ(vertical, 2);
    EXPECT_EQ(s.report().dof, 4);
    ASSERT_EQ(s.profiles().size(), 1u);
    EXPECT_NEAR(s.profiles()[0].area, 800.0, 800.0 * 1e-9);
}

TEST(SketchSession, CircleInsideRectangleGivesRingAndDisc) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addRect2pt({0, 0}, {40, 20}));
    ASSERT_TRUE(s.addCircleCenter({20, 10}, 5));
    EXPECT_EQ(s.profiles().size(), 2u);
    EXPECT_EQ(s.report().dof, 7);   // rectangle 4 + circle centre 2 + radius 1
}

TEST(SketchSession, ConstructionEntitiesAreExcludedFromProfiles) {
    SketchSession s = makeSession();
    auto rect = s.addRect2pt({0, 0}, {40, 20});
    ASSERT_TRUE(rect);
    // A construction side leaves the rectangle open: no profile at all, the constraints stay.
    ASSERT_TRUE(s.toggleConstruction((*rect)[0]));
    EXPECT_TRUE(s.sketch().findEntity((*rect)[0])->construction);
    EXPECT_EQ(s.profiles().size(), 0u);
    EXPECT_EQ(s.sketch().constraints().size(), 8u);

    // Back to normal: one profile again. A construction circle inside the rectangle is ignored.
    ASSERT_TRUE(s.toggleConstruction((*rect)[0]));
    EXPECT_EQ(s.profiles().size(), 1u);
    ASSERT_TRUE(s.addCircleCenter({20, 10}, 5));
    auto circleId = s.sketch().entities().rbegin()->first;
    ASSERT_TRUE(s.toggleConstruction(circleId));
    EXPECT_EQ(s.profiles().size(), 1u);
}

TEST(SketchSession, UndoRedoRestoresEntitiesConstraintsAndProfiles) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addRect2pt({0, 0}, {40, 20}));
    ASSERT_TRUE(s.addCircleCenter({20, 10}, 5));
    EXPECT_EQ(s.undoStack().size(), 2u);
    EXPECT_EQ(s.profiles().size(), 2u);

    ASSERT_TRUE(s.undo());
    EXPECT_EQ(s.sketch().entities().size(), 4u);
    EXPECT_EQ(s.profiles().size(), 1u);
    EXPECT_EQ(s.undoStack().undoText(), "Create Rectangle");
    EXPECT_TRUE(s.canRedo());

    ASSERT_TRUE(s.undo());
    EXPECT_EQ(s.sketch().entities().size(), 0u);
    EXPECT_EQ(s.sketch().constraints().size(), 0u);
    EXPECT_EQ(s.profiles().size(), 0u);
    EXPECT_FALSE(s.canUndo());

    ASSERT_TRUE(s.redo());
    ASSERT_TRUE(s.redo());
    EXPECT_EQ(s.sketch().entities().size(), 5u);
    EXPECT_EQ(s.sketch().constraints().size(), 8u);
    EXPECT_EQ(s.profiles().size(), 2u);
    EXPECT_FALSE(s.canRedo());
}

TEST(SketchSession, FailedEditLeavesSketchAndHistoryUnchanged) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addLine({0, 0}, {10, 0}));
    const std::size_t undoSize = s.undoStack().size();
    EXPECT_FALSE(s.addRect2pt({5, 5}, {5, 5}));          // degenerate corners
    EXPECT_FALSE(s.addLine({1, 1}, {1, 1}));             // zero length
    EXPECT_FALSE(s.addCircleCenter({0, 0}, 0));           // zero radius
    EXPECT_FALSE(s.toggleConstruction(sketch::SketchEntityId{999}));
    EXPECT_EQ(s.sketch().entities().size(), 1u);
    EXPECT_EQ(s.undoStack().size(), undoSize);
}

TEST(SketchSession, RectCenterUsesCentreAndCorner) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addRectCenter({0, 0}, {10, 5}));
    EXPECT_EQ(s.sketch().entities().size(), 4u);
    EXPECT_EQ(s.sketch().constraints().size(), 8u);
    ASSERT_EQ(s.profiles().size(), 1u);
    EXPECT_NEAR(s.profiles()[0].area, 200.0, 200.0 * 1e-9);
    EXPECT_NEAR(s.profiles()[0].centroid.x, 0.0, kTol);
    EXPECT_NEAR(s.profiles()[0].centroid.y, 0.0, kTol);
}

TEST(SketchSession, LineToolPolylineJoinsSegments) {
    SketchSession s = makeSession();
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({0, 0}));           // starts the polyline, no entity yet
    EXPECT_EQ(s.sketch().entities().size(), 0u);
    ASSERT_TRUE(s.click({10, 0}));
    ASSERT_TRUE(s.click({10, 10}));
    EXPECT_TRUE(s.hasPending());
    s.endPolyline();                        // double click / Enter
    EXPECT_FALSE(s.hasPending());
    EXPECT_EQ(countKind(s.sketch(), sketch::EntityKind::Line), 2);
    // Inference is on by default: the joint plus the first segment's Horizontal and a Perpendicular snap for the second.
    ASSERT_EQ(s.sketch().constraints().size(), 3u);
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Coincident), 1);
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Horizontal), 1);
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Perpendicular), 1);
    EXPECT_EQ(s.undoStack().size(), 2u);    // one undo step per segment

    // Starting again after the polyline ended does not join to the old one.
    ASSERT_TRUE(s.click({50, 50}));
    ASSERT_TRUE(s.click({60, 50}));
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Coincident), 1);   // still only the first joint
    EXPECT_EQ(s.sketch().constraints().size(), 4u);                           // + the new segment's Horizontal
}

TEST(SketchSession, EscCancelsToolThenReportsNothingLeft) {
    SketchSession s = makeSession();
    EXPECT_FALSE(s.cancel());               // Select tool: nothing to cancel, the shell exits sketch mode
    s.setTool(SketchTool::Rectangle);
    ASSERT_TRUE(s.click({0, 0}));
    EXPECT_TRUE(s.hasPending());
    EXPECT_TRUE(s.cancel());                // drops the half-placed rectangle and returns to Select
    EXPECT_EQ(s.tool(), SketchTool::Select);
    EXPECT_FALSE(s.hasPending());
    EXPECT_EQ(s.sketch().entities().size(), 0u);
    EXPECT_FALSE(s.cancel());
}

TEST(SketchSession, RectangleToolTakesTwoClicks) {
    SketchSession s = makeSession();
    s.setTool(SketchTool::Rectangle);
    ASSERT_TRUE(s.click({0, 0}));
    EXPECT_EQ(s.sketch().entities().size(), 0u);
    s.setCursor(Point2{30, 15});
    ASSERT_EQ(s.preview().size(), 4u);      // rubber band: four sides
    ASSERT_TRUE(s.click({30, 15}));
    EXPECT_EQ(s.sketch().entities().size(), 4u);
    EXPECT_FALSE(s.hasPending());
    EXPECT_EQ(s.tool(), SketchTool::Rectangle);   // the tool stays active for the next rectangle
}

TEST(SketchSession, CircleToolCentreThenRadiusPoint) {
    SketchSession s = makeSession();
    s.setTool(SketchTool::Circle);
    ASSERT_TRUE(s.click({0, 0}));
    s.setCursor(Point2{3, 4});
    ASSERT_EQ(s.preview().size(), 1u);
    ASSERT_TRUE(s.click({3, 4}));
    ASSERT_EQ(s.sketch().entities().size(), 1u);
    const auto& e = s.sketch().entities().begin()->second;
    ASSERT_EQ(e.kind(), sketch::EntityKind::Circle);
    EXPECT_NEAR(std::get<sketch::CircleGeom>(e.geometry).radius, 5.0, kTol);
}

TEST(SketchSession, PointToolAddsOnePoint) {
    SketchSession s = makeSession();
    s.setTool(SketchTool::Point);
    ASSERT_TRUE(s.click({7, 8}));
    EXPECT_EQ(countKind(s.sketch(), sketch::EntityKind::Point), 1);
    EXPECT_EQ(s.sketch().constraints().size(), 0u);
}

TEST(SketchSession, SelectPicksNearestEntityWithinTolerance) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addRect2pt({0, 0}, {40, 20}));
    ASSERT_TRUE(s.addCircleCenter({20, 10}, 5));
    // 0.3 mm from the bottom side (y = 0), well inside a 0.5 mm tolerance.
    const auto bottom = s.selectAt({20, 0.3}, 0.5);
    ASSERT_TRUE(bottom);
    EXPECT_EQ(s.selected(), bottom);
    // The circle: 0.2 mm off its rim.
    const auto circle = s.selectAt({25.2, 10}, 0.5);
    ASSERT_TRUE(circle);
    EXPECT_EQ(s.sketch().findEntity(*circle)->kind(), sketch::EntityKind::Circle);
    // Empty space clears the selection.
    EXPECT_FALSE(s.selectAt({200, 200}, 0.5));
    EXPECT_FALSE(s.selected());
}

TEST(SketchSession, SelectToolClickSelects) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addRect2pt({0, 0}, {40, 20}));
    ASSERT_TRUE(s.click({40, 10}, 0.5));    // Select tool is the default
    ASSERT_TRUE(s.selected());
    ASSERT_TRUE(s.toggleConstruction(*s.selected()));
    EXPECT_EQ(s.profiles().size(), 0u);
}

// ===================================================================================================================
// B3a part 2 (Qt-free): snapping, auto-constraints, arcs, trim and offset.
// ===================================================================================================================


InferenceSettings inferOn(double radius = 0.5) { return InferenceSettings{true, radius}; }

TEST(SketchInference, EndpointSnapBeatsOnCurveAtTheSamePlace) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addRect2pt({0, 0}, {40, 20}));
    s.setInference(inferOn());
    // (39.8, 0.1) is 0.1 mm from the bottom side (on-curve) and 0.22 mm from the corner (endpoint).
    const auto hit = s.snapAt({39.8, 0.1});
    ASSERT_TRUE(hit);
    EXPECT_EQ(hit->kind, SnapKind::Endpoint);
    EXPECT_NEAR(hit->point.x, 40, kTol);
    EXPECT_NEAR(hit->point.y, 0, kTol);
    // Away from every endpoint, the same cursor snaps to the curve.
    const auto onCurve = s.snapAt({15, 0.1});   // the bottom side's midpoint is at x = 20
    ASSERT_TRUE(onCurve);
    EXPECT_EQ(onCurve->kind, SnapKind::OnCurve);
    // Off: no inference at all.
    s.setInference(InferenceSettings{false, 0.5});
    EXPECT_FALSE(s.snapAt({39.8, 0.1}));
}

TEST(SketchInference, SnapToEndpointAddsCoincidentToTheSnappedEnd) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addRect2pt({0, 0}, {40, 20}));
    s.setInference(inferOn());
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({39.8, 0.1}));          // snapped to the corner (40, 0)
    ASSERT_TRUE(s.click({60, 30}));             // free end
    s.endPolyline();
    const sketch::SketchEntityId line = s.sketch().entities().rbegin()->first;
    const int before = static_cast<int>(s.sketch().constraints().size());
    EXPECT_GT(before, 8);
    // Some coincident constraint joins the new line's start to an existing point at (40, 0).
    bool found = false;
    for (const auto& [id, c] : s.sketch().constraints()) {
        if (c.kind != ConstraintKind::Coincident || c.refs.size() != 2) continue;
        if (c.refs[0].entity != line || c.refs[0].role != PointRole::Start) continue;
        auto p = s.sketch().pointOf(c.refs[1]);
        found = found || (p && sketch::distance(*p, Point2{40, 0}) < kTol);
    }
    EXPECT_TRUE(found);
    EXPECT_EQ(s.profiles().size(), 1u);   // the rectangle is still one profile
}

TEST(SketchInference, NearHorizontalSegmentGetsHorizontal) {
    SketchSession s = makeSession();
    s.setInference(inferOn(0.2));
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({0, 0}));
    ASSERT_TRUE(s.click({30, 0.3}));            // 0.57 degrees off the x axis
    s.endPolyline();
    const sketch::SketchEntityId line = s.sketch().entities().rbegin()->first;
    bool horizontal = false;
    for (const auto& [id, c] : s.sketch().constraints())
        horizontal = horizontal || (c.kind == ConstraintKind::Horizontal && c.refs[0].entity == line);
    EXPECT_TRUE(horizontal);
}

TEST(SketchInference, SegmentOutsideOneAndAHalfDegreesGetsNoAxisConstraint) {
    SketchSession s = makeSession();
    s.setInference(inferOn(0.01));
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({0, 0}));
    ASSERT_TRUE(s.click({30, 1.5}));            // 2.9 degrees
    s.endPolyline();
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Horizontal), 0);
}

TEST(SketchInference, AlignmentWithAPendingStartIsAHorizontalGuide) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addPoint({5, 12}));
    s.setInference(inferOn(0.5));
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({0, 10}));
    s.setCursor(Point2{30, 10.3});
    ASSERT_TRUE(s.snapHint());
    EXPECT_EQ(s.snapHint()->kind, SnapKind::Horizontal);
    EXPECT_FALSE(s.snapHint()->guides.empty());
    EXPECT_NEAR(s.snapHint()->point.y, 10.0, kTol);
}

TEST(SketchInference, PerpendicularSnapAddsPerpendicularConstraint) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addLine({0, 0}, {40, 0}));
    s.setInference(inferOn());
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({10, 20}));
    ASSERT_TRUE(s.click({10.2, 0.1}));         // foot of the perpendicular from (10,20) onto the x axis
    s.endPolyline();
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Perpendicular), 1);
    EXPECT_TRUE(s.report().ok());
}

TEST(SketchInference, TangentSnapAddsTangentConstraint) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addCircleCenter({0, 0}, 10));
    s.setInference(inferOn());
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({20, 0}));
    s.setCursor(Point2{5, 8.5});
    ASSERT_TRUE(s.snapHint());
    EXPECT_EQ(s.snapHint()->kind, SnapKind::Tangent);   // beats the nearby curve point
    ASSERT_TRUE(s.click({5, 8.5}));
    s.endPolyline();
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Tangent), 1);
    EXPECT_NEAR(sketch::distance(Point2{5, std::sqrt(75.0)}, Point2{5, 8.5}), 0.1603, 1e-3);
}

TEST(SketchInference, MidpointSnapAddsMidpointConstraint) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addLine({0, 0}, {40, 0}));
    s.setInference(inferOn());
    s.setTool(SketchTool::Point);
    const auto hit = s.snapAt({20.1, 0.1});
    ASSERT_TRUE(hit);
    EXPECT_EQ(hit->kind, SnapKind::Midpoint);
    ASSERT_TRUE(s.click({20.1, 0.1}));
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Midpoint), 1);
}

TEST(SketchArc, ThreePointArcIsCcwAndThroughAllThreePoints) {
    SketchSession s = makeSession();
    s.setTool(SketchTool::Arc);
    ASSERT_TRUE(s.click({10, 0}));
    ASSERT_TRUE(s.click({0, 10}));
    EXPECT_EQ(s.sketch().entities().size(), 0u);
    ASSERT_TRUE(s.click({-10, 0}));
    ASSERT_EQ(s.sketch().entities().size(), 1u);
    const auto& arc = std::get<sketch::ArcGeom>(s.sketch().entities().begin()->second.geometry);
    EXPECT_NEAR(arc.radius, 10, kTol);
    EXPECT_NEAR(arc.center.x, 0, kTol);
    EXPECT_NEAR(arc.center.y, 0, kTol);
    EXPECT_NEAR(arc.sweep, std::numbers::pi, 1e-6);
    EXPECT_NEAR(sketch::arcMid(arc).y, 10, 1e-6);
    EXPECT_FALSE(s.hasPending());
}

TEST(SketchArc, CollinearThreePointArcIsRejected) {
    SketchSession s = makeSession();
    EXPECT_FALSE(s.addArc3({0, 0}, {5, 0}, {10, 0}));
    EXPECT_EQ(s.sketch().entities().size(), 0u);
}

TEST(SketchArc, TangentArcFromLineEndIsTangentAndG1) {
    SketchSession s = makeSession();
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({0, 0}));
    ASSERT_TRUE(s.click({10, 0}));
    ASSERT_TRUE(s.canTangentArcAt({10, 0}, 0.5));
    EXPECT_FALSE(s.canTangentArcAt({0, 0}, 0.5));
    // The drag: the arc leaves (10,0) along +x and reaches (15,5): radius 5, centre (10,5), a quarter turn.
    auto arc = s.tangentArcTo({15, 5});
    ASSERT_TRUE(arc) << arc.error().message;
    const auto& g = std::get<sketch::ArcGeom>(s.sketch().findEntity(*arc)->geometry);
    EXPECT_NEAR(g.radius, 5, kTol);
    EXPECT_NEAR(g.center.x, 10, kTol);
    EXPECT_NEAR(g.center.y, 5, kTol);
    EXPECT_NEAR(g.sweep, std::numbers::pi / 2, 1e-6);
    // Tangent constraint between the line and the arc, and the arc starts at the line end.
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Tangent), 1);
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Coincident), 1);
    // G1 at the joint: the arc's tangent there is the line direction (+x): the radius is vertical.
    const Point2 joint = sketch::arcStart(g);
    EXPECT_NEAR(joint.x, 10, kTol);
    EXPECT_NEAR(joint.y, 0, kTol);
    EXPECT_TRUE(s.report().ok());
    // The polyline continues from the arc's far end.
    EXPECT_TRUE(s.canTangentArcAt({15, 5}, 0.5));
}

TEST(SketchArc, TangentArcToTheOtherSideRunsClockwise) {
    SketchSession s = makeSession();
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({0, 0}));
    ASSERT_TRUE(s.click({10, 0}));
    auto arc = s.tangentArcTo({15, -5});
    ASSERT_TRUE(arc) << arc.error().message;
    const auto& g = std::get<sketch::ArcGeom>(s.sketch().findEntity(*arc)->geometry);
    EXPECT_NEAR(g.center.y, -5, kTol);
    EXPECT_NEAR(g.radius, 5, kTol);
    EXPECT_TRUE(s.report().ok());
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Tangent), 1);
}

TEST(SketchArc, TangentArcOnTheTangentLineIsRejected) {
    SketchSession s = makeSession();
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({0, 0}));
    ASSERT_TRUE(s.click({10, 0}));
    EXPECT_FALSE(s.tangentArcTo({30, 0}));
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Tangent), 0);
}

TEST(SketchTrim, TrimmingAMiddlePieceKeepsIdsAndCarriesHorizontal) {
    SketchSession s = makeSession();
    // A line at y = 10 drawn first, so its horizontal comes from the near-axis rule (inference on).
    s.setInference(inferOn());
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({-10, 10}));
    ASSERT_TRUE(s.click({50, 10}));
    s.endPolyline();
    s.setInference(InferenceSettings{false, 0.5});
    ASSERT_TRUE(s.addRect2pt({0, 0}, {40, 20}));   // crosses the line at (0,10) and (40,10)
    const sketch::SketchEntityId line{1};
    ASSERT_EQ(countConstraints(s.sketch(), ConstraintKind::Horizontal), 3);   // rectangle's 2 + the line's

    s.setTool(SketchTool::Trim);
    ASSERT_TRUE(s.click({20, 10}));              // removes (0..40, 10)
    ASSERT_EQ(s.sketch().entities().size(), 6u);  // 4 sides + 2 pieces
    // The left piece keeps the line's id, the right piece is new.
    const auto& left = std::get<LineGeom>(s.sketch().findEntity(line)->geometry);
    EXPECT_NEAR(left.start.x, -10, kTol);
    EXPECT_NEAR(left.end.x, 0, kTol);
    EXPECT_NEAR(left.end.y, 10, kTol);
    const sketch::SketchEntityId right = s.sketch().entities().rbegin()->first;
    EXPECT_NE(right, line);
    const auto& rg = std::get<LineGeom>(s.sketch().findEntity(right)->geometry);
    EXPECT_NEAR(rg.start.x, 40, kTol);
    EXPECT_NEAR(rg.end.x, 50, kTol);
    // Both pieces carry the horizontal; the rectangle's constraints are untouched.
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Horizontal), 4);
    EXPECT_EQ(s.sketch().constraints().size(), 8u + 2u);
    EXPECT_TRUE(s.report().ok());
    ASSERT_EQ(s.profiles().size(), 1u);
    EXPECT_NEAR(s.profiles()[0].area, 800.0, 800.0 * 1e-9);
    EXPECT_TRUE(s.undo());
    EXPECT_EQ(s.sketch().entities().size(), 5u);
}

TEST(SketchTrim, CrossingLinesSplitTheTargetAtBothIntersections) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addLine({0, -10}, {0, 30}));   // vertical, the target
    ASSERT_TRUE(s.addLine({-10, 0}, {10, 0}));   // crosses at (0,0)
    ASSERT_TRUE(s.addLine({-10, 20}, {10, 20})); // crosses at (0,20)
    const sketch::SketchEntityId target = s.sketch().entities().begin()->first;
    ASSERT_TRUE(s.trimAt({0, 10}));              // removes (0..20) of the target
    const auto& a = std::get<LineGeom>(s.sketch().findEntity(target)->geometry);
    EXPECT_NEAR(a.start.y, -10, kTol);
    EXPECT_NEAR(a.end.y, 0, kTol);
    EXPECT_EQ(s.sketch().entities().size(), 4u);  // target kept + one new piece + two crossers
}

TEST(SketchTrim, TrimmingWithoutIntersectionsRemovesTheWholeLine) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addLine({0, 0}, {10, 0}));
    ASSERT_TRUE(s.trimAt({5, 0}));
    EXPECT_EQ(s.sketch().entities().size(), 0u);
}

TEST(SketchOffset, OffsetOfTwentyByTenRectByTwoMakesRingAndTwentyFourByFourteen) {
    SketchSession s = makeSession();
    auto rect = s.addRect2pt({0, 0}, {20, 10});
    ASSERT_TRUE(rect);
    ASSERT_EQ(s.profiles().size(), 1u);
    auto ids = s.offsetLoop((*rect)[0], 2.0);
    ASSERT_TRUE(ids) << ids.error().message;
    ASSERT_EQ(ids->size(), 4u);
    double minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
    for (auto id : *ids) {
        const auto& l = std::get<LineGeom>(s.sketch().findEntity(id)->geometry);
        for (Point2 q : {l.start, l.end}) {
            minx = std::min(minx, q.x);
            maxx = std::max(maxx, q.x);
            miny = std::min(miny, q.y);
            maxy = std::max(maxy, q.y);
        }
    }
    EXPECT_NEAR(maxx - minx, 24, kTol);
    EXPECT_NEAR(maxy - miny, 14, kTol);
    EXPECT_NEAR(minx, -2, kTol);
    EXPECT_NEAR(miny, -2, kTol);
    // Two regions: the original 20 x 10 (200 mm^2) and the ring between it and the offset 24 x 14 loop
    // (24*14 - 20*10 = 136 mm^2). The ring is the offset loop's region with the original loop as its hole.
    ASSERT_EQ(s.profiles().size(), 2u);
    std::vector<double> areas;
    for (const auto& pr : s.profiles()) areas.push_back(pr.area);
    std::sort(areas.begin(), areas.end());
    EXPECT_NEAR(areas[0], 136.0, 136.0 * 1e-9);
    EXPECT_NEAR(areas[1], 200.0, 200.0 * 1e-9);
    EXPECT_TRUE(s.report().ok());
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Coincident), 8);   // 4 original + 4 offset joints
}

// Offset ring: the offset loop encloses the original loop, so the ring is one region with the original as a hole (F28).
TEST(SketchOffset, OffsetRingIsOneProfileWithAHole) {
    SketchSession s = makeSession();
    auto rect = s.addRect2pt({0, 0}, {20, 10});
    ASSERT_TRUE(rect);
    ASSERT_TRUE(s.offsetLoop((*rect)[0], 2.0));
    ASSERT_EQ(s.profiles().size(), 2u);
    int rings = 0;
    for (const auto& pr : s.profiles()) {
        if (!pr.holes.empty()) {
            ++rings;
            EXPECT_NEAR(pr.area, 136.0, 136.0 * 1e-9);
        }
    }
    EXPECT_EQ(rings, 1);
}

TEST(SketchOffset, NegativeOffsetShrinksAndCircleOffsetIsConcentric) {
    SketchSession s = makeSession();
    auto rect = s.addRect2pt({0, 0}, {20, 10});
    ASSERT_TRUE(rect);
    auto shrunk = s.offsetLoop((*rect)[0], -2.0);
    ASSERT_TRUE(shrunk) << shrunk.error().message;
    ASSERT_EQ(s.profiles().size(), 2u);   // the 20 x 10 loop and the 16 x 6 loop
    auto circle = s.addCircleCenter({50, 50}, 5);
    ASSERT_TRUE(circle);
    auto grown = s.offsetLoop(*circle, 1.5);
    ASSERT_TRUE(grown) << grown.error().message;
    const auto& g = std::get<sketch::CircleGeom>(s.sketch().findEntity((*grown)[0])->geometry);
    EXPECT_NEAR(g.radius, 6.5, kTol);
    EXPECT_NEAR(g.center.x, 50, kTol);
}

TEST(SketchOffset, OffsetOfAnOpenChainIsRejected) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addLine({0, 0}, {10, 0}));
    EXPECT_FALSE(s.offsetLoop(s.sketch().entities().begin()->first, 1.0));
}

TEST(SketchTools, CenterRectangleByClicksIsCentredOnTheFirstClick) {
    SketchSession s = makeSession();
    s.setTool(SketchTool::CenterRectangle);
    ASSERT_TRUE(s.click({0, 0}));
    ASSERT_TRUE(s.click({10, 5}));
    EXPECT_EQ(s.sketch().entities().size(), 4u);
    ASSERT_EQ(s.profiles().size(), 1u);
    EXPECT_NEAR(s.profiles()[0].area, 200.0, 200.0 * 1e-9);
    EXPECT_NEAR(s.profiles()[0].centroid.x, 0.0, kTol);
}

TEST(SketchTools, TrimAndOffsetClicksUseTheTools) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addRect2pt({0, 0}, {20, 10}));
    s.setOffsetDistance(1.0);
    s.setTool(SketchTool::Offset);
    ASSERT_TRUE(s.click({10, 0}));              // bottom side: the loop it belongs to grows by 1
    EXPECT_EQ(s.sketch().entities().size(), 8u);
    s.setTool(SketchTool::Select);
}

// ===================================================================================================================
// Sketch mode in the shell (offscreen): the viewport receives real mouse and key events.
// ===================================================================================================================

/// Runs "Create Sketch", picks XY in the chooser and lets the camera settle.
void startXySketch(MainWindow& win) {
    ASSERT_TRUE(win.actions()->action("sketch")->isEnabled());
    win.actions()->run("sketch");
    auto* chooser = qobject_cast<SketchPlaneChooser*>(win.commandDock()->widget());
    ASSERT_NE(chooser, nullptr);
    auto* xy = chooser->findChild<QPushButton*>("planeXY");
    ASSERT_NE(xy, nullptr);
    QTest::mouseClick(xy, Qt::LeftButton);
    ASSERT_TRUE(win.sketchMode()->active());
    settle(win.viewport());
}

std::unique_ptr<MainWindow> makeWindow() {
    auto win = std::make_unique<MainWindow>();
    win->resize(1280, 800);
    win->show();
    processFor(60);
    return win;
}

TEST(SketchMode, CreateSketchStartsOnChosenPlaneAndAnimatesCamera) {
    auto win = makeWindow();
    startXySketch(*win);
    SketchSession* s = win->sketchMode()->activeSession();
    ASSERT_NE(s, nullptr);
    EXPECT_NEAR(s->plane().normal.z, 1.0, kTol);
    // The camera now looks along the plane normal: the eye sits on the +Z side of the origin.
    const render::Camera& cam = win->viewport()->camera();
    EXPECT_NEAR(cam.target().x, 0.0, 1e-6);
    EXPECT_NEAR(cam.forward().z, -1.0, 1e-3);
    auto* dock = win->findChild<QDockWidget*>("sketchDock");
    ASSERT_NE(dock, nullptr);
    EXPECT_TRUE(dock->isVisible());
    EXPECT_TRUE(win->commandDock()->isHidden());
}

TEST(SketchMode, RectangleAndCircleByMouseClicksAndEscExit) {
    auto win = makeWindow();
    startXySketch(*win);
    render::Viewport* vp = win->viewport();
    SketchSession* s = win->sketchMode()->activeSession();

    // Rectangle: two corner clicks.
    QTest::keyClick(vp, Qt::Key_R);
    EXPECT_EQ(s->tool(), SketchTool::Rectangle);
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {-30, -20}));
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {40, 25}));
    EXPECT_EQ(s->sketch().entities().size(), 4u);
    EXPECT_EQ(s->sketch().constraints().size(), 8u);
    EXPECT_EQ(s->report().dof, 4);
    ASSERT_EQ(s->profiles().size(), 1u);

    // Pixel rounding is at most a fraction of a millimetre at this zoom.
    const Point2 corner = sketchAt(*win, screenOf(*win, {-30, -20}));
    EXPECT_NEAR(corner.x, -30, 0.5);
    EXPECT_NEAR(corner.y, -20, 0.5);

    // Centre circle inside the rectangle.
    QTest::keyClick(vp, Qt::Key_C);
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {0, 0}));
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {12, 0}));
    EXPECT_EQ(s->sketch().entities().size(), 5u);
    EXPECT_EQ(s->profiles().size(), 2u);   // ring + disc
    EXPECT_EQ(s->report().dof, 7);

    // First Esc cancels the tool (back to Select), the sketch stays open; second Esc finishes.
    QTest::keyClick(vp, Qt::Key_Escape);
    EXPECT_TRUE(win->sketchMode()->active());
    EXPECT_EQ(s->tool(), SketchTool::Select);
    QTest::keyClick(vp, Qt::Key_Escape);
    EXPECT_FALSE(win->sketchMode()->active());
    EXPECT_FALSE(win->findChild<QDockWidget*>("sketchDock")->isVisible());
    ASSERT_EQ(win->sketchMode()->sessions().size(), 1u);   // kept for later features
}

TEST(SketchMode, PolylineByDoubleClickAndConstructionToggleWithUndo) {
    auto win = makeWindow();
    startXySketch(*win);
    render::Viewport* vp = win->viewport();
    SketchSession* s = win->sketchMode()->activeSession();

    QTest::keyClick(vp, Qt::Key_L);
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {0, 0}));
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {20, 0}));
    // A real double click is press, release, double-click at the same spot. QTest::mouseDClick sends only the
    // double-click event, so the first press is sent explicitly: it adds the last segment, the double click ends.
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {20, 15}));
    QTest::mouseDClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {20, 15}));
    EXPECT_EQ(s->sketch().entities().size(), 2u);
    EXPECT_FALSE(s->hasPending());
    EXPECT_EQ(countConstraints(s->sketch(), ConstraintKind::Coincident), 1);   // + inferred H / Perpendicular

    // Select tool: click the first segment, X makes it construction, Ctrl+Z undoes that step.
    QTest::keyClick(vp, Qt::Key_Escape);             // Line -> Select
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {10, 0}));
    ASSERT_TRUE(s->selected());
    QTest::keyClick(vp, Qt::Key_X);
    EXPECT_TRUE(s->sketch().findEntity(*s->selected())->construction);
    QTest::keyClick(vp, Qt::Key_Z, Qt::ControlModifier);
    EXPECT_FALSE(s->sketch().findEntity(*s->selected())->construction);
    EXPECT_TRUE(win->sketchMode()->active());
}

TEST(SketchMode, Part2PaletteControlsAndSnapToggle) {
    auto win = makeWindow();
    startXySketch(*win);
    for (const char* name : {"tool_arc", "tool_trim", "tool_offset", "tool_center_rect"})
        EXPECT_NE(win->findChild<QToolButton*>(name), nullptr) << name;
    EXPECT_EQ(win->findChild<QDoubleSpinBox*>("offsetDistance"), nullptr);   // the distance lives in a command panel now
    for (const char* name : {"tool_dimension", "con_horizontal", "con_vertical", "con_coincident", "con_tangent", "con_equal",
                             "con_parallel", "con_perpendicular", "con_fix", "con_midpoint", "con_concentric",
                             "con_collinear", "con_symmetric", "con_pointonobject"})
        EXPECT_NE(win->findChild<QToolButton*>(name), nullptr) << name;
    SketchSession* s = win->sketchMode()->activeSession();
    EXPECT_TRUE(s->inference().enabled);        // snapping and auto-constraints are on by default
    EXPECT_GT(s->inference().radiusMm, 0.0);
    auto* snap = win->findChild<QCheckBox*>("toggleSnap");
    ASSERT_NE(snap, nullptr);
    EXPECT_TRUE(snap->isChecked());
    snap->click();   // a synthetic mouse click is not delivered to palette widgets offscreen; click() runs the same slot
    EXPECT_FALSE(s->inference().enabled);
    snap->click();
    EXPECT_TRUE(s->inference().enabled);
    QTest::keyClick(win->viewport(), Qt::Key_T);
    EXPECT_EQ(s->tool(), SketchTool::Trim);
    QTest::keyClick(win->viewport(), Qt::Key_O);
    EXPECT_EQ(s->tool(), SketchTool::Offset);
    QTest::keyClick(win->viewport(), Qt::Key_A);
    EXPECT_EQ(s->tool(), SketchTool::Arc);
    QTest::keyClick(win->viewport(), Qt::Key_Escape);
}

TEST(SketchMode, TangentArcByDraggingFromALineEnd) {
    auto win = makeWindow();
    startXySketch(*win);
    render::Viewport* vp = win->viewport();
    SketchSession* s = win->sketchMode()->activeSession();
    QTest::keyClick(vp, Qt::Key_L);
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {0, 0}));
    processFor(500);   // keep the next press from reading as a double click
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {10, 0}));
    processFor(500);
    ASSERT_EQ(s->sketch().entities().size(), 1u);
    // Press on the polyline end, drag to (15, 5): a tangent arc, not a new straight segment.
    QTest::mousePress(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {10, 0}));
    QTest::mouseMove(vp, screenOf(*win, {12, 2}));
    QTest::mouseMove(vp, screenOf(*win, {15, 5}));
    QTest::mouseRelease(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {15, 5}));
    ASSERT_EQ(s->sketch().entities().size(), 2u);
    int arcs = 0, tangents = 0;
    for (const auto& [id, e] : s->sketch().entities()) arcs += e.kind() == sketch::EntityKind::Arc;
    for (const auto& [id, c] : s->sketch().constraints()) tangents += c.kind == ConstraintKind::Tangent;
    EXPECT_EQ(arcs, 1);
    EXPECT_EQ(tangents, 1);
    EXPECT_TRUE(s->report().ok());
}

TEST(SketchMode, FinishSketchButtonLeavesModeAndKeepsSession) {
    auto win = makeWindow();
    startXySketch(*win);
    SketchSession* s = win->sketchMode()->activeSession();
    ASSERT_TRUE(s->addRect2pt({0, 0}, {10, 10}));
    auto* finish = win->findChild<QPushButton*>("finishSketch");
    ASSERT_NE(finish, nullptr);
    QTest::mouseClick(finish, Qt::LeftButton);
    EXPECT_FALSE(win->sketchMode()->active());
    EXPECT_EQ(win->sketchMode()->sessions().size(), 1u);
    EXPECT_EQ(win->sketchMode()->sessions()[0]->profiles().size(), 1u);
}

TEST(SketchMode, ScreenshotOfRectangleAndCircle) {
    const char* dir = std::getenv("IW_TEST_DUMP_DIR");
    auto win = makeWindow();
    startXySketch(*win);
    render::Viewport* vp = win->viewport();
    SketchSession* s = win->sketchMode()->activeSession();
    ASSERT_TRUE(s->addRect2pt({-30, -20}, {40, 25}));
    ASSERT_TRUE(s->addCircleCenter({0, 0}, 12));
    win->sketchMode()->refresh();
    if (!dir) GTEST_SKIP() << "set IW_TEST_DUMP_DIR to save the screenshot";

    // Offscreen paint of the sketch layer with the camera framed on the rectangle.
    render::Camera cam = vp->camera();
    cam.setViewportSize(800, 600);
    sketch::Sketch copy = s->sketch();
    Box3 box;
    for (const Point2& q : {Point2{-30, -20}, Point2{40, 25}}) box.extend(copy.plane().toWorld(q));
    cam.fit(box, 1.3);
    QImage img(800, 600, QImage::Format_ARGB32_Premultiplied);
    img.fill(QColor(30, 34, 40));
    {
        QPainter p(&img);
        paintSketches(p, cam, 800, 600, {s}, s, SketchViewOptions{});
    }
    EXPECT_TRUE(img.save(QString::fromLocal8Bit(dir) + "/ui_sketch_rect_circle.png"));
    EXPECT_TRUE(win->grab().save(QString::fromLocal8Bit(dir) + "/ui_sketch_window.png"));
}

// ===================================================================================================================
// B3b: PointOnObject inference, circle trim, constraints, dimensions, colouring, drag, offset dialog.
// ===================================================================================================================

TEST(SketchInference, OnCurveAndIntersectionSnapsAddPointOnObject) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addLine({0, 0}, {20, 0}));
    ASSERT_TRUE(s.addLine({15, -10}, {15, 20}));
    s.setInference(inferOn());
    s.setTool(SketchTool::Point);
    ASSERT_TRUE(s.click({5, 0.3}));                       // on the first line, away from its end and middle
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::PointOnObject), 1);
    ASSERT_TRUE(s.click({15.2, 0.2}));                    // crossing of the two lines
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::PointOnObject), 3);
    EXPECT_TRUE(s.report().ok());
}

TEST(SketchInference, TangentToCircleSnapPinsTheEndOnTheCircle) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addCircleCenter({30, 0}, 5));
    s.setInference(inferOn(1.0));
    s.setTool(SketchTool::Line);
    ASSERT_TRUE(s.click({0, 0}));
    const auto hit = s.snapAt({29.17, 4.93});
    ASSERT_TRUE(hit);
    ASSERT_EQ(hit->kind, SnapKind::Tangent);
    ASSERT_TRUE(s.click(hit->point));
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Tangent), 1);
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::PointOnObject), 1);
    EXPECT_TRUE(s.report().ok());
}

TEST(SketchTrim, CircleBecomesAnArcBetweenTheTwoCrossings) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addCircleCenter({0, 0}, 10));
    ASSERT_TRUE(s.addLine({-6, -15}, {-6, 15}));
    ASSERT_TRUE(s.addLine({6, -15}, {6, 15}));
    ASSERT_TRUE(s.trimAt({10, 0}));                       // the right piece between the x = 6 crossings goes
    EXPECT_EQ(countKind(s.sketch(), sketch::EntityKind::Circle), 0);
    ASSERT_EQ(countKind(s.sketch(), sketch::EntityKind::Arc), 1);
    for (const auto& [id, e] : s.sketch().entities()) {
        if (e.kind() != sketch::EntityKind::Arc) continue;
        const auto& g = std::get<sketch::ArcGeom>(e.geometry);
        const double removed = 2 * std::atan2(8.0, 6.0);
        EXPECT_NEAR(g.sweep, 2 * std::numbers::pi - removed, 1e-6);
        EXPECT_NEAR(g.radius, 10, 1e-6);
    }
    EXPECT_TRUE(s.report().ok());
    ASSERT_TRUE(s.undo());
    EXPECT_EQ(countKind(s.sketch(), sketch::EntityKind::Circle), 1);
}

TEST(SketchTrim, CircleWithOneCrossingCannotBeTrimmed) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addCircleCenter({0, 0}, 10));
    ASSERT_TRUE(s.addLine({10, -5}, {10, 5}));            // touches the circle at one point
    EXPECT_FALSE(s.trimAt({-10, 0}));
    EXPECT_EQ(countKind(s.sketch(), sketch::EntityKind::Circle), 1);
}

TEST(SketchConstraints, ToolbarRulesAndDof) {
    SketchSession s = makeSession();
    auto a = s.addLine({0, 0}, {20, 3});
    auto b = s.addLine({0, 15}, {20, 10});
    ASSERT_TRUE(a && b);
    const int dof0 = s.report().dof;
    s.selectAt({10, 1.5}, 0.5);
    s.selectAt({10, 12.5}, 0.5, true);
    ASSERT_EQ(s.selection().size(), 2u);
    auto r = s.applyConstraint(ConstraintKind::Parallel);
    ASSERT_TRUE(r) << r.error().message;
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Parallel), 1);
    EXPECT_EQ(s.report().dof, dof0 - 1);
    // The same constraint again is redundant and refused; so are kinds the selection does not fit.
    EXPECT_FALSE(s.applyConstraint(ConstraintKind::Parallel));
    EXPECT_FALSE(s.applyConstraint(ConstraintKind::Tangent));
    EXPECT_FALSE(s.applyConstraint(ConstraintKind::Concentric));
    EXPECT_EQ(s.sketch().constraints().size(), 1u);
    EXPECT_TRUE(s.applyConstraint(ConstraintKind::Horizontal));
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Horizontal), 2);
    EXPECT_TRUE(s.applyConstraint(ConstraintKind::Equal));
    s.select(*a);
    ASSERT_TRUE(s.toggleFix());
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Fix), 1);
    ASSERT_TRUE(s.toggleFix());
    EXPECT_EQ(countConstraints(s.sketch(), ConstraintKind::Fix), 0);

    SketchSession t = makeSession();
    auto l1 = t.addLine({0, 0}, {10, 0});
    auto l2 = t.addLine({11, 1}, {20, 5});
    ASSERT_TRUE(l1 && l2);
    t.setSelection({{*l1, PointRole::End}, {*l2, PointRole::Start}});
    ASSERT_TRUE(t.applyConstraint(ConstraintKind::Coincident));
    EXPECT_NEAR(sketch::distance(*t.sketch().pointOf({*l1, PointRole::End}), *t.sketch().pointOf({*l2, PointRole::Start})),
                0.0, kTol);

    SketchSession u = makeSession();
    auto base = u.addLine({-10, 0}, {10, 0});
    auto pa = u.addPoint({-3, 4});
    auto pb = u.addPoint({5, -2});
    ASSERT_TRUE(base && pa && pb);
    u.select(*base);
    ASSERT_TRUE(u.toggleFix());
    u.setSelection({{*pa, std::nullopt}, {*pb, std::nullopt}, {*base, std::nullopt}});
    auto sym = u.applyConstraint(ConstraintKind::Symmetric);
    ASSERT_TRUE(sym) << sym.error().message;
    const auto qa = *u.sketch().pointOf({*pa, std::nullopt}), qb = *u.sketch().pointOf({*pb, std::nullopt});
    EXPECT_NEAR(qa.x, qb.x, 1e-6);      // mirrored about the x axis line
    EXPECT_NEAR(qa.y, -qb.y, 1e-6);
}

TEST(SketchConstraints, GlyphsAreSelectableAndDeletable) {
    SketchSession s = makeSession();
    auto l1 = s.addLine({0, 0}, {20, 3});
    ASSERT_TRUE(l1);
    ASSERT_TRUE(s.addLine({0, 15}, {20, 10}));
    s.selectAt({10, 1.5}, 0.5);
    s.selectAt({10, 12.5}, 0.5, true);
    ASSERT_TRUE(s.applyConstraint(ConstraintKind::Parallel));
    const auto glyphs = s.glyphs();
    ASSERT_EQ(glyphs.size(), 1u);
    EXPECT_EQ(glyphs[0].kind, ConstraintKind::Parallel);
    EXPECT_FALSE(glyphs[0].conflicting);
    s.selectConstraint(glyphs[0].id);
    ASSERT_TRUE(s.deleteSelection());
    EXPECT_EQ(s.sketch().constraints().size(), 0u);
    ASSERT_TRUE(s.undo());
    EXPECT_EQ(s.sketch().constraints().size(), 1u);
    // With no glyph selected, Delete removes the selected entities and what references them.
    s.select(*l1);
    ASSERT_TRUE(s.deleteSelection());
    EXPECT_EQ(s.sketch().entities().size(), 1u);
    EXPECT_EQ(s.sketch().constraints().size(), 0u);
}

TEST(SketchDimensions, RectangleWidthFollowsTheParameterAndColouringFlipsAtDofZero) {
    ParameterTable params;
    ASSERT_TRUE(params.add("w", "40 mm"));
    ASSERT_TRUE(params.add("h", "20 mm"));
    SketchSession s = makeSession();
    s.setParameters(&params);
    auto rect = s.addRect2pt({0, 0}, {30, 15});
    ASSERT_TRUE(rect);
    EXPECT_EQ(s.constraintState(), ConstraintState::Under);

    auto bottom = s.planDimension({sketch::SketchRef{(*rect)[0], std::nullopt}}, {15, -5});
    ASSERT_TRUE(bottom) << bottom.error().message;
    EXPECT_EQ(bottom->kind, sketch::DimensionKind::Distance);
    auto dw = s.addDimension(bottom->kind, bottom->refs, "w");
    ASSERT_TRUE(dw) << dw.error().message;
    ASSERT_TRUE(dw->id);
    auto left = s.planDimension({sketch::SketchRef{(*rect)[3], std::nullopt}}, {-5, 7});
    ASSERT_TRUE(left);
    auto dh = s.addDimension(left->kind, left->refs, "h");
    ASSERT_TRUE(dh && dh->id);

    auto widthOf = [&] {
        const auto& g = std::get<LineGeom>(s.sketch().findEntity((*rect)[0])->geometry);
        return sketch::distance(g.start, g.end);
    };
    EXPECT_NEAR(widthOf(), 40.0, 1e-6);
    ASSERT_TRUE(params.setExpression("w", "55 mm"));
    ASSERT_TRUE(s.resolve());
    EXPECT_NEAR(widthOf(), 55.0, 1e-6);
    const auto views = s.dimensionViews();
    ASSERT_EQ(views.size(), 2u);
    EXPECT_NEAR(views[0].value, 55.0, 1e-6);
    EXPECT_EQ(views[0].text, "w = 55");

    // The position is free: two degrees of freedom left, under-constrained (blue).
    EXPECT_EQ(s.report().dof, 2);
    EXPECT_EQ(s.constraintState(), ConstraintState::Under);
    EXPECT_EQ(sketchCurveColor(s.constraintState(), false), QColor(74, 144, 226));

    // Pin the bottom-left corner of the rectangle to a fixed point: DOF 0, fully constrained.
    auto origin = s.addPoint({0, 0});
    ASSERT_TRUE(origin);
    s.select(*origin);
    ASSERT_TRUE(s.toggleFix());
    s.setSelection({{*origin, std::nullopt}, {(*rect)[0], PointRole::Start}});
    ASSERT_TRUE(s.applyConstraint(ConstraintKind::Coincident));
    EXPECT_EQ(s.report().dof, 0);
    EXPECT_EQ(s.constraintState(), ConstraintState::Fully);
    EXPECT_EQ(sketchCurveColor(s.constraintState(), false), QColor(20, 20, 20));
    EXPECT_EQ(sketchCurveColor(s.constraintState(), true), QColor(245, 245, 245));
    // Undo takes it back to blue.
    ASSERT_TRUE(s.undo());
    EXPECT_EQ(s.constraintState(), ConstraintState::Under);
}

TEST(SketchDimensions, PlannerPicksKindFromGeometryAndMouse) {
    SketchSession s = makeSession();
    auto l = s.addLine({0, 0}, {30, 10});
    auto c = s.addCircleCenter({60, 0}, 5);
    auto arc = s.addArc3({0, 40}, {10, 50}, {20, 40});
    auto m = s.addLine({0, 80}, {30, 80});
    auto n = s.addLine({0, 90}, {20, 110});
    ASSERT_TRUE(l && c && arc && m && n);
    using DK = sketch::DimensionKind;
    const sketch::SketchRef p0{*l, PointRole::Start}, p1{*l, PointRole::End};
    EXPECT_EQ(s.planDimension({sketch::SketchRef{*l, std::nullopt}}, {0, 0})->kind, DK::Distance);
    EXPECT_EQ(s.planDimension({sketch::SketchRef{*c, std::nullopt}}, {0, 0})->kind, DK::Diameter);
    EXPECT_EQ(s.planDimension({sketch::SketchRef{*arc, std::nullopt}}, {0, 0})->kind, DK::Radius);
    EXPECT_EQ(s.planDimension({sketch::SketchRef{*m, std::nullopt}, sketch::SketchRef{*n, std::nullopt}}, {0, 0})->kind,
              DK::Angle);
    EXPECT_EQ(s.planDimension({p0, p1}, {15, 30})->kind, DK::DistanceX);    // above the points: horizontal
    EXPECT_EQ(s.planDimension({p0, p1}, {45, 5})->kind, DK::DistanceY);     // beside them: vertical
    EXPECT_EQ(s.planDimension({p0, p1}, {15, 5})->kind, DK::Distance);      // between them: aligned
    EXPECT_FALSE(s.planDimension({sketch::SketchRef{*l, std::nullopt}, sketch::SketchRef{*c, std::nullopt}}, {0, 0}));
    // Measures.
    EXPECT_NEAR(*s.measure(DK::Diameter, {sketch::SketchRef{*c, std::nullopt}}), 10.0, kTol);
    EXPECT_NEAR(*s.measure(DK::DistanceX, {p0, p1}), 30.0, kTol);
}

TEST(SketchDimensions, EveryDimensionKindSolvesAndEditsByExpression) {
    ParameterTable params;
    ASSERT_TRUE(params.add("d1", "15 mm"));
    SketchSession s = makeSession();
    s.setParameters(&params);
    using DK = sketch::DimensionKind;
    auto c = s.addCircleCenter({60, 0}, 5);
    auto arc = s.addArc3({0, 40}, {10, 50}, {20, 40});
    auto m = s.addLine({0, 80}, {30, 80});
    auto n = s.addLine({0, 90}, {20, 110});
    ASSERT_TRUE(c && arc && m && n);
    auto dd = s.addDimension(DK::Diameter, {sketch::SketchRef{*c, std::nullopt}}, "d1*2");
    ASSERT_TRUE(dd && dd->id) << (dd ? dd->message : dd.error().message);
    EXPECT_NEAR(std::get<sketch::CircleGeom>(s.sketch().findEntity(*c)->geometry).radius, 15.0, 1e-6);
    ASSERT_TRUE(s.setDimensionExpression(*dd->id, "10 mm + d1"));
    EXPECT_NEAR(std::get<sketch::CircleGeom>(s.sketch().findEntity(*c)->geometry).radius, 12.5, 1e-6);
    EXPECT_FALSE(s.setDimensionExpression(*dd->id, "2 deg"));             // an angle for a length
    EXPECT_FALSE(s.setDimensionExpression(*dd->id, "nope + 1"));          // unknown parameter
    EXPECT_NEAR(std::get<sketch::CircleGeom>(s.sketch().findEntity(*c)->geometry).radius, 12.5, 1e-6);
    auto dr = s.addDimension(DK::Radius, {sketch::SketchRef{*arc, std::nullopt}}, "d1");
    ASSERT_TRUE(dr && dr->id);
    EXPECT_NEAR(std::get<sketch::ArcGeom>(s.sketch().findEntity(*arc)->geometry).radius, 15.0, 1e-6);
    // Pin one line so the angle has something to turn against.
    s.select(*m);
    ASSERT_TRUE(s.toggleFix());
    auto plan = s.planDimension({sketch::SketchRef{*m, std::nullopt}, sketch::SketchRef{*n, std::nullopt}}, {0, 0});
    ASSERT_TRUE(plan);
    auto da = s.addDimension(plan->kind, plan->refs, "30 deg");
    ASSERT_TRUE(da && da->id) << (da ? da->message : da.error().message);
    const auto& ln = std::get<LineGeom>(s.sketch().findEntity(*n)->geometry);
    const auto& lm = std::get<LineGeom>(s.sketch().findEntity(*m)->geometry);
    const sketch::Point2 d1 = lm.end - lm.start, d2 = ln.end - ln.start;
    EXPECT_NEAR(std::abs(std::atan2(d1.x * d2.y - d1.y * d2.x, d1.x * d2.x + d1.y * d2.y)) * 180 / std::numbers::pi, 30.0, 1e-5);
}

TEST(SketchDimensions, OverConstrainingDimensionIsRefusedAndCanBecomeDriven) {
    SketchSession s = makeSession();
    auto rect = s.addRect2pt({0, 0}, {40, 20});
    ASSERT_TRUE(rect);
    using DK = sketch::DimensionKind;
    const std::vector<sketch::SketchRef> bottom{{(*rect)[0], PointRole::Start}, {(*rect)[0], PointRole::End}};
    auto first = s.addDimension(DK::Distance, bottom, "40");
    ASSERT_TRUE(first && first->id);
    const std::size_t dims = s.sketch().dimensions().size();
    // The same measure again: redundant. A different value: a conflict. Both are refused, nothing is added.
    auto again = s.addDimension(DK::Distance, bottom, "40");
    ASSERT_TRUE(again);
    EXPECT_TRUE(again->overConstrains);
    EXPECT_FALSE(again->id);
    auto clash = s.addDimension(DK::Distance, bottom, "50");
    ASSERT_TRUE(clash);
    EXPECT_TRUE(clash->overConstrains);
    EXPECT_EQ(s.sketch().dimensions().size(), dims);
    // Driven: shown, measured, never constraining; deletable; part of undo.
    auto driven = s.addDrivenDimension(DK::Distance, bottom);
    ASSERT_TRUE(driven);
    EXPECT_EQ(s.drivenDimensions().size(), 1u);
    EXPECT_EQ(s.sketch().dimensions().size(), dims);
    const auto views = s.dimensionViews();
    ASSERT_EQ(views.size(), 2u);
    EXPECT_TRUE(views.back().driven);
    EXPECT_EQ(views.back().text, "(40)");
    ASSERT_TRUE(s.undo());
    EXPECT_TRUE(s.drivenDimensions().empty());
    ASSERT_TRUE(s.redo());
    EXPECT_EQ(s.drivenDimensions().size(), 1u);
    s.selectDimension(SketchSession::DimensionRef{*driven, true});
    ASSERT_TRUE(s.deleteSelection());
    EXPECT_TRUE(s.drivenDimensions().empty());
}

TEST(SketchDimensions, DimensionToolClicksCreateLengthDiameterAndAngle) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addLine({0, 0}, {30, 0}));
    ASSERT_TRUE(s.addCircleCenter({60, 0}, 5));
    ASSERT_TRUE(s.addLine({0, 20}, {20, 40}));
    s.setInference(InferenceSettings{false, 0.5});
    s.setTool(SketchTool::Dimension);
    ASSERT_TRUE(s.click({15, 0}));                      // the line
    ASSERT_TRUE(s.click({15, -8}));                     // placement
    auto ev = s.takeDimensionEvent();
    ASSERT_TRUE(ev);
    EXPECT_EQ(ev->type, DimensionEvent::Type::Created);
    EXPECT_EQ(ev->kind, sketch::DimensionKind::Distance);
    ASSERT_TRUE(s.click({60, 5}));                      // the circle
    ASSERT_TRUE(s.click({60, 15}));
    ev = s.takeDimensionEvent();
    ASSERT_TRUE(ev);
    EXPECT_EQ(ev->kind, sketch::DimensionKind::Diameter);
    EXPECT_FALSE(s.takeDimensionEvent());
    ASSERT_TRUE(s.click({15, 0}));                      // two lines: angle
    ASSERT_TRUE(s.click({10, 30}));
    EXPECT_TRUE(s.hasPending());
    ASSERT_TRUE(s.click({25, 10}));
    ev = s.takeDimensionEvent();
    ASSERT_TRUE(ev);
    EXPECT_EQ(ev->kind, sketch::DimensionKind::Angle);
    EXPECT_EQ(s.sketch().dimensions().size(), 3u);
    EXPECT_TRUE(s.report().ok());
}

TEST(SketchDrag, DraggingACornerKeepsTheRectangleClosedAndUndoes) {
    SketchSession s = makeSession();
    auto rect = s.addRect2pt({0, 0}, {40, 20});
    ASSERT_TRUE(rect);
    const std::size_t undoSize = s.undoStack().size();
    ASSERT_TRUE(s.beginDrag({(*rect)[1], PointRole::End}));              // top-right corner
    ASSERT_TRUE(s.dragTo({45, 24}));
    ASSERT_TRUE(s.dragTo({50, 30}));
    EXPECT_TRUE(s.dragging());
    s.endDrag();
    EXPECT_FALSE(s.dragging());
    const auto corner = *s.sketch().pointOf({(*rect)[1], PointRole::End});
    EXPECT_NEAR(corner.x, 50, 1e-5);
    EXPECT_NEAR(corner.y, 30, 1e-5);
    EXPECT_NEAR(sketch::distance(corner, *s.sketch().pointOf({(*rect)[2], PointRole::Start})), 0.0, 1e-6);
    EXPECT_EQ(s.profiles().size(), 1u);
    EXPECT_EQ(s.undoStack().size(), undoSize + 1);
    ASSERT_TRUE(s.undo());
    EXPECT_NEAR(s.sketch().pointOf({(*rect)[1], PointRole::End})->x, 40, kTol);
}

TEST(SketchDrag, SolverStepStaysWithinTheSixtyHertzBudget) {
    SketchSession s = makeSession();
    ASSERT_TRUE(s.addRect2pt({0, 0}, {40, 20}));
    ASSERT_TRUE(s.beginDrag({sketch::SketchEntityId{1}, PointRole::End}));
    QElapsedTimer t;
    t.start();
    for (int i = 1; i <= 20; ++i) ASSERT_TRUE(s.dragTo({40.0 + i * 0.5, 0.0 + i * 0.3}));
    EXPECT_LT(t.elapsed() / 20.0, 16.0);
    s.endDrag();
}

TEST(SketchMode, ConstraintToolbarAddsParallelAndGlyphClickPlusDeleteRemovesIt) {
    auto win = makeWindow();
    startXySketch(*win);
    render::Viewport* vp = win->viewport();
    SketchSession* s = win->sketchMode()->activeSession();
    ASSERT_TRUE(s->addLine({-20, -10}, {20, -6}));
    ASSERT_TRUE(s->addLine({-20, 10}, {20, 4}));
    win->sketchMode()->refresh();
    // Select both lines (Ctrl adds), then the palette button.
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {0, -8}));
    QTest::mouseClick(vp, Qt::LeftButton, Qt::ControlModifier, screenOf(*win, {0, 7}));
    ASSERT_EQ(s->selection().size(), 2u);
    auto* parallel = win->findChild<QToolButton*>("con_parallel");
    ASSERT_NE(parallel, nullptr);
    parallel->click();
    EXPECT_EQ(countConstraints(s->sketch(), ConstraintKind::Parallel), 1);

    // Click the glyph (it is drawn next to the first line), then Delete.
    const auto glyphs = s->glyphs();
    ASSERT_EQ(glyphs.size(), 1u);
    const auto at = glyphScreenPos(vp->camera(), vp->width(), vp->height(), s->plane(), glyphs[0]);
    ASSERT_TRUE(at);
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, at->toPoint());
    ASSERT_TRUE(s->selectedConstraint());
    QTest::keyClick(vp, Qt::Key_Delete);
    EXPECT_EQ(countConstraints(s->sketch(), ConstraintKind::Parallel), 0);
    EXPECT_EQ(s->sketch().entities().size(), 2u);
}

TEST(SketchMode, DimensionToolEditorAcceptsExpressionsAndDoubleClickReEdits) {
    auto win = makeWindow();
    ParameterTable params;
    ASSERT_TRUE(params.add("w", "40 mm"));
    ASSERT_TRUE(params.add("d1", "5 mm"));
    win->sketchMode()->setParameters(&params);
    startXySketch(*win);
    render::Viewport* vp = win->viewport();
    SketchSession* s = win->sketchMode()->activeSession();
    auto rect = s->addRect2pt({-20, -10}, {10, 10});
    ASSERT_TRUE(rect);
    win->sketchMode()->refresh();
    auto widthOf = [&] {
        const auto& g = std::get<LineGeom>(s->sketch().findEntity((*rect)[0])->geometry);
        return sketch::distance(g.start, g.end);
    };

    QTest::keyClick(vp, Qt::Key_D);
    EXPECT_EQ(s->tool(), SketchTool::Dimension);
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {-5, -10}));   // bottom side
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {-5, -18}));   // placement
    QLineEdit* editor = win->sketchMode()->dimensionEditor();
    ASSERT_NE(editor, nullptr);
    editor->setText("w");
    ASSERT_TRUE(win->sketchMode()->commitDimensionEditor());
    EXPECT_EQ(win->sketchMode()->dimensionEditor(), nullptr);
    EXPECT_NEAR(widthOf(), 40.0, 1e-6);

    // Parameter change -> geometry follows.
    ASSERT_TRUE(params.setExpression("w", "50 mm"));
    win->sketchMode()->parametersChanged();
    EXPECT_NEAR(widthOf(), 50.0, 1e-6);

    // An invalid expression keeps the editor open with the reason; Esc closes it.
    const auto views = s->dimensionViews();
    ASSERT_EQ(views.size(), 1u);
    win->sketchMode()->editDimension({views[0].id, false});
    editor = win->sketchMode()->dimensionEditor();
    ASSERT_NE(editor, nullptr);
    editor->setText("nope * 2");
    EXPECT_FALSE(win->sketchMode()->commitDimensionEditor());
    EXPECT_FALSE(editor->toolTip().isEmpty());
    QTest::keyClick(editor, Qt::Key_Escape);
    EXPECT_EQ(win->sketchMode()->dimensionEditor(), nullptr);

    // Double click on the dimension text re-opens the editor; `10 mm + d1` is accepted.
    QTest::keyClick(vp, Qt::Key_Escape);                                                // Dimension -> Select
    ASSERT_EQ(s->tool(), SketchTool::Select);
    const DimensionView dv = s->dimensionViews()[0];
    const auto lay = layoutDimension(vp->camera(), vp->width(), vp->height(), *s, dv);
    ASSERT_TRUE(lay);
    QTest::mouseDClick(vp, Qt::LeftButton, Qt::NoModifier, lay->textRect.center().toPoint());
    editor = win->sketchMode()->dimensionEditor();
    ASSERT_NE(editor, nullptr);
    editor->setText("10 mm + d1 * 2");
    ASSERT_TRUE(win->sketchMode()->commitDimensionEditor());
    EXPECT_NEAR(widthOf(), 20.0, 1e-6);
}

TEST(SketchMode, OverConstrainingDimensionOffersDrivenAndAnswerDecides) {
    auto win = makeWindow();
    startXySketch(*win);
    render::Viewport* vp = win->viewport();
    SketchSession* s = win->sketchMode()->activeSession();
    auto rect = s->addRect2pt({-20, -10}, {20, 10});
    ASSERT_TRUE(rect);
    ASSERT_TRUE(s->addDimension(sketch::DimensionKind::Distance,
                                {{(*rect)[0], PointRole::Start}, {(*rect)[0], PointRole::End}}));
    win->sketchMode()->refresh();
    int asked = 0;
    bool answer = false;
    win->sketchMode()->setDrivenPrompt([&](const QString&) {
        ++asked;
        return answer;
    });
    auto dimensionBottom = [&] {
        QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {0, -10}));
        QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {0, -22}));
    };
    QTest::keyClick(vp, Qt::Key_D);
    dimensionBottom();                         // bottom side again: redundant
    EXPECT_EQ(asked, 1);
    EXPECT_TRUE(s->drivenDimensions().empty());
    EXPECT_EQ(s->sketch().dimensions().size(), 1u);
    answer = true;
    dimensionBottom();
    EXPECT_EQ(asked, 2);
    EXPECT_EQ(s->drivenDimensions().size(), 1u);
    EXPECT_EQ(s->sketch().dimensions().size(), 1u);
    EXPECT_EQ(win->sketchMode()->dimensionEditor(), nullptr);
}

TEST(SketchMode, OffsetDistanceComesFromACommandPanel) {
    auto win = makeWindow();
    startXySketch(*win);
    render::Viewport* vp = win->viewport();
    SketchSession* s = win->sketchMode()->activeSession();
    ASSERT_TRUE(s->addRect2pt({-20, -10}, {20, 10}));
    win->sketchMode()->refresh();
    QTest::keyClick(vp, Qt::Key_O);
    EXPECT_EQ(win->sketchMode()->offsetPanel(), nullptr);
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, screenOf(*win, {0, -10}));
    CommandPanel* panel = win->sketchMode()->offsetPanel();
    ASSERT_NE(panel, nullptr);
    EXPECT_EQ(s->sketch().entities().size(), 4u);                        // nothing happens until OK
    panel->setText("distance", "d + 1");
    EXPECT_FALSE(panel->okEnabled());                                    // unknown parameter
    panel->setText("distance", "3 mm");
    EXPECT_TRUE(panel->okEnabled());
    panel->accept();
    EXPECT_EQ(s->sketch().entities().size(), 8u);
    double minx = 1e9, maxx = -1e9;
    for (const auto& [id, e] : s->sketch().entities()) {
        if (const auto* l = std::get_if<LineGeom>(&e.geometry)) {
            minx = std::min({minx, l->start.x, l->end.x});
            maxx = std::max({maxx, l->start.x, l->end.x});
        }
    }
    EXPECT_NEAR(minx, -23, kTol);
    EXPECT_NEAR(maxx, 23, kTol);
    EXPECT_EQ(win->sketchMode()->offsetPanel(), nullptr);                // finished -> closed
    EXPECT_NEAR(s->offsetDistance(), 3.0, kTol);                         // remembered as the next default
}

TEST(SketchMode, ScreenshotOfAFullyDimensionedSketch) {
    const char* dir = std::getenv("IW_TEST_DUMP_DIR");
    ParameterTable params;
    ASSERT_TRUE(params.add("w", "60 mm"));
    ASSERT_TRUE(params.add("h", "30 mm"));
    SketchSession s = makeSession();
    s.setParameters(&params);
    using DK = sketch::DimensionKind;
    auto rect = s.addRect2pt({0, 0}, {50, 25});
    ASSERT_TRUE(rect);
    auto origin = s.addPoint({0, 0});
    ASSERT_TRUE(origin);
    s.select(*origin);
    ASSERT_TRUE(s.toggleFix());
    s.setSelection({{*origin, std::nullopt}, {(*rect)[0], PointRole::Start}});
    ASSERT_TRUE(s.applyConstraint(ConstraintKind::Coincident));
    ASSERT_TRUE(s.addDimension(DK::Distance, {{(*rect)[0], PointRole::Start}, {(*rect)[0], PointRole::End}}, "w"));
    ASSERT_TRUE(s.addDimension(DK::Distance, {{(*rect)[3], PointRole::Start}, {(*rect)[3], PointRole::End}}, "h"));
    auto circle = s.addCircleCenter({30, 15}, 5);
    ASSERT_TRUE(circle);
    s.setSelection({{*circle, PointRole::Center}, {(*rect)[0], PointRole::Start}});
    // Dimension the centre from the origin and the circle's diameter.
    ASSERT_TRUE(s.addDimension(DK::DistanceX, {{*origin, std::nullopt}, {*circle, PointRole::Center}}, "w / 2"));
    ASSERT_TRUE(s.addDimension(DK::DistanceY, {{*origin, std::nullopt}, {*circle, PointRole::Center}}, "h / 2"));
    ASSERT_TRUE(s.addDimension(DK::Diameter, {{*circle, std::nullopt}}, "10 mm"));
    ASSERT_TRUE(s.resolve());
    s.clearSelection();
    EXPECT_EQ(s.report().dof, 0);
    EXPECT_EQ(s.constraintState(), ConstraintState::Fully);
    EXPECT_NEAR(std::get<sketch::CircleGeom>(s.sketch().findEntity(*circle)->geometry).center.x, 30.0, 1e-6);
    if (!dir) GTEST_SKIP() << "set IW_TEST_DUMP_DIR to save the screenshot";

    auto win = makeWindow();
    for (const bool dark : {false, true}) {
        render::Camera c = win->viewport()->camera();
        c.setViewportSize(900, 600);
        c.setState(c.poseForEyeDirection(s.plane().normal));
        Box3 box;
        for (const Point2& q : {Point2{-12, -18}, Point2{66, 40}}) box.extend(s.plane().toWorld(q));
        c.fit(box, 1.1);
        QImage img(900, 600, QImage::Format_ARGB32_Premultiplied);
        img.fill(dark ? QColor(30, 34, 40) : QColor(236, 238, 242));
        {
            QPainter p(&img);
            SketchViewOptions o;
            o.dark = dark;
            paintSketches(p, c, 900, 600, {&s}, &s, o);
        }
        EXPECT_TRUE(img.save(QString::fromLocal8Bit(dir) + (dark ? "/ui_sketch_dimensioned_dark.png" : "/ui_sketch_dimensioned_light.png")));
    }
}

} // namespace
