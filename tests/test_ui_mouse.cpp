// SPDX-License-Identifier: GPL-3.0-or-later
// UX1: real mouse input on the viewport. Every interaction below is a QTest mouse or key event sent to the Viewport (or
// to one of its child widgets: ViewCube, manipulator overlay, dialog lists) at projected screen coordinates of known
// faces, edges and profiles. Picks go through the offscreen GPU-pick path (QPA offscreen, Null RHI).
// Offscreen only (QT_QPA_PLATFORM=offscreen is set by the environment below).
#include <gtest/gtest.h>

#include <QApplication>
#include <QCoreApplication>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QLabel>
#include <QStatusBar>
#include <QListWidget>
#include <QPushButton>
#include <QTest>

#include <cmath>
#include <cstdlib>
#include <numbers>
#include <string>

#include "core/math.hpp"
#include "kernel/shape.hpp"
#include "render/camera.hpp"
#include "render/viewcube_widget.hpp"
#include "render/viewport.hpp"
#include "sketch/sketch.hpp"
#include "ui/command_dialog.hpp"
#include "ui/main_window.hpp"
#include "ui/manipulators.hpp"
#include "ui/sketch/plane_chooser.hpp"
#include "ui/sketch/session.hpp"
#include "ui/sketch/sketch_mode.hpp"
#include "ui/sketch/view_math.hpp"
#include "ui/timeline/doc_eval.hpp"

namespace {

using namespace iw;

class QtEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
        app_ = new QApplication(argc_, argv_);
        QCoreApplication::setOrganizationName("IronwoodTest");
        QCoreApplication::setApplicationName("ironwood-ui-mouse-test");
    }
    void TearDown() override { delete app_; }

private:
    int argc_ = 1;
    char name_[16] = "test_ui_mouse";
    char* argv_[2] = {name_, nullptr};
    QApplication* app_ = nullptr;
};
const auto* const kEnv = ::testing::AddGlobalTestEnvironment(new QtEnv);

/// Waits (processing events) until `pred` holds or `ms` elapse. Returns the final value of `pred`.
template <class Pred>
bool waitFor(Pred pred, int ms = 3000) {
    QElapsedTimer t;
    t.start();
    while (!pred() && t.elapsed() < ms) QTest::qWait(5);
    return pred();
}

bool sameView(const render::CameraState& a, const render::CameraState& b) {
    constexpr double eps = 1e-9;
    return std::abs(a.target.x - b.target.x) < eps && std::abs(a.target.y - b.target.y) < eps &&
           std::abs(a.target.z - b.target.z) < eps && std::abs(a.distance - b.distance) < eps &&
           std::abs(a.yaw - b.yaw) < eps && std::abs(a.pitch - b.pitch) < eps;
}

/// Widget pixel of a world point (y down), as the viewport draws it.
QPoint worldPx(const render::Viewport* vp, const Vec3& p) {
    auto ndc = vp->camera().project(p);
    EXPECT_TRUE(ndc.has_value());
    if (!ndc) return {};
    return QPoint(static_cast<int>(std::lround((ndc->x + 1.0) * 0.5 * vp->width())),
                  static_cast<int>(std::lround((1.0 - ndc->y) * 0.5 * vp->height())));
}

/// Mesh of the first scene body (the body the feature tests pick on).
const tess::MeshData& firstMesh(const render::Viewport* vp) {
    static const tess::MeshData empty;
    if (vp->scene().empty() || !vp->scene().front().mesh) return empty;
    return *vp->scene().front().mesh;
}

/// World midpoint of B-rep edge `index` of the first scene body (mean of its polyline's end points).
Vec3 edgeMidpoint(const render::Viewport* vp, std::uint32_t index) {
    const auto& mesh = firstMesh(vp);
    const auto& pts = mesh.edgePolylines.at(index).points;
    const std::size_t n = pts.size() / 3;
    const Vec3 a{pts[0], pts[1], pts[2]}, b{pts[3 * (n - 1)], pts[3 * (n - 1) + 1], pts[3 * (n - 1) + 2]};
    return vp->scene().front().transform.apply((a + b) * 0.5);
}

/// World centroid of the planar face `index` (mean of its triangle vertices) of the first scene body.
Vec3 faceCentroid(const render::Viewport* vp, std::uint32_t index) {
    const auto& mesh = firstMesh(vp);
    const auto& r = mesh.faceRanges.at(index);
    Vec3 sum{};
    for (std::uint32_t k = 0; k < r.indexCount; ++k) {
        const std::size_t v = 3 * static_cast<std::size_t>(mesh.indices[r.firstIndex + k]);
        sum = sum + Vec3{mesh.positions[v], mesh.positions[v + 1], mesh.positions[v + 2]};
    }
    return vp->scene().front().transform.apply(sum * (1.0 / r.indexCount));
}

/// Index of the planar face of the first body whose vertices all lie at height `z` (mm) on the Z axis.
std::uint32_t faceAtHeight(const render::Viewport* vp, double z) {
    const auto& mesh = firstMesh(vp);
    for (std::uint32_t f = 0; f < mesh.faceRanges.size(); ++f) {
        const auto& r = mesh.faceRanges[f];
        if (r.indexCount == 0) continue;
        bool flat = true;
        for (std::uint32_t k = 0; k < r.indexCount && flat; ++k) {
            const std::size_t v = 3 * static_cast<std::size_t>(mesh.indices[r.firstIndex + k]);
            flat = std::abs(mesh.positions[v + 2] - z) < 1e-3;
        }
        if (flat) return f;
    }
    ADD_FAILURE() << "no face at height " << z;
    return 0;
}

double shownVolume(ui::MainWindow& w) {
    const auto& ev = w.shownEvaluation().eval;
    if (ev.bodies.empty()) return -1;
    auto v = ui::timeline::volumeOfBody(ev, ev.bodies.begin()->first);
    return v ? *v : -1;
}

ui::CommandPanel* openPanel(ui::MainWindow& w) { return qobject_cast<ui::CommandPanel*>(w.commandDock()->widget()); }

QPushButton* buttonNamed(QWidget* root, const QString& text) {
    for (QPushButton* b : root->findChildren<QPushButton*>())
        if (b->text() == text) return b;
    return nullptr;
}

/// True when some status-bar label shows `text` (the shell's status line).
bool statusShows(ui::MainWindow& w, const QString& text) {
    for (QLabel* l : w.statusBar()->findChildren<QLabel*>())
        if (l->text() == text) return true;
    return false;
}

/// Sketch 40 x 20 on the XY plane (user rectangle, no dimensions), finished into a sketch feature.
void sketchRectangle(ui::MainWindow& w) {
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    auto* s = w.sketchMode()->activeSession();
    auto rect = s->addRect2pt({0, 0}, {40, 20});
    EXPECT_TRUE(rect.has_value());
    w.sketchMode()->finish();
    EXPECT_TRUE(w.settle(5000));
}

/// Sketch rectangle extruded 10 mm (a 40 x 20 x 10 block) with the default Extrude dialog.
void buildBlock(ui::MainWindow& w) {
    sketchRectangle(w);
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_NEAR(shownVolume(w), 40.0 * 20.0 * 10.0, 0.05);
}

/// Left drag from `from` to `to` in `steps` moves, as the window system delivers it.
void leftDrag(QWidget* target, QPoint from, QPoint to, int steps = 8) {
    QTest::mousePress(target, Qt::LeftButton, Qt::NoModifier, from);
    for (int i = 1; i <= steps; ++i) {
        const QPoint p = from + (to - from) * i / steps;
        QTest::mouseMove(target, p);
    }
    QTest::mouseRelease(target, Qt::LeftButton, Qt::NoModifier, to);
}

/// Hovers `px` on the viewport until the highlighted edge is `edge`, or the timeout passes.
bool hoverEdge(render::Viewport* vp, QPoint px, std::uint32_t edge) {
    QTest::mouseMove(vp, px);
    return waitFor([vp, edge] { return vp->hover().edgeIndex && *vp->hover().edgeIndex == edge; });
}

/// A 10 mm edge of the block that is visible from the camera: the first whose midpoint hovers as an edge.
/// Returns the edge index and its screen point (the hover state is left on it).
std::pair<std::uint32_t, QPoint> visibleEdgeOfLength(ui::MainWindow& w, double length) {
    render::Viewport* vp = w.viewport();
    const auto& ev = w.shownEvaluation().eval;
    const kernel::Shape& s = ev.bodies.begin()->second;
    for (std::uint32_t i = 0; i < s.subShapeCount(kernel::SubShapeKind::Edge); ++i) {
        if (std::abs(s.subShapeKey(kernel::SubShapeKind::Edge, i)[3] - length) > 1e-6) continue;
        const QPoint px = worldPx(vp, edgeMidpoint(vp, i));
        if (hoverEdge(vp, px, i)) return {i, px};
    }
    ADD_FAILURE() << "no visible edge of length " << length;
    return {0, QPoint()};
}

// ---- sketch: flat view ----------------------------------------------------------------------------------------------

TEST(UiMouseSketch, LeftDragNeverMovesTheCameraButMiddleDragStillPans) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    render::Viewport* vp = w.viewport();
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ASSERT_TRUE(waitFor([vp] { return !vp->isAnimating(); }));
    EXPECT_EQ(vp->projectionMode(), render::Projection::Orthographic) << "sketch mode is a flat (orthographic) view";
    const render::CameraState flat = vp->camera().state();

    const QPoint centre(vp->width() / 2, vp->height() / 2);
    leftDrag(vp, centre, centre + QPoint(160, 90));
    leftDrag(vp, centre, centre - QPoint(200, 0));
    leftDrag(vp, centre, centre + QPoint(0, 120));
    EXPECT_TRUE(sameView(flat, vp->camera().state())) << "left drag moved the camera in sketch mode";

    // The ViewCube is no way round it either: a left drag that starts on the cube does not orbit.
    auto* cube = vp->findChild<render::ViewCubeWidget*>();
    ASSERT_NE(cube, nullptr);
    leftDrag(cube, cube->rect().center(), cube->rect().center() + QPoint(60, 40));
    EXPECT_TRUE(sameView(flat, vp->camera().state())) << "a left drag on the ViewCube orbited the camera";

    // Positive control: the middle button still pans the flat view.
    QTest::mousePress(vp, Qt::MiddleButton, Qt::NoModifier, centre);
    QTest::mouseMove(vp, centre + QPoint(80, 0));
    QTest::mouseRelease(vp, Qt::MiddleButton, Qt::NoModifier, centre + QPoint(80, 0));
    EXPECT_FALSE(sameView(flat, vp->camera().state())) << "middle drag should still pan";

    // Finishing the sketch restores the camera and projection from before it started.
    w.sketchMode()->finish();
    ASSERT_TRUE(waitFor([vp] { return !vp->isAnimating(); }));
    EXPECT_EQ(vp->projectionMode(), render::Projection::Perspective);
    EXPECT_FALSE(vp->flatViewLocked());
}

TEST(UiMouseSketch, LookAtButtonFacesThePlaneAgain) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    render::Viewport* vp = w.viewport();
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ASSERT_TRUE(waitFor([vp] { return !vp->isAnimating(); }));
    const QPoint centre(vp->width() / 2, vp->height() / 2);
    // Shift + middle orbits (Fusion); the flat view turns away from the plane.
    QTest::mousePress(vp, Qt::MiddleButton, Qt::ShiftModifier, centre);
    QTest::mouseMove(vp, centre + QPoint(60, 40));
    QTest::mouseMove(vp, centre + QPoint(120, 80));
    QTest::mouseRelease(vp, Qt::MiddleButton, Qt::ShiftModifier, centre + QPoint(120, 80));
    ASSERT_LT(std::abs(vp->camera().forward().z), 0.999) << "Shift+middle should orbit the view";

    QPushButton* lookAt = w.findChild<QPushButton*>("sketchLookAt");
    ASSERT_NE(lookAt, nullptr);
    QTest::mouseClick(lookAt, Qt::LeftButton);
    ASSERT_TRUE(waitFor([vp] { return !vp->isAnimating(); }));
    EXPECT_NEAR(std::abs(vp->camera().forward().z), 1.0, 1e-6) << "Look At faces the XY plane";
    w.sketchMode()->finish();
}

TEST(UiMouseSketch, RectangleDrawnByTwoClicks) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    render::Viewport* vp = w.viewport();
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ASSERT_TRUE(waitFor([vp] { return !vp->isAnimating(); }));
    auto* s = w.sketchMode()->activeSession();
    QTest::keyClick(vp, Qt::Key_R);
    ASSERT_EQ(s->tool(), ui::SketchTool::Rectangle);

    const sketch::SketchPlane plane = s->plane();
    const auto a = ui::planeToScreen(vp->camera(), vp->width(), vp->height(), plane, sketch::Point2{0, 0});
    const auto b = ui::planeToScreen(vp->camera(), vp->width(), vp->height(), plane, sketch::Point2{40, 20});
    ASSERT_TRUE(a && b);
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, QPoint(static_cast<int>(a->x), static_cast<int>(a->y)));
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, QPoint(static_cast<int>(b->x), static_cast<int>(b->y)));
    std::size_t lines = 0;
    for (const auto& [id, e] : s->sketch().entities())
        if (!sketch::isReferenceEntity(id) && std::holds_alternative<sketch::LineGeom>(e.geometry)) ++lines;
    EXPECT_EQ(lines, 4u) << "two clicks make a rectangle of four lines";
    w.sketchMode()->finish();
    EXPECT_TRUE(w.settle(5000));
}

// ---- command dialogs: selection with hover, click, Esc --------------------------------------------------------------

TEST(UiMouseFillet, HoverHighlightsEdgesOnlyThenClickSelectsOneEdgeAndOkChangesTheBody) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildBlock(w);
    render::Viewport* vp = w.viewport();
    ASSERT_TRUE(w.startFeature("fillet").has_value());
    ui::CommandPanel* fi = openPanel(w);
    ASSERT_NE(fi, nullptr);
    EXPECT_EQ(fi->activeSelection(), "edges") << "the first selection input is active when the dialog opens";
    EXPECT_TRUE(statusShows(w, QStringLiteral("Select edges to fillet"))) << "status hint";
    EXPECT_TRUE(vp->leftButtonReserved());

    // Hovering a face highlights nothing: the filter accepts edges only.
    const Vec3 faceC = faceCentroid(vp, faceAtHeight(vp, 10.0));
    QTest::mouseMove(vp, worldPx(vp, faceC));
    QTest::qWait(150);
    EXPECT_FALSE(vp->hover().faceIndex.has_value()) << "faces are not highlighted by a fillet";
    EXPECT_FALSE(vp->hover().edgeIndex.has_value());

    // Hovering a visible edge highlights it.
    const auto [edge, onEdge] = visibleEdgeOfLength(w, 10.0);
    ASSERT_TRUE(hoverEdge(vp, onEdge, edge)) << "hovered edge should be highlighted";

    // A click selects it: the input shows one edge.
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, onEdge);
    ASSERT_TRUE(waitFor([fi] { return fi->model().selection("edges").size() == 1; }));
    auto* count = fi->findChild<QLabel*>("count_edges");
    ASSERT_NE(count, nullptr);
    EXPECT_EQ(count->text(), QStringLiteral("1 edge"));

    // Clicking the same edge again toggles it off; then select it again.
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, onEdge);
    ASSERT_TRUE(waitFor([fi] { return fi->model().selection("edges").empty(); }));
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, onEdge);
    ASSERT_TRUE(waitFor([fi] { return fi->model().selection("edges").size() == 1; }));

    // OK (through its button) makes the fillet: the body loses the fillet's material.
    fi->setText("radius", QStringLiteral("1 mm"));
    QPushButton* ok = buttonNamed(fi, QStringLiteral("OK"));
    ASSERT_NE(ok, nullptr);
    QTest::mouseClick(ok, Qt::LeftButton);
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.model()->timeline().size(), 3u);
    const double filletRemoved = (1.0 - std::numbers::pi / 4.0) * 1.0 * 1.0 * 10.0;
    EXPECT_NEAR(shownVolume(w), 40.0 * 20.0 * 10.0 - filletRemoved, 0.05);
    EXPECT_FALSE(vp->leftButtonReserved()) << "the dialog released the left button";
}

TEST(UiMouseFillet, EscapeClearsTheSelectionThenCancelsTheDialog) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildBlock(w);
    render::Viewport* vp = w.viewport();
    ASSERT_TRUE(w.startFeature("fillet").has_value());
    ui::CommandPanel* fi = openPanel(w);
    const auto [edge, onEdge] = visibleEdgeOfLength(w, 10.0);
    ASSERT_TRUE(hoverEdge(vp, onEdge, edge));
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, onEdge);
    ASSERT_TRUE(waitFor([fi] { return fi->model().selection("edges").size() == 1; }));

    QTest::keyClick(vp, Qt::Key_Escape);
    EXPECT_TRUE(fi->model().selection("edges").empty()) << "Escape with items clears the selection";
    EXPECT_TRUE(w.commandDock()->isVisible()) << "the dialog stays open";
    EXPECT_FALSE(vp->selection().edgeIndex.has_value()) << "the orange pick follows the empty list";

    QTest::keyClick(vp, Qt::Key_Escape);
    EXPECT_FALSE(w.commandDock()->isVisible()) << "Escape with nothing selected cancels the dialog";
    EXPECT_FALSE(vp->leftButtonReserved());
    EXPECT_EQ(w.model()->timeline().size(), 2u) << "cancelled: no fillet";
}

TEST(UiMouseFillet, LeftDragWhileDialogOpenNeverOrbits) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildBlock(w);
    render::Viewport* vp = w.viewport();
    ASSERT_TRUE(w.startFeature("fillet").has_value());
    const render::CameraState before = vp->camera().state();
    const QPoint centre(vp->width() / 2, vp->height() / 2);
    leftDrag(vp, centre, centre + QPoint(140, 70));
    leftDrag(vp, centre + QPoint(-200, -60), centre + QPoint(200, 60));
    EXPECT_TRUE(sameView(before, vp->camera().state())) << "a left drag in a pick dialog must not orbit or pan";
}

TEST(UiMouseFillet, RightClickDonesTheSelectionWithoutTheMarkingMenu) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildBlock(w);
    render::Viewport* vp = w.viewport();
    ASSERT_TRUE(w.startFeature("fillet").has_value());
    ui::CommandPanel* fi = openPanel(w);
    fi->setText("radius", QStringLiteral("1 mm"));
    const auto [edge, onEdge] = visibleEdgeOfLength(w, 10.0);
    ASSERT_TRUE(hoverEdge(vp, onEdge, edge));
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, onEdge);
    ASSERT_TRUE(waitFor([fi] { return fi->model().selection("edges").size() == 1; }));

    QTest::mouseClick(vp, Qt::RightButton, Qt::NoModifier, onEdge);
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(w.model()->timeline().size(), 3u) << "right click accepts the dialog when it is valid";
    EXPECT_NEAR(shownVolume(w), 40.0 * 20.0 * 10.0 - (1.0 - std::numbers::pi / 4.0) * 10.0, 0.05);
}

TEST(UiMouseHole, HoverFaceThenClickItAndOkCutsTheBody) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildBlock(w);
    render::Viewport* vp = w.viewport();
    ASSERT_TRUE(w.startFeature("hole").has_value());
    ui::CommandPanel* hi = openPanel(w);
    ASSERT_NE(hi, nullptr);
    EXPECT_TRUE(statusShows(w, QStringLiteral("Select a face to place the hole")));

    // The top face (z = 10): hovering a nearby edge highlights nothing for a hole.
    const std::uint32_t top = faceAtHeight(vp, 10.0);
    const QPoint onFace = worldPx(vp, faceCentroid(vp, top));
    QTest::mouseMove(vp, onFace);
    ASSERT_TRUE(waitFor([vp] { return vp->hover().faceIndex.has_value(); })) << "the top face is highlighted";
    EXPECT_EQ(*vp->hover().faceIndex, top);

    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, onFace);
    ASSERT_TRUE(waitFor([hi] { return hi->model().selection("face").size() == 1; }));
    hi->setText("through", QStringLiteral("1"));
    QPushButton* ok = buttonNamed(hi, QStringLiteral("OK"));
    ASSERT_NE(ok, nullptr);
    const double before = shownVolume(w);
    QTest::mouseClick(ok, Qt::LeftButton);
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(w.model()->timeline().size(), 3u);
    EXPECT_LT(shownVolume(w), before - 1.0) << "the hole removes material";
}

TEST(UiMouseExtrude, ClickProfileDragArrowThenOk) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    sketchRectangle(w);
    render::Viewport* vp = w.viewport();
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    ui::CommandPanel* ei = openPanel(w);
    ASSERT_NE(ei, nullptr);
    ASSERT_FALSE(vp->overlayFaces().empty()) << "the sketch profile is drawn for picking";

    // Hover the profile: it is highlighted and the status names the hint.
    Vec3 centre{};
    std::size_t n = 0;
    for (const auto& t : vp->overlayFaces().front().triangles) {
        centre = centre + t;
        ++n;
    }
    centre = centre * (1.0 / static_cast<double>(n));
    const QPoint onProfile = worldPx(vp, centre);
    QTest::mouseMove(vp, onProfile);
    QTest::qWait(150);
    QTest::mouseClick(vp, Qt::LeftButton, Qt::NoModifier, onProfile);
    ASSERT_TRUE(waitFor([ei] { return !ei->model().text("profile").empty(); })) << "a click chooses the profile";

    // Drag the distance arrow along the normal: the field follows and OK extrudes to that distance.
    ui::ManipulatorOverlay* arrow = w.manipulators();
    ASSERT_EQ(arrow->mode(), ui::ManipulatorOverlay::Mode::Distance);
    const QPoint handle = arrow->handlePos().toPoint();
    // Unit screen direction of the sketch normal at the profile centre (the arrow's own direction).
    const QPointF a = worldPx(vp, centre), b = worldPx(vp, centre + Vec3{0, 0, 1});
    const QPointF dirPx = (b - a) / std::hypot(b.x() - a.x(), b.y() - a.y());
    const QPoint tip = handle + (dirPx * 30.0).toPoint();
    QTest::mousePress(arrow, Qt::LeftButton, Qt::NoModifier, handle);
    QTest::mouseMove(arrow, handle + (dirPx * 15.0).toPoint());
    QTest::mouseMove(arrow, tip);
    QTest::mouseRelease(arrow, Qt::LeftButton, Qt::NoModifier, tip);
    ASSERT_TRUE(waitFor([ei] { return ei->model().valueOf("distance").has_value(); }));
    const double dist = ei->model().valueOf("distance")->value;
    EXPECT_GT(dist, 10.0) << "dragging the arrow along its normal lengthens the extrusion";

    QPushButton* ok = buttonNamed(ei, QStringLiteral("OK"));
    ASSERT_NE(ok, nullptr);
    QTest::mouseClick(ok, Qt::LeftButton);
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(w.model()->timeline().size(), 2u);
    EXPECT_NEAR(shownVolume(w), 40.0 * 20.0 * dist, 0.5);
}

} // namespace
