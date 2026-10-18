// SPDX-License-Identifier: GPL-3.0-or-later
// UI2: Edit Sketch from the timeline (SketchSession from an existing sketch, Finish replaces it as one undo step, Esc
// discards) and the extrude distance arrow (drag sets the field and the preview, typing moves the arrow).
// Offscreen only (QT_QPA_PLATFORM=offscreen is set by the environment below).
#include <gtest/gtest.h>

#include <QAbstractItemView>
#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QDockWidget>
#include <QEventLoop>
#include <QMouseEvent>
#include <QTest>
#include <QThread>

#include <cmath>
#include <cstdlib>
#include <numbers>
#include <optional>
#include <string>

#include "core/math.hpp"
#include "kernel/shape.hpp"
#include "recompute/evaluator.hpp"
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
#include "ui/timeline/timeline_widget.hpp"

namespace {

// Entities the user made: every sketch also holds the fixed reference geometry (origin + two axes, F46).
std::size_t countUserEntities(const iw::sketch::Sketch& s) {
    std::size_t n = 0;
    for (const auto& [id, e] : s.entities())
        if (!iw::sketch::isReferenceEntity(id)) ++n;
    return n;
}

using namespace iw;

class QtEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
        app_ = new QApplication(argc_, argv_);
        QCoreApplication::setOrganizationName("IronwoodTest");
        QCoreApplication::setApplicationName("ironwood-ui-edit-test");
    }
    void TearDown() override { delete app_; }

private:
    int argc_ = 1;
    char name_[16] = "test_ui_edit";
    char* argv_[2] = {name_, nullptr};
    QApplication* app_ = nullptr;
};
const auto* const kEnv = ::testing::AddGlobalTestEnvironment(new QtEnv);

ui::CommandPanel* openPanel(ui::MainWindow& w) { return qobject_cast<ui::CommandPanel*>(w.commandDock()->widget()); }

double shownVolume(ui::MainWindow& w) {
    const auto& ev = w.shownEvaluation().eval;
    if (ev.bodies.empty()) return -1;
    auto v = ui::timeline::volumeOfBody(ev, ev.bodies.begin()->first);
    return v ? *v : -1;
}

/// Index of a box edge of the first body whose length is `length` (mm).
std::uint32_t edgeOfLength(ui::MainWindow& w, double length) {
    const auto& ev = w.shownEvaluation().eval;
    const kernel::Shape& s = ev.bodies.begin()->second;
    for (std::uint32_t i = 0; i < s.subShapeCount(kernel::SubShapeKind::Edge); ++i)
        if (std::abs(s.subShapeKey(kernel::SubShapeKind::Edge, i)[3] - length) < 1e-6) return i;
    ADD_FAILURE() << "no edge of length " << length;
    return 0;
}

void doubleClickItem(ui::MainWindow& w, std::size_t index) {
    auto* tw = w.timelineWidget();
    QTest::mouseDClick(tw, Qt::LeftButton, Qt::NoModifier, tw->itemRect(index).center());
}

/// Sketch 40 x 20 on XY with a driving length on the bottom (40) and right (20) lines, finished. Returns the entity
/// ids of the four lines, the bottom dimension id and the id of the sketch feature.
struct Block {
    sketch::SketchEntityId bottom, right, top, left;
    sketch::SketchDimensionId bottomDim;
    FeatureId sketchId;
};

Block buildSketch(ui::MainWindow& w) {
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    auto* s = w.sketchMode()->activeSession();
    auto rect = s->addRect2pt({0, 0}, {40, 20});
    EXPECT_TRUE(rect.has_value());
    Block b{};
    if (rect) {
        b.bottom = (*rect)[0];
        b.right = (*rect)[1];
        b.top = (*rect)[2];
        b.left = (*rect)[3];
    }
    auto d1 = s->addDimension(sketch::DimensionKind::Distance,
                          {sketch::SketchRef{b.bottom, sketch::PointRole::Start}, sketch::SketchRef{b.bottom, sketch::PointRole::End}},
                          "40 mm");
    EXPECT_TRUE(d1.has_value() && d1->id.has_value());
    if (d1 && d1->id) b.bottomDim = *d1->id;
    auto d2 = s->addDimension(sketch::DimensionKind::Distance,
                          {sketch::SketchRef{b.right, sketch::PointRole::Start}, sketch::SketchRef{b.right, sketch::PointRole::End}},
                          "20 mm");
    EXPECT_TRUE(d2.has_value() && d2->id.has_value());
    w.sketchMode()->finish();
    EXPECT_TRUE(w.settle(5000));
    b.sketchId = w.model()->timeline().at(0)->id();
    return b;
}

/// Expression text of the dimension `id` of `sk` (empty when missing).
std::string expressionOf(const sketch::Sketch& sk, sketch::SketchDimensionId id) {
    auto it = sk.dimensions().find(id);
    return it == sk.dimensions().end() ? std::string() : it->second.expression;
}

// ---- Edit Sketch from the timeline ----------------------------------------------------------------------------

TEST(UiEditSketch, DoubleClickSketchReentersItAndFinishReplacesItInOneUndoStep) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    const Block b = buildSketch(w);
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_TRUE(w.startFeature("fillet").has_value());
    ui::CommandPanel* fi = openPanel(w);
    render::PickResult pick;
    pick.bodyIndex = 0;
    pick.edgeIndex = edgeOfLength(w, 10.0);
    w.routePick(pick);
    fi->setText("radius", QStringLiteral("1 mm"));
    fi->accept();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.model()->timeline().size(), 3u);
    const double filletRemoved = (1.0 - std::numbers::pi / 4.0) * 1.0 * 1.0 * 10.0;
    ASSERT_NEAR(shownVolume(w), 40.0 * 20.0 * 10.0 - filletRemoved, 0.05);
    const std::string undoBefore = w.model()->undoText();

    // Double-click the sketch item: sketch mode opens with that sketch (same ids, same dimensions).
    doubleClickItem(w, 0);
    ASSERT_TRUE(w.sketchMode()->active());
    ASSERT_TRUE(w.sketchMode()->editing());
    auto* session = w.sketchMode()->activeSession();
    ASSERT_NE(session, nullptr);
    EXPECT_EQ(countUserEntities(session->sketch()), 4u);  // reference geometry (origin, axes) excluded
    EXPECT_TRUE(session->sketch().findEntity(b.bottom) != nullptr);
    EXPECT_TRUE(session->sketch().findEntity(b.left) != nullptr);
    EXPECT_EQ(session->sketch().dimensions().size(), 2u);
    EXPECT_TRUE(session->sketch().dimensions().count(b.bottomDim) == 1) << "dimension ids are kept";

    // Change the bottom length: the rectangle becomes 50 x 20 (the solver moves the geometry).
    ASSERT_TRUE(session->setDimensionExpression(b.bottomDim, "50 mm").has_value());
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));

    ASSERT_EQ(w.model()->timeline().size(), 3u) << "editing does not add or remove features";
    const auto& st = w.shownEvaluation().eval.statuses;
    ASSERT_EQ(st.size(), 3u);
    for (const auto& s : st) EXPECT_NE(s.state, recompute::FeatureStatus::State::Failed) << s.error.message;
    EXPECT_NEAR(shownVolume(w), 50.0 * 20.0 * 10.0 - filletRemoved, 0.05) << "extrude and fillet re-evaluated";

    // The stored sketch keeps the ids and carries the new expression.
    auto stored = w.model()->sketchForEdit(b.sketchId);
    ASSERT_TRUE(stored.has_value());
    EXPECT_NE(stored->findEntity(b.bottom), nullptr);
    EXPECT_NE(stored->findEntity(b.top), nullptr);
    EXPECT_EQ(expressionOf(*stored, b.bottomDim), "50 mm");

    // One undo step restores the previous sketch and the previous volume.
    EXPECT_NE(w.model()->undoText(), undoBefore);
    w.modelUndo();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_NEAR(shownVolume(w), 40.0 * 20.0 * 10.0 - filletRemoved, 0.05);
    auto back = w.model()->sketchForEdit(b.sketchId);
    ASSERT_TRUE(back.has_value());
    EXPECT_EQ(expressionOf(*back, b.bottomDim), "40 mm");
    EXPECT_EQ(w.model()->timeline().size(), 3u);
}

TEST(UiEditSketch, EscapeDiscardsTheEditAndLeavesTheDocumentUnchanged) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    const Block b = buildSketch(w);
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));
    const double before = shownVolume(w);
    const std::string undoBefore = w.model()->undoText();
    const bool canUndoBefore = w.model()->canUndo();

    doubleClickItem(w, 0);
    ASSERT_TRUE(w.sketchMode()->editing());
    auto* session = w.sketchMode()->activeSession();
    ASSERT_TRUE(session->setDimensionExpression(b.bottomDim, "70 mm").has_value());
    ASSERT_NEAR(session->profiles().empty() ? 0.0 : session->profiles().front().area, 70.0 * 20.0, 1e-6)
        << "the edit is live in the session";

    // Esc: no tool is pending, so the edit is cancelled.
    EXPECT_TRUE(w.sketchMode()->escape());
    EXPECT_FALSE(w.sketchMode()->active());
    ASSERT_TRUE(w.settle(5000));

    EXPECT_EQ(w.model()->undoText(), undoBefore) << "no undo step was recorded";
    EXPECT_EQ(w.model()->canUndo(), canUndoBefore);
    EXPECT_NEAR(shownVolume(w), before, 1e-6);
    auto stored = w.model()->sketchForEdit(b.sketchId);
    ASSERT_TRUE(stored.has_value());
    EXPECT_EQ(expressionOf(*stored, b.bottomDim), "40 mm");
}

TEST(UiEditSketch, FinishWithoutChangesDoesNotTouchTheDocument) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildSketch(w);
    const std::string undoBefore = w.model()->undoText();
    doubleClickItem(w, 0);
    ASSERT_TRUE(w.sketchMode()->editing());
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(w.model()->undoText(), undoBefore);
    EXPECT_EQ(w.model()->timeline().size(), 1u);
}

TEST(UiEditSketch, ReferenceDimensionsRoundTripThroughTheTimeline) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    auto* s = w.sketchMode()->activeSession();
    auto rect = s->addRect2pt({0, 0}, {40, 20});
    ASSERT_TRUE(rect.has_value());
    auto driven = s->addDrivenDimension(sketch::DimensionKind::Distance,
                                                 {sketch::SketchRef{(*rect)[0], sketch::PointRole::Start},
                                                  sketch::SketchRef{(*rect)[0], sketch::PointRole::End}});
    ASSERT_TRUE(driven.has_value());
    const std::uint64_t drivenId = *driven;
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));

    auto stored = w.model()->sketchForEdit(w.model()->timeline().at(0)->id());
    ASSERT_TRUE(stored.has_value());
    ASSERT_EQ(stored->dimensions().size(), 1u);
    const auto& d = stored->dimensions().begin()->second;
    EXPECT_TRUE(d.driven) << "saved as a driven dimension";
    EXPECT_EQ(d.id.value(), drivenId) << "same id";

    doubleClickItem(w, 0);
    auto* again = w.sketchMode()->activeSession();
    ASSERT_NE(again, nullptr);
    ASSERT_EQ(again->drivenDimensions().size(), 1u) << "re-entered as a reference dimension";
    EXPECT_EQ(again->drivenDimensions().front().id, drivenId);
    EXPECT_TRUE(again->sketch().dimensions().empty()) << "driven dimensions are not solved";
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));
}

// ---- Extrude distance arrow -----------------------------------------------------------------------------------

/// Sends a left mouse event with the given type straight to the overlay (the arrow only sees widget events).
void sendMouse(QWidget* target, QEvent::Type type, QPointF pos) {
    QMouseEvent ev(type, pos, pos, Qt::LeftButton, type == QEvent::MouseButtonRelease ? Qt::NoButton : Qt::LeftButton,
                   Qt::NoModifier);
    QApplication::sendEvent(target, &ev);
}

/// Screen point of a world point of the viewport, in widget pixels.
QPointF worldToWidget(ui::MainWindow& w, const Vec3& p) {
    auto ndc = w.viewport()->camera().project(p);
    EXPECT_TRUE(ndc.has_value());
    return {(ndc->x + 1.0) * 0.5 * w.viewport()->width(), (1.0 - ndc->y) * 0.5 * w.viewport()->height()};
}

/// Unit screen direction of the sketch plane normal at the profile centre (the same direction the arrow uses).
QPointF normalDirPx(ui::MainWindow& w, const Vec3& anchor, const Vec3& normal) {
    const QPointF a = worldToWidget(w, anchor);
    const QPointF b = worldToWidget(w, anchor + normal);
    const QPointF d = b - a;
    const double len = std::hypot(d.x(), d.y());
    return len > 0 ? QPointF(d.x() / len, d.y() / len) : QPointF(1, 0);
}

/// Opens Extrude on the 40 x 20 sketch (profile centroid (20, 10, 0), normal +Z) and returns its panel.
ui::CommandPanel* openExtrudeOnBlock(ui::MainWindow& w) {
    buildSketch(w);
    EXPECT_TRUE(w.startFeature("extrude").has_value());
    ui::CommandPanel* panel = openPanel(w);
    EXPECT_NE(panel, nullptr);
    return panel;
}

TEST(UiExtrudeArrow, ArrowSitsOnTheProfileAndDragSetsTheDistanceAndPreview) {
    ui::MainWindow w;
    w.resize(1280, 800);
    w.show();
    ASSERT_TRUE(w.settle(5000));
    ui::CommandPanel* panel = openExtrudeOnBlock(w);
    ASSERT_NE(panel, nullptr);
    ui::ManipulatorOverlay* arrow = w.manipulators();
    ASSERT_NE(arrow, nullptr);
    ASSERT_EQ(arrow->mode(), ui::ManipulatorOverlay::Mode::Distance) << "arrow shown while Extrude is open";
    EXPECT_NEAR(arrow->value(), 10.0, 1e-9) << "starts at the distance field";

    const Vec3 centroid{20, 10, 0};
    const QPointF dir = normalDirPx(w, centroid, Vec3{0, 0, 1});
    const double pxPerMm = ui::sketchPixelsPerMm(w.viewport()->camera(), w.viewport()->height());
    const QPointF anchor = worldToWidget(w, centroid);
    const QPointF handle = arrow->handlePos();
    EXPECT_NEAR(std::hypot(handle.x() - anchor.x(), handle.y() - anchor.y()), 10.0 * pxPerMm, 1.0)
        << "handle 10 mm from the centroid along the normal";

    // Drag the handle 5 mm further out along the normal (in pixels, along the arrow).
    const QPointF target = handle + dir * (5.0 * pxPerMm);
    sendMouse(arrow, QEvent::MouseButtonPress, handle);
    sendMouse(arrow, QEvent::MouseMove, target);
    sendMouse(arrow, QEvent::MouseButtonRelease, target);

    auto field = panel->model().valueOf("distance");
    ASSERT_TRUE(field.has_value());
    EXPECT_NEAR(field->value, 15.0, 0.1) << "distance field follows the drag";
    EXPECT_EQ(panel->model().text("distance"), "15.0 mm") << "snapped to 0.1 mm and written as a plain number";

    // The live preview follows: volume of 40 x 20 x 15. The preview runs on a job: pump until it delivers.
    for (int i = 0; i < 1000 && !panel->previewText().contains("12000"); ++i) {
        panel->pollPreview();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(5);
    }
    EXPECT_TRUE(panel->previewText().contains("12000")) << panel->previewText().toStdString();
    EXPECT_TRUE(panel->okEnabled());
}

TEST(UiExtrudeArrow, TypingInTheFieldMovesTheArrowAndSavesAScreenshot) {
    ui::MainWindow w;
    w.resize(1280, 800);
    w.show();
    ASSERT_TRUE(w.settle(5000));
    ui::CommandPanel* panel = openExtrudeOnBlock(w);
    ASSERT_NE(panel, nullptr);
    ui::ManipulatorOverlay* arrow = w.manipulators();
    ASSERT_NE(arrow, nullptr);
    const QPointF before = arrow->handlePos();
    const QPointF dir = normalDirPx(w, Vec3{20, 10, 0}, Vec3{0, 0, 1});
    const double pxPerMm = ui::sketchPixelsPerMm(w.viewport()->camera(), w.viewport()->height());

    panel->setText("distance", QStringLiteral("25 mm"));
    EXPECT_NEAR(arrow->value(), 25.0, 1e-9) << "the arrow takes the typed value";
    const QPointF after = arrow->handlePos();
    EXPECT_NEAR(std::hypot(after.x() - before.x(), after.y() - before.y()), 15.0 * pxPerMm, 1.0);
    EXPECT_NEAR((after.x() - before.x()) * dir.x() + (after.y() - before.y()) * dir.y(), 15.0 * pxPerMm, 1.0)
        << "moved along the normal";

    // Dialog-open screenshot with the arrow, when a dump directory is set.
    for (int i = 0; i < 5; ++i) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    if (const char* dir = std::getenv("IW_TEST_DUMP_DIR")) {
        const QString base = QString::fromLocal8Bit(dir);
        EXPECT_TRUE(w.grab().save(base + "/ui_extrude_distance_arrow.png"));
    }
    panel->reject();
    for (int i = 0; i < 3; ++i) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    EXPECT_EQ(arrow->mode(), ui::ManipulatorOverlay::Mode::None) << "arrow goes with the dialog";
}

// ---- right-click Edit Sketch is testable through the timeline action -----------------------------------------

TEST(UiEditSketch, TimelineEditSketchActionReentersTheSketchWithoutTheMenu) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    const Block b = buildSketch(w);
    ASSERT_FALSE(w.sketchMode()->active());

    // The right-click menu's "Edit Sketch" item runs this action; the test injects it directly.
    w.runTimelineAction(b.sketchId, ui::MainWindow::TimelineAction::Edit);
    ASSERT_TRUE(w.sketchMode()->active());
    ASSERT_TRUE(w.sketchMode()->editing());
    auto* session = w.sketchMode()->activeSession();
    ASSERT_NE(session, nullptr);
    for (sketch::SketchEntityId line : {b.bottom, b.right, b.top, b.left}) {
        const sketch::SketchEntity* ent = session->sketch().findEntity(line);
        ASSERT_NE(ent, nullptr) << "the block's line " << line.value() << " is the sketch being edited";
        EXPECT_NE(std::get_if<sketch::LineGeom>(&ent->geometry), nullptr);
    }
    EXPECT_TRUE(session->sketch().findEntity(b.bottom) != nullptr);
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_FALSE(w.sketchMode()->active());
    EXPECT_EQ(w.model()->timeline().size(), 1u) << "finishing an unchanged edit adds nothing";
}

TEST(UiEditSketch, TimelineEditOfAFeatureOpensItsDialogThroughTheSameAction) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildSketch(w);
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.model()->timeline().size(), 2u);
    w.runTimelineAction(w.model()->timeline().at(1)->id(), ui::MainWindow::TimelineAction::Edit);
    ui::CommandPanel* panel = openPanel(w);
    ASSERT_NE(panel, nullptr);
    EXPECT_EQ(panel->model().text("operation"), "New body");
    panel->reject();
}

// ---- Profile dropdown: "Profile N" labels, keys as item data, hover highlights the profile -------------------

/// Two 40 x 20 rectangles in one sketch, finished: the left profile's centroid is (20, 10), the right one's (80, 10).
Block buildTwoRectangles(ui::MainWindow& w) {
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    auto* s = w.sketchMode()->activeSession();
    auto left = s->addRect2pt({0, 0}, {40, 20});
    auto right = s->addRect2pt({60, 0}, {100, 20});
    EXPECT_TRUE(left.has_value() && right.has_value());
    Block b{};
    if (left) {
        b.bottom = (*left)[0];
    }
    w.sketchMode()->finish();
    EXPECT_TRUE(w.settle(5000));
    b.sketchId = w.model()->timeline().at(0)->id();
    return b;
}

QComboBox* dropdown(ui::CommandPanel* panel, const char* field) {
    return panel->findChild<QComboBox*>(QStringLiteral("input_%1").arg(field));
}

TEST(UiProfileDropdown, ProfilesAreNumberedPerSketchByCentroidAndKeepTheirKeyAsData) {
    ui::MainWindow w;
    w.resize(1280, 800);
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildTwoRectangles(w);
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    ui::CommandPanel* panel = openPanel(w);
    ASSERT_NE(panel, nullptr);
    QComboBox* box = dropdown(panel, "profile");
    ASSERT_NE(box, nullptr);
    ASSERT_EQ(box->count(), 2);
    EXPECT_EQ(box->itemText(0), QStringLiteral("Profile 1")) << "left rectangle: smaller centroid x";
    EXPECT_EQ(box->itemText(1), QStringLiteral("Profile 2"));
    const QString key0 = box->itemData(0).toString(), key1 = box->itemData(1).toString();
    EXPECT_FALSE(key0.isEmpty());
    EXPECT_NE(key0, key1);
    EXPECT_NE(key0, QStringLiteral("Profile 1")) << "the internal name is the data, not the label";
    EXPECT_EQ(QString::fromStdString(panel->model().text("profile")), key0) << "the default is Profile 1";

    // Choosing Profile 2 moves the arrow to the right rectangle's centroid (80, 10, 0).
    box->setCurrentIndex(1);
    EXPECT_EQ(QString::fromStdString(panel->model().text("profile")), key1);
    ui::ManipulatorOverlay* arrow = w.manipulators();
    ASSERT_NE(arrow, nullptr);
    const QPointF anchor = worldToWidget(w, Vec3{80, 10, 0});
    const QPointF handle = arrow->handlePos();
    const double pxPerMm = ui::sketchPixelsPerMm(w.viewport()->camera(), w.viewport()->height());
    const QPointF dir = normalDirPx(w, Vec3{80, 10, 0}, Vec3{0, 0, 1});
    EXPECT_NEAR(std::hypot(handle.x() - anchor.x() - dir.x() * 10.0 * pxPerMm, handle.y() - anchor.y() - dir.y() * 10.0 * pxPerMm), 0.0, 1.5)
        << "arrow starts at the Profile 2 centroid";

    // Setting the value from code selects the item with that key too.
    panel->setText("profile", key0);
    EXPECT_EQ(box->currentIndex(), 0);
    panel->reject();
}

TEST(UiProfileDropdown, HoveringAnOptionHighlightsItsProfileOverlayUntilTheMouseLeaves) {
    ui::MainWindow w;
    w.resize(1280, 800);
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildTwoRectangles(w);
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    ui::CommandPanel* panel = openPanel(w);
    ASSERT_NE(panel, nullptr);
    QComboBox* box = dropdown(panel, "profile");
    ASSERT_NE(box, nullptr);
    const auto& faces = w.viewport()->overlayFaces();
    ASSERT_EQ(faces.size(), 2u) << "one overlay per profile";
    const float base = faces[0].rgba[3];
    EXPECT_NEAR(faces[1].rgba[3], base, 1e-6);

    // Hovering the second item (the popup's view reports `entered`) brightens exactly that profile's overlay.
    QAbstractItemView* view = box->view();
    emit view->entered(view->model()->index(1, 0));
    EXPECT_GT(w.viewport()->overlayFaces()[1].rgba[3], base) << "hovered profile is highlighted";
    EXPECT_NEAR(w.viewport()->overlayFaces()[0].rgba[3], base, 1e-6) << "the other profile is not";

    // The mouse leaving the list clears the highlight.
    QEvent leave(QEvent::Leave);
    QApplication::sendEvent(view->viewport(), &leave);
    EXPECT_NEAR(w.viewport()->overlayFaces()[1].rgba[3], base, 1e-6);
    panel->reject();
}

// ---- Revolve angle ring ----------------------------------------------------------------------------------------

TEST(UiRevolveRing, RingIsBoundToTheAngleFieldAndDragWritesIt) {
    ui::MainWindow w;
    w.resize(1280, 800);
    w.show();
    ASSERT_TRUE(w.settle(5000));
    const Block b = buildSketch(w);
    ASSERT_TRUE(w.startFeature("revolve").has_value());
    ui::CommandPanel* panel = openPanel(w);
    ASSERT_NE(panel, nullptr);
    ui::ManipulatorOverlay* ring = w.manipulators();
    ASSERT_NE(ring, nullptr);
    ASSERT_EQ(ring->mode(), ui::ManipulatorOverlay::Mode::Angle) << "ring shown while Revolve is open";

    // Axis: the bottom line of the block (y = 0, along X). The ring sits on the axis at the centroid's projection.
    QComboBox* axes = dropdown(panel, "axis");
    ASSERT_NE(axes, nullptr);
    const std::string suffix = " / line " + std::to_string(b.bottom.value());
    int axisRow = -1;
    for (int i = 0; i < axes->count(); ++i)
        if (axes->itemText(i).toStdString().ends_with(suffix)) axisRow = i;
    ASSERT_GE(axisRow, 0);
    axes->setCurrentIndex(axisRow);
    const QPointF centre = worldToWidget(w, Vec3{20, 0, 0});

    // The angle field drives the ring (the default is a full turn).
    EXPECT_NEAR(ring->value(), 360.0, 1e-6);
    panel->setText("angle", QStringLiteral("90 deg"));
    EXPECT_NEAR(ring->value(), 90.0, 1e-6) << "the ring takes the typed angle";

    // Dragging the handle 30 degrees further (in screen angle about the ring centre) writes 120 deg into the field.
    const QPointF handle = ring->handlePos();
    const double r = std::hypot(handle.x() - centre.x(), handle.y() - centre.y());
    const double a = std::atan2(-(handle.y() - centre.y()), handle.x() - centre.x()) + std::numbers::pi / 6.0;
    const QPointF target(centre.x() + r * std::cos(a), centre.y() - r * std::sin(a));
    sendMouse(ring, QEvent::MouseButtonPress, handle);
    sendMouse(ring, QEvent::MouseMove, target);
    sendMouse(ring, QEvent::MouseButtonRelease, target);
    auto angle = panel->model().valueOf("angle");
    ASSERT_TRUE(angle.has_value());
    EXPECT_NEAR(angle->value * 180.0 / std::numbers::pi, 120.0, 0.2) << "drag sets the angle field";
    panel->reject();
    for (int i = 0; i < 3; ++i) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    EXPECT_EQ(ring->mode(), ui::ManipulatorOverlay::Mode::None) << "ring goes with the dialog";
}

// ---- Live preview body: translucent, green / red, gone on Cancel / OK, errors without a preview ----------------

/// Waits (pumping the preview) until the panel shows a preview whose text contains `needle`, or the budget runs out.
bool waitForPreviewText(ui::CommandPanel* panel, const char* needle) {
    for (int i = 0; i < 1000 && !panel->previewText().contains(QString::fromUtf8(needle)); ++i) {
        panel->pollPreview();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(5);
    }
    return panel->previewText().contains(QString::fromUtf8(needle));
}

TEST(UiPreviewBody, ExtrudePreviewIsATranslucentGreenBodyThatGoesAwayOnCancel) {
    ui::MainWindow w;
    w.resize(1280, 800);
    w.show();
    ASSERT_TRUE(w.settle(5000));
    ui::CommandPanel* panel = openExtrudeOnBlock(w);
    ASSERT_NE(panel, nullptr);
    ASSERT_TRUE(waitForPreviewText(panel, "8000")) << panel->previewText().toStdString();   // 40 x 20 x 10
    for (int i = 0; i < 5; ++i) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);

    // Profile overlay first (index 0 = the profile, pickable), then the preview body last.
    const auto& faces = w.viewport()->overlayFaces();
    ASSERT_EQ(faces.size(), 2u);
    const render::OverlayFace& body = faces.back();
    EXPECT_NEAR(body.rgba[3], 0.45f, 1e-3) << "about 45% alpha";
    EXPECT_GT(body.rgba[1], body.rgba[0]) << "green for a new body";
    EXPECT_GT(body.triangles.size(), 0u) << "the produced body is tessellated";
    // The triangles are the surface of the 40 x 20 x 10 box: every vertex lies within its bounds.
    for (const Vec3& p : body.triangles) {
        EXPECT_GE(p.x, -1e-3);
        EXPECT_LE(p.x, 40.0 + 1e-3);
        EXPECT_GE(p.z, -1e-3);
        EXPECT_LE(p.z, 10.0 + 1e-3);
    }

    if (const char* dir = std::getenv("IW_TEST_DUMP_DIR")) {
        EXPECT_TRUE(w.grab().save(QString::fromLocal8Bit(dir) + "/ui_extrude_translucent_preview.png"));
    }
    panel->reject();
    for (int i = 0; i < 3; ++i) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    EXPECT_EQ(w.viewport()->overlayFaces().size(), 0u) << "preview and profiles go with the dialog";
}

TEST(UiPreviewBody, CutPreviewIsRedAndOkClearsIt) {
    ui::MainWindow w;
    w.resize(1280, 800);
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildSketch(w);
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));
    // A second sketch-based extrude that cuts the first body.
    buildSketch(w);
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    ui::CommandPanel* panel = openPanel(w);
    ASSERT_NE(panel, nullptr);
    panel->setText("distance", QStringLiteral("5 mm"));
    panel->setText("operation", QStringLiteral("Cut"));
    QComboBox* ops = dropdown(panel, "operation");
    ASSERT_NE(ops, nullptr);
    EXPECT_EQ(ops->currentText(), QStringLiteral("Cut"));
    // Wait for the preview of the Cut (red), not the earlier New body one (green).
    bool red = false;
    for (int i = 0; i < 1000 && !red; ++i) {
        panel->pollPreview();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(5);
        const auto& faces = w.viewport()->overlayFaces();
        red = faces.size() >= 2 && faces.back().rgba[3] > 0.4f && faces.back().rgba[0] > faces.back().rgba[1];
    }
    ASSERT_TRUE(red) << "the cut preview is drawn red: " << panel->previewText().toStdString() << " faces=" << w.viewport()->overlayFaces().size();
    const auto& faces = w.viewport()->overlayFaces();
    const render::OverlayFace& body = faces.back();
    EXPECT_GT(body.rgba[0], body.rgba[1]) << "red for a cut";
    EXPECT_NEAR(body.rgba[3], 0.45f, 1e-3);

    // OK adds the feature and clears the preview.
    ASSERT_TRUE(panel->okEnabled());
    panel->accept();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(w.viewport()->overlayFaces().size(), 0u);
}

TEST(UiPreviewBody, FailedPreviewShowsTheErrorAndNoBody) {
    ui::MainWindow w;
    w.resize(1280, 800);
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildSketch(w);
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_TRUE(w.startFeature("fillet").has_value());
    ui::CommandPanel* fi = openPanel(w);
    ASSERT_NE(fi, nullptr);
    render::PickResult pick;
    pick.bodyIndex = 0;
    pick.edgeIndex = edgeOfLength(w, 10.0);   // a 10 mm high edge
    w.routePick(pick);
    fi->setText("radius", QStringLiteral("30 mm"));   // larger than the body: the fillet cannot exist
    for (int i = 0; i < 1000 && fi->previewText().isEmpty(); ++i) {
        fi->pollPreview();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(5);
    }
    ASSERT_FALSE(fi->previewText().isEmpty());
    EXPECT_FALSE(fi->previewText().contains("Volume")) << fi->previewText().toStdString();
    EXPECT_TRUE(w.viewport()->overlayFaces().empty()) << "no preview body for a failed feature";
    fi->reject();
}

} // namespace
