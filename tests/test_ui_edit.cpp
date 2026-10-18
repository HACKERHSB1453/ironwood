// SPDX-License-Identifier: GPL-3.0-or-later
// UI2: Edit Sketch from the timeline (SketchSession from an existing sketch, Finish replaces it as one undo step, Esc
// discards) and the extrude distance arrow (drag sets the field and the preview, typing moves the arrow).
// Offscreen only (QT_QPA_PLATFORM=offscreen is set by the environment below).
#include <gtest/gtest.h>

#include <QApplication>
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
    EXPECT_EQ(session->sketch().entities().size(), 4u);
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

} // namespace
