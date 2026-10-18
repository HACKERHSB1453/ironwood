// SPDX-License-Identifier: GPL-3.0-or-later
// B2 timeline UI and the bound document: scripted end-to-end flow through the shell's API (sketch -> extrude ->
// rollback -> fillet -> edit through the timeline -> undo/redo -> save/reopen), plus the timeline strip, browser and
// failure display. Offscreen only: QT_QPA_PLATFORM=offscreen is set by the environment below.

#include <gtest/gtest.h>

#include <QApplication>
#include <QDockWidget>
#include <QEventLoop>
#include <QImage>
#include <QPixmap>
#include <QPainter>
#include <QTest>
#include <QTreeWidget>

#include <cmath>
#include <cstdlib>
#include <numbers>
#include <filesystem>
#include <string>
#include <vector>

#include "kernel/shape.hpp"
#include "recompute/evaluator.hpp"
#include "render/offscreen.hpp"
#include "render/viewport.hpp"
#include "ui/command_dialog.hpp"
#include "ui/main_window.hpp"
#include "ui/sketch/plane_chooser.hpp"
#include "ui/sketch/session.hpp"
#include "ui/sketch/sketch_mode.hpp"
#include "ui/timeline/doc_eval.hpp"
#include "ui/timeline/timeline_widget.hpp"
#include "core/ids.hpp"

namespace {

using namespace iw;

/// One QApplication for the binary; the offscreen platform unless the caller picked one.
class QtEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
        app_ = new QApplication(argc_, argv_);
    }
    void TearDown() override { delete app_; }
    static inline int argc_ = 1;
    static inline char arg0_[] = "test_ui_timeline";
    static inline char* argv_[] = {arg0_, nullptr};
private:
    QApplication* app_ = nullptr;
};
const auto* const kEnv = ::testing::AddGlobalTestEnvironment(new QtEnv);

std::filesystem::path scratchDir(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("ironwood-ui-timeline-" + name);
}

ui::CommandPanel* openPanel(ui::MainWindow& w) {
    return qobject_cast<ui::CommandPanel*>(w.commandDock()->widget());
}

/// Volume (mm^3) of the first body of the shown evaluation; -1 when there is no body.
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

/// Screen centre of timeline item `i`.
QPoint itemCentre(ui::timeline::TimelineWidget* tw, std::size_t i) { return tw->itemRect(i).center(); }

/// Drags the rollback marker from its current position to the boundary `target` (0..size) with mouse events.
void dragMarkerTo(ui::MainWindow& w, std::size_t target) {
    auto* tw = w.timelineWidget();
    const QPoint from = tw->markerRect().center();
    // Boundary x positions: left of item m, or right of the last item.
    const std::size_t n = tw->entries().size();
    const int x = target == 0 ? tw->itemRect(0).left() - 8
                : target < n  ? tw->itemRect(target).left() - 8
                              : tw->itemRect(n - 1).right() + 8;
    QTest::mousePress(tw, Qt::LeftButton, Qt::NoModifier, from);
    QTest::mouseMove(tw, QPoint(x, from.y()));
    QTest::mouseRelease(tw, Qt::LeftButton, Qt::NoModifier, QPoint(x, from.y()));
}

} // namespace

TEST(UiTimeline, EndToEndSketchExtrudeRollbackFilletEditUndoSaveReopen) {
    ui::MainWindow w;
    w.show();
    ASSERT_NE(w.model(), nullptr);
    ASSERT_TRUE(w.settle(5000));

    // 1. Sketch a 40 x 20 rectangle on the XY plane and finish it: a SketchFeature is added.
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ASSERT_TRUE(w.sketchMode()->active());
    auto* session = w.sketchMode()->activeSession();
    ASSERT_NE(session, nullptr);
    ASSERT_TRUE(session->addRect2pt({-20, -10}, {20, 10}).has_value());
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.model()->timeline().size(), 1u);
    EXPECT_EQ(w.model()->timeline().at(0)->kind(), "sketch");

    // 2. Extrude the profile by 10 mm with the Extrude command dialog (New body).
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    ui::CommandPanel* ex = openPanel(w);
    ASSERT_NE(ex, nullptr);
    ASSERT_TRUE(ex->okEnabled());
    ex->setText("distance", QStringLiteral("10 mm"));
    ex->accept();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.model()->timeline().size(), 2u) << "timeline must hold sketch + extrude";
    ASSERT_EQ(w.timelineWidget()->entries().size(), 2u);
    EXPECT_NEAR(shownVolume(w), 40.0 * 20.0 * 10.0, 1e-6);

    // 3. Rollback: drag the marker to 1 -> the extrude is not evaluated, the body is absent.
    dragMarkerTo(w, 1);
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(w.model()->timeline().rollbackMarker(), 1u);
    EXPECT_TRUE(w.shownEvaluation().meshes.empty()) << "rolled back before the extrude: no body";
    EXPECT_EQ(w.timelineWidget()->activeCount(), 1u);

    // 4. Back to the end: the body returns, and the evaluator reuses the cached prefix.
    dragMarkerTo(w, 2);
    ASSERT_TRUE(w.settle(5000));
    EXPECT_FALSE(w.model()->timeline().rollbackMarker() < w.model()->timeline().size());
    EXPECT_NEAR(shownVolume(w), 8000.0, 1e-6);
    EXPECT_GT(w.shownEvaluation().eval.reusedCount, 0u) << "rollback back to the end is a cache hit";

    // 5. Fillet one 10 mm edge: pick it in the viewport while the dialog is open (routed through the pick handler).
    ASSERT_TRUE(w.startFeature("fillet").has_value());
    ui::CommandPanel* fi = openPanel(w);
    ASSERT_NE(fi, nullptr);
    const std::uint32_t edge = edgeOfLength(w, 10.0);
    render::PickResult pick;
    pick.bodyIndex = 0;
    pick.edgeIndex = edge;
    w.routePick(pick);
    ASSERT_EQ(fi->model().selection("edges").size(), 1u);
    fi->setText("radius", QStringLiteral("1 mm"));
    fi->accept();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.model()->timeline().size(), 3u);
    const auto& st = w.shownEvaluation().eval.statuses;
    ASSERT_EQ(st.size(), 3u);
    EXPECT_EQ(st[2].state, recompute::FeatureStatus::State::Ok) << st[2].error.message;
    const double fillet = (1.0 - std::numbers::pi / 4.0) * 1.0 * 1.0 * 10.0;   // corner removed by r = 1 on a 10 mm edge
    const double volumeFillet = shownVolume(w);
    EXPECT_NEAR(volumeFillet, 8000.0 - fillet, 0.05);

    // 6. Edit the extrude distance through the timeline: double-click the extrude item.
    QTest::mouseDClick(w.timelineWidget(), Qt::LeftButton, Qt::NoModifier, itemCentre(w.timelineWidget(), 1));
    ui::CommandPanel* edit = openPanel(w);
    ASSERT_NE(edit, nullptr);
    EXPECT_EQ(edit->model().text("distance"), "10 mm") << "dialog prefilled from the feature";
    edit->setText("distance", QStringLiteral("15 mm"));
    edit->accept();
    ASSERT_TRUE(w.settle(5000));
    const double volumeEdited = shownVolume(w);
    EXPECT_NE(volumeEdited, volumeFillet) << "volume changes with the distance";
    EXPECT_NEAR(volumeEdited, 12000.0 - fillet * 1.5, 0.05) << "fillet still applied to the 15 mm edge";
    EXPECT_EQ(w.shownEvaluation().eval.statuses[2].state, recompute::FeatureStatus::State::Ok);

    // 7. Undo / redo the edit.
    ASSERT_TRUE(w.modelCanUndo());
    w.modelUndo();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_NEAR(shownVolume(w), volumeFillet, 1e-6) << "undo restores the 10 mm distance";
    ASSERT_TRUE(w.modelCanRedo());
    w.modelRedo();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_NEAR(shownVolume(w), volumeEdited, 1e-6) << "redo restores the 15 mm distance";

    // 8. Save, reopen, and compare the document: same features, same parameters, same volume.
    std::vector<std::pair<std::string, std::string>> before;
    for (std::size_t i = 0; i < w.model()->timeline().size(); ++i) {
        const auto& f = w.model()->timeline().at(i);
        before.emplace_back(std::string(f->kind()), f->paramsJson());
    }
    const double volumeBefore = shownVolume(w);
    const auto dir = scratchDir("part");
    std::filesystem::remove_all(dir);
    ASSERT_TRUE(w.saveDocument(QString::fromStdString(dir.string()))) << "save";
    ASSERT_TRUE(w.openDocument(QString::fromStdString(dir.string()))) << "reopen";
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.model()->timeline().size(), before.size());
    for (std::size_t i = 0; i < before.size(); ++i) {
        const auto& f = w.model()->timeline().at(i);
        EXPECT_EQ(std::string(f->kind()), before[i].first);
        EXPECT_EQ(f->paramsJson(), before[i].second) << "feature " << i << " round-trips";
    }
    EXPECT_NEAR(shownVolume(w), volumeBefore, 1e-9) << "reopened part has the same volume";
    for (const auto& st2 : w.shownEvaluation().eval.statuses)
        EXPECT_EQ(st2.state, recompute::FeatureStatus::State::Ok);
    std::filesystem::remove_all(dir);

    // 9. Screenshot of the window (timeline + browser + filleted part) for the user's review.
    if (const char* dumpDir = std::getenv("IW_TEST_DUMP_DIR")) {
        w.resize(1280, 800);
        QApplication::processEvents();
        QPixmap shot = w.grab();
        // The viewport needs a GPU frame, which the offscreen platform does not produce: draw a headless render of
        // the same scene into the viewport's rectangle of the window grab.
        render::Viewport* vp = w.viewport();
        render::Camera cam = vp->camera();
        const QSize size = vp->size();
        cam.setViewportSize(size.width(), size.height());
        auto frame = render::renderOffscreen(vp->scene(), cam, size);
        if (frame) {
            QPainter p(&shot);
            const QPoint origin = vp->mapTo(&w, QPoint(0, 0));
            p.drawImage(origin, frame->image);
            p.end();
        }
        EXPECT_TRUE(shot.save(QString::fromLocal8Bit(dumpDir) + "/ui-timeline.png"));
    }
}

TEST(UiTimeline, TimelineShowsStatusesWarningsAndHoverText) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ASSERT_TRUE(w.sketchMode()->activeSession()->addRect2pt({0, 0}, {10, 10}).has_value());
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->setText("distance", QStringLiteral("5 mm"));
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.timelineWidget()->entries().size(), 2u);

    // Rename (display name) and suppress through the feature operations the timeline menu uses.
    const FeatureId extrude = w.model()->timeline().at(1)->id();
    w.renameFeatureTo(extrude, "Base plate");
    EXPECT_EQ(w.timelineWidget()->entries()[1].label.toStdString(), "Base plate");
    w.setFeatureSuppressed(extrude, true);
    ASSERT_TRUE(w.settle(5000));
    EXPECT_TRUE(w.timelineWidget()->entries()[1].suppressed);
    EXPECT_TRUE(w.shownEvaluation().meshes.empty()) << "suppressed extrude makes no body";
    w.setFeatureSuppressed(extrude, false);
    ASSERT_TRUE(w.settle(5000));
    EXPECT_FALSE(w.shownEvaluation().meshes.empty());

    // Failure: deleting the sketch the extrude references makes the extrude fail (red), with the message on hover.
    const FeatureId sketch = w.model()->timeline().at(0)->id();
    w.deleteFeature(sketch);
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.timelineWidget()->entries().size(), 1u);
    EXPECT_EQ(w.timelineWidget()->entries()[0].state, recompute::FeatureStatus::State::Failed);
    // Undo brings the sketch back and the extrude is fine again.
    w.modelUndo();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(w.timelineWidget()->entries()[1].state, recompute::FeatureStatus::State::Ok);
}

TEST(UiTimeline, HoverHighlightsTheProducedBody) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ASSERT_TRUE(w.sketchMode()->activeSession()->addRect2pt({0, 0}, {10, 10}).has_value());
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));

    auto* tw = w.timelineWidget();
    QTest::mouseMove(tw, itemCentre(tw, 1));
    EXPECT_EQ(tw->hovered(), std::optional<FeatureId>(w.model()->timeline().at(1)->id()));
    EXPECT_EQ(w.viewport()->selection().bodyIndex.value_or(99u), 0u) << "hovering the extrude highlights its body";
    QTest::mouseMove(tw, QPoint(tw->width() - 2, tw->height() - 2));
    EXPECT_FALSE(tw->hovered().has_value());
}

TEST(UiTimeline, BrowserListsTheDocumentAndHidesBodies) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ASSERT_TRUE(w.sketchMode()->activeSession()->addRect2pt({0, 0}, {10, 10}).has_value());
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));

    QTreeWidget* tree = w.browser();
    std::vector<std::string> groups;
    QTreeWidgetItem* bodies = nullptr;
    for (int i = 0; i < tree->topLevelItemCount(); ++i) {
        groups.push_back(tree->topLevelItem(i)->text(0).toStdString());
        if (tree->topLevelItem(i)->text(0) == "Bodies") bodies = tree->topLevelItem(i);
    }
    EXPECT_EQ(groups, (std::vector<std::string>{"Origin", "Sketches", "Bodies", "Construction"}));
    ASSERT_NE(bodies, nullptr);
    ASSERT_EQ(bodies->childCount(), 1);
    EXPECT_EQ(w.viewport()->scene().size(), 1u);
    bodies->child(0)->setCheckState(0, Qt::Unchecked);
    EXPECT_FALSE(w.viewport()->scene()[0].visible);
    // The visibility survives a re-evaluation.
    w.setFeatureSuppressed(w.model()->timeline().at(1)->id(), true);
    w.setFeatureSuppressed(w.model()->timeline().at(1)->id(), false);
    ASSERT_TRUE(w.settle(5000));
    EXPECT_FALSE(w.viewport()->scene()[0].visible);
}

TEST(UiTimeline, DocumentUndoAndTheParameterStackAreSeparate) {
    ui::MainWindow w;
    ASSERT_TRUE(w.settle(5000));
    EXPECT_FALSE(w.modelCanUndo());
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ASSERT_TRUE(w.sketchMode()->activeSession()->addRect2pt({0, 0}, {10, 10}).has_value());
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_TRUE(w.modelCanUndo());
    w.modelUndo();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(w.model()->timeline().size(), 0u);
    EXPECT_FALSE(w.modelCanUndo());
}

TEST(UiTimeline, DialogsRefuseAddingWhileRolledBack) {
    ui::MainWindow w;
    ASSERT_TRUE(w.settle(5000));
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ASSERT_TRUE(w.sketchMode()->activeSession()->addRect2pt({0, 0}, {10, 10}).has_value());
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));
    dragMarkerTo(w, 1);
    ASSERT_TRUE(w.settle(5000));
    EXPECT_FALSE(w.startFeature("fillet").has_value()) << "a new feature must not be added behind the marker";
    dragMarkerTo(w, 2);
    ASSERT_TRUE(w.settle(5000));
}
