// SPDX-License-Identifier: GPL-3.0-or-later
// W2b1 UI integration: parameters on the document's undo stack, feature names, construction edits, inspect commands
// wired to the viewport, profile overlays, and the icon set in the ribbon / timeline / browser / S-box / marking menu.
// Offscreen only (QT_QPA_PLATFORM=offscreen is set by the environment below).
#include <gtest/gtest.h>

#include <QApplication>
#include <QCoreApplication>
#include <QDockWidget>
#include <QEventLoop>
#include <QImage>
#include <QListWidget>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTest>
#include <QToolButton>
#include <QTreeWidget>

#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

#include "core/math.hpp"
#include "features/construction_features.hpp"
#include "kernel/measure.hpp"
#include "kernel/shape.hpp"
#include "naming/element_name.hpp"
#include "render/viewport.hpp"
#include "sketch/sketch.hpp"
#include "ui/command_actions.hpp"
#include "ui/command_dialog.hpp"
#include "ui/icons.hpp"
#include "ui/inspect/inspect_commands.hpp"
#include "ui/inspect/measure_dialog.hpp"
#include "ui/inspect/properties_panel.hpp"
#include "ui/inspect/section_dialog.hpp"
#include "ui/main_window.hpp"
#include "ui/marking_menu.hpp"
#include "ui/parameters_dialog.hpp"
#include "ui/ribbon.hpp"
#include "ui/sbox.hpp"
#include "ui/sketch/session.hpp"
#include "ui/sketch/sketch_mode.hpp"
#include "ui/sketch/plane_chooser.hpp"
#include "ui/timeline/timeline_widget.hpp"

namespace {
using namespace iw;

class QtEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
        app_ = new QApplication(argc_, argv_);
        QCoreApplication::setOrganizationName("IronwoodTest");
        QCoreApplication::setApplicationName("ironwood-ui-integration-test");
    }
    void TearDown() override { delete app_; }

private:
    int argc_ = 1;
    char name_[24] = "test_ui_integration";
    char* argv_[2] = {name_, nullptr};
    QApplication* app_ = nullptr;
};
const auto* const kEnv = ::testing::AddGlobalTestEnvironment(new QtEnv);

ui::CommandPanel* openPanel(ui::MainWindow& w) { return qobject_cast<ui::CommandPanel*>(w.commandDock()->widget()); }

/// Sketch rectangle (0,0)-(10,10) on XY, finished, then extruded 10 mm: timeline = [sketch, extrude].
void buildBlock(ui::MainWindow& w) {
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ASSERT_TRUE(w.sketchMode()->activeSession()->addRect2pt({0, 0}, {10, 10}).has_value());
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.model()->timeline().size(), 2u);
}

/// Index of the planar face of `shape` whose outward normal is `n`, if any.
std::optional<std::uint32_t> planarFace(const kernel::Shape& shape, Vec3 n) {
    for (std::uint32_t i = 0; i < shape.subShapeCount(kernel::SubShapeKind::Face); ++i) {
        const kernel::FaceGeometry g = shape.faceGeometry(i);
        if (g.kind != kernel::FaceGeometry::Kind::Plane) continue;
        if (g.normal.x * n.x + g.normal.y * n.y + g.normal.z * n.z > 0.99) return i;
    }
    return std::nullopt;
}

/// Entity count of the first body of the shown evaluation: (body id, shape, naming).
struct ShownBody {
    BodyId id;
    kernel::Shape shape;
    naming::NamingMap naming;
};
ShownBody firstBody(ui::MainWindow& w) {
    const auto& eval = w.shownEvaluation().eval;
    const auto it = eval.bodies.begin();
    return ShownBody{it->first, it->second, eval.namings.at(it->first)};
}

naming::ElementRef refOf(const naming::NamingMap& nm, naming::ElementType type, std::uint32_t index) {
    auto n = nm.nameOf(type, index);
    return naming::ElementRef{n->feature, type, *n};
}

// ---- 1. parameters: one undo stack, dialog edits the document ---------------------------------------------------

TEST(UiIntegrationParameters, ParametersAndFeaturesShareOneUndoStack) {
    ui::MainWindow w;
    ASSERT_TRUE(w.settle(5000));
    ASSERT_TRUE(w.model()->addParameter("d1", "5 mm"));
    buildBlock(w);
    w.modelUndo();   // extrude
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(w.model()->timeline().size(), 1u);
    w.modelUndo();   // sketch
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(w.model()->timeline().size(), 0u);
    EXPECT_NE(w.model()->parameters().find("d1"), nullptr) << "the parameter edit is still on the stack";
    w.modelUndo();   // the parameter itself
    EXPECT_EQ(w.model()->parameters().find("d1"), nullptr);
    EXPECT_FALSE(w.modelCanUndo());
}

TEST(UiIntegrationParameters, DialogRenameRewritesFeatureReferencesAndUndoesInOneStep) {
    ui::MainWindow w;
    ASSERT_TRUE(w.settle(5000));
    ASSERT_TRUE(w.model()->addParameter("h", "10 mm"));
    buildBlock(w);
    ASSERT_TRUE(w.startFeature("extrude").has_value());
    openPanel(w)->setText("distance", QStringLiteral("h"));
    openPanel(w)->accept();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.model()->timeline().size(), 3u);
    ASSERT_NE(w.model()->timeline().at(2)->paramsJson().find("\"h\""), std::string::npos)
        << w.model()->timeline().at(2)->paramsJson();

    ui::ParametersDialog dialog(*w.model());
    ASSERT_EQ(dialog.table()->rowCount(), 1);
    ASSERT_TRUE(dialog.editCell(0, ui::ParametersDialog::Name, QStringLiteral("height")));
    ASSERT_TRUE(w.settle(5000));
    EXPECT_NE(w.model()->timeline().at(2)->paramsJson().find("height"), std::string::npos);

    w.modelUndo();   // one step reverts the rename everywhere
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(dialog.table()->item(0, ui::ParametersDialog::Name)->text().toStdString(), "h");
    EXPECT_EQ(w.model()->timeline().at(2)->paramsJson().find("height"), std::string::npos);
}

TEST(UiIntegrationParameters, SketchDimensionsFollowTheDocumentParameterTable) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_TRUE(w.model()->addParameter("w", "40 mm"));
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ui::SketchSession* s = w.sketchMode()->activeSession();
    ASSERT_NE(s, nullptr);
    auto rect = s->addRect2pt({0, 0}, {30, 15});
    ASSERT_TRUE(rect);
    auto bottom = s->planDimension({sketch::SketchRef{(*rect)[0], std::nullopt}}, {15, -5});
    ASSERT_TRUE(bottom) << bottom.error().message;
    auto dw = s->addDimension(bottom->kind, bottom->refs, "w");
    ASSERT_TRUE(dw && dw->id);
    auto widthOf = [&] {
        const auto& g = std::get<sketch::LineGeom>(s->sketch().findEntity((*rect)[0])->geometry);
        return sketch::distance(g.start, g.end);
    };
    EXPECT_NEAR(widthOf(), 40.0, 1e-6);

    // Editing the document parameter re-solves the sketch without any manual call.
    ASSERT_TRUE(w.model()->setParameterExpression("w", "55 mm"));
    EXPECT_NEAR(widthOf(), 55.0, 1e-6);
}

TEST(UiIntegrationParameters, NewDocumentReleasesOpenParameterDialogs) {
    ui::MainWindow w;
    ASSERT_TRUE(w.settle(5000));
    auto* dialog = new ui::ParametersDialog(*w.model(), &w);
    ASSERT_TRUE(dialog->addParameter("a", "1 mm"));
    EXPECT_EQ(dialog->table()->rowCount(), 1);
    w.newDocument();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_EQ(dialog->table()->rowCount(), 0) << "the dialog no longer follows the replaced document";
    EXPECT_FALSE(dialog->addParameter("b", "1 mm"));
    EXPECT_EQ(w.model()->parameters().find("a"), nullptr) << "the new document is untouched";
}

// ---- 2. feature names ----------------------------------------------------------------------------------------------

TEST(UiIntegrationNames, TimelineLabelsAreFeatureNamesAndRenamePersists) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString path = dir.filePath("named.iwd");
    {
        ui::MainWindow w;
        w.show();
        ASSERT_TRUE(w.settle(5000));
        buildBlock(w);
        ASSERT_EQ(w.timelineWidget()->entries().size(), 2u);
        EXPECT_EQ(w.model()->timeline().at(1)->name(), "Extrude 1");
        EXPECT_EQ(w.timelineWidget()->entries()[1].label.toStdString(), "Extrude 1");

        const FeatureId extrude = w.model()->timeline().at(1)->id();
        w.renameFeatureTo(extrude, "Base plate");
        EXPECT_EQ(w.model()->timeline().at(1)->name(), "Base plate") << "Document::renameFeature";
        EXPECT_EQ(w.timelineWidget()->entries()[1].label.toStdString(), "Base plate") << "shown without waiting";
        ASSERT_TRUE(w.settle(5000));
        w.modelUndo();   // the rename is an undoable command
        EXPECT_EQ(w.model()->timeline().at(1)->name(), "Extrude 1");
        ASSERT_TRUE(w.settle(5000));
        EXPECT_EQ(w.timelineWidget()->entries()[1].label.toStdString(), "Extrude 1");
        w.modelRedo();
        ASSERT_TRUE(w.settle(5000));
        EXPECT_EQ(w.timelineWidget()->entries()[1].label.toStdString(), "Base plate");
        ASSERT_TRUE(w.saveDocument(path));
    }
    {
        ui::MainWindow w;
        ASSERT_TRUE(w.openDocument(path));
        ASSERT_TRUE(w.settle(5000));
        ASSERT_EQ(w.timelineWidget()->entries().size(), 2u);
        EXPECT_EQ(w.timelineWidget()->entries()[1].label.toStdString(), "Base plate") << "persisted in the document";
    }
}

// ---- 3. construction edit: double-click opens the prefilled dialog ------------------------------------------------

void doubleClickItem(ui::MainWindow& w, std::size_t index) {
    auto* tw = w.timelineWidget();
    QTest::mouseDClick(tw, Qt::LeftButton, Qt::NoModifier, tw->itemRect(index).center());
}

TEST(UiIntegrationConstruction, OffsetPlaneDoubleClickOpensPrefilledDialogAndEditsInPlace) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildBlock(w);
    const ShownBody body = firstBody(w);
    auto face = planarFace(body.shape, Vec3{0, 0, 1});
    ASSERT_TRUE(face.has_value());
    features::ConstructionPlaneParams p;
    p.kind = features::ConstructionPlaneParams::Kind::Offset;
    p.base.isFace = true;
    p.base.body = body.id;
    p.base.face = refOf(body.naming, naming::ElementType::Face, *face);
    p.distance = "5 mm";
    const FeatureId id = w.model()->allocateFeatureId();
    ASSERT_TRUE(w.model()->addFeature(std::make_shared<features::ConstructionPlaneFeature>(id, p)));
    ASSERT_TRUE(w.settle(5000));
    ASSERT_EQ(w.timelineWidget()->entries().size(), 3u);

    doubleClickItem(w, 2);
    ui::CommandPanel* panel = openPanel(w);
    ASSERT_NE(panel, nullptr) << "double-click opens the offset plane dialog";
    EXPECT_EQ(panel->model().text("distance"), "5 mm") << "prefilled from the feature";
    panel->setText("distance", QStringLiteral("7 mm"));
    panel->accept();
    ASSERT_TRUE(w.settle(5000));

    const auto* edited = dynamic_cast<const features::ConstructionPlaneFeature*>(w.model()->timeline().at(2).get());
    ASSERT_NE(edited, nullptr);
    EXPECT_EQ(edited->id(), id) << "edited in place (same id)";
    EXPECT_EQ(edited->params().distance, "7 mm") << "through Document::editConstructionPlane";
}

TEST(UiIntegrationConstruction, AxisAndPointDoubleClickOpenPrefilledDialogs) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildBlock(w);
    const ShownBody body = firstBody(w);

    features::ConstructionAxisParams ap;
    ap.kind = features::ConstructionAxisParams::Kind::TwoPoints;
    ap.a = {"0 mm", "0 mm", "0 mm"};
    ap.b = {"0 mm", "0 mm", "20 mm"};
    const FeatureId axisId = w.model()->allocateFeatureId();
    ASSERT_TRUE(w.model()->addFeature(std::make_shared<features::ConstructionAxisFeature>(axisId, ap)));
    ASSERT_TRUE(w.settle(5000));

    doubleClickItem(w, 2);
    ui::CommandPanel* axisPanel = openPanel(w);
    ASSERT_NE(axisPanel, nullptr);
    EXPECT_EQ(axisPanel->model().text("kind"), "Two points");
    EXPECT_EQ(axisPanel->model().text("bz"), "20 mm");
    axisPanel->setText("bz", QStringLiteral("30 mm"));
    axisPanel->accept();
    ASSERT_TRUE(w.settle(5000));
    const auto* axis = dynamic_cast<const features::ConstructionAxisFeature*>(w.model()->timeline().at(2).get());
    ASSERT_NE(axis, nullptr);
    EXPECT_EQ(axis->id(), axisId);
    EXPECT_EQ(axis->params().b[2], "30 mm");

    // A construction point on a vertex of the body.
    auto vertexName = body.naming.nameOf(naming::ElementType::Vertex, 0);
    ASSERT_TRUE(vertexName.has_value()) << "box vertices are named";
    features::ConstructionPointParams pp;
    pp.kind = features::ConstructionPointParams::Kind::Vertex;
    pp.body = body.id;
    pp.ref = refOf(body.naming, naming::ElementType::Vertex, 0);
    const FeatureId pointId = w.model()->allocateFeatureId();
    ASSERT_TRUE(w.model()->addFeature(std::make_shared<features::ConstructionPointFeature>(pointId, pp)));
    ASSERT_TRUE(w.settle(5000));

    doubleClickItem(w, 3);
    ui::CommandPanel* pointPanel = openPanel(w);
    ASSERT_NE(pointPanel, nullptr) << "double-click opens the point dialog";
    pointPanel->accept();
    ASSERT_TRUE(w.settle(5000));
    const auto* point = dynamic_cast<const features::ConstructionPointFeature*>(w.model()->timeline().at(3).get());
    ASSERT_NE(point, nullptr);
    EXPECT_EQ(point->id(), pointId);
    EXPECT_EQ(point->params().kind, features::ConstructionPointParams::Kind::Vertex);
    EXPECT_EQ(point->params().ref, pp.ref) << "the prefilled vertex is kept";
}

// ---- 4. inspect: Measure, Properties, Section Analysis --------------------------------------------------------------

TEST(UiIntegrationInspect, CommandsAreRegisteredAndMeasureWorksOnViewportPicks) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildBlock(w);
    for (const char* id : {"measure", "properties", "section_analysis"}) {
        ASSERT_NE(w.commands().find(id), nullptr) << id;
        EXPECT_TRUE(w.commands().find(id)->implemented()) << id;
    }
    EXPECT_EQ(w.commands().findByShortcut(QKeySequence(QStringLiteral("I")))->id, "measure");

    const ShownBody body = firstBody(w);
    auto top = planarFace(body.shape, Vec3{0, 0, 1});
    auto front = planarFace(body.shape, Vec3{0, -1, 0});
    ASSERT_TRUE(top && front);
    render::PickResult a;
    a.bodyIndex = 0u;
    a.faceIndex = *top;
    w.routePick(a);   // no dialog is routing picks: the viewport selection feeds Measure
    render::PickResult b = a;
    b.faceIndex = *front;
    w.routePick(b);
    ASSERT_EQ(w.picks().size(), 2u);

    w.actions()->run("measure");
    auto* measure = qobject_cast<ui::inspect::MeasureDialog*>(w.commandDock()->widget());
    ASSERT_NE(measure, nullptr) << "Measure docks its dialog";
    ASSERT_TRUE(measure->pickFromProvider(1));
    ASSERT_TRUE(measure->pickFromProvider(2));
    EXPECT_FALSE(measure->rows().empty()) << "two faces give distance and angle rows";
}

TEST(UiIntegrationInspect, PropertiesUsesTheActiveBodyAndSectionDrivesTheViewport) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildBlock(w);

    w.actions()->refresh();
    ASSERT_TRUE(w.actions()->action("properties")->isEnabled()) << "one body in the scene is the active body";
    w.actions()->run("properties");
    EXPECT_NE(qobject_cast<ui::inspect::PropertiesPanel*>(w.commandDock()->widget()), nullptr);

    EXPECT_FALSE(w.viewport()->sectionRequest().enabled);
    w.actions()->run("section_analysis");
    auto* section = qobject_cast<ui::inspect::SectionDialog*>(w.commandDock()->widget());
    ASSERT_NE(section, nullptr);
    section->setNormalAxis(2);
    section->setEnabled(true);
    ASSERT_TRUE(w.sectionRequest().has_value());
    EXPECT_TRUE(w.sectionRequest()->enabled);
    // The request reaches the viewport (field by field).
    const render::SectionRequest r = w.viewport()->sectionRequest();
    EXPECT_TRUE(r.enabled);
    EXPECT_NEAR(std::abs(r.clipPlane[2]), 1.0, 1e-9) << "Z normal";
    EXPECT_EQ(r.clipPlane, w.sectionRequest()->clipPlane);
}

// ---- profile overlays while Extrude is open -----------------------------------------------------------------------

TEST(UiIntegrationOverlays, ProfilesAreShadedWhileExtrudeIsOpenAndAClickChoosesOne) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    w.startSketch(ui::originPlane(ui::PlaneChoice::XY), ui::planeName(ui::PlaneChoice::XY));
    ASSERT_TRUE(w.sketchMode()->activeSession()->addRect2pt({0, 0}, {10, 10}).has_value());
    ASSERT_TRUE(w.sketchMode()->activeSession()->addRect2pt({20, 0}, {30, 10}).has_value());
    w.sketchMode()->finish();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_TRUE(w.viewport()->overlayFaces().empty()) << "no dialog open, no overlays";

    ASSERT_TRUE(w.startFeature("extrude").has_value());
    ui::CommandPanel* panel = openPanel(w);
    ASSERT_NE(panel, nullptr);
    const auto& overlays = w.viewport()->overlayFaces();
    ASSERT_EQ(overlays.size(), 2u) << "one shaded face per finished profile";
    for (const auto& o : overlays) {
        EXPECT_FALSE(o.triangles.empty());
        EXPECT_NEAR(o.rgba[3], 0.35f, 1e-6) << "light orange at ~35% alpha";
    }

    render::PickResult first;
    first.overlayIndex = 0u;
    w.routePick(first);
    const std::string afterFirst = panel->model().text("profile");
    render::PickResult second;
    second.overlayIndex = 1u;
    w.routePick(second);
    const std::string afterSecond = panel->model().text("profile");
    EXPECT_NE(afterFirst, afterSecond) << "a click on a profile chooses it";

    panel->accept();
    ASSERT_TRUE(w.settle(5000));
    EXPECT_TRUE(w.viewport()->overlayFaces().empty()) << "the overlays go with the dialog";
    EXPECT_EQ(w.model()->timeline().size(), 2u);
    EXPECT_FALSE(w.shownEvaluation().meshes.empty());
}

// ---- 5. icons ----------------------------------------------------------------------------------------------------

bool hasAccentGreen(const QImage& img) {
    for (int y = 0; y < img.height(); ++y)
        for (int x = 0; x < img.width(); ++x) {
            const QColor c = img.pixelColor(x, y);
            if (c.alpha() > 200 && c.green() > 120 && c.red() < 100 && c.blue() < 150) return true;
        }
    return false;
}

TEST(UiIntegrationIcons, RibbonShowsShortLabelsAndKeepsGroupAccentColours) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    ASSERT_NE(w.commands().find("parameters"), nullptr);
    EXPECT_EQ(w.commands().find("parameters")->shortLabel.toStdString(), "Parameters");
    auto* params = w.ribbon()->findChild<QToolButton*>("btn_parameters");
    ASSERT_NE(params, nullptr);
    EXPECT_EQ(params->text().toStdString(), "Parameters");
    EXPECT_EQ(w.actions()->action("parameters")->text().toStdString(), "Change Parameters...") << "menus keep the full label";

    auto* extrude = w.ribbon()->findChild<QToolButton*>("btn_extrude");
    ASSERT_NE(extrude, nullptr);
    ASSERT_TRUE(extrude->isEnabled());
    EXPECT_TRUE(hasAccentGreen(extrude->icon().pixmap(32, 32).toImage())) << "the extrude accent (green) is drawn";
    EXPECT_FALSE(w.actions()->action("extrude")->icon().isNull()) << "command actions use ui::icon";
}

TEST(UiIntegrationIcons, TimelineAndBrowserUseTheirIconSet) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildBlock(w);
    ASSERT_EQ(w.timelineWidget()->entries().size(), 2u);
    EXPECT_EQ(w.timelineWidget()->entries()[0].icon, "timeline_sketch");
    EXPECT_EQ(w.timelineWidget()->entries()[1].icon, "timeline_extrude");
    EXPECT_TRUE(ui::iconExists("timeline_extrude"));

    QTreeWidget* tree = w.browser();
    QTreeWidgetItem* bodies = nullptr;
    for (int i = 0; i < tree->topLevelItemCount(); ++i)
        if (tree->topLevelItem(i)->text(0) == "Bodies") bodies = tree->topLevelItem(i);
    ASSERT_NE(bodies, nullptr);
    ASSERT_EQ(bodies->childCount(), 1);
    EXPECT_FALSE(bodies->child(0)->icon(0).isNull()) << "bodies carry browser_body";
    EXPECT_FALSE(bodies->icon(0).isNull()) << "groups carry browser_folder";
}

TEST(UiIntegrationIcons, SboxRowsAndMarkingSlotsCarryIcons) {
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.settle(5000));
    w.openCommandSearch();
    w.commandSearch()->setQuery("extrude");
    auto* list = w.commandSearch()->findChild<QListWidget*>();
    ASSERT_NE(list, nullptr);
    ASSERT_GT(list->count(), 0);
    EXPECT_FALSE(list->item(0)->icon().isNull()) << "S-box rows show the command icon";

    for (std::string_view id : ui::markingSlotCommands()) {
        const ui::Command* c = w.commands().find(id);
        ASSERT_NE(c, nullptr) << id;
        EXPECT_TRUE(ui::iconExists(c->icon)) << id << " -> " << c->icon;
    }
}

// ---- screenshots -------------------------------------------------------------------------------------------------

TEST(UiIntegrationScreenshots, WindowTimelineParametersAndMeasure) {
    const char* dir = std::getenv("IW_TEST_DUMP_DIR");
    if (!dir) GTEST_SKIP() << "set IW_TEST_DUMP_DIR to save the screenshots";
    const QString base = QString::fromLocal8Bit(dir);

    ui::MainWindow w;
    w.resize(1280, 800);
    w.show();
    ASSERT_TRUE(w.settle(5000));
    buildBlock(w);
    ASSERT_TRUE(w.model()->addParameter("height", "10 mm", "", "Block height"));
    for (int i = 0; i < 5; ++i) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    EXPECT_TRUE(w.grab().save(base + "/ui_integration_window.png"));

    // Heap-owned: the dialog deletes itself on close (WA_DeleteOnClose), as it does when opened from the shell.
    auto* params = new ui::ParametersDialog(*w.model(), &w);
    params->resize(760, 420);
    params->show();
    for (int i = 0; i < 3; ++i) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    EXPECT_TRUE(params->grab().save(base + "/ui_integration_parameters.png"));
    params->close();

    const ShownBody body = firstBody(w);
    render::PickResult a;
    a.bodyIndex = 0u;
    a.faceIndex = *planarFace(body.shape, Vec3{0, 0, 1});
    w.routePick(a);
    render::PickResult b = a;
    b.faceIndex = *planarFace(body.shape, Vec3{0, -1, 0});
    w.routePick(b);
    w.actions()->run("measure");
    auto* measure = qobject_cast<ui::inspect::MeasureDialog*>(w.commandDock()->widget());
    ASSERT_NE(measure, nullptr);
    measure->pickFromProvider(1);
    measure->pickFromProvider(2);
    for (int i = 0; i < 3; ++i) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    EXPECT_TRUE(measure->grab().save(base + "/ui_integration_measure.png"));
}

} // namespace
