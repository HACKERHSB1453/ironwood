// SPDX-License-Identifier: GPL-3.0-or-later
// B6: Inspect widgets (measure dialog, properties panel, section dialog) and command registration. Offscreen only.
#include <gtest/gtest.h>

#include <QApplication>
#include <QCoreApplication>
#include <QClipboard>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QKeySequence>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QCheckBox>
#include <QWidget>
#include <QGuiApplication>

#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

#include "kernel/measure.hpp"
#include "kernel/ops.hpp"
#include "ui/command.hpp"
#include "ui/inspect/inspect_commands.hpp"
#include "ui/inspect/measure_dialog.hpp"
#include "ui/inspect/properties_panel.hpp"
#include "ui/inspect/section_dialog.hpp"

namespace iw::ui::inspect {
namespace {

namespace km = kernel::measure;

/// One QApplication for the binary (offscreen).
class InspectEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
        app_ = new QApplication(argc_, argv_);
    }
    void TearDown() override { delete app_; }

private:
    int argc_ = 1;
    char name_[16] = "test_ui_inspect";
    char* argv_[2] = {name_, nullptr};
    QApplication* app_ = nullptr;
};
const auto* const kEnv = ::testing::AddGlobalTestEnvironment(new InspectEnv);

constexpr double kTol = 1e-6;

kernel::Shape box(Vec3 o, double dx, double dy, double dz) {
    auto r = kernel::makeBox(o, dx, dy, dz);
    EXPECT_TRUE(r.has_value());
    return r ? r->shape : kernel::Shape{};
}

std::optional<std::uint32_t> faceWithNormal(const kernel::Shape& s, Vec3 n, double planeCoord, int axis) {
    for (std::uint32_t i = 0; i < s.subShapeCount(kernel::SubShapeKind::Face); ++i) {
        const kernel::FaceGeometry g = s.faceGeometry(i);
        if (g.kind != kernel::FaceGeometry::Kind::Plane) continue;
        if (length(g.normal - n) > 1e-7) continue;
        const double c = axis == 0 ? g.point.x : axis == 1 ? g.point.y : g.point.z;
        if (std::abs(c - planeCoord) < 1e-7) return i;
    }
    return std::nullopt;
}

std::optional<std::uint32_t> vertexAt(const kernel::Shape& s, Vec3 p) {
    for (std::uint32_t i = 0; i < s.subShapeCount(kernel::SubShapeKind::Vertex); ++i) {
        auto v = km::vertexPoint(km::Entity{km::EntityKind::Vertex, s, i});
        if (v && length(*v - p) < 1e-7) return i;
    }
    return std::nullopt;
}

// ----------------------------------------------------------------------------------------- measure dialog

TEST(MeasureDialogTest, TwoFacesGiveAngleAndDistanceRows) {
    const kernel::Shape b = box({0, 0, 0}, 10, 10, 10);
    const auto top = faceWithNormal(b, {0, 0, 1}, 10, 2);
    const auto side = faceWithNormal(b, {1, 0, 0}, 10, 0);
    ASSERT_TRUE(top && side);
    const std::vector<km::Entity> picks{{km::EntityKind::Face, b, *top}, {km::EntityKind::Face, b, *side}};
    MeasureDialog dlg([&picks] { return picks; });
    EXPECT_TRUE(dlg.pickFromProvider(1));
    EXPECT_TRUE(dlg.pickFromProvider(2));
    ASSERT_TRUE(dlg.selection(1).has_value());
    ASSERT_TRUE(dlg.selection(2).has_value());
    EXPECT_EQ(dlg.findChild<QLabel*>("slotLabel1")->text(), entityLabel(picks[0]));
    bool sawAngle = false;
    for (const auto& r : dlg.rows())
        if (r.label == "Angle") {
            sawAngle = true;
            EXPECT_NEAR(r.value, 90.0, kTol);
        }
    EXPECT_TRUE(sawAngle);
    const auto lines = dlg.resultLines();
    bool shown = false;
    for (const auto& l : lines)
        if (l == "Angle: 90.000 deg") shown = true;
    EXPECT_TRUE(shown) << dlg.copyText().toStdString();
    EXPECT_EQ(dlg.findChild<QListWidget*>("results")->count(), static_cast<int>(lines.size()));
}

TEST(MeasureDialogTest, PrecisionChangesFormatting) {
    const kernel::Shape b = box({0, 0, 0}, 10, 10, 10);
    MeasureDialog dlg;
    dlg.setSelection(1, {km::EntityKind::Face, b, *faceWithNormal(b, {0, 0, 1}, 10, 2)});
    ASSERT_FALSE(dlg.rows().empty());
    EXPECT_EQ(dlg.precision(), 3);
    EXPECT_EQ(dlg.resultLines().at(0), "Area: 100.000 mm²");
    dlg.setPrecision(1);
    EXPECT_EQ(dlg.resultLines().at(0), "Area: 100.0 mm²");
    dlg.setPrecision(42); // clamped
    EXPECT_EQ(dlg.precision(), 9);
}

TEST(MeasureDialogTest, SingleVertexShowsCoordinates) {
    const kernel::Shape b = box({0, 0, 0}, 10, 20, 30);
    MeasureDialog dlg;
    dlg.setSelection(2, {km::EntityKind::Vertex, b, *vertexAt(b, {10, 20, 30})});
    EXPECT_EQ(entityLabel(*dlg.selection(2)).left(6), "Vertex");
    const auto lines = dlg.resultLines();
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_EQ(lines[0], "Position X: 10.000 mm");
    EXPECT_EQ(lines[2], "Position Z: 30.000 mm");
}

TEST(MeasureDialogTest, EmptySelectionAndProviderEdgeCases) {
    MeasureDialog none;
    EXPECT_FALSE(none.pickFromProvider(1));
    EXPECT_TRUE(none.resultLines().isEmpty());
    EXPECT_FALSE(none.copyToClipboard());
    EXPECT_FALSE(none.selection(1).has_value());
    EXPECT_FALSE(none.pickFromProvider(3)); // invalid slot
    none.clearSelection(1);
    EXPECT_TRUE(none.rows().empty());
}

TEST(MeasureDialogTest, FailureIsReportedNotThrown) {
    const kernel::Shape b = box({0, 0, 0}, 1, 1, 1);
    MeasureDialog dlg;
    dlg.setSelection(1, {km::EntityKind::Face, b, 999});
    EXPECT_TRUE(dlg.rows().empty());
    EXPECT_FALSE(dlg.statusText().isEmpty());
}

TEST(MeasureDialogTest, CopyPutsResultsOnClipboard) {
    const kernel::Shape b = box({0, 0, 0}, 10, 10, 10);
    MeasureDialog dlg;
    dlg.setSelection(1, {km::EntityKind::Face, b, *faceWithNormal(b, {0, 0, 1}, 10, 2)});
    dlg.setPrecision(2);
    ASSERT_TRUE(dlg.copyToClipboard());
    const QString clip = QGuiApplication::clipboard()->text();
    EXPECT_EQ(clip, dlg.copyText());
    EXPECT_EQ(clip, QStringLiteral("Area: 100.00 mm²"));
}

TEST(MeasureDialogTest, SelectionChangedEmitsResults) {
    const kernel::Shape b = box({0, 0, 0}, 10, 10, 10);
    MeasureDialog dlg;
    int emitted = 0;
    QObject::connect(&dlg, &MeasureDialog::resultsChanged, [&emitted] { ++emitted; });
    dlg.setSelection(1, {km::EntityKind::Face, b, *faceWithNormal(b, {0, 0, 1}, 10, 2)});
    dlg.clearSelection(1);
    EXPECT_EQ(emitted, 2);
    EXPECT_TRUE(dlg.rows().empty());
}

// ----------------------------------------------------------------------------------------- properties

TEST(PropertiesPanelTest, BoxValuesAndMaterialMass) {
    const kernel::Shape b = box({0, 0, 0}, 10, 20, 30);
    PropertiesPanel panel;
    EXPECT_EQ(panel.findChild<QLabel*>("name")->text(), "No body");
    EXPECT_EQ(panel.statusText(), "No body selected.");

    panel.setBody(b, "Bracket");
    ASSERT_TRUE(panel.properties().has_value());
    EXPECT_EQ(panel.findChild<QLabel*>("name")->text(), "Bracket");
    EXPECT_NEAR(panel.properties()->volume, 6000.0, kTol);
    EXPECT_EQ(panel.findChild<QLabel*>("volume")->text(), "6000.000 mm³");
    // Default material is Steel (7850 kg/m^3): 6000 mm^3 = 0.0471 kg.
    EXPECT_EQ(panel.findChild<QLabel*>("mass")->text(), "0.0471 kg");
    EXPECT_EQ(panel.findChild<QLabel*>("bboxSize")->text(), "X 10.000  Y 20.000  Z 30.000 mm");

    EXPECT_NEAR(panel.density(), 7850.0, kTol);
    ASSERT_TRUE(panel.setMaterial("Water"));
    EXPECT_NEAR(panel.density(), 1000.0, kTol);
    EXPECT_NEAR(panel.properties()->mass, 6000e-9 * 1000.0, 1e-12); // 0.006 kg
    EXPECT_FALSE(panel.setMaterial("Unobtainium"));
}

TEST(PropertiesPanelTest, CustomDensityAndNullBody) {
    const kernel::Shape b = box({0, 0, 0}, 1000, 1000, 1000); // 1 m^3
    PropertiesPanel panel;
    panel.setBody(b);
    panel.setCustomDensity(2700.0);
    EXPECT_EQ(panel.materialName(), "Custom");
    EXPECT_NEAR(panel.properties()->mass, 2700.0, 1e-6); // 1 m^3 of aluminium-ish density
    EXPECT_TRUE(panel.findChild<QDoubleSpinBox*>("customDensity")->isEnabled());

    panel.setBody(kernel::Shape{});
    EXPECT_FALSE(panel.properties().has_value());
    EXPECT_EQ(panel.statusText(), "No body selected.");
}

TEST(PropertiesPanelTest, MaterialTableHasCustomLast) {
    const auto& m = materials();
    ASSERT_FALSE(m.empty());
    EXPECT_EQ(m.back().name, "Custom");
    for (const auto& x : m) EXPECT_GE(x.densityKgPerM3, 0.0);
}

// ----------------------------------------------------------------------------------------- section

TEST(SectionDialogTest, DisabledByDefaultThenPublishesClipPlane) {
    SectionDialog dlg;
    EXPECT_FALSE(dlg.request().enabled);

    std::vector<km::section::RenderRequest> seen;
    dlg.setRequestSink([&seen](const km::section::RenderRequest& r) { seen.push_back(r); });
    ASSERT_EQ(seen.size(), 1u); // set once on attach

    dlg.setEnabled(true);
    dlg.setNormalAxis(0); // X
    dlg.setOffset(3.0);
    const km::section::RenderRequest r = dlg.request();
    EXPECT_TRUE(r.enabled);
    EXPECT_NEAR(r.clipPlane[0], 1.0, kTol);
    EXPECT_NEAR(r.clipPlane[3], -3.0, kTol);
    EXPECT_FALSE(seen.empty());
    EXPECT_EQ(seen.back(), r);
}

TEST(SectionDialogTest, FlipReversesClipAndCapColourIsRgba) {
    SectionDialog dlg;
    dlg.setEnabled(true);
    dlg.setNormalAxis(2);
    dlg.setOrigin({0, 0, 5});
    const auto before = dlg.request();
    EXPECT_NEAR(before.clipPlane[3], -5.0, kTol);
    dlg.flip();
    const auto after = dlg.request();
    EXPECT_NEAR(after.clipPlane[2], -1.0, kTol);
    EXPECT_NEAR(after.clipPlane[3], 5.0, kTol);
    EXPECT_FLOAT_EQ(after.capColour[3], 1.0f);
    dlg.findChild<QComboBox*>("cap")->setCurrentIndex(1);
    EXPECT_FLOAT_EQ(dlg.request().capColour[0], 0.90f);
}

TEST(SectionDialogTest, RequestSignalEmitted) {
    SectionDialog dlg;
    int count = 0;
    QObject::connect(&dlg, &SectionDialog::requestChanged, [&count](const km::section::RenderRequest&) { ++count; });
    dlg.setEnabled(true);
    dlg.setOffset(1.0);
    dlg.flip();
    EXPECT_GE(count, 3);
}

// ----------------------------------------------------------------------------------------- commands

TEST(InspectCommandsTest, RegistersThreeCommandsWithFusionShortcut) {
    ui::CommandRegistry reg;
    InspectContext ctx;
    ASSERT_TRUE(registerCommands(reg, ctx).has_value());
    ASSERT_NE(reg.find("measure"), nullptr);
    ASSERT_NE(reg.find("properties"), nullptr);
    ASSERT_NE(reg.find("section_analysis"), nullptr);
    EXPECT_EQ(reg.shortcutOf("measure"), QKeySequence(QStringLiteral("I")));
    EXPECT_EQ(reg.findByShortcut(QKeySequence(QStringLiteral("I")))->id, "measure");
    // Registering twice reports the duplicate instead of silently dropping it.
    EXPECT_FALSE(registerCommands(reg, ctx).has_value());
}

TEST(InspectCommandsTest, MeasureDocksDialogWithContextSelection) {
    const kernel::Shape b = box({0, 0, 0}, 10, 10, 10);
    const std::vector<km::Entity> sel{{km::EntityKind::Face, b, *faceWithNormal(b, {0, 0, 1}, 10, 2)}};
    ui::CommandRegistry reg;
    InspectContext ctx;
    ctx.selection = [&sel] { return sel; };
    ASSERT_TRUE(registerCommands(reg, ctx).has_value());

    QWidget* docked = nullptr;
    ui::CommandContext cc;
    cc.showPanel = [&docked](QWidget* w) { docked = w; };
    ASSERT_TRUE(reg.run("measure", cc).has_value());
    ASSERT_NE(docked, nullptr);
    auto* dlg = dynamic_cast<MeasureDialog*>(docked);
    ASSERT_NE(dlg, nullptr);
    EXPECT_TRUE(dlg->pickFromProvider(1));
    EXPECT_EQ(dlg->resultLines().at(0), "Area: 100.000 mm²");
    delete docked;
}

TEST(InspectCommandsTest, PropertiesEnabledOnlyWithBody) {
    const kernel::Shape b = box({0, 0, 0}, 10, 10, 10);
    std::optional<kernel::Shape> body;
    ui::CommandRegistry reg;
    InspectContext ctx;
    ctx.activeBody = [&body] { return body; };
    ASSERT_TRUE(registerCommands(reg, ctx).has_value());

    const ui::Command* props = reg.find("properties");
    ui::CommandContext cc;
    EXPECT_FALSE(props->isEnabled(cc));
    body = b;
    EXPECT_TRUE(props->isEnabled(cc));

    QWidget* docked = nullptr;
    cc.showPanel = [&docked](QWidget* w) { docked = w; };
    ASSERT_TRUE(reg.run("properties", cc).has_value());
    auto* panel = dynamic_cast<PropertiesPanel*>(docked);
    ASSERT_NE(panel, nullptr);
    ASSERT_TRUE(panel->properties().has_value());
    EXPECT_NEAR(panel->properties()->volume, 1000.0, kTol);
    delete docked;
}

TEST(InspectCommandsTest, SectionSinkReceivesRequestsFromDockedDialog) {
    ui::CommandRegistry reg;
    InspectContext ctx;
    int calls = 0;
    ctx.sectionSink = [&calls](const km::section::RenderRequest&) { ++calls; };
    ASSERT_TRUE(registerCommands(reg, ctx).has_value());

    QWidget* docked = nullptr;
    ui::CommandContext cc;
    cc.showPanel = [&docked](QWidget* w) { docked = w; };
    ASSERT_TRUE(reg.run("section_analysis", cc).has_value());
    auto* dlg = dynamic_cast<SectionDialog*>(docked);
    ASSERT_NE(dlg, nullptr);
    const int before = calls;
    dlg->setEnabled(true);
    EXPECT_GT(calls, before);
    delete docked;
}

} // namespace
} // namespace iw::ui::inspect
