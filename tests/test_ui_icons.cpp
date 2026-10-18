// SPDX-License-Identifier: GPL-3.0-or-later
// F25: Ironwood's own icon set, ui::icon() loader with fallback, ribbon icon layout. Offscreen only.
#include <gtest/gtest.h>

#include <QApplication>
#include <QColor>
#include <QCoreApplication>
#include <QFile>
#include <QFont>
#include <QGridLayout>
#include <QImage>
#include <QLabel>
#include <QPainter>
#include <QPalette>
#include <QToolButton>

#include <cstdlib>
#include <string>
#include <vector>

#include "ui/command.hpp"
#include "ui/command_actions.hpp"
#include "ui/default_commands.hpp"
#include "ui/icons.hpp"
#include "ui/ribbon.hpp"

namespace {

using iw::ui::IconTheme;

/// Every icon the set ships (tool, constraint, timeline, browser, fallback and the two extras used by the S-box and
/// Repeat Last commands). Keep in sync with resources/icons/ (tests check that each one resolves).
const std::vector<std::string> kAllIcons = {
    // sketch tools
    "sketch", "line", "rectangle", "circle", "arc", "point", "trim", "offset", "construction", "dimension",
    // sketch constraints
    "constraint_horizontal", "constraint_vertical", "constraint_coincident", "constraint_tangent",
    "constraint_equal", "constraint_parallel", "constraint_perpendicular", "constraint_fix", "constraint_midpoint",
    "constraint_concentric", "constraint_collinear", "constraint_symmetric", "constraint_point_on",
    // solid, construction, inspect, insert, edit, assemble
    "extrude", "revolve", "hole", "press_pull", "fillet", "chamfer", "shell", "move", "delete", "combine",
    "offset_plane", "axis", "construction_point", "measure", "section", "import", "export", "parameters", "undo",
    "redo", "finish_sketch", "joint", "new_component", "select",
    // timeline feature glyphs
    "timeline_sketch", "timeline_extrude", "timeline_revolve", "timeline_fillet", "timeline_chamfer",
    "timeline_hole", "timeline_move", "timeline_plane", "timeline_axis", "timeline_point",
    // browser tree
    "browser_origin", "browser_plane", "browser_axis", "browser_body", "browser_sketch", "browser_folder",
    // extras and fallback
    "search", "repeat", "missing",
};

class IconsEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
        app_ = new QApplication(argc_, argv_);
    }
    void TearDown() override { delete app_; }

private:
    int argc_ = 1;
    char name_[16] = "test_ui_icons";
    char* argv_[2] = {name_, nullptr};
    QApplication* app_ = nullptr;
};

const auto* kEnv = ::testing::AddGlobalTestEnvironment(new IconsEnv);

QString dumpDir() {
    const char* dir = std::getenv("IW_TEST_DUMP_DIR");
    return dir ? QString::fromLocal8Bit(dir) : QString();
}

QByteArray readResource(const std::string& name) {
    QFile f(QString::fromStdString(":/icons/" + name + ".svg"));
    return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
}

/// Contact sheet: every icon at 48 px with its name, on a light and a dark background.
QImage contactSheet(IconTheme theme) {
    const int cols = 8, cell = 96, cellH = 84;
    const int rows = static_cast<int>((kAllIcons.size() + cols - 1) / cols);
    QImage sheet(cols * cell, rows * cellH, QImage::Format_ARGB32_Premultiplied);
    const QColor bg = theme == IconTheme::Dark ? QColor("#1e2127") : QColor("#f5f6f8");
    const QColor fg = theme == IconTheme::Dark ? QColor("#e8eaed") : QColor("#2b2f36");
    sheet.fill(bg);
    QPainter p(&sheet);
    QFont small = p.font();
    small.setPointSizeF(7.0);
    p.setFont(small);
    p.setPen(fg);
    for (std::size_t i = 0; i < kAllIcons.size(); ++i) {
        const int x = static_cast<int>(i % cols) * cell, y = static_cast<int>(i / cols) * cellH;
        p.drawImage(x + (cell - 48) / 2, y + 6, iw::ui::renderIcon(kAllIcons[i], 48, theme));
        p.drawText(QRect(x, y + 58, cell, cellH - 58), Qt::AlignHCenter | Qt::AlignTop,
                   QString::fromStdString(kAllIcons[i]));
    }
    p.end();
    return sheet;
}

/// The shell's default commands, registered before anything that reads them (base class => initialised first).
struct DefaultRegistry {
    DefaultRegistry() {
        auto r = iw::ui::registerDefaultCommands(registry);
        EXPECT_TRUE(r) << (r ? "" : r.error().message);
    }
    iw::ui::CommandRegistry registry;
};

/// A ribbon built from the real default commands, the way the shell builds it.
struct RibbonFixture : DefaultRegistry {
    iw::ui::CommandContext ctx;
    iw::ui::CommandActions actions{registry, ctx};
    iw::ui::RibbonBar ribbon{registry, actions};
};

QPalette lightPalette() {
    QPalette p;
    p.setColor(QPalette::Window, QColor("#f3f4f6"));
    p.setColor(QPalette::WindowText, QColor("#1f2328"));
    p.setColor(QPalette::Button, QColor("#ffffff"));
    p.setColor(QPalette::ButtonText, QColor("#1f2328"));
    p.setColor(QPalette::Text, QColor("#1f2328"));
    p.setColor(QPalette::Base, QColor("#ffffff"));
    return p;
}

QPalette darkPalette() {
    QPalette p;
    p.setColor(QPalette::Window, QColor("#1e2127"));
    p.setColor(QPalette::WindowText, QColor("#e8eaed"));
    p.setColor(QPalette::Button, QColor("#2a2e36"));
    p.setColor(QPalette::ButtonText, QColor("#e8eaed"));
    p.setColor(QPalette::Text, QColor("#e8eaed"));
    p.setColor(QPalette::Base, QColor("#16181d"));
    return p;
}

} // namespace

TEST(UiIcons, EveryPlannedIconIsBundledAsA24GridSvg) {
    for (const std::string& name : kAllIcons) {
        EXPECT_TRUE(iw::ui::iconExists(name)) << name;
        const QByteArray svg = readResource(name);
        ASSERT_FALSE(svg.isEmpty()) << name;
        EXPECT_NE(svg.indexOf("viewBox=\"0 0 24 24\""), -1) << name << " is not on the 24x24 grid";
        EXPECT_NE(svg.indexOf("stroke-width=\"1.5\""), -1) << name << " does not use the 1.5px stroke";
        EXPECT_NE(svg.indexOf("currentColor"), -1) << name << " must stroke with currentColor (theme-aware)";
    }
    EXPECT_EQ(kAllIcons.size(), 66u);
}

TEST(UiIcons, UnknownNamesFallBackToTheMissingGlyph) {
    EXPECT_FALSE(iw::ui::iconExists("no_such_icon"));
    EXPECT_FALSE(iw::ui::iconExists(""));
    const QImage fallback = iw::ui::renderIcon("missing", 32, IconTheme::Light);
    ASSERT_FALSE(fallback.isNull());
    EXPECT_EQ(iw::ui::renderIcon("no_such_icon", 32, IconTheme::Light), fallback);
    EXPECT_EQ(iw::ui::renderIcon("", 32, IconTheme::Light), fallback);
    const QIcon icon = iw::ui::icon("no_such_icon", 24, IconTheme::Dark);
    EXPECT_FALSE(icon.isNull());
    EXPECT_EQ(icon.pixmap(24, 24).size(), QSize(24, 24));
    EXPECT_TRUE(iw::ui::renderIcon("extrude", 0, IconTheme::Light).isNull());
}

TEST(UiIcons, RenderedSizeAndThemeColourFollowTheRequest) {
    const QImage light = iw::ui::renderIcon("extrude", 32, IconTheme::Light);
    const QImage dark = iw::ui::renderIcon("extrude", 32, IconTheme::Dark);
    ASSERT_FALSE(light.isNull());
    EXPECT_EQ(light.size(), QSize(32, 32));
    EXPECT_EQ(dark.size(), QSize(32, 32));
    EXPECT_NE(light, dark) << "strokes must change with the theme";
    // A fixed accent is shared by both themes: the extrude arrow is accent-coloured, so some pixel matches in both.
    bool accentInBoth = false;
    for (int y = 0; y < 32 && !accentInBoth; ++y)
        for (int x = 0; x < 32 && !accentInBoth; ++x)
            accentInBoth = light.pixelColor(x, y).alpha() == 255 && light.pixelColor(x, y) == dark.pixelColor(x, y) &&
                           light.pixelColor(x, y).green() > light.pixelColor(x, y).red() + 40;
    EXPECT_TRUE(accentInBoth) << "the accent (green group) must stay the same in both themes";
}

TEST(UiIcons, EveryRegistryCommandResolvesToAnIcon) {
    iw::ui::CommandRegistry reg;
    ASSERT_TRUE(iw::ui::registerDefaultCommands(reg));
    ASSERT_FALSE(reg.all().empty());
    for (const iw::ui::Command& c : reg.all()) {
        EXPECT_FALSE(c.icon.empty()) << c.id << " has no icon name";
        EXPECT_TRUE(iw::ui::iconExists(c.icon)) << c.id << " -> " << c.icon << " is not in resources/icons";
        EXPECT_FALSE(iw::ui::renderIcon(c.icon, 32, IconTheme::Light).isNull()) << c.id;
    }
}

TEST(UiIcons, ContactSheetLightAndDark) {
    const QImage light = contactSheet(IconTheme::Light);
    const QImage dark = contactSheet(IconTheme::Dark);
    EXPECT_GT(light.width(), 0);
    const QString dir = dumpDir();
    if (dir.isEmpty()) GTEST_SKIP() << "set IW_TEST_DUMP_DIR to save the contact sheets";
    EXPECT_TRUE(light.save(dir + "/icons_light.png"));
    EXPECT_TRUE(dark.save(dir + "/icons_dark.png"));
}

TEST(UiIcons, RibbonShowsLargeIconsAboveGroupRowAndGreysDisabled) {
    RibbonFixture f;
    f.ribbon.resize(1100, 180);
    f.ribbon.show();
    for (int i = 0; i < 3; ++i) QCoreApplication::processEvents();

    QToolButton* extrude = f.ribbon.findChild<QToolButton*>("btn_extrude");
    ASSERT_NE(extrude, nullptr);
    EXPECT_EQ(extrude->iconSize(), QSize(32, 32));
    EXPECT_FALSE(extrude->icon().isNull()) << "ribbon buttons carry the command icon";

    auto* title = f.ribbon.findChild<QLabel*>("title_CREATE");
    ASSERT_NE(title, nullptr);
    EXPECT_GE(title->mapTo(&f.ribbon, title->rect().topLeft()).y(),
              extrude->mapTo(&f.ribbon, extrude->rect().bottomLeft()).y())
        << "group label row sits underneath the large icons";
    QToolButton* more = f.ribbon.findChild<QToolButton*>("more_CREATE");
    ASSERT_NE(more, nullptr);
    EXPECT_EQ(more->text(), QStringLiteral("▾")) << "dropdown arrow on the group row";

    // press_pull has no handler yet: disabled, so its icon must render in the Disabled (grey) state.
    QToolButton* pp = f.ribbon.findChild<QToolButton*>("btn_press_pull");
    ASSERT_NE(pp, nullptr);
    EXPECT_FALSE(pp->isEnabled());
    const QIcon ic = pp->icon();
    EXPECT_NE(ic.pixmap(32, 32, QIcon::Normal).toImage(), ic.pixmap(32, 32, QIcon::Disabled).toImage());

    const QString dir = dumpDir();
    if (!dir.isEmpty()) {
        f.ribbon.setPalette(lightPalette());
        for (int i = 0; i < 3; ++i) QCoreApplication::processEvents();
        EXPECT_TRUE(f.ribbon.grab().save(dir + "/ribbon_light.png"));
        f.ribbon.setPalette(darkPalette());
        for (int i = 0; i < 3; ++i) QCoreApplication::processEvents();
        EXPECT_TRUE(f.ribbon.grab().save(dir + "/ribbon_dark.png"));
    }
}
