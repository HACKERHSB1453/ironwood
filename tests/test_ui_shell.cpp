// SPDX-License-Identifier: GPL-3.0-or-later
// B4: command registry, ribbon, S-box, marking menu, command dialogs, manipulators, parameters dialog, shell.
#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QKeySequence>
#include <QMenu>
#include <QMouseEvent>
#include <QSettings>
#include <QTableWidget>
#include <QTemporaryDir>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <string>
#include <vector>

#include "core/job.hpp"
#include "core/params.hpp"
#include "core/undo.hpp"
#include "ui/command.hpp"
#include "ui/command_actions.hpp"
#include "ui/command_dialog.hpp"
#include "ui/default_commands.hpp"
#include "ui/doc_state.hpp"
#include "ui/fuzzy.hpp"
#include "ui/manipulators.hpp"
#include "ui/marking_menu.hpp"
#include "ui/parameters_dialog.hpp"
#include "ui/ribbon.hpp"
#include "ui/sbox.hpp"
#include "render/viewport.hpp"
#include "ui/main_window.hpp"

using namespace iw;
using namespace iw::ui;

namespace {

/// One QApplication for the binary. Settings go to a test-only organisation, cleared before every run.
class ShellEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
        app_ = new QApplication(argc_, argv_);
        QCoreApplication::setOrganizationName("IronwoodTest");
        QCoreApplication::setApplicationName("ironwood-ui-shell-test");
        QSettings().clear();
    }
    void TearDown() override {
        QSettings().clear();
        delete app_;
    }

private:
    int argc_ = 1;
    char name_[16] = "test_ui_shell";
    char* argv_[2] = {name_, nullptr};
    QApplication* app_ = nullptr;
};
const auto* const kEnv = ::testing::AddGlobalTestEnvironment(new ShellEnv);

/// A registry with the shell's default commands; fails the test (and aborts) when registration fails.
CommandRegistry defaultRegistry() {
    CommandRegistry reg;
    auto r = registerDefaultCommands(reg);
    EXPECT_TRUE(r) << (r ? "" : r.error().message);
    return reg;
}

iw::ui::Command makeCommand(std::string id, std::string label, QKeySequence shortcut = {}) {
    iw::ui::Command c;
    c.id = std::move(id);
    c.label = QString::fromStdString(label);
    c.shortcut = shortcut;
    c.start = [](CommandContext&) {};
    return c;
}

/// Sends a mouse event to a widget as the windowing system would.
void sendMouse(QWidget* w, QEvent::Type type, QPointF pos, Qt::MouseButton button) {
    QMouseEvent e(type, pos, w->mapToGlobal(pos.toPoint()), button,
                  type == QEvent::MouseButtonRelease ? Qt::NoButton : button, Qt::NoModifier);
    QApplication::sendEvent(w, &e);
}

} // namespace

// ---- registry & shortcuts -------------------------------------------------------------------------------------

TEST(UiShellRegistry, DefaultCommandsExistWithTheirImplementationState) {
    CommandRegistry reg = defaultRegistry();
    for (const char* id : {"import", "undo", "redo", "parameters", "sketch"}) {
        ASSERT_NE(reg.find(id), nullptr) << id;
        EXPECT_TRUE(reg.find(id)->implemented()) << id;
    }
    for (const char* id : {"measure", "extrude", "revolve", "fillet", "chamfer", "hole", "plane", "axis",
                           "point", "move"}) {
        const iw::ui::Command* c = reg.find(id);
        if (!c) {
            ADD_FAILURE() << "missing command " << id;
            continue;
        }
        EXPECT_FALSE(c->implemented()) << id << " should be a disabled placeholder";
    }
    CommandContext ctx;
    EXPECT_FALSE(reg.find("extrude")->isEnabled(ctx));
    EXPECT_EQ(reg.run("extrude", ctx).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(reg.run("nope", ctx).error().code, ErrorCode::NotFound);
}

TEST(UiShellRegistry, DefaultRegistryHasNoShortcutConflicts) {
    CommandRegistry reg = defaultRegistry();
    EXPECT_TRUE(reg.shortcutConflicts().empty());
    EXPECT_EQ(reg.shortcutOf("undo").toString(QKeySequence::PortableText), "Ctrl+Z");
    EXPECT_EQ(reg.findByShortcut(QKeySequence("E"))->id, "extrude");
    EXPECT_EQ(reg.findByShortcut(QKeySequence("S"))->id, "sbox");
}

TEST(UiShellRegistry, GroupLayoutOfTheSolidTab) {
    CommandRegistry reg = defaultRegistry();
    EXPECT_EQ(reg.groups("DESIGN", "SOLID"),
              (std::vector<std::string>{"CREATE", "MODIFY", "ASSEMBLE", "CONSTRUCT", "INSPECT", "INSERT", "SELECT"}));
    EXPECT_TRUE(reg.groups("DESIGN", "SURFACE").empty()); // listed in the ribbon as "coming soon"
}

TEST(UiShellRegistry, DuplicateIdIsRejected) {
    CommandRegistry reg;
    ASSERT_TRUE(reg.add(makeCommand("a", "A")));
    auto r = reg.add(makeCommand("a", "A again"));
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
    EXPECT_FALSE(reg.add(makeCommand("", "empty id")));
}

TEST(UiShellRegistry, DuplicateDefaultShortcutsAreReportedAndNewOnesRefused) {
    CommandRegistry reg;
    ASSERT_TRUE(reg.add(makeCommand("first", "First", QKeySequence("X"))));
    ASSERT_TRUE(reg.add(makeCommand("second", "Second", QKeySequence("X"))));
    ASSERT_TRUE(reg.add(makeCommand("third", "Third", QKeySequence("Y"))));

    const auto conflicts = reg.shortcutConflicts();
    ASSERT_EQ(conflicts.size(), 1u);
    EXPECT_EQ(conflicts[0].first, "first");
    EXPECT_EQ(conflicts[0].second, "second");

    // A user remap onto a taken key is refused, and the message names the owner.
    auto r = reg.setShortcut("third", QKeySequence("X"));
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(r.error().message.find("First"), std::string::npos);
    EXPECT_EQ(reg.shortcutOf("third").toString(), "Y");
}

TEST(UiShellRegistry, RemapResetAndUnbind) {
    CommandRegistry reg = defaultRegistry();
    ASSERT_TRUE(reg.setShortcut("extrude", QKeySequence("Ctrl+Shift+E")));
    EXPECT_EQ(reg.shortcutOf("extrude").toString(QKeySequence::PortableText), "Ctrl+Shift+E");
    EXPECT_EQ(reg.findByShortcut(QKeySequence("Ctrl+Shift+E"))->id, "extrude");
    EXPECT_EQ(reg.findByShortcut(QKeySequence("E")), nullptr); // the default no longer triggers anything

    reg.resetShortcut("extrude");
    EXPECT_EQ(reg.shortcutOf("extrude").toString(QKeySequence::PortableText), "E");

    // Unbinding frees the key for another command.
    ASSERT_TRUE(reg.setShortcut("fillet", QKeySequence()));
    EXPECT_TRUE(reg.shortcutOf("fillet").isEmpty());
    ASSERT_TRUE(reg.setShortcut("hole", QKeySequence("F")));
    EXPECT_TRUE(reg.shortcutConflicts().empty());

    EXPECT_EQ(reg.setShortcut("missing", QKeySequence("G")).error().code, ErrorCode::NotFound);
}

TEST(UiShellRegistry, ShortcutOverridesRoundTripThroughSettings) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString path = dir.filePath("shortcuts.ini");
    {
        CommandRegistry reg = defaultRegistry();
        ASSERT_TRUE(reg.setShortcut("extrude", QKeySequence("Ctrl+E")));
        ASSERT_TRUE(reg.setShortcut("fillet", QKeySequence()));
        QSettings s(path, QSettings::IniFormat);
        reg.saveShortcuts(s);
    }
    CommandRegistry reg = defaultRegistry();
    QSettings s(path, QSettings::IniFormat);
    reg.loadShortcuts(s);
    EXPECT_EQ(reg.shortcutOf("extrude").toString(QKeySequence::PortableText), "Ctrl+E");
    EXPECT_TRUE(reg.shortcutOf("fillet").isEmpty());
    EXPECT_EQ(reg.shortcutOf("hole").toString(QKeySequence::PortableText), "H");
}

TEST(UiShellRegistry, ImportRunsItsHookAndRepeatLastRepeatsIt) {
    CommandRegistry reg = defaultRegistry();
    DocumentState doc;
    int imports = 0;
    CommandContext ctx;
    ctx.doc = &doc;
    ctx.importFile = [&] { ++imports; };

    EXPECT_FALSE(reg.find("repeat_last")->isEnabled(ctx)); // nothing to repeat yet
    ASSERT_TRUE(reg.run("import", ctx));
    EXPECT_EQ(imports, 1);
    EXPECT_EQ(reg.lastRun(), "import");

    EXPECT_TRUE(reg.find("repeat_last")->isEnabled(ctx));
    ASSERT_TRUE(reg.run("repeat_last", ctx));
    EXPECT_EQ(imports, 2);
    EXPECT_EQ(reg.lastRun(), "import") << "repeat_last itself must not become the last command";
}

TEST(UiShellRegistry, UndoAndRedoFollowTheUndoStack) {
    CommandRegistry reg = defaultRegistry();
    DocumentState doc;
    CommandContext ctx;
    ctx.doc = &doc;
    EXPECT_FALSE(reg.find("undo")->isEnabled(ctx));
    EXPECT_FALSE(reg.find("redo")->isEnabled(ctx));

    ASSERT_TRUE(editParameters(doc, "Add d1", [](ParameterTable& t) { return t.add("d1", "5 mm"); }));
    EXPECT_TRUE(reg.find("undo")->isEnabled(ctx));
    ASSERT_TRUE(reg.run("undo", ctx));
    EXPECT_EQ(doc.params.find("d1"), nullptr);
    EXPECT_TRUE(reg.find("redo")->isEnabled(ctx));
    ASSERT_TRUE(reg.run("redo", ctx));
    EXPECT_NE(doc.params.find("d1"), nullptr);
}

// ---- S-box fuzzy ranking -----------------------------------------------------------------------------------

TEST(UiShellSbox, FuzzyScoreMatchesSubsequencesOnly) {
    EXPECT_EQ(fuzzyScore(QStringLiteral("xyz"), QStringLiteral("Extrude")), 0);
    EXPECT_GT(fuzzyScore(QStringLiteral("ext"), QStringLiteral("Extrude")), 0);
    EXPECT_GT(fuzzyScore(QStringLiteral("ex"), QStringLiteral("Index Exchange")), 0);
    // Word starts and consecutive characters beat scattered matches.
    EXPECT_GT(fuzzyScore(QStringLiteral("pp"), QStringLiteral("Press Pull")),
              fuzzyScore(QStringLiteral("pp"), QStringLiteral("Hopper")));
    EXPECT_GT(fuzzyScore(QStringLiteral("hol"), QStringLiteral("Hole")),
              fuzzyScore(QStringLiteral("hol"), QStringLiteral("Chamfer Holder Long Name")));
}

TEST(UiShellSbox, RankingPutsTheBestMatchFirst) {
    CommandRegistry reg = defaultRegistry();
    auto first = [&](const char* q) {
        auto ranked = rankCommands(reg, QString::fromLatin1(q), {});
        return ranked.empty() ? std::string("<none>") : ranked.front()->id;
    };
    EXPECT_EQ(first("fil"), "fillet");
    EXPECT_EQ(first("sk"), "sketch");
    EXPECT_EQ(first("ex"), "extrude");
    EXPECT_EQ(first("hol"), "hole");
    EXPECT_EQ(first("pp"), "press_pull");
    EXPECT_EQ(first("undo"), "undo");
    EXPECT_EQ(first("zzzq"), "<none>");
}

TEST(UiShellSbox, EmptyQueryListsRecentFirstAndNeverTheSearchItself) {
    CommandRegistry reg = defaultRegistry();
    auto ranked = rankCommands(reg, QString(), {"move", "undo"});
    ASSERT_GE(ranked.size(), 3u);
    EXPECT_EQ(ranked[0]->id, "move");
    EXPECT_EQ(ranked[1]->id, "undo");
    for (const iw::ui::Command* c : ranked) EXPECT_NE(c->id, "sbox");
}

TEST(UiShellSbox, PaletteRunsImplementedCommandsAndPinsPlaceholders) {
    CommandRegistry reg = defaultRegistry();
    CommandPalette palette(reg);
    std::vector<std::string> chosen;
    std::vector<std::string> pinned;
    QObject::connect(&palette, &CommandPalette::commandChosen, [&](const QString& id) {
        chosen.push_back(id.toStdString());
    });
    QObject::connect(&palette, &CommandPalette::pinRequested, [&](const QString& id) {
        pinned.push_back(id.toStdString());
    });

    palette.setQuery("hol");
    ASSERT_FALSE(palette.results().empty());
    EXPECT_EQ(palette.results().front(), "hole");
    EXPECT_FALSE(palette.chooseCurrent()) << "placeholder must not run";
    EXPECT_TRUE(chosen.empty());
    EXPECT_TRUE(palette.pinCurrent());
    EXPECT_EQ(pinned, (std::vector<std::string>{"hole"}));

    palette.setQuery("import");
    EXPECT_TRUE(palette.chooseCurrent());
    EXPECT_EQ(chosen, (std::vector<std::string>{"import"}));
    ASSERT_FALSE(palette.recent().empty());
    EXPECT_EQ(palette.recent().front(), "import");
}

// ---- command dialog: expressions, preview -------------------------------------------------------------------

TEST(UiShellDialog, ExpressionFieldEvaluatesParametersAndUnits) {
    ParameterTable params;
    ASSERT_TRUE(params.add("d1", "5 mm"));

    auto v = evaluateValueText("10 mm + d1", Dimension::Length, &params);
    ASSERT_TRUE(v) << v.error().message;
    EXPECT_NEAR(v->value, 15.0, 1e-12);
    EXPECT_EQ(v->dim, Dimension::Length);

    auto inches = evaluateValueText("2 in + 5 mm", Dimension::Length, &params);
    ASSERT_TRUE(inches);
    EXPECT_NEAR(inches->value, 55.8, 1e-9);

    // Bare numbers take the field's unit: mm for lengths, degrees for angles.
    EXPECT_NEAR(evaluateValueText("12", Dimension::Length, &params)->value, 12.0, 1e-12);
    EXPECT_NEAR(evaluateValueText("30", Dimension::Angle, &params)->value, std::numbers::pi / 6, 1e-12);
}

TEST(UiShellDialog, ExpressionValidationErrors) {
    ParameterTable params;
    ASSERT_TRUE(params.add("d1", "5 mm"));
    ASSERT_TRUE(params.add("a1", "5 deg"));

    auto mismatch = evaluateValueText("10 mm + a1", Dimension::Length, &params);
    ASSERT_FALSE(mismatch);
    EXPECT_EQ(mismatch.error().code, ErrorCode::InvalidArgument);

    auto unknown = evaluateValueText("10 mm + nope", Dimension::Length, &params);
    ASSERT_FALSE(unknown);
    EXPECT_EQ(unknown.error().code, ErrorCode::NotFound);

    auto broken = evaluateValueText("10 mm +", Dimension::Length, &params);
    ASSERT_FALSE(broken);
    EXPECT_EQ(broken.error().code, ErrorCode::ParseError);

    EXPECT_FALSE(evaluateValueText("   ", Dimension::Length, &params));
}

TEST(UiShellDialog, ModelTracksPerFieldErrorsAndSnapshots) {
    ParameterTable params;
    ASSERT_TRUE(params.add("d1", "5 mm"));
    std::vector<InputSpec> specs;
    specs.push_back(InputSpec{.name = "distance", .label = QStringLiteral("Distance"), .defaultText = "10 mm + d1", .options = {}});
    specs.push_back(InputSpec{.name = "flip", .label = QStringLiteral("Flip"), .kind = InputKind::Checkbox,
                              .defaultText = "0", .options = {}});
    DialogModel model(specs, &params);

    EXPECT_TRUE(model.canAccept());
    auto snap = model.snapshot();
    ASSERT_TRUE(snap);
    EXPECT_NEAR(snap->values.at("distance").value, 15.0, 1e-12);
    EXPECT_EQ(snap->choices.at("flip"), "0");

    model.setText("distance", "10 mm +");
    EXPECT_FALSE(model.canAccept());
    ASSERT_TRUE(model.errorFor("distance").has_value());
    EXPECT_FALSE(model.snapshot());

    model.setText("distance", "10 mm + d1");
    EXPECT_TRUE(model.canAccept());
}

TEST(UiShellDialog, PanelPreviewFollowsTheValidExpressionAndOkIsGated) {
    ParameterTable params;
    ASSERT_TRUE(params.add("d1", "5 mm"));
    JobSystem jobs(2);

    CommandDialogSpec spec;
    spec.title = QStringLiteral("Test");
    spec.inputs.push_back(InputSpec{.name = "distance", .label = QStringLiteral("Distance"),
                                    .defaultText = "10 mm", .options = {}});
    spec.preview = [](const DialogModel::Snapshot& s, CancelToken) -> Result<std::string> {
        return formatQuantity(s.values.at("distance"));
    };
    CommandPanel panel(spec, &params, jobs);
    panel.setText("distance", QStringLiteral("10 mm + d1"));
    EXPECT_TRUE(panel.okEnabled());
    jobs.waitAll();
    panel.pollPreview();
    EXPECT_EQ(panel.previewText().toStdString(), formatQuantity(Quantity::mm(15.0)));

    panel.setText("distance", QStringLiteral("10 mm + "));
    EXPECT_FALSE(panel.okEnabled());
    panel.accept(); // refused while invalid: no finished signal
}

TEST(UiShellDialog, PanelAcceptReportsFailuresAndFinishesOnSuccess) {
    ParameterTable params;
    JobSystem jobs(1);
    CommandDialogSpec spec;
    spec.title = QStringLiteral("Test");
    spec.inputs.push_back(InputSpec{.name = "distance", .label = QStringLiteral("Distance"), .defaultText = "3 mm", .options = {}});
    int calls = 0;
    bool refuse = true;
    spec.accept = [&](const DialogModel::Snapshot&) -> Result<void> {
        ++calls;
        if (refuse) return makeError(ErrorCode::KernelFailure, "fillet radius too large");
        return {};
    };
    CommandPanel panel(spec, &params, jobs);
    std::vector<bool> finished;
    QObject::connect(&panel, &CommandPanel::finished, [&](bool ok) { finished.push_back(ok); });

    panel.accept();
    EXPECT_EQ(calls, 1);
    EXPECT_TRUE(finished.empty());
    EXPECT_EQ(panel.errorText().toStdString(), "fillet radius too large");

    refuse = false;
    panel.accept();
    EXPECT_EQ(calls, 2);
    EXPECT_EQ(finished, (std::vector<bool>{true}));
}

TEST(UiShellDialog, LatestWinsPreviewDeliversOnlyTheNewest) {
    JobSystem jobs(2);
    std::vector<PreviewResult> delivered;
    LivePreview preview(
        jobs,
        [](const DialogModel::Snapshot& s, CancelToken token) -> Result<std::string> {
            if (token.cancelled()) return makeError(ErrorCode::Cancelled, "superseded");
            return s.choices.at("n");
        },
        [&](const PreviewResult& r) { delivered.push_back(r); });

    for (int i = 0; i < 30; ++i) {
        DialogModel::Snapshot s;
        s.choices["n"] = std::to_string(i);
        preview.request(std::move(s));
    }
    jobs.waitAll();
    EXPECT_EQ(preview.requests(), 30u);
    EXPECT_TRUE(preview.poll());
    EXPECT_FALSE(preview.poll()) << "a delivered preview is not delivered twice";
    ASSERT_EQ(delivered.size(), 1u);
    EXPECT_EQ(delivered[0].seq, 30u);
    EXPECT_EQ(delivered[0].text, "29");
    EXPECT_FALSE(delivered[0].failed);
}

TEST(UiShellDialog, LatestWinsPreviewReportsFailuresAsFailedResults) {
    JobSystem jobs(1);
    std::vector<PreviewResult> delivered;
    LivePreview preview(
        jobs, [](const DialogModel::Snapshot&, CancelToken) -> Result<std::string> {
            return makeError(ErrorCode::KernelFailure, "boolean failed");
        },
        [&](const PreviewResult& r) { delivered.push_back(r); });
    preview.request(DialogModel::Snapshot{});
    jobs.waitAll();
    EXPECT_TRUE(preview.poll());
    ASSERT_EQ(delivered.size(), 1u);
    EXPECT_TRUE(delivered[0].failed);
    EXPECT_EQ(delivered[0].text, "boolean failed");
}

// ---- manipulators ----------------------------------------------------------------------------------------------

TEST(UiShellManipulators, DragMathIsRelativeToTheStartOfTheDrag) {
    EXPECT_NEAR(distanceFromDrag(10.0, {0, 0}, {50, 0}, {1, 0}, 5.0), 20.0, 1e-12);
    EXPECT_NEAR(distanceFromDrag(10.0, {0, 0}, {0, 50}, {1, 0}, 5.0), 10.0, 1e-12) << "perpendicular drag";
    // Screen y grows down: dragging up from the centre is a positive 90 degrees.
    EXPECT_NEAR(screenAngleDeg({0, 0}, {0, -100}), 90.0, 1e-9);
    EXPECT_NEAR(angleFromDrag(30.0, {0, 0}, {100, 0}, {0, -100}), 120.0, 1e-9);
    // Crossing the 180-degree seam moves by the short way round (+0.57 deg), not by -359 deg.
    EXPECT_NEAR(angleFromDrag(170.0, {0, 0}, {-100, 0}, {-100, 1}), 170.57, 0.05);
}

TEST(UiShellManipulators, DistanceArrowDragReportsTheNewValueAndFinishes) {
    ManipulatorOverlay overlay;
    overlay.resize(400, 300);
    overlay.show();
    EXPECT_TRUE(overlay.testAttribute(Qt::WA_TransparentForMouseEvents)) << "idle overlay must not steal clicks";

    overlay.showDistance({100, 100}, {1, 0}, 10.0, 10.0, QStringLiteral("10 mm"));
    EXPECT_FALSE(overlay.testAttribute(Qt::WA_TransparentForMouseEvents));
    EXPECT_EQ(overlay.handlePos(), QPointF(200, 100));

    std::vector<double> values;
    int finished = 0;
    QObject::connect(&overlay, &ManipulatorOverlay::distanceChanged, [&](double mm) { values.push_back(mm); });
    QObject::connect(&overlay, &ManipulatorOverlay::dragFinished, [&] { ++finished; });

    sendMouse(&overlay, QEvent::MouseButtonPress, {200, 100}, Qt::LeftButton);
    EXPECT_TRUE(overlay.dragging());
    sendMouse(&overlay, QEvent::MouseMove, {250, 100}, Qt::LeftButton);
    ASSERT_EQ(values.size(), 1u);
    EXPECT_NEAR(values.back(), 15.0, 1e-9);
    EXPECT_NEAR(overlay.value(), 15.0, 1e-9);
    sendMouse(&overlay, QEvent::MouseButtonRelease, {250, 100}, Qt::LeftButton);
    EXPECT_FALSE(overlay.dragging());
    EXPECT_EQ(finished, 1);

    // A press away from the handle is not taken by the overlay (navigation keeps working).
    sendMouse(&overlay, QEvent::MouseButtonPress, {20, 20}, Qt::LeftButton);
    EXPECT_FALSE(overlay.dragging());

    overlay.clear();
    EXPECT_EQ(overlay.mode(), ManipulatorOverlay::Mode::None);
    EXPECT_TRUE(overlay.testAttribute(Qt::WA_TransparentForMouseEvents));
}

TEST(UiShellManipulators, AngleRingDragChangesTheAngle) {
    ManipulatorOverlay overlay;
    overlay.resize(400, 300);
    overlay.show();
    overlay.showAngle({100, 100}, 0.0, 30.0, 50.0);
    const QPointF h = overlay.handlePos();
    EXPECT_NEAR(h.x(), 100 + 50 * std::cos(std::numbers::pi / 6), 1e-9);
    EXPECT_NEAR(h.y(), 100 - 50 * std::sin(std::numbers::pi / 6), 1e-9);

    double last = 0;
    QObject::connect(&overlay, &ManipulatorOverlay::angleChanged, [&](double deg) { last = deg; });
    sendMouse(&overlay, QEvent::MouseButtonPress, h, Qt::LeftButton);
    const QPointF to60(100 + 50 * std::cos(std::numbers::pi / 3), 100 - 50 * std::sin(std::numbers::pi / 3));
    sendMouse(&overlay, QEvent::MouseMove, to60, Qt::LeftButton);
    EXPECT_NEAR(last, 60.0, 1e-6);
    sendMouse(&overlay, QEvent::MouseButtonRelease, to60, Qt::LeftButton);
}

// ---- marking menu ----------------------------------------------------------------------------------------------

TEST(UiShellMarkingMenu, SlotsMapToCompassDirections) {
    const auto& slotIds = markingSlotCommands();
    EXPECT_EQ(slotIds[0], "repeat_last");
    EXPECT_EQ(slotIds[1], "delete");
    EXPECT_EQ(slotIds[2], "press_pull");
    EXPECT_EQ(slotIds[3], "undo");
    EXPECT_EQ(slotIds[4], "redo");
    EXPECT_EQ(slotIds[5], "move");
    EXPECT_EQ(slotIds[6], "hole");
    EXPECT_EQ(slotIds[7], "sketch");

    const QPointF c(0, 0);
    EXPECT_EQ(markingSlotAt(c, {0, -100}, 36), 0);  // north (screen up)
    EXPECT_EQ(markingSlotAt(c, {70, -70}, 36), 1);  // north-east
    EXPECT_EQ(markingSlotAt(c, {100, 0}, 36), 2);   // east
    EXPECT_EQ(markingSlotAt(c, {0, 100}, 36), 4);   // south
    EXPECT_EQ(markingSlotAt(c, {-100, 0}, 36), 6);  // west
    EXPECT_EQ(markingSlotAt(c, {-70, -70}, 36), 7); // north-west
    EXPECT_EQ(markingSlotAt(c, {5, 5}, 36), -1);    // centre dead zone

    EXPECT_NEAR(markingSlotDirection(0).y(), -1.0, 1e-12);
    EXPECT_NEAR(markingSlotDirection(2).x(), 1.0, 1e-12);
}

TEST(UiShellMarkingMenu, SlotsRunEnabledCommandsOnly) {
    CommandRegistry reg = defaultRegistry();
    DocumentState doc;
    int imports = 0;
    CommandContext ctx;
    ctx.doc = &doc;
    ctx.importFile = [&] { ++imports; };
    MarkingMenu menu(reg, ctx);

    std::vector<std::string> chosen;
    QObject::connect(&menu, &MarkingMenu::commandChosen, [&](const QString& id) { chosen.push_back(id.toStdString()); });

    EXPECT_EQ(menu.slotLabel(3).toStdString(), "Undo");
    EXPECT_FALSE(menu.slotEnabled(3)) << "nothing to undo";
    EXPECT_FALSE(menu.chooseSlot(3));
    EXPECT_FALSE(menu.slotEnabled(6)) << "hole is a placeholder";
    EXPECT_FALSE(menu.chooseSlot(0)) << "no last command to repeat";

    ASSERT_TRUE(editParameters(doc, "Add d1", [](ParameterTable& t) { return t.add("d1", "5 mm"); }));
    EXPECT_TRUE(menu.slotEnabled(3));
    EXPECT_TRUE(menu.chooseSlot(3));
    EXPECT_EQ(chosen, (std::vector<std::string>{"undo"}));

    // Repeat last becomes available after a command ran.
    ASSERT_TRUE(reg.run("import", ctx));
    EXPECT_TRUE(menu.slotEnabled(0));
    EXPECT_TRUE(menu.chooseSlot(0));
    EXPECT_EQ(chosen.back(), "repeat_last");
    EXPECT_EQ(imports, 1);
}

TEST(UiShellMarkingMenu, OverflowListHoldsEveryOtherCommand) {
    CommandRegistry reg = defaultRegistry();
    DocumentState doc;
    CommandContext ctx;
    ctx.doc = &doc;
    MarkingMenu menu(reg, ctx);
    QMenu* menuList = menu.overflowMenu();
    int entries = 0;
    for (QAction* a : menuList->actions())
        if (!a->isSeparator()) ++entries;
    // Everything except the eight slot commands and the S-box itself.
    EXPECT_EQ(entries, static_cast<int>(reg.all().size()) - kMarkingSlotCount - 1);
}

// ---- parameters dialog --------------------------------------------------------------------------------------------

TEST(UiShellParameters, AddEditAndCycleErrorsAreShownInlineAndLeaveTheTableIntact) {
    DocumentState doc;
    ParametersDialog dialog(doc);

    ASSERT_TRUE(dialog.addParameter("a", "5 mm"));
    ASSERT_TRUE(dialog.addParameter("b", "a*2"));
    EXPECT_EQ(dialog.table()->rowCount(), 2);
    EXPECT_EQ(dialog.table()->item(1, ParametersDialog::Value)->text().toStdString(), "10 mm");
    EXPECT_TRUE(dialog.errorText().isEmpty());

    // Cycle: a = b + 1 while b = a * 2.
    auto cycle = dialog.editCell(0, ParametersDialog::Expression, QStringLiteral("b+1"));
    ASSERT_FALSE(cycle);
    EXPECT_EQ(cycle.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(dialog.errorText().toStdString().find("cyclic"), std::string::npos) << dialog.errorText().toStdString();
    EXPECT_EQ(doc.params.find("a")->expression.source(), "5 mm");

    // Parse error keeps the old expression.
    EXPECT_FALSE(dialog.editCell(0, ParametersDialog::Expression, QStringLiteral("5 mm *")));
    EXPECT_FALSE(dialog.errorText().isEmpty());
    EXPECT_EQ(doc.params.find("a")->expression.source(), "5 mm");

    // A valid edit clears the error and updates dependents.
    ASSERT_TRUE(dialog.editCell(0, ParametersDialog::Expression, QStringLiteral("7 mm")));
    EXPECT_TRUE(dialog.errorText().isEmpty());
    EXPECT_EQ(dialog.table()->item(1, ParametersDialog::Value)->text().toStdString(), "14 mm");

    // Editing a cell through the table (what the user does) goes through the same path.
    dialog.table()->item(1, ParametersDialog::Expression)->setText(QStringLiteral("a*3"));
    EXPECT_NEAR(doc.params.evaluate("b")->value, 21.0, 1e-12);

    // Duplicate name is refused.
    EXPECT_FALSE(dialog.addParameter("a", "1 mm"));
    EXPECT_FALSE(dialog.errorText().isEmpty());
}

TEST(UiShellParameters, RenameDeleteAndUndoAreRecorded) {
    DocumentState doc;
    ParametersDialog dialog(doc);
    ASSERT_TRUE(dialog.addParameter("a", "5 mm"));
    ASSERT_TRUE(dialog.addParameter("b", "a*2"));

    ASSERT_TRUE(dialog.editCell(0, ParametersDialog::Name, QStringLiteral("x")));
    EXPECT_EQ(doc.params.find("b")->expression.source(), "x*2") << "rename rewrites references";

    // Deleting a referenced parameter is refused while the reference exists.
    EXPECT_FALSE(dialog.deleteParameter("x"));
    EXPECT_NE(doc.params.find("x"), nullptr);

    // Undo steps back through the edits.
    ASSERT_TRUE(doc.undo.undo());
    EXPECT_NE(doc.params.find("a"), nullptr);
    EXPECT_EQ(doc.params.find("b")->expression.source(), "a*2");

    ASSERT_TRUE(dialog.deleteParameter("b"));
    EXPECT_EQ(doc.params.find("b"), nullptr);
    EXPECT_EQ(dialog.table()->rowCount(), 1);
    ASSERT_TRUE(doc.undo.undo());
    EXPECT_NE(doc.params.find("b"), nullptr);
}

TEST(UiShellParameters, RowsStayInTableOrderAndOutOfRangeEditsAreRejected) {
    DocumentState doc;
    ParametersDialog dialog(doc);
    ASSERT_TRUE(dialog.addParameter("p", "1 mm"));
    EXPECT_FALSE(dialog.editCell(5, ParametersDialog::Expression, QStringLiteral("2 mm")));
    EXPECT_EQ(dialog.table()->item(0, ParametersDialog::Name)->text().toStdString(), "p");
}

// ---- shell -----------------------------------------------------------------------------------------------------

TEST(UiShellWindow, RibbonShowsTabsGroupsAndDisabledPlaceholders) {
    MainWindow win;
    RibbonBar* ribbon = win.ribbon();
    ASSERT_NE(ribbon, nullptr);
    EXPECT_EQ(ribbon->tabBar()->count(), 7);
    EXPECT_EQ(ribbon->tabBar()->tabText(0).toStdString(), "SOLID");
    EXPECT_EQ(ribbon->tabBar()->tabText(3).toStdString(), "SHEET METAL");
    EXPECT_EQ(ribbon->groupTitles("SOLID").size(), 7u);
    EXPECT_EQ(ribbon->workspaceBox()->count(), 5);
    EXPECT_TRUE(ribbon->workspaceBox()->model()->index(0, 0).flags().testFlag(Qt::ItemIsEnabled));
    EXPECT_FALSE(ribbon->workspaceBox()->model()->index(1, 0).flags().testFlag(Qt::ItemIsEnabled));

    ASSERT_NE(win.actions()->action("extrude"), nullptr);
    EXPECT_FALSE(win.actions()->action("extrude")->isEnabled());
    EXPECT_NE(win.actions()->action("extrude")->toolTip().toStdString().find("coming soon"), std::string::npos);
    EXPECT_TRUE(win.actions()->action("import")->isEnabled());
    EXPECT_FALSE(win.actions()->action("undo")->isEnabled());
    EXPECT_EQ(ribbon->quickAccessIds(), (std::vector<std::string>{"undo", "redo"}));
}

TEST(UiShellWindow, UndoActionFollowsTheDocumentAndEditMenuRuns) {
    MainWindow win;
    ASSERT_TRUE(editParameters(win.document(), "Add d1", [](ParameterTable& t) { return t.add("d1", "5 mm"); }));
    win.actions()->refresh();
    EXPECT_TRUE(win.actions()->action("undo")->isEnabled());
    win.actions()->action("undo")->trigger();
    EXPECT_EQ(win.document().params.find("d1"), nullptr);
    win.actions()->refresh();
    EXPECT_TRUE(win.actions()->action("redo")->isEnabled());
}

TEST(UiShellWindow, SboxIsBoundToSAndOpensThePalette) {
    MainWindow win;
    win.show();
    EXPECT_EQ(win.actions()->action("sbox")->shortcut().toString(QKeySequence::PortableText), "S");
    win.openCommandSearch();
    EXPECT_TRUE(win.commandSearch()->isVisible());
    win.commandSearch()->setQuery("measure");
    EXPECT_TRUE(win.commandSearch()->pinCurrent());
    const auto ids = win.ribbon()->quickAccessIds();
    EXPECT_NE(std::find(ids.begin(), ids.end(), "measure"), ids.end()) << "pinned command joins the quick access row";
}

TEST(UiShellWindow, RightClickOpensTheMarkingMenuButARightDragDoesNot) {
    MainWindow win;
    win.resize(1000, 700);
    win.show();
    QApplication::processEvents();
    QWidget* vp = win.viewport();

    sendMouse(vp, QEvent::MouseButtonPress, {300, 300}, Qt::RightButton);
    sendMouse(vp, QEvent::MouseButtonRelease, {300, 300}, Qt::RightButton);
    EXPECT_TRUE(win.markingMenu()->isVisible());
    win.markingMenu()->hide();

    sendMouse(vp, QEvent::MouseButtonPress, {300, 300}, Qt::RightButton);
    sendMouse(vp, QEvent::MouseMove, {340, 300}, Qt::RightButton); // orbit drag
    sendMouse(vp, QEvent::MouseButtonRelease, {340, 300}, Qt::RightButton);
    EXPECT_FALSE(win.markingMenu()->isVisible());
}

TEST(UiShellWindow, ShellScreenshot) {
    MainWindow win;
    win.resize(1280, 800);
    win.show();
    for (int i = 0; i < 5; ++i) QApplication::processEvents(QEventLoop::AllEvents, 20);

    if (const char* dir = std::getenv("IW_TEST_DUMP_DIR")) {
        const QString base = QString::fromLocal8Bit(dir);
        EXPECT_TRUE(win.grab().save(base + "/ui_shell.png"));
        win.openCommandSearch();
        for (int i = 0; i < 3; ++i) QApplication::processEvents(QEventLoop::AllEvents, 20);
        EXPECT_TRUE(win.grab().save(base + "/ui_shell_sbox.png"));
    }
    SUCCEED();
}
