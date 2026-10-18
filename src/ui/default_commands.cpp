// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/default_commands.hpp"

#include <QCoreApplication>
#include <QKeySequence>

#include "features/document.hpp"
#include "ui/main_window.hpp"
#include "ui/parameters_dialog.hpp"

namespace iw::ui {
namespace {

QString T(const char* text) { return QCoreApplication::translate("Commands", text); }

constexpr const char* kSolid = "SOLID";

MainWindow* winOf(const CommandContext& c) { return qobject_cast<MainWindow*>(c.parent); }

/// Starts a modelling command dialog on the shell (refusals are reported in the status bar).
void startFeature(CommandContext& ctx, const char* kind) {
    if (MainWindow* win = winOf(ctx)) (void)win->startFeature(kind);
}

/// Enabled when a document is open.
bool docOpen(const CommandContext& c) { return c.doc != nullptr; }

/// Short ribbon labels (Command::shortLabel) of the commands whose full label is long.
const char* shortLabelOf(const std::string& id) {
    if (id == "parameters") return "Parameters";
    if (id == "axis") return "Axis";
    return "";
}

} // namespace

Result<void> registerDefaultCommands(CommandRegistry& registry) {
    // Captured by Repeat Last; the registry outlives the commands that refer to it.
    CommandRegistry* reg = &registry;
    Result<void> status;

    // A command without `start` is a placeholder: listed, disabled, "coming soon".
    auto put = [&](const char* id, const char* label, const char* tab, const char* group, const char* shortcut,
                   const char* icon, std::function<void(CommandContext&)> start = {},
                   std::function<bool(const CommandContext&)> enabled = {}, bool inToolbar = true,
                   bool repeatable = true) {
        Command c;
        c.id = id;
        c.label = T(label);
        c.icon = icon;
        c.tab = tab;
        c.group = group;
        if (*shortcut) c.shortcut = QKeySequence(QString::fromLatin1(shortcut));
        c.inToolbar = inToolbar;
        c.repeatable = repeatable;
        c.shortLabel = T(shortLabelOf(c.id));
        c.start = std::move(start);
        c.enabled = std::move(enabled);
        if (auto r = registry.add(std::move(c)); !r && status) status = std::unexpected(r.error());
    };
    auto hasDoc = [](const CommandContext& c) { return c.doc != nullptr; };
    // One undo stack: the document's. Feature edits, rollback, sketches and parameter edits are all on it.
    auto canUndo = [](const CommandContext& c) { return c.doc && c.doc->canUndo(); };
    auto canRedo = [](const CommandContext& c) { return c.doc && c.doc->canRedo(); };

    // CREATE
    put("sketch", "Create Sketch", kSolid, "CREATE", "", "sketch",
        [](CommandContext& ctx) {
            if (auto* win = qobject_cast<MainWindow*>(ctx.parent)) win->chooseSketchPlane();
        }, hasDoc);
    put("extrude", "Extrude", kSolid, "CREATE", "E", "extrude",
        [](CommandContext& ctx) { startFeature(ctx, "extrude"); }, docOpen);
    put("revolve", "Revolve", kSolid, "CREATE", "", "revolve",
        [](CommandContext& ctx) { startFeature(ctx, "revolve"); }, docOpen);
    put("hole", "Hole", kSolid, "CREATE", "H", "hole",
        [](CommandContext& ctx) { startFeature(ctx, "hole"); }, docOpen);
    // MODIFY
    put("press_pull", "Press Pull", kSolid, "MODIFY", "Q", "press_pull");
    put("fillet", "Fillet", kSolid, "MODIFY", "F", "fillet",
        [](CommandContext& ctx) { startFeature(ctx, "fillet"); }, docOpen);
    put("chamfer", "Chamfer", kSolid, "MODIFY", "", "chamfer",
        [](CommandContext& ctx) { startFeature(ctx, "chamfer"); }, docOpen);
    put("move", "Move/Copy", kSolid, "MODIFY", "M", "move",
        [](CommandContext& ctx) { startFeature(ctx, "move"); }, docOpen);
    put("delete", "Delete", kSolid, "MODIFY", "Del", "delete");
    put("parameters", "Change Parameters...", kSolid, "MODIFY", "", "parameters",
        [](CommandContext& ctx) {
            if (ctx.doc) (new ParametersDialog(*ctx.doc, ctx.parent))->show();
        }, hasDoc);
    // ASSEMBLE
    put("new_component", "New Component", kSolid, "ASSEMBLE", "", "new_component");
    put("joint", "Joint", kSolid, "ASSEMBLE", "", "joint");
    // CONSTRUCT
    put("plane", "Offset Plane", kSolid, "CONSTRUCT", "", "offset_plane",
        [](CommandContext& ctx) { startFeature(ctx, "plane"); }, docOpen);
    put("axis", "Axis Through...", kSolid, "CONSTRUCT", "", "axis",
        [](CommandContext& ctx) { startFeature(ctx, "axis"); }, docOpen);
    put("point", "Point", kSolid, "CONSTRUCT", "", "construction_point",
        [](CommandContext& ctx) { startFeature(ctx, "point"); }, docOpen);
    // INSPECT: Measure, Properties and Section Analysis are registered by inspect::registerCommands (MainWindow).
    // INSERT
    put("import", "Import...", kSolid, "INSERT", "", "import",
        [](CommandContext& ctx) {
            if (ctx.importFile) ctx.importFile();
        });
    // SELECT
    put("select", "Select", kSolid, "SELECT", "", "select");
    // Quick access (Fusion puts undo/redo at the top left); not repeatable.
    put("undo", "Undo", "", "", "Ctrl+Z", "undo",
        [](CommandContext& ctx) {
            if (ctx.doc) (void)ctx.doc->undo();
        }, canUndo, true, false);
    put("redo", "Redo", "", "", "Ctrl+Y", "redo",
        [](CommandContext& ctx) {
            if (ctx.doc) (void)ctx.doc->redo();
        }, canRedo, true, false);
    // Hidden from the toolbar: S-box and Repeat Last.
    put("sbox", "Search Commands", "", "", "S", "search",
        [](CommandContext& ctx) {
            if (ctx.openCommandSearch) ctx.openCommandSearch();
        },
        {}, false, false);
    put("repeat_last", "Repeat Last", "", "", "", "repeat",
        [reg](CommandContext& ctx) {
            const std::string last = reg->lastRun();
            if (!last.empty()) (void)reg->run(last, ctx);
        },
        [reg](const CommandContext&) { return !reg->lastRun().empty(); }, false, false);

    return status;
}

} // namespace iw::ui
