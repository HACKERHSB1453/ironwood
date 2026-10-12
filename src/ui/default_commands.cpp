// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/default_commands.hpp"

#include <QCoreApplication>
#include <QKeySequence>

#include "ui/doc_state.hpp"
#include "ui/main_window.hpp"
#include "ui/parameters_dialog.hpp"

namespace iw::ui {
namespace {

QString T(const char* text) { return QCoreApplication::translate("Commands", text); }

constexpr const char* kSolid = "SOLID";

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
        c.start = std::move(start);
        c.enabled = std::move(enabled);
        if (auto r = registry.add(std::move(c)); !r && status) status = std::unexpected(r.error());
    };
    auto hasDoc = [](const CommandContext& c) { return c.doc != nullptr; };
    auto canUndo = [](const CommandContext& c) { return c.doc && c.doc->undo.canUndo(); };
    auto canRedo = [](const CommandContext& c) { return c.doc && c.doc->undo.canRedo(); };

    // CREATE
    put("sketch", "Create Sketch", kSolid, "CREATE", "", "document-edit",
        [](CommandContext& ctx) {
            if (auto* win = qobject_cast<MainWindow*>(ctx.parent)) win->chooseSketchPlane();
        }, hasDoc);
    put("extrude", "Extrude", kSolid, "CREATE", "E", "");
    put("revolve", "Revolve", kSolid, "CREATE", "", "");
    put("hole", "Hole", kSolid, "CREATE", "H", "");
    // MODIFY
    put("press_pull", "Press Pull", kSolid, "MODIFY", "Q", "");
    put("fillet", "Fillet", kSolid, "MODIFY", "F", "");
    put("chamfer", "Chamfer", kSolid, "MODIFY", "", "");
    put("move", "Move/Copy", kSolid, "MODIFY", "M", "");
    put("delete", "Delete", kSolid, "MODIFY", "Del", "edit-delete");
    put("parameters", "Change Parameters...", kSolid, "MODIFY", "", "document-properties",
        [](CommandContext& ctx) { (new ParametersDialog(*ctx.doc, ctx.parent))->show(); }, hasDoc);
    // ASSEMBLE
    put("new_component", "New Component", kSolid, "ASSEMBLE", "", "");
    put("joint", "Joint", kSolid, "ASSEMBLE", "", "");
    // CONSTRUCT
    put("plane", "Offset Plane", kSolid, "CONSTRUCT", "", "");
    put("axis", "Axis Through...", kSolid, "CONSTRUCT", "", "");
    put("point", "Point", kSolid, "CONSTRUCT", "", "");
    // INSPECT
    put("measure", "Measure", kSolid, "INSPECT", "I", "");
    // INSERT
    put("import", "Import...", kSolid, "INSERT", "", "document-import",
        [](CommandContext& ctx) {
            if (ctx.importFile) ctx.importFile();
        });
    // SELECT
    put("select", "Select", kSolid, "SELECT", "", "");
    // Quick access (Fusion puts undo/redo at the top left); not repeatable.
    put("undo", "Undo", "", "", "Ctrl+Z", "edit-undo",
        [](CommandContext& ctx) { (void)ctx.doc->undo.undo(); }, canUndo, true, false);
    put("redo", "Redo", "", "", "Ctrl+Y", "edit-redo",
        [](CommandContext& ctx) { (void)ctx.doc->undo.redo(); }, canRedo, true, false);
    // Hidden from the toolbar: S-box and Repeat Last.
    put("sbox", "Search Commands", "", "", "S", "system-search",
        [](CommandContext& ctx) {
            if (ctx.openCommandSearch) ctx.openCommandSearch();
        },
        {}, false, false);
    put("repeat_last", "Repeat Last", "", "", "", "view-refresh",
        [reg](CommandContext& ctx) {
            const std::string last = reg->lastRun();
            if (!last.empty()) (void)reg->run(last, ctx);
        },
        [reg](const CommandContext&) { return !reg->lastRun().empty(); }, false, false);

    return status;
}

} // namespace iw::ui
