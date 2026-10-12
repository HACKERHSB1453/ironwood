// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QKeySequence>
#include <QString>

#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "core/result.hpp"

class QSettings;
class QWidget;

namespace iw {
class JobSystem;
}

namespace iw::ui {

struct DocumentState;

/// What a command can reach when it starts: the document and a few shell hooks. The shell fills it in.
struct CommandContext {
    QWidget* parent = nullptr;
    DocumentState* doc = nullptr;
    JobSystem* jobs = nullptr;                    ///< Background work for previews.
    std::function<void()> importFile;             ///< Insert > Import: opens the file dialog.
    std::function<void(QWidget*)> showPanel;      ///< Docks a command dialog on the right (takes ownership).
    std::function<void()> openCommandSearch;      ///< S-box.
};

/// One user operation. The registry is the single source for the toolbar, S-box, marking menu and shortcuts.
/// A command without `start` is shown but disabled with a "coming soon" tooltip (B1/B3 register handlers).
struct Command {
    std::string id;                   ///< Stable key, e.g. "extrude". Never shown to the user.
    QString label;                    ///< Translated display name.
    QString tooltip;                  ///< Optional extra text.
    std::string icon;                 ///< Freedesktop theme icon name (may be missing -> text only).
    QKeySequence shortcut;            ///< Default shortcut; the user may remap it (see CommandRegistry).
    std::string workspace = "DESIGN";
    std::string tab;                  ///< "SOLID", "SURFACE", ... ; empty = quick access toolbar / hidden.
    std::string group;                ///< "CREATE", "MODIFY", ...
    bool inToolbar = true;            ///< false = only in S-box / marking menu / shortcut.
    bool repeatable = true;           ///< false = "Repeat last" ignores it (undo, S-box, repeat itself).
    std::function<bool(const CommandContext&)> enabled; ///< Extra predicate; empty = enabled when implemented.
    std::function<void(CommandContext&)> start;         ///< Empty = not implemented.

    bool implemented() const { return static_cast<bool>(start); }
    bool isEnabled(const CommandContext& ctx) const { return implemented() && (!enabled || enabled(ctx)); }
};

struct ShortcutConflict {
    std::string first;
    std::string second;
    QString sequence; ///< Portable text of the shared shortcut.
};

/// Ordered command table plus user shortcut overrides (persisted in QSettings under "shortcuts/<id>").
/// GUI thread only. Ids are unique; shortcut duplicates are allowed in defaults but reported by
/// shortcutConflicts(), and setShortcut() refuses to create a new one.
class CommandRegistry {
public:
    /// Errors: InvalidArgument (empty or duplicate id).
    Result<void> add(Command cmd);
    const Command* find(std::string_view id) const;
    const std::vector<Command>& all() const noexcept { return cmds_; }

    /// Commands of one ribbon group, in registration order.
    std::vector<const Command*> inGroup(std::string_view workspace, std::string_view tab, std::string_view group) const;
    /// Group names of a tab, in order of first registration.
    std::vector<std::string> groups(std::string_view workspace, std::string_view tab) const;
    /// Commands of a tab (or of the quick-access area when `tab` is empty).
    std::vector<const Command*> inTab(std::string_view workspace, std::string_view tab) const;

    /// Effective shortcut: the user's override if any (empty = unbound), else the default.
    QKeySequence shortcutOf(std::string_view id) const;
    /// Sets (or with an empty sequence unbinds) the user shortcut. Errors: NotFound (id), InvalidArgument when the
    /// sequence is already bound to another command (message names it).
    Result<void> setShortcut(std::string_view id, const QKeySequence& seq);
    /// Drops the override so the default applies again.
    void resetShortcut(std::string_view id);
    const Command* findByShortcut(const QKeySequence& seq) const;
    std::vector<ShortcutConflict> shortcutConflicts() const;

    void loadShortcuts(QSettings& settings);
    void saveShortcuts(QSettings& settings) const;

    /// Runs a command. Errors: NotFound (unknown id), InvalidArgument (not available right now).
    Result<void> run(std::string_view id, CommandContext& ctx);
    /// Id of the last command run successfully ("" if none). Used by "Repeat last".
    const std::string& lastRun() const noexcept { return last_; }

private:
    std::vector<Command> cmds_;
    std::map<std::string, QString, std::less<>> overrides_; ///< id -> portable sequence text ("" = unbound)
    std::string last_;
};

} // namespace iw::ui
