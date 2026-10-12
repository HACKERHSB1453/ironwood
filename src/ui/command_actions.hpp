// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QObject>
#include <QString>

#include <map>
#include <string>
#include <string_view>

#include "ui/command.hpp"

class QAction;

namespace iw::ui {

/// One QAction per registry command. Toolbar buttons, menus and shortcuts all use these actions, so enabled state,
/// text and shortcuts come from the registry in one place. Call refresh() after anything that can change enablement
/// (a command ran, undo history changed, a parameter edit). The owning widget must add the actions to itself for the
/// shortcuts to work.
class CommandActions : public QObject {
    Q_OBJECT
public:
    CommandActions(CommandRegistry& registry, CommandContext& ctx, QObject* parent = nullptr);

    /// nullptr for an unknown id.
    QAction* action(std::string_view id) const;
    /// Updates enabled state, text, tooltip ("coming soon" for unimplemented commands) and shortcut.
    void refresh();
    /// Runs the command through the registry; failures are reported through failed().
    void run(std::string_view id);

    CommandRegistry& registry() noexcept { return registry_; }
    CommandContext& context() noexcept { return ctx_; }

signals:
    void executed(const QString& id);
    void failed(const QString& message);

private:
    CommandRegistry& registry_;
    CommandContext& ctx_;
    std::map<std::string, QAction*, std::less<>> actions_;
};

} // namespace iw::ui
