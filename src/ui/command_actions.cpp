// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/command_actions.hpp"

#include <QAction>
#include <QIcon>

namespace iw::ui {

CommandActions::CommandActions(CommandRegistry& registry, CommandContext& ctx, QObject* parent)
    : QObject(parent), registry_(registry), ctx_(ctx) {
    for (const Command& c : registry_.all()) {
        auto* a = new QAction(c.label, this);
        a->setObjectName(QString::fromStdString("cmd_" + c.id));
        if (!c.icon.empty()) a->setIcon(QIcon::fromTheme(QString::fromStdString(c.icon)));
        actions_[c.id] = a;
        const std::string id = c.id;
        connect(a, &QAction::triggered, this, [this, id] { run(id); });
    }
    refresh();
}

QAction* CommandActions::action(std::string_view id) const {
    auto it = actions_.find(id);
    return it == actions_.end() ? nullptr : it->second;
}

void CommandActions::refresh() {
    for (const Command& c : registry_.all()) {
        QAction* a = action(c.id);
        if (!a) continue;
        a->setEnabled(c.isEnabled(ctx_));
        const QKeySequence seq = registry_.shortcutOf(c.id);
        if (a->shortcut() != seq) a->setShortcut(seq);
        QString tip = c.label;
        if (!seq.isEmpty()) tip += QStringLiteral(" (%1)").arg(seq.toString(QKeySequence::NativeText));
        if (!c.implemented()) tip += QStringLiteral(" - ") + tr("coming soon");
        else if (!c.tooltip.isEmpty()) tip += QStringLiteral("\n") + c.tooltip;
        a->setToolTip(tip);
        a->setStatusTip(c.tooltip.isEmpty() ? c.label : c.tooltip);
    }
}

void CommandActions::run(std::string_view id) {
    const std::string key(id);
    if (auto r = registry_.run(key, ctx_); !r) {
        emit failed(QString::fromStdString(r.error().message));
        return;
    }
    refresh();
    emit executed(QString::fromStdString(key));
}

} // namespace iw::ui
