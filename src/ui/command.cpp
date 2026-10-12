// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/command.hpp"

#include <QSettings>

#include <algorithm>
#include <set>

namespace iw::ui {
namespace {
constexpr const char* kShortcutGroup = "shortcuts";

QKeySequence fromPortable(const QString& text) {
    return text.isEmpty() ? QKeySequence() : QKeySequence::fromString(text, QKeySequence::PortableText);
}
QString toPortable(const QKeySequence& seq) { return seq.toString(QKeySequence::PortableText); }
} // namespace

Result<void> CommandRegistry::add(Command cmd) {
    if (cmd.id.empty()) return makeError(ErrorCode::InvalidArgument, "command id must not be empty");
    if (find(cmd.id)) return makeError(ErrorCode::InvalidArgument, "duplicate command id: " + cmd.id);
    cmds_.push_back(std::move(cmd));
    return {};
}

const Command* CommandRegistry::find(std::string_view id) const {
    for (const Command& c : cmds_)
        if (c.id == id) return &c;
    return nullptr;
}

std::vector<const Command*> CommandRegistry::inGroup(std::string_view workspace, std::string_view tab,
                                                     std::string_view group) const {
    std::vector<const Command*> out;
    for (const Command& c : cmds_)
        if (c.workspace == workspace && c.tab == tab && c.group == group && c.inToolbar) out.push_back(&c);
    return out;
}

std::vector<std::string> CommandRegistry::groups(std::string_view workspace, std::string_view tab) const {
    std::vector<std::string> out;
    for (const Command& c : cmds_) {
        if (c.workspace != workspace || c.tab != tab || c.group.empty() || !c.inToolbar) continue;
        if (std::find(out.begin(), out.end(), c.group) == out.end()) out.push_back(c.group);
    }
    return out;
}

std::vector<const Command*> CommandRegistry::inTab(std::string_view workspace, std::string_view tab) const {
    std::vector<const Command*> out;
    for (const Command& c : cmds_)
        if (c.workspace == workspace && c.tab == tab && c.inToolbar) out.push_back(&c);
    return out;
}

QKeySequence CommandRegistry::shortcutOf(std::string_view id) const {
    if (auto it = overrides_.find(id); it != overrides_.end()) return fromPortable(it->second);
    const Command* c = find(id);
    return c ? c->shortcut : QKeySequence();
}

Result<void> CommandRegistry::setShortcut(std::string_view id, const QKeySequence& seq) {
    if (!find(id)) return makeError(ErrorCode::NotFound, "unknown command: " + std::string(id));
    if (!seq.isEmpty()) {
        for (const Command& other : cmds_) {
            if (other.id == id || shortcutOf(other.id) != seq) continue;
            return makeError(ErrorCode::InvalidArgument,
                             "shortcut " + toPortable(seq).toStdString() + " is already used by " +
                                 other.label.toStdString());
        }
    }
    overrides_[std::string(id)] = toPortable(seq);
    return {};
}

void CommandRegistry::resetShortcut(std::string_view id) {
    if (auto it = overrides_.find(id); it != overrides_.end()) overrides_.erase(it);
}

const Command* CommandRegistry::findByShortcut(const QKeySequence& seq) const {
    if (seq.isEmpty()) return nullptr;
    for (const Command& c : cmds_)
        if (shortcutOf(c.id) == seq) return &c;
    return nullptr;
}

std::vector<ShortcutConflict> CommandRegistry::shortcutConflicts() const {
    std::vector<ShortcutConflict> out;
    for (std::size_t i = 0; i < cmds_.size(); ++i) {
        const QKeySequence si = shortcutOf(cmds_[i].id);
        if (si.isEmpty()) continue;
        for (std::size_t j = i + 1; j < cmds_.size(); ++j) {
            if (shortcutOf(cmds_[j].id) == si)
                out.push_back({cmds_[i].id, cmds_[j].id, toPortable(si)});
        }
    }
    return out;
}

void CommandRegistry::loadShortcuts(QSettings& settings) {
    settings.beginGroup(kShortcutGroup);
    for (const QString& key : settings.childKeys()) {
        const std::string id = key.toStdString();
        if (find(id)) overrides_[id] = settings.value(key).toString();
    }
    settings.endGroup();
}

void CommandRegistry::saveShortcuts(QSettings& settings) const {
    settings.remove(kShortcutGroup);
    settings.beginGroup(kShortcutGroup);
    for (const auto& [id, text] : overrides_) settings.setValue(QString::fromStdString(id), text);
    settings.endGroup();
}

Result<void> CommandRegistry::run(std::string_view id, CommandContext& ctx) {
    const Command* c = find(id);
    if (!c) return makeError(ErrorCode::NotFound, "unknown command: " + std::string(id));
    if (!c->isEnabled(ctx))
        return makeError(ErrorCode::InvalidArgument, c->label.toStdString() + " is not available");
    c->start(ctx);
    if (c->repeatable) last_ = c->id;
    return {};
}

} // namespace iw::ui
