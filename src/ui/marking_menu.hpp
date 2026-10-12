// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QPointF>
#include <QWidget>

#include <array>
#include <memory>
#include <string_view>

#include "ui/command.hpp"

class QMenu;

namespace iw::ui {

inline constexpr int kMarkingSlotCount = 8;

/// Command id of each marking-menu slot, clockwise from the top: N, NE, E, SE, S, SW, W, NW.
/// Repeat last, Delete, Press Pull, Undo, Redo, Move, Hole, Sketch [VERIFY the Fusion order].
const std::array<std::string_view, kMarkingSlotCount>& markingSlotCommands();

/// Unit direction of a slot in screen coordinates (y down): slot 0 points up.
QPointF markingSlotDirection(int slot);

/// Slot under `p` for a menu centred on `center`, or -1 when `p` lies within `innerRadiusPx` of the centre.
/// Sectors are 45 degrees wide and centred on each slot direction.
int markingSlotAt(QPointF center, QPointF p, double innerRadiusPx);

/// Right-click radial menu. Eight slots around the centre; the centre opens an overflow menu with every other command.
/// Choosing a slot emits commandChosen with its command id; unimplemented or currently disabled commands are drawn
/// greyed out and cannot be chosen.
class MarkingMenu : public QWidget {
    Q_OBJECT
public:
    static constexpr int kSize = 260;
    static constexpr double kInnerRadiusPx = 36.0;

    MarkingMenu(const CommandRegistry& registry, CommandContext& ctx, QWidget* parent = nullptr);
    ~MarkingMenu() override;

    /// Shows the menu centred on `centre` (in parent coordinates).
    void popup(QPoint centre);
    /// Label of a slot (empty when the slot has no command).
    QString slotLabel(int slot) const;
    /// True when the slot's command is implemented and enabled right now.
    bool slotEnabled(int slot) const;
    /// Emits commandChosen for an enabled slot and hides the menu. Returns whether it did.
    bool chooseSlot(int slot);
    /// Menu of every command that is not in a slot (grouped, separators between groups).
    QMenu* overflowMenu();

signals:
    void commandChosen(const QString& id);

protected:
    void paintEvent(QPaintEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void keyPressEvent(QKeyEvent* e) override;

private:
    const Command* slotCommand(int slot) const;

    const CommandRegistry& registry_;
    CommandContext& ctx_;
    std::unique_ptr<QMenu> overflow_;
};

} // namespace iw::ui
