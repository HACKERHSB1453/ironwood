// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/marking_menu.hpp"

#include <QAction>
#include <QKeyEvent>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>

#include <cmath>
#include <numbers>
#include <string>

#include "ui/icons.hpp"

namespace iw::ui {
namespace {

constexpr double kDeg = 180.0 / std::numbers::pi;
constexpr double kSlotRadius = 84.0;   ///< Distance of slot centres from the menu centre (px).
constexpr double kSlotBadgeRadius = 34.0;
constexpr int kSlotIconPx = 22;        ///< Command icon drawn in each slot badge.

bool inSlots(std::string_view id) {
    for (std::string_view s : markingSlotCommands())
        if (s == id) return true;
    return false;
}

} // namespace

const std::array<std::string_view, kMarkingSlotCount>& markingSlotCommands() {
    static constexpr std::array<std::string_view, kMarkingSlotCount> kSlots = {
        "repeat_last", "delete", "press_pull", "undo", "redo", "move", "hole", "sketch"};
    return kSlots;
}

QPointF markingSlotDirection(int slot) {
    const double angle = (90.0 - 45.0 * slot) / kDeg; // slot 0 = north, slots run clockwise
    return {std::cos(angle), -std::sin(angle)};
}

int markingSlotAt(QPointF center, QPointF p, double innerRadiusPx) {
    const QPointF d = p - center;
    if (std::hypot(d.x(), d.y()) < innerRadiusPx) return -1;
    const double angle = std::atan2(-d.y(), d.x()) * kDeg; // counter-clockwise from east
    double delta = 90.0 - angle;                            // clockwise from north
    while (delta < 0.0) delta += 360.0;
    while (delta >= 360.0) delta -= 360.0;
    return static_cast<int>(std::lround(delta / 45.0)) % kMarkingSlotCount;
}

MarkingMenu::MarkingMenu(const CommandRegistry& registry, CommandContext& ctx, QWidget* parent)
    : QWidget(parent), registry_(registry), ctx_(ctx) {
    setObjectName("markingMenu");
    setFixedSize(kSize, kSize);
    setAttribute(Qt::WA_TranslucentBackground);
    setFocusPolicy(Qt::StrongFocus);
    hide();
}

MarkingMenu::~MarkingMenu() = default;

void MarkingMenu::popup(QPoint centre) {
    move(centre.x() - width() / 2, centre.y() - height() / 2);
    show();
    raise();
    setFocus();
    update();
}

const Command* MarkingMenu::slotCommand(int slot) const {
    if (slot < 0 || slot >= kMarkingSlotCount) return nullptr;
    return registry_.find(markingSlotCommands()[static_cast<std::size_t>(slot)]);
}

QString MarkingMenu::slotLabel(int slot) const {
    const Command* c = slotCommand(slot);
    return c ? c->label : QString();
}

bool MarkingMenu::slotEnabled(int slot) const {
    const Command* c = slotCommand(slot);
    return c && c->isEnabled(ctx_);
}

bool MarkingMenu::chooseSlot(int slot) {
    if (!slotEnabled(slot)) return false;
    const QString id = QString::fromStdString(slotCommand(slot)->id);
    hide();
    emit commandChosen(id);
    return true;
}

QMenu* MarkingMenu::overflowMenu() {
    overflow_ = std::make_unique<QMenu>();
    overflow_->setObjectName("markingOverflow");
    std::string lastGroup;
    bool first = true;
    for (const Command& c : registry_.all()) {
        if (c.id == "sbox" || inSlots(c.id)) continue;
        if (!first && c.group != lastGroup) overflow_->addSeparator();
        lastGroup = c.group;
        first = false;
        QString text = c.label;
        if (!c.implemented()) text += QStringLiteral(" (") + tr("coming soon") + QStringLiteral(")");
        QAction* a = overflow_->addAction(text);
        if (!c.icon.empty()) a->setIcon(icon(c.icon, 16, iconThemeFor(palette())));
        a->setEnabled(c.isEnabled(ctx_));
        const std::string id = c.id;
        connect(a, &QAction::triggered, this, [this, id] { emit commandChosen(QString::fromStdString(id)); });
    }
    return overflow_.get();
}

void MarkingMenu::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    const QPointF c(width() / 2.0, height() / 2.0);

    p.setPen(QPen(QColor(90, 90, 96), 1));
    p.setBrush(QColor(30, 30, 34, 200));
    p.drawEllipse(c, kSlotRadius + kSlotBadgeRadius + 8, kSlotRadius + kSlotBadgeRadius + 8);

    for (int slot = 0; slot < kMarkingSlotCount; ++slot) {
        const QPointF at = c + markingSlotDirection(slot) * kSlotRadius;
        const bool on = slotEnabled(slot);
        p.setPen(QPen(on ? QColor(0x2d, 0x8c, 0xff) : QColor(90, 90, 96), 1));
        p.setBrush(on ? QColor(45, 45, 52) : QColor(36, 36, 40));
        p.drawEllipse(at, kSlotBadgeRadius, kSlotBadgeRadius);
        // Command icon in the upper part of the badge (ui::icon), the label underneath.
        const Command* cmd = slotCommand(slot);
        const bool hasIcon = cmd && !cmd->icon.empty();
        if (hasIcon) {
            const QImage glyph = renderIcon(cmd->icon, kSlotIconPx, iconThemeFor(palette()));
            if (!glyph.isNull()) {
                p.setOpacity(on ? 1.0 : 0.4);
                p.drawImage(QPointF(at.x() - kSlotIconPx / 2.0, at.y() - kSlotBadgeRadius + 5), glyph);
                p.setOpacity(1.0);
            }
        }
        p.setPen(on ? QColor(235, 235, 240) : QColor(120, 120, 126));
        const double top = hasIcon ? at.y() - kSlotBadgeRadius + 5 + kSlotIconPx : at.y() - kSlotBadgeRadius + 4;
        const QRectF text(at.x() - kSlotBadgeRadius + 4, top, 2 * kSlotBadgeRadius - 8,
                          at.y() + kSlotBadgeRadius - 4 - top);
        p.drawText(text, Qt::AlignCenter | Qt::TextWordWrap, slotLabel(slot));
    }

    p.setPen(QPen(QColor(90, 90, 96), 1));
    p.setBrush(QColor(55, 55, 62));
    p.drawEllipse(c, MarkingMenu::kInnerRadiusPx, MarkingMenu::kInnerRadiusPx);
    p.setPen(QColor(235, 235, 240));
    p.drawText(QRectF(c.x() - 30, c.y() - 10, 60, 20), Qt::AlignCenter, tr("More"));
}

void MarkingMenu::mouseReleaseEvent(QMouseEvent* e) {
    const QPointF center(width() / 2.0, height() / 2.0);
    const QPointF pos = e->position();
    const int slot = markingSlotAt(center, pos, kInnerRadiusPx);
    if (slot >= 0) {
        chooseSlot(slot);
    } else if (std::hypot(pos.x() - center.x(), pos.y() - center.y()) < kInnerRadiusPx) {
        QMenu* menu = overflowMenu();
        hide();
        menu->popup(e->globalPosition().toPoint()); // non-blocking
    } else {
        hide();
    }
    e->accept();
}

void MarkingMenu::keyPressEvent(QKeyEvent* e) {
    if (e->key() == Qt::Key_Escape) {
        hide();
        e->accept();
        return;
    }
    QWidget::keyPressEvent(e);
}

} // namespace iw::ui
