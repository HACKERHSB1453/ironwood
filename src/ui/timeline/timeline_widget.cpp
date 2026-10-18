// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/timeline/timeline_widget.hpp"

#include "ui/icons.hpp"

#include <QEvent>
#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>

#include <algorithm>
#include <cstdlib>

namespace iw::ui::timeline {
namespace {

constexpr int kItemW = 92;
constexpr int kGap = 16;        ///< Space between items; the rollback marker sits in these gaps.
constexpr int kLeft = 14;
constexpr int kTop = 6;
constexpr int kIconH = 40;
constexpr int kLabelH = 20;
constexpr int kHeight = kTop + kIconH + 4 + kLabelH + 8;
constexpr int kGrab = 9;        ///< Half-width of the marker's grab zone.

QColor familyColour(IconFamily f) {
    switch (f) {
    case IconFamily::Sketch: return QColor("#3d7ab8");
    case IconFamily::Solid: return QColor("#5b6b7a");
    case IconFamily::Edit: return QColor("#2e8b6e");
    case IconFamily::Construction: return QColor("#8e6bb0");
    }
    return QColor("#5b6b7a");
}

/// Small glyph per family (own drawing; not an icon-theme image).
void drawGlyph(QPainter& p, const QRectF& r, IconFamily f, const QColor& ink) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);
    QPen pen(ink, 2.0);
    pen.setJoinStyle(Qt::RoundJoin);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    const QPointF c = r.center();
    const double s = r.height() * 0.32;
    switch (f) {
    case IconFamily::Sketch:
        p.drawRoundedRect(QRectF(c.x() - s, c.y() - s, 2 * s, 2 * s), 2, 2);
        break;
    case IconFamily::Solid: {
        QPainterPath path;
        path.moveTo(c.x() - s, c.y() + s * 0.5);
        path.lineTo(c.x(), c.y() - s);
        path.lineTo(c.x() + s, c.y() + s * 0.5);
        path.lineTo(c.x() - s, c.y() + s * 0.5);
        p.setBrush(ink);
        p.drawPath(path);
        break;
    }
    case IconFamily::Edit:
        p.drawArc(QRectF(c.x() - s, c.y() - s, 2 * s, 2 * s), 30 * 16, 240 * 16);
        break;
    case IconFamily::Construction:
        p.drawLine(QPointF(c.x() - s, c.y()), QPointF(c.x() + s, c.y()));
        p.drawLine(QPointF(c.x(), c.y() - s), QPointF(c.x(), c.y() + s));
        break;
    }
    p.restore();
}

} // namespace

TimelineWidget::TimelineWidget(QWidget* parent) : QWidget(parent) {
    setMouseTracking(true);
    setObjectName("timelineWidget");
    setMinimumHeight(kHeight + 12);
}

void TimelineWidget::setEntries(std::vector<TimelineEntry> entries, std::size_t activeCount) {
    entries_ = std::move(entries);
    activeCount_ = std::min(activeCount, entries_.size());
    if (hovered_) {
        bool still = false;
        for (const TimelineEntry& e : entries_) still = still || e.id == *hovered_;
        if (!still) hovered_.reset();
    }
    update();
}

QRect TimelineWidget::itemRect(std::size_t index) const {
    const int x = kLeft + static_cast<int>(index) * (kItemW + kGap);
    return QRect(x, kTop, kItemW, kHeight - kTop - 4);
}

std::size_t TimelineWidget::markerAt(int x) const {
    const std::size_t n = entries_.size();
    std::size_t best = 0;
    int bestDist = 1 << 30;
    for (std::size_t m = 0; m <= n; ++m) {
        int bx = 0;
        if (m == 0) bx = kLeft - kGap / 2;
        else if (m < n) bx = itemRect(m).left() - kGap / 2;
        else bx = itemRect(n - 1).right() + kGap / 2;
        const int d = std::abs(bx - x);
        if (d < bestDist) {
            bestDist = d;
            best = m;
        }
    }
    return best;
}

QRect TimelineWidget::markerRect() const {
    const std::size_t n = entries_.size();
    int bx = kLeft - kGap / 2;
    if (activeCount_ >= n && n > 0) bx = itemRect(n - 1).right() + kGap / 2;
    else if (activeCount_ > 0) bx = itemRect(activeCount_).left() - kGap / 2;
    return QRect(bx - kGrab, 0, 2 * kGrab, height());
}

std::optional<std::size_t> TimelineWidget::itemAt(QPoint p) const {
    for (std::size_t i = 0; i < entries_.size(); ++i)
        if (itemRect(i).contains(p)) return i;
    return std::nullopt;
}

QSize TimelineWidget::sizeHint() const {
    const int n = static_cast<int>(entries_.size());
    return QSize(std::max(480, kLeft * 2 + n * (kItemW + kGap)), kHeight + 12);
}

QSize TimelineWidget::minimumSizeHint() const { return QSize(240, kHeight + 12); }

void TimelineWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), palette().window());
    if (entries_.empty()) {
        p.setPen(palette().color(QPalette::PlaceholderText));
        p.drawText(rect(), Qt::AlignCenter, tr("The timeline is empty. Create a sketch, then extrude it."));
        return;
    }
    QFont small = font();
    small.setPointSizeF(font().pointSizeF() * 0.9);
    p.setFont(small);
    const QFontMetrics fm(small);
    const IconTheme theme = iconThemeFor(palette());

    for (std::size_t i = 0; i < entries_.size(); ++i) {
        const TimelineEntry& e = entries_[i];
        const QRect r = itemRect(i);
        const bool rolledBack = i >= activeCount_;
        const bool failed = e.state == recompute::FeatureStatus::State::Failed;
        const bool hovered = hovered_ && *hovered_ == e.id;

        // Tile: the feature's icon from the icon set on a neutral tile, outlined in its family colour.
        QRect icon(r.left() + 6, r.top() + 2, r.width() - 12, kIconH);
        const QColor family = familyColour(e.family);
        p.setPen(QPen(rolledBack || e.suppressed ? family.lighter(170) : family, 2));
        p.setBrush(palette().base());
        p.drawRoundedRect(icon.adjusted(1, 1, -1, -1), 6, 6);
        const QImage glyph = e.icon.empty() ? QImage() : renderIcon(e.icon, kIconH - 10, theme);
        p.setOpacity(rolledBack || e.suppressed ? 0.45 : 1.0);
        if (!glyph.isNull()) {
            p.drawImage(QPointF(icon.center().x() - glyph.width() / 2.0, icon.center().y() - glyph.height() / 2.0), glyph);
        } else {
            drawGlyph(p, icon, e.family, family);
        }
        p.setOpacity(1.0);

        // State badge: failed = red outline, warning = yellow badge with '!'
        if (failed) {
            QPen pen(QColor("#c0392b"), 3);
            p.setPen(pen);
            p.setBrush(Qt::NoBrush);
            p.drawRoundedRect(icon.adjusted(-2, -2, 2, 2), 7, 7);
        } else if (e.warning) {
            QRect badge(icon.right() - 12, icon.top() - 5, 18, 18);
            p.setPen(QPen(QColor("#7d6608"), 1));
            p.setBrush(QColor("#f1c40f"));
            p.drawEllipse(badge);
            p.setPen(QColor("#202020"));
            p.drawText(badge, Qt::AlignCenter, QStringLiteral("!"));
        }
        if (hovered) {
            p.setPen(QPen(palette().highlight().color(), 2));
            p.setBrush(Qt::NoBrush);
            p.drawRoundedRect(icon.adjusted(-3, -3, 3, 3), 8, 8);
        }

        // Label (elided), dimmed when rolled back or suppressed
        QColor ink = palette().windowText().color();
        if (rolledBack || e.suppressed) ink.setAlpha(120);
        if (failed) ink = QColor("#c0392b");
        p.setPen(ink);
        const QString text = fm.elidedText(e.label, Qt::ElideRight, r.width());
        QRect labelRect(r.left(), icon.bottom() + 4, r.width(), kLabelH);
        p.drawText(labelRect, Qt::AlignHCenter | Qt::AlignVCenter, text);
        if (e.suppressed) {
            p.setPen(QPen(ink, 1));
            p.drawLine(labelRect.left() + 6, labelRect.center().y(), labelRect.right() - 6, labelRect.center().y());
        }
    }

    // Rollback marker
    const QRect mr = markerRect();
    const int x = mr.center().x();
    QPen pen(palette().highlight().color(), 2);
    p.setPen(pen);
    p.drawLine(x, 0, x, height() - 1);
    QPainterPath grip;
    grip.moveTo(x - 6, 0);
    grip.lineTo(x + 6, 0);
    grip.lineTo(x, 8);
    grip.closeSubpath();
    p.setPen(Qt::NoPen);
    p.setBrush(palette().highlight());
    p.drawPath(grip);
}

void TimelineWidget::mousePressEvent(QMouseEvent* e) {
    if (e->button() == Qt::RightButton) {
        if (auto i = itemAt(e->pos())) emit contextRequested(entries_[*i].id, e->globalPosition().toPoint());
        return;
    }
    if (e->button() != Qt::LeftButton) return;
    if (markerRect().contains(e->pos())) {
        dragging_ = true;
        committedAtPress_ = activeCount_;
        dragValue_ = activeCount_;
        return;
    }
    // A click on an item only selects it (hover/tooltip); the marker moves by dragging its handle.
}

void TimelineWidget::mouseMoveEvent(QMouseEvent* e) {
    if (dragging_) {
        const std::size_t v = markerAt(e->pos().x());
        if (v != dragValue_) {
            dragValue_ = v;
            activeCount_ = v;   // provisional: keeps the marker under the pointer while the preview runs
            update();
            emit rollbackPreview(v);
        }
        return;
    }
    std::optional<FeatureId> now;
    if (auto i = itemAt(e->pos())) now = entries_[*i].id;
    if (now != hovered_) {
        hovered_ = now;
        update();
        emit hoverChanged(now);
    }
    if (auto i = itemAt(e->pos())) setToolTip(entries_[*i].message);
    else setToolTip(QString());
}

void TimelineWidget::mouseReleaseEvent(QMouseEvent* e) {
    if (e->button() != Qt::LeftButton || !dragging_) return;
    dragging_ = false;
    if (dragValue_ != committedAtPress_) emit rollbackCommitted(dragValue_);
}

void TimelineWidget::mouseDoubleClickEvent(QMouseEvent* e) {
    if (e->button() != Qt::LeftButton) return;
    if (auto i = itemAt(e->pos())) emit editRequested(entries_[*i].id);
}

void TimelineWidget::leaveEvent(QEvent*) {
    if (hovered_) {
        hovered_.reset();
        update();
        emit hoverChanged(std::nullopt);
    }
}

} // namespace iw::ui::timeline
