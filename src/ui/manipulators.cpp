// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/manipulators.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>

#include <cmath>
#include <numbers>

namespace iw::ui {
namespace {

constexpr double kDeg = 180.0 / std::numbers::pi;

double wrapDeg(double d) {
    while (d > 180.0) d -= 360.0;
    while (d <= -180.0) d += 360.0;
    return d;
}

QPointF ringPoint(QPointF center, double radiusPx, double deg) {
    const double r = deg / kDeg;
    return {center.x() + radiusPx * std::cos(r), center.y() - radiusPx * std::sin(r)};
}

} // namespace

double screenAngleDeg(QPointF center, QPointF p) {
    return std::atan2(-(p.y() - center.y()), p.x() - center.x()) * kDeg;
}

double distanceFromDrag(double startMm, QPointF startMouse, QPointF mouse, QPointF dirUnit, double pxPerMm) {
    if (pxPerMm <= 0.0) return startMm;
    const QPointF d = mouse - startMouse;
    return startMm + (d.x() * dirUnit.x() + d.y() * dirUnit.y()) / pxPerMm;
}

double angleFromDrag(double startDeg, QPointF center, QPointF startMouse, QPointF mouse) {
    return startDeg + wrapDeg(screenAngleDeg(center, mouse) - screenAngleDeg(center, startMouse));
}

ManipulatorOverlay::ManipulatorOverlay(QWidget* parent) : QWidget(parent) {
    setAttribute(Qt::WA_TranslucentBackground);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_TransparentForMouseEvents);
}

void ManipulatorOverlay::showDistance(QPointF anchor, QPointF dirUnit, double valueMm, double pxPerMm,
                                      const QString& label) {
    mode_ = Mode::Distance;
    anchor_ = anchor;
    const double len = std::hypot(dirUnit.x(), dirUnit.y());
    dir_ = len > 0.0 ? QPointF(dirUnit.x() / len, dirUnit.y() / len) : QPointF(1.0, 0.0);
    value_ = valueMm;
    pxPerMm_ = pxPerMm > 0.0 ? pxPerMm : 1.0;
    label_ = label;
    setAttribute(Qt::WA_TransparentForMouseEvents, false);
    update();
}

void ManipulatorOverlay::showAngle(QPointF center, double startDeg, double valueDeg, double radiusPx,
                                   const QString& label) {
    mode_ = Mode::Angle;
    anchor_ = center;
    startDeg_ = startDeg;
    value_ = valueDeg;
    radiusPx_ = radiusPx > 0.0 ? radiusPx : 60.0;
    label_ = label;
    setAttribute(Qt::WA_TransparentForMouseEvents, false);
    update();
}

void ManipulatorOverlay::clear() {
    mode_ = Mode::None;
    dragging_ = false;
    label_.clear();
    setAttribute(Qt::WA_TransparentForMouseEvents, true);
    update();
}

QPointF ManipulatorOverlay::handlePos() const {
    switch (mode_) {
    case Mode::Distance: return anchor_ + dir_ * (value_ * pxPerMm_);
    case Mode::Angle: return ringPoint(anchor_, radiusPx_, startDeg_ + value_);
    case Mode::None: break;
    }
    return anchor_;
}

void ManipulatorOverlay::paintEvent(QPaintEvent*) {
    if (mode_ == Mode::None) return;
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    const QColor accent(0x2d, 0x8c, 0xff);
    QPen pen(accent, 2);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);

    if (mode_ == Mode::Distance) {
        const QPointF end = handlePos();
        p.drawLine(anchor_, end);
        // Extension ticks perpendicular to the arrow at both ends.
        const QPointF perp(-dir_.y(), dir_.x());
        p.drawLine(anchor_ - perp * 8, anchor_ + perp * 8);
        p.drawLine(end - perp * 8, end + perp * 8);
        // Arrow head at the handle.
        const QPointF back = end - dir_ * 10;
        p.drawLine(end, back + perp * 5);
        p.drawLine(end, back - perp * 5);
        const QPointF mid = (anchor_ + end) / 2.0;
        if (!label_.isEmpty()) p.drawText(mid + QPointF(6, -6), label_);
    } else {
        p.setPen(QPen(accent, 1, Qt::DashLine));
        p.drawEllipse(anchor_, radiusPx_, radiusPx_);
        p.setPen(pen);
        const QRectF rect(anchor_.x() - radiusPx_, anchor_.y() - radiusPx_, 2 * radiusPx_, 2 * radiusPx_);
        p.drawArc(rect, static_cast<int>(std::lround(startDeg_ * 16)), static_cast<int>(std::lround(value_ * 16)));
        if (!label_.isEmpty()) p.drawText(ringPoint(anchor_, radiusPx_ + 14, startDeg_ + value_ / 2), label_);
    }

    const QPointF h = handlePos();
    p.setBrush(dragging_ ? accent : QColor(255, 255, 255));
    p.drawEllipse(h, kHandleRadiusPx * 0.6, kHandleRadiusPx * 0.6);
}

void ManipulatorOverlay::mousePressEvent(QMouseEvent* e) {
    if (mode_ == Mode::None || e->button() != Qt::LeftButton) {
        e->ignore();
        return;
    }
    const QPointF pos = e->position();
    const QPointF d = pos - handlePos();
    if (std::hypot(d.x(), d.y()) > kHandleRadiusPx) {
        e->ignore(); // let the viewport navigate
        return;
    }
    dragging_ = true;
    dragStartMouse_ = pos;
    dragStartValue_ = value_;
    e->accept();
    update();
}

void ManipulatorOverlay::mouseMoveEvent(QMouseEvent* e) {
    if (!dragging_) {
        e->ignore();
        return;
    }
    const QPointF pos = e->position();
    if (mode_ == Mode::Distance) {
        value_ = distanceFromDrag(dragStartValue_, dragStartMouse_, pos, dir_, pxPerMm_);
        emit distanceChanged(value_);
    } else if (mode_ == Mode::Angle) {
        value_ = angleFromDrag(dragStartValue_, anchor_, dragStartMouse_, pos);
        emit angleChanged(value_);
    }
    e->accept();
    update();
}

void ManipulatorOverlay::mouseReleaseEvent(QMouseEvent* e) {
    if (!dragging_) {
        e->ignore();
        return;
    }
    dragging_ = false;
    e->accept();
    update();
    emit dragFinished();
}

} // namespace iw::ui
