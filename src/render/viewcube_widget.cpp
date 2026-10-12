// SPDX-License-Identifier: GPL-3.0-or-later
#include "render/viewcube_widget.hpp"

#include <QFont>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>
#include <QTransform>
#include <cmath>

#include "render/viewcube_model.hpp"

namespace iw::render {

namespace {
constexpr double kCx = ViewCubeWidget::kSize / 2.0 + 6, kCy = ViewCubeWidget::kSize / 2.0 + 4;
constexpr double kHalf = ViewCubeWidget::kSize / 2.0 - 8;

QPolygonF toPoly(const ProjectedCell& c) {
    QPolygonF p;
    for (const auto& q : c.pts) p << QPointF(q[0], q[1]);
    return p;
}
bool sameDir(const Vec3& a, const Vec3& b) { return a == b; }

Vec3 neg(const Vec3& v) { return v * -1.0; }
} // namespace

ViewCubeWidget::ViewCubeWidget(const Camera* camera, QWidget* parent) : QWidget(parent), camera_(camera) {
    setFixedSize(kSize, kSize);
    setMouseTracking(true);
    setAttribute(Qt::WA_NoSystemBackground);
    setAutoFillBackground(false);
    setToolTip(tr("ViewCube: click a face, edge or corner to look from there; drag to orbit"));
}

QRectF ViewCubeWidget::homeRect() const { return QRectF(4, 4, 22, 22); }

bool ViewCubeWidget::homeButtonAt(const QPointF& p) const { return homeRect().contains(p); }

std::optional<Vec3> ViewCubeWidget::directionAt(const QPointF& p) const {
    const auto cells = projectViewCube(camera_->viewMatrix(), kCx, kCy, kHalf);
    const int i = hitTestViewCube(cells, p.x(), p.y());
    if (i < 0) return std::nullopt;
    return cells[static_cast<std::size_t>(i)].cell->dir;
}

void ViewCubeWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const Transform view = camera_->viewMatrix();
    const auto cells = projectViewCube(view, kCx, kCy, kHalf);
    const auto labels = viewCubeFaceLabels(camera_->upAxis());
    const Vec3 U = camera_->worldUp();
    const Vec3 F = camera_->frontEyeDirection();
    const double scale = kHalf * 0.58;
    auto rot = [&](const Vec3& v) {
        const auto& m = view.m;
        return Vec3{m[0] * v.x + m[1] * v.y + m[2] * v.z, m[4] * v.x + m[5] * v.y + m[6] * v.z,
                    m[8] * v.x + m[9] * v.y + m[10] * v.z};
    };
    auto scr = [&](const Vec3& v) { const Vec3 r = rot(v); return QPointF(kCx + r.x * scale, kCy - r.y * scale); };

    for (const auto& pc : cells) {
        const auto& cell = *pc.cell;
        const bool hot = hoverDir_ && sameDir(*hoverDir_, cell.dir);
        QColor fill = cell.kind == ViewCubeCell::Kind::Face   ? QColor(238, 241, 246)
                      : cell.kind == ViewCubeCell::Kind::Edge ? QColor(208, 214, 224)
                                                              : QColor(184, 192, 206);
        if (hot) fill = QColor(86, 156, 255);
        p.setPen(QPen(QColor(120, 128, 142), 0.8));
        p.setBrush(fill);
        p.drawPolygon(toPoly(pc));
        if (cell.kind == ViewCubeCell::Kind::Face) {
            // label mapped onto the face (affine: orthographic projection)
            const Vec3 n = cell.faceNormal;
            int idx = n.x > 0 ? 0 : n.x < 0 ? 1 : n.y > 0 ? 2 : n.y < 0 ? 3 : n.z > 0 ? 4 : 5;
            Vec3 up = U, right;
            if (sameDir(n, U)) up = neg(F);
            else if (sameDir(n, neg(U))) up = F;
            right = cross(up, n);
            const QPointF o = scr(n), pr = scr(n + right), pu = scr(n + up);
            const double k = 1.0 / 100.0;
            // local (lx, ly): lx along `right`, ly DOWN (= -up); one local unit = k cube units
            QTransform t((pr.x() - o.x()) * k, (pr.y() - o.y()) * k, -(pu.x() - o.x()) * k, -(pu.y() - o.y()) * k,
                         o.x(), o.y());
            p.save();
            p.setTransform(t, true);
            QFont f = font();
            f.setPixelSize(24);
            f.setBold(true);
            p.setFont(f);
            p.setPen(hot ? QColor(255, 255, 255) : QColor(70, 78, 92));
            p.drawText(QRectF(-95, -50, 190, 100), Qt::AlignCenter, tr(labels[static_cast<std::size_t>(idx)]));
            p.restore();
        }
    }

    // Home button (house glyph)
    p.setPen(QPen(QColor(120, 128, 142), 0.8));
    p.setBrush(hoverHome_ ? QColor(86, 156, 255) : QColor(238, 241, 246));
    p.drawRoundedRect(homeRect(), 4, 4);
    const QRectF h = homeRect();
    QPolygonF house;
    house << QPointF(h.center().x(), h.top() + 4.5) << QPointF(h.right() - 4, h.center().y() - 0.5)
          << QPointF(h.right() - 6.5, h.center().y() - 0.5) << QPointF(h.right() - 6.5, h.bottom() - 4.5)
          << QPointF(h.left() + 6.5, h.bottom() - 4.5) << QPointF(h.left() + 6.5, h.center().y() - 0.5)
          << QPointF(h.left() + 4, h.center().y() - 0.5);
    p.setPen(Qt::NoPen);
    p.setBrush(hoverHome_ ? QColor(255, 255, 255) : QColor(70, 78, 92));
    p.drawPolygon(house);
}

void ViewCubeWidget::mousePressEvent(QMouseEvent* e) {
    if (e->button() != Qt::LeftButton) { e->ignore(); return; }
    const QPointF pos = e->position();
    pressedHome_ = homeButtonAt(pos);
    pressDir_ = directionAt(pos);
    if (!pressedHome_ && !pressDir_) { e->ignore(); return; }
    pressed_ = true;
    dragging_ = false;
    pressPos_ = lastPos_ = pos;
    e->accept();
}

void ViewCubeWidget::mouseMoveEvent(QMouseEvent* e) {
    const QPointF pos = e->position();
    if (pressed_) {
        if (!dragging_ && (pos - pressPos_).manhattanLength() > 4 && !pressedHome_) dragging_ = true;
        if (dragging_) {
            emit orbitDragged(pos.x() - lastPos_.x(), pos.y() - lastPos_.y());
            lastPos_ = pos;
            update();
        }
        e->accept();
        return;
    }
    const auto d = directionAt(pos);
    const bool home = homeButtonAt(pos);
    if (d != hoverDir_ || home != hoverHome_) {
        hoverDir_ = d;
        hoverHome_ = home;
        setCursor((d || home) ? Qt::PointingHandCursor : Qt::ArrowCursor);
        update();
    }
    if (!d && !home) e->ignore();
}

void ViewCubeWidget::mouseReleaseEvent(QMouseEvent* e) {
    if (!pressed_ || e->button() != Qt::LeftButton) { e->ignore(); return; }
    pressed_ = false;
    if (!dragging_) {
        if (pressedHome_ && homeButtonAt(e->position())) emit homeRequested();
        else if (pressDir_) emit viewRequested(pressDir_->x, pressDir_->y, pressDir_->z);
    }
    dragging_ = false;
    e->accept();
}

void ViewCubeWidget::leaveEvent(QEvent*) {
    hoverDir_.reset();
    hoverHome_ = false;
    update();
}

} // namespace iw::render
