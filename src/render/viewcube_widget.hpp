// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QWidget>
#include <optional>

#include "core/math.hpp"
#include "render/camera.hpp"

namespace iw::render {

/// QPainter overlay drawn on top of the Viewport: the cube follows the camera orientation; hovering highlights a
/// face/edge/corner, a click requests the matching view, dragging orbits, the house button requests Home.
/// It never owns the camera: it reads it through a pointer valid for the widget's lifetime.
class ViewCubeWidget : public QWidget {
    Q_OBJECT
public:
    static constexpr int kSize = 176;
    explicit ViewCubeWidget(const Camera* camera, QWidget* parent = nullptr);

    /// Cell under the widget-local point, as eye-direction sign vector (test hook / shared with the mouse code).
    std::optional<Vec3> directionAt(const QPointF& p) const;
    bool homeButtonAt(const QPointF& p) const;

signals:
    void viewRequested(double dx, double dy, double dz);  ///< Look from this (sign-vector) direction.
    void homeRequested();
    void orbitDragged(double dxPixels, double dyPixels);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void leaveEvent(QEvent*) override;

private:
    QRectF homeRect() const;
    const Camera* camera_;
    std::optional<Vec3> hoverDir_;
    bool hoverHome_ = false;
    QPointF pressPos_, lastPos_;
    bool pressed_ = false, dragging_ = false, pressedHome_ = false;
    std::optional<Vec3> pressDir_;
};

} // namespace iw::render
