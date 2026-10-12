// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QPointF>
#include <QWidget>

namespace iw::ui {

// Screen-space helpers. Screen y grows downwards; angles are in degrees, counter-clockwise as seen on screen.

/// Screen angle of `p` seen from `center`, in degrees (-180, 180].
double screenAngleDeg(QPointF center, QPointF p);
/// New distance (mm) after dragging from `startMouse` to `mouse` along the unit direction `dirUnit`, where the
/// value was `startMm` when the drag began and `pxPerMm` is the screen scale.
double distanceFromDrag(double startMm, QPointF startMouse, QPointF mouse, QPointF dirUnit, double pxPerMm);
/// New angle (degrees) after dragging around `center` from `startMouse` to `mouse`, starting at `startDeg`.
double angleFromDrag(double startDeg, QPointF center, QPointF startMouse, QPointF mouse);

/// Canvas manipulators drawn over the viewport (like the ViewCube): a distance arrow between two points and an angle
/// ring around a vertex. Dragging the handle reports the new value; the owner binds it to a dialog field. The overlay
/// is transparent to the mouse while idle, so navigation keeps working.
class ManipulatorOverlay : public QWidget {
    Q_OBJECT
public:
    enum class Mode { None, Distance, Angle };

    explicit ManipulatorOverlay(QWidget* parent = nullptr);

    /// Shows a distance arrow from `anchor` along `dirUnit`; the handle sits at value*pxPerMm from the anchor.
    void showDistance(QPointF anchor, QPointF dirUnit, double valueMm, double pxPerMm, const QString& label = {});
    /// Shows an angle ring at `center` with radius `radiusPx`; the handle sits at `startDeg + valueDeg` (screen).
    void showAngle(QPointF center, double startDeg, double valueDeg, double radiusPx, const QString& label = {});
    void clear();

    Mode mode() const noexcept { return mode_; }
    bool dragging() const noexcept { return dragging_; }
    /// Screen position of the drag handle (undefined when mode() is None).
    QPointF handlePos() const;
    /// Current value shown by the manipulator (mm or degrees).
    double value() const noexcept { return value_; }
    /// Handle radius for hit testing.
    static constexpr double kHandleRadiusPx = 9.0;

signals:
    void distanceChanged(double mm);
    void angleChanged(double deg);
    void dragFinished();

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;

private:
    Mode mode_ = Mode::None;
    QPointF anchor_;     ///< Distance: start point. Angle: centre.
    QPointF dir_;        ///< Distance: unit direction (screen).
    double value_ = 0.0; ///< mm or degrees.
    double pxPerMm_ = 1.0;
    double startDeg_ = 0.0;
    double radiusPx_ = 60.0;
    QString label_;
    bool dragging_ = false;
    QPointF dragStartMouse_;
    double dragStartValue_ = 0.0;
};

} // namespace iw::ui
