// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// QPainter overlay drawing sketches over the viewport (like the ViewCube). Curves are projected through the viewport
// camera; construction geometry is dashed; points are small squares; profiles are filled translucent light orange.

#include <QColor>
#include <QLineF>
#include <QPolygonF>
#include <QRectF>
#include <QString>
#include <QWidget>

#include <optional>
#include <vector>

#include "ui/sketch/session.hpp"

namespace iw::render {
class Camera;
class Viewport;
}

class QPainter;

namespace iw::ui {

class SketchSession;

/// Display toggles of the sketch palette.
struct SketchViewOptions {
    bool profiles = true;     ///< Fill closed profiles.
    bool points = true;       ///< Draw sketch points and curve end dots.
    bool constraints = true;  ///< Draw constraint glyphs (H, V, coincident, ...).
    bool dimensions = true;   ///< Draw dimensions.
    bool dark = false;        ///< Dark theme: fully constrained curves are white instead of black.
};

/// Curve colour of the active sketch: blue while under-constrained, black (light theme) / white (dark theme) when
/// fully constrained. Conflicts keep the blue; the offending constraints are what turns red.
QColor sketchCurveColor(ConstraintState state, bool dark);

/// Widget-pixel position of a constraint glyph (nullopt when behind the eye).
std::optional<QPointF> glyphScreenPos(const render::Camera& cam, double width, double height,
                                      const sketch::SketchPlane& plane, const ConstraintGlyph& g);

/// Screen-space drawing of one dimension: extension and dimension lines, arrow heads and the text box.
struct DimensionLayout {
    std::vector<QLineF> lines;
    std::vector<QPolygonF> arrows;
    QString text;
    QRectF textRect;   ///< Hit area of the text (double click edits).
};
std::optional<DimensionLayout> layoutDimension(const render::Camera& cam, double width, double height,
                                               const SketchSession& session, const DimensionView& dim);

/// Paints `sessions` into `p` for a widget of size `width` x `height`. `active` is drawn in full colour with its
/// selection and preview; the other sessions are dimmed. Pure painting: the overlay and the tests both use it.
void paintSketches(QPainter& p, const render::Camera& cam, double width, double height,
                   const std::vector<const SketchSession*>& sessions, const SketchSession* active,
                   const SketchViewOptions& opts);

/// Transparent-to-mouse child of the viewport that paints the sketches on top of it.
class SketchOverlay : public QWidget {
    Q_OBJECT
public:
    explicit SketchOverlay(render::Viewport* viewport);

    /// Replaces what is drawn. Pointers must stay valid until the next call (the sketch mode owns the sessions).
    void setContent(std::vector<const SketchSession*> sessions, const SketchSession* active, SketchViewOptions opts);

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    render::Viewport* viewport_;
    std::vector<const SketchSession*> sessions_;
    const SketchSession* active_ = nullptr;
    SketchViewOptions opts_;
};

} // namespace iw::ui
