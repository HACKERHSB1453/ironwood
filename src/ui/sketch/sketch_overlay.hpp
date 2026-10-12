// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// QPainter overlay drawing sketches over the viewport (like the ViewCube). Curves are projected through the viewport
// camera; construction geometry is dashed; points are small squares; profiles are filled translucent light orange.

#include <QWidget>

#include <vector>

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
    bool constraints = true;  ///< Draw constraint glyphs (H, V, coincident).
};

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
