// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Screen <-> sketch-plane mapping through the viewport camera. Qt-free: widgets pass plain doubles in widget pixels
// (x right, y down) together with the widget size.

#include <optional>

#include "render/camera.hpp"
#include "sketch/sketch.hpp"

namespace iw::ui {

struct ScreenPoint {
    double x = 0, y = 0;   ///< Widget pixels, y down.
};

/// Pixels per millimetre at the camera target depth (what a click tolerance of N pixels means in mm).
double sketchPixelsPerMm(const render::Camera& cam, double widgetHeight);

/// Sketch point -> widget pixels; nullopt when the point is behind the eye.
std::optional<ScreenPoint> planeToScreen(const render::Camera& cam, double widgetWidth, double widgetHeight,
                                         const sketch::SketchPlane& plane, sketch::Point2 p);

/// Widget pixels -> sketch point: the camera ray through the pixel, intersected with the sketch plane.
/// nullopt when the ray is parallel to the plane.
std::optional<sketch::Point2> screenToPlane(const render::Camera& cam, double widgetWidth, double widgetHeight,
                                            const sketch::SketchPlane& plane, ScreenPoint s);

} // namespace iw::ui
