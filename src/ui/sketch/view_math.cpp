// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/sketch/view_math.hpp"

#include <cmath>

#include "core/math.hpp"

namespace iw::ui {

double sketchPixelsPerMm(const render::Camera& cam, double widgetHeight) {
    const double halfHeight = cam.halfHeightAtTarget();
    if (halfHeight <= 0 || widgetHeight <= 0) return 1.0;
    return widgetHeight / (2.0 * halfHeight);
}

std::optional<ScreenPoint> planeToScreen(const render::Camera& cam, double widgetWidth, double widgetHeight,
                                         const sketch::SketchPlane& plane, sketch::Point2 p) {
    const auto ndc = cam.project(plane.toWorld(p));
    if (!ndc) return std::nullopt;
    return ScreenPoint{(ndc->x + 1.0) * 0.5 * widgetWidth, (1.0 - ndc->y) * 0.5 * widgetHeight};
}

std::optional<sketch::Point2> screenToPlane(const render::Camera& cam, double widgetWidth, double widgetHeight,
                                            const sketch::SketchPlane& plane, ScreenPoint s) {
    if (widgetWidth <= 0 || widgetHeight <= 0) return std::nullopt;
    const double ndcX = 2.0 * s.x / widgetWidth - 1.0;
    const double ndcY = 1.0 - 2.0 * s.y / widgetHeight;
    const render::Ray ray = cam.rayThrough(ndcX, ndcY);
    const double denom = dot(ray.dir, plane.normal);
    if (std::abs(denom) < 1e-9) return std::nullopt;
    const double t = dot(plane.origin - ray.origin, plane.normal) / denom;
    const Vec3 hit = ray.origin + ray.dir * t;
    return plane.toLocal(hit);
}

} // namespace iw::ui
