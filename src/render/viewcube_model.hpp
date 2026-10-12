// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <optional>
#include <vector>

#include "core/math.hpp"
#include "render/camera.hpp"

namespace iw::render {

/// Qt-free geometry + hit-testing of the ViewCube. The cube is [-1,1]^3 in world axes; each of its six faces is
/// split into a 3x3 grid: centre cell = face view, edge cells = edge views, corner cells = corner views.
/// A cell's `dir` is the sign vector (-1/0/1 per world axis) pointing from the cube centre to the cell; clicking
/// the cell asks the camera to look FROM that direction (26 distinct directions).
struct ViewCubeCell {
    enum class Kind { Face, Edge, Corner };
    std::array<Vec3, 4> corners;  ///< Cell quad in cube coordinates, CCW seen from outside.
    Vec3 dir;                     ///< Sign vector of the view this cell selects.
    Vec3 faceNormal;              ///< Unit normal of the cube face the cell lies on.
    Kind kind = Kind::Face;
};

struct ProjectedCell {
    const ViewCubeCell* cell = nullptr;
    std::array<std::array<double, 2>, 4> pts{}; ///< Screen pixels (y down).
    double depth = 0;                           ///< Larger = closer to the viewer.
};

/// All 54 cells (6 faces x 9). Static storage.
const std::vector<ViewCubeCell>& viewCubeCells();

/// Orthographically projects the front-facing cells for a camera whose view matrix is `view`, onto a square of
/// `halfSizePx` (cube half-extent maps to ~halfSizePx*0.58 so the rotated cube stays inside). Result is sorted
/// back-to-front (paint order).
std::vector<ProjectedCell> projectViewCube(const Transform& view, double cx, double cy, double halfSizePx);

/// Topmost cell containing the point, or -1.
int hitTestViewCube(const std::vector<ProjectedCell>& cells, double x, double y);

/// Text of the six face labels for an up axis, in the order +X,-X,+Y,-Y,+Z,-Z ("RIGHT","LEFT",...).
std::array<const char*, 6> viewCubeFaceLabels(UpAxis up);

} // namespace iw::render
