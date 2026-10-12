// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <memory>

#include "core/math.hpp"
#include "tess/tessellate.hpp"

namespace iw::render {

/// Hand-built axis-aligned box mesh with the full MeshData topology contract (6 face ranges in the order
/// -X,+X,-Y,+Y,-Z,+Z; 12 edge polylines; per-face vertices with flat normals; CCW from outside). Used by the
/// demo scene and by tests so the viewport can be driven without kernel/tessellation.
std::shared_ptr<const tess::MeshData> makeBoxMesh(const Vec3& min, const Vec3& max);

} // namespace iw::render
