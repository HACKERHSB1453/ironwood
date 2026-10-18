// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Kernel-side support for persistent naming (N1 / F27). Lives in its own file so it does not touch ops.cpp / shape.cpp:
//  * stableSubShapeKey: Shape::subShapeKey with a defined key for degenerate edges (cone apex circle, seams of zero length),
//    which subShapeKey reports as {0,0,0,0}.
//  * holeWithTool: kernel::hole that ALSO returns the hole tool (the revolved solid) and the tool's profile with a role label
//    per profile segment, so the naming layer can keep the tool (operand 1) history and name hole faces by ROLE.
// FOLLOW-UP for the owner of ops.cpp: kernel::hole() should be reduced to `holeWithTool(...)->result` (the two share the
// profile construction logic by copy today).

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "core/cancel.hpp"
#include "core/math.hpp"
#include "core/result.hpp"
#include "kernel/ops.hpp"
#include "kernel/shape.hpp"

namespace iw::kernel {

/// As Shape::subShapeKey, except that a degenerate edge (zero length: the apex circle of a cone, a collapsed seam) gets
/// its start-vertex position as centroid and measure 0, never the (0,0,0) origin.
std::array<double, 4> stableSubShapeKey(const Shape& shape, SubShapeKind kind, std::size_t index);

/// Closed polygon (points in world space) that is revolved about the hole axis to make the tool. Segment k runs from
/// points[k] to points[(k + 1) % n]; segmentRoles[k] names what it becomes: "top" (the cap outside the entry face),
/// "wall", "floor" (flat bottom), "drill_cone", "cbore_wall", "cbore_floor", "csink_cone", "end" (far end of a through
/// hole) or "axis" (the segment on the rotation axis, which produces no face). Roles are unique within one profile.
struct HoleToolProfile {
    std::vector<Vec3> points;
    std::vector<std::string> segmentRoles;
};

struct HoleDetail {
    ShapeResult result;     ///< Identical to kernel::hole(): history operand 0 = target, operand 1 = `tool.shape`.
    ShapeResult tool;       ///< The revolved tool; history operand 0 = `profileFace`.
    Shape profileFace;      ///< Planar face of `profile`.
    HoleToolProfile profile;
};

/// Same contract and errors as kernel::hole, plus the tool and its profile.
Result<HoleDetail> holeWithTool(const Shape& shape, const Axis& placement, const HoleSpec& spec, CancelToken cancel = {});

} // namespace iw::kernel
