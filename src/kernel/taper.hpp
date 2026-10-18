// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "core/cancel.hpp"
#include "core/math.hpp"
#include "core/result.hpp"
#include "kernel/history.hpp"
#include "kernel/shape.hpp"

namespace iw::kernel {

/// Extrudes a planar `profile` (a planar Face, or a closed planar Wire) by `direction` (mm; |direction| is the
/// distance) with a draft (taper) angle on every side. `taperRadians` is the angle between a side and the extrusion
/// direction: positive tapers inward (the profile shrinks as it moves along `direction`), negative outward. The
/// magnitude must be below 89 degrees. Zero delegates to extrudeProfile().
///
/// Construction: the top outline is the profile's mitred offset by |direction| * tan(taper), moved along `direction`,
/// and the sides are ruled between the two outlines (exact planar sides for polygons).
///
/// `direction` must be normal to the profile plane: an oblique tapered extrude is InvalidArgument. An inward taper
/// whose offset empties the profile fails with KernelFailure ("taper too large: profile collapses at distance X mm",
/// X = the largest inward offset that still exists divided by tan(taper)). Faces with holes are InvalidArgument.
/// History (operand 0 = profile) follows extrudeProfile(): profile edge -> generated side face; profile face -> start
/// cap modified; end cap generated.
Result<ShapeResult> extrudeProfileTapered(const Shape& profile, const Vec3& direction, double taperRadians,
                                          const CancelToken& cancel = {});

} // namespace iw::kernel
