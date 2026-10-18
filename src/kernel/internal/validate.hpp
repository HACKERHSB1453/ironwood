// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Result validation for topology-changing operations (F38, F39). OCCT-free interface: the OCCT work happens in
// kernel/ops.cpp, these functions are exposed so the kernel tests can exercise the checks directly.

#include <cstdint>

#include "core/math.hpp"
#include "kernel/shape.hpp"

namespace iw::kernel::check {

enum class BooleanKind : std::uint8_t { Fuse, Cut, Common };

/// Structural validity of a result: BRepCheck_Analyzer passes, the shape contains at least one solid, and every
/// solid is a closed shell (each non-degenerate edge is shared by at least two faces). False for a null shape.
bool isSolidValid(const Shape& s);

/// Volume invariant of a boolean (F38), with a relative tolerance of 1e-5 of (|A|+|B|) plus 1e-6 mm^3:
///   fuse:   max(|A|,|B|) <= |R| <= |A|+|B|
///   cut:    |R| <= |A|
///   common: |R| <= min(|A|,|B|)
/// A non-positive or NaN result volume is implausible.
bool volumesPlausible(BooleanKind kind, double volumeA, double volumeB, double volumeResult);

/// True when `result` lies inside `input` grown by `allowance` mm on every side (F39: a fillet or chamfer cannot put
/// material far from its input). An empty input box is never a match.
bool boundsWithin(const Box3& input, const Box3& result, double allowance);

} // namespace iw::kernel::check
