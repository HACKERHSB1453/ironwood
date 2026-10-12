// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Kernel-internal-but-OCCT-free helpers to build simple profiles (closed polygon wires / planar faces) until a
// sketch module exists. Declared here, defined in ops.cpp. Usable by tests and the proof feature (no OCCT types).

#include <span>

#include "core/math.hpp"
#include "core/result.hpp"
#include "kernel/shape.hpp"

namespace iw::kernel {

/// Closed polygonal Wire through `points` (>= 3, consecutive points distinct; closing edge added automatically).
/// Edge i runs points[i] -> points[(i+1) % n], so profile edge indices follow that order (verified by tests).
Result<Shape> makePolygonWire(std::span<const Vec3> points);

/// Planar Face bounded by a closed planar polygon through `points` (same edge order as makePolygonWire).
Result<Shape> makePolygonFace(std::span<const Vec3> points);

} // namespace iw::kernel
