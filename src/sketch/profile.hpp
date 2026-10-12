// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Profile detection: closed regions of a sketch's non-construction curves (ARCHITECTURE section 5, P1a contract
// "Sketch naming"). OCCT-free header; the region -> kernel face conversion lives in profile_face.cpp.

#include <cstdint>
#include <string>
#include <vector>

#include "core/result.hpp"
#include "kernel/shape.hpp"
#include "sketch/sketch.hpp"

namespace iw::sketch {

/// One directed piece of a profile boundary, in loop traversal order. A piece is a whole entity or a sub-segment
/// of a split entity. `subIndex` is the piece's position along its entity (lines: from start to end; arcs: from
/// startAngle CCW; circles: from angle 0 CCW), so `(entity, subIndex)` identifies the piece for naming.
struct ProfileEdge {
    SketchEntityId entity;
    std::uint32_t subIndex = 0;
    bool isArc = false;
    Point2 start, end;        ///< Traversal start and end points (equal for a full circle).
    Point2 center;            ///< Arcs only.
    double radius = 0;        ///< Arcs only.
    double startAngle = 0;    ///< Arcs only: angle of `start` about `center`.
    double sweep = 0;         ///< Arcs only: signed angle travelled from `start` to `end` (|sweep| <= 2*pi).
};

/// A closed boundary loop. `area` is signed: outer loops are CCW (> 0), holes are CW (< 0), mm^2.
struct ProfileLoop {
    std::vector<ProfileEdge> edges;
    double area = 0;
};

/// A bounded region of the sketch: one outer loop (CCW) and zero or more hole loops (CW).
struct SketchProfile {
    /// Deterministic name from the bounding pieces, sorted: "e<entity>.<sub>+e<entity>.<sub>+...".
    /// Unaffected by adding unrelated entities or by region index.
    std::string name;
    ProfileLoop outer;
    std::vector<ProfileLoop> holes;
    double area = 0;          ///< Region area = outer area - sum of hole areas (mm^2, > 0).
    Point2 centroid;          ///< Region centroid (mm, sketch coordinates).
};

/// Detects all bounded regions of the sketch's non-construction line/arc/circle entities. Curves are split at
/// every mutual intersection and at every endpoint lying on another curve (T-junctions); dangling and bridge
/// edges do not form regions. Regions are the bounded faces of the planar arrangement; nested loops are
/// classified by even-odd containment (a loop directly inside a region is a hole of it, one inside a hole is a
/// new region). Result is sorted by name. Points and construction entities are ignored.
/// Errors: none expected for a valid sketch; reserved for internal failures (ErrorCode::Internal).
Result<std::vector<SketchProfile>> detectProfiles(const Sketch& sketch);

/// Builds a planar kernel face (outer wire + hole wires) for `profile` on `plane`. Errors: KernelFailure.
Result<kernel::Shape> profileFace(const SketchPlane& plane, const SketchProfile& profile);

} // namespace iw::sketch
