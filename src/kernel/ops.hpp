// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "core/cancel.hpp"
#include "core/math.hpp"
#include "core/result.hpp"
#include "kernel/history.hpp"
#include "kernel/shape.hpp"

/// Modelling operations. Common contract for all functions here:
///  * Inputs are never modified (Shape is immutable); results are new Shapes.
///  * Failure -> ErrorCode::KernelFailure / InvalidArgument, never an exception.
///  * Thread-safety: UNVERIFIED for concurrent calls (see matrix in THREAD_SAFETY.md). Until the
///    matrix is filled in, callers must treat these as safe only for *disjoint* inputs on distinct threads.
///  * `CancelToken` is polled between internal stages where the kernel allows; granularity is coarse.
namespace iw::kernel {

/// Axis-aligned box with its minimum corner at `origin`, extents (dx,dy,dz) mm, all > 0.
Result<ShapeResult> makeBox(const Vec3& origin, double dx, double dy, double dz);

/// Cylinder with base-centre `origin`, axis direction `axis` (non-zero, normalised internally),
/// `radius` > 0 and `height` > 0 (mm).
Result<ShapeResult> makeCylinder(const Vec3& origin, const Vec3& axis, double radius, double height);

/// Sphere centred at `center` with `radius` > 0 (mm). Non-finite centre or radius <= 0 -> InvalidArgument.
/// Single solid bounded by one spherical face; its two poles are degenerate edges of the parametrisation.
/// Volume = 4/3 pi r^3, surface area = 4 pi r^2.
Result<ShapeResult> makeSphere(const Vec3& center, double radius);

/// Extrudes a planar `profile` (a closed planar Wire -> solid, or a planar Face -> solid) along `vector`
/// (mm; non-zero; need not be perpendicular to the plane). History (operand 0): profile edge ->
/// generated side face; profile face -> modified start-cap face; end cap is generated from the face.
Result<ShapeResult> extrudeProfile(const Shape& profile, const Vec3& vector, CancelToken cancel = {});

/// Boolean union / subtraction (a minus b) / intersection of two solids. Operand 0 = a, 1 = b.
/// History follows OCCT's BOPAlgo history. Failure (e.g. non-manifold, tolerance) -> KernelFailure.
Result<ShapeResult> fuse(const Shape& a, const Shape& b, CancelToken cancel = {});
Result<ShapeResult> cut(const Shape& a, const Shape& b, CancelToken cancel = {});
Result<ShapeResult> common(const Shape& a, const Shape& b, CancelToken cancel = {});

/// Volume / area / centre of mass / inertia of a shape, assuming unit density (scale by density yourself).
struct MassProperties {
    double volume = 0;                       ///< mm^3 (solids)
    double surfaceArea = 0;                  ///< mm^2
    Vec3 centerOfMass;                       ///< mm
    std::array<double, 9> inertia{};         ///< row-major 3x3 about the centre of mass, unit density, mm^5
};
Result<MassProperties> massProperties(const Shape& shape);

// ---------------------------------------------------------------------------------------------------------------
// P1a additions (A4). Everything below is additive; the functions above are unchanged.
// Sub-shape indices in these APIs are 0-based indices into the INPUT shape's ordering contract (shape.hpp).

/// Infinite-plane placement in mm. `normal` and `xDir` are unit vectors, orthogonal to each other.
struct Plane {
    Vec3 origin;
    Vec3 normal{0, 0, 1};
    Vec3 xDir{1, 0, 0};
};

/// Infinite line in mm. `direction` is a unit vector.
struct Axis {
    Vec3 origin;
    Vec3 direction{0, 0, 1};
};

/// Plane through `origin` with the given normal (non-zero, finite). xDir is derived deterministically from the
/// normal (the world axis least aligned with it, projected) so the same normal always yields the same frame.
Result<Plane> planeFromPointNormal(const Vec3& origin, const Vec3& normal);
/// Plane through three non-collinear points. normal = (b-a) x (c-a) normalised; xDir = normalised (b-a).
Result<Plane> planeFromThreePoints(const Vec3& a, const Vec3& b, const Vec3& c);
/// Plane of a PLANAR face of `shape` (face index per the ordering contract). The normal follows the face's
/// orientation (reversed faces flip the normal, i.e. it points out of the solid for a solid's boundary face).
/// Non-planar face -> InvalidArgument; out-of-range index -> InvalidArgument.
Result<Plane> planeFromFace(const Shape& shape, std::uint32_t faceIndex);
/// Axis through two distinct points, direction from `a` to `b`.
Result<Axis> axisFromPoints(const Vec3& a, const Vec3& b);
/// Axis along a LINEAR edge of `shape` (edge index per the ordering contract), oriented start -> end
/// following the edge's orientation. Non-linear edge -> InvalidArgument; out-of-range index -> InvalidArgument.
Result<Axis> axisFromEdge(const Shape& shape, std::uint32_t edgeIndex);

/// Revolves a planar `profile` (a planar Face, or a closed planar Wire which is converted to a face) about `axis`
/// by `angle` radians, 0 < angle <= 2*pi. The profile must not cross the axis (result must have positive volume).
/// History (operand 0 = profile): profile edge -> generated face; for a face profile the start cap is modified
/// and the end cap generated (partial revolve); vertices -> generated edges.
Result<ShapeResult> revolve(const Shape& profile, const Axis& axis, double angle, CancelToken cancel = {});

/// Constant-radius fillet of the given edges of a SOLID. Edge indices are de-duplicated; empty list or out-of-range
/// index -> InvalidArgument. If OCCT cannot build the fillet, the failing edge is identified and reported as
/// KernelFailure ("fillet radius too large for edge N ..."). History (operand 0): filleted edge -> generated face;
/// adjacent faces -> modified.
Result<ShapeResult> filletEdges(const Shape& shape, std::span<const std::uint32_t> edgeIndices, double radius,
                                CancelToken cancel = {});

/// Asymmetric chamfer with distances d1 (measured on the adjacent face with the lowest face index) and d2
/// (measured on the other adjacent face). Both > 0. Failure reporting as filletEdges.
Result<ShapeResult> chamferEdges(const Shape& shape, std::span<const std::uint32_t> edgeIndices, double d1, double d2,
                                 CancelToken cancel = {});
/// Chamfer defined by `distance` on the lowest-indexed adjacent face and `angle` (radians, 0 < angle < pi, measured
/// from that face) to the other face.
Result<ShapeResult> chamferEdgesByAngle(const Shape& shape, std::span<const std::uint32_t> edgeIndices, double distance,
                                        double angle, CancelToken cancel = {});

/// Hole specification. Kinds:
///  * Simple: cylinder of `diameter`, depth `depth` (or through), drill point of `drillPointAngle` (full included
///    angle, 0 < a <= pi; pi = flat bottom). Through holes have no drill point.
///  * Counterbore: additionally a counterbore of `counterboreDiameter` (> diameter) and `counterboreDepth`
///    (0 < cbDepth < depth) at the entry.
///  * Countersink: additionally a countersink of `countersinkDiameter` (> diameter), full included angle
///    `countersinkAngle` (0 < a < pi) at the entry.
enum class HoleKind : std::uint8_t { Simple, Counterbore, Countersink };
struct HoleSpec {
    HoleKind kind = HoleKind::Simple;
    double diameter = 0;              ///< mm
    double depth = 0;                 ///< mm, measured from the entry plane; ignored when `through`
    bool through = false;
    double drillPointAngle = 2.0943951023931953; ///< rad (118 deg); Simple/Counterbore/Countersink, below the hole
    double counterboreDiameter = 0;   ///< mm (Counterbore)
    double counterboreDepth = 0;      ///< mm (Counterbore)
    double countersinkDiameter = 0;   ///< mm (Countersink)
    double countersinkAngle = 0;      ///< rad, full included angle (Countersink)
};

/// Drills a hole into `shape` (a SOLID). `placement.origin` is the entry point on the surface, `placement.direction`
/// points INTO the material (for a face use -planeFromFace(...).normal). Implemented as BRepAlgoAPI_Cut with a
/// revolved tool. History: operand 0 = target shape, operand 1 = the hole tool (its surviving faces are the hole walls).
Result<ShapeResult> hole(const Shape& shape, const Axis& placement, const HoleSpec& spec, CancelToken cancel = {});

/// Rigid transform (rotation + translation) of `shape`. `matrix` must be affine with last row (0,0,0,1), orthonormal
/// linear part with determinant +1; otherwise InvalidArgument. Topology is preserved: history maps input sub-shape i
/// to output sub-shape i (kind "modified" for every element).
Result<ShapeResult> transform(const Shape& shape, const Transform& matrix, CancelToken cancel = {});

/// Tangent-continuous (G1) expansion of the given edges: returns the input edges plus every edge reachable from them
/// through vertices where exactly the tangent directions meet (angle between the edges' outgoing tangents within 1e-4 rad
/// of pi). Output: unique indices, ascending. Empty or out-of-range input -> InvalidArgument.
Result<std::vector<std::uint32_t>> tangentChain(const Shape& shape, std::span<const std::uint32_t> edgeIndices,
                                                CancelToken cancel = {});

/// Mirror of `shape` across `plane` (reflection through the plane; `plane.normal` need not be unit). Topology is
/// preserved: history maps input sub-shape i to output sub-shape i (kind "modified"), as for transform(). Solids come
/// out with positive volume. Null shape, zero normal or non-finite input -> InvalidArgument.
Result<ShapeResult> mirror(const Shape& shape, const Plane& plane, CancelToken cancel = {});

/// Uniform scale of `shape` about `center` by `factor` (finite, > 0). Topology preserved; history as mirror().
/// Non-finite input or factor <= 0 -> InvalidArgument.
Result<ShapeResult> scale(const Shape& shape, const Vec3& center, double factor, CancelToken cancel = {});

/// Intersection of two solids; same as common(a, b). Provided under the P1a contract name.
Result<ShapeResult> intersectWith(const Shape& a, const Shape& b, CancelToken cancel = {});

} // namespace iw::kernel
