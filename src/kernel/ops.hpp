// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

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

} // namespace iw::kernel
