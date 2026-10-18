// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// B6 Inspect: Qt-free measurement, mass properties and section-plane model.
// Units: mm, mm^2, mm^3, degrees (angles in rows), kg (mass), kg*mm^2 (moments). Density in kg/m^3.
// Failures are returned as Result errors (KernelFailure / InvalidArgument / NotFound), never exceptions.

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/math.hpp"
#include "core/result.hpp"
#include "kernel/shape.hpp"

namespace iw::kernel::measure {

/// What a measurement is taken on. `Body` ignores the index.
enum class EntityKind : std::uint8_t { Vertex, Edge, Face, Body };

/// One pickable entity. For Vertex/Edge/Face, `index` addresses `body`'s sub-shapes (shape.hpp ordering contract).
struct Entity {
    EntityKind kind = EntityKind::Body;
    Shape body;
    std::uint32_t index = 0;
};

/// Closest pair between two entities: distance (mm) and the two witness points (mm, on A and on B).
struct Closest {
    double distance = 0;
    Vec3 onA;
    Vec3 onB;
};

/// Circle described by an edge whose curve is a circle (full or arc).
struct CircleInfo {
    Vec3 centre;
    Vec3 normal;
    double radius = 0;
    double diameter = 0;
};

/// Mass properties of a solid body. `density` and `mass` are zero when no density was given.
struct BodyProperties {
    double volume = 0;               ///< mm^3
    double surfaceArea = 0;          ///< mm^2
    Vec3 centreOfMass;               ///< mm
    Box3 boundingBox;                ///< mm
    double density = 0;              ///< kg/m^3 (input)
    double mass = 0;                 ///< kg = volume * density
    Vec3 principalMoments;           ///< kg*mm^2 about the centre of mass (sorted as OCCT reports them)
};

/// One displayable number. `unit` is a plain ASCII/UTF-8 unit suffix ("mm", "deg", "mm^2", "mm^3", "kg", "kg*mm^2").
struct MeasureRow {
    std::string label;
    double value = 0;
    std::string unit;
};

/// Minimum distance between two entities of any kind (vertex, edge, face, body), with the closest points.
/// Overlapping bodies report distance 0 with both points at the bounding-box centre of A (points not meaningful).
Result<Closest> minimumDistance(const Entity& a, const Entity& b);

/// Angle in degrees, in [0, 180], between two planar faces (angle between the outward normals).
/// Two linear edges: angle between their directions, in [0, 90] (lines are unoriented).
/// Linear edge and planar face: angle between the edge and the face plane, in [0, 90] (0 = edge lies in the plane).
/// Errors: InvalidArgument for non-planar faces, non-linear edges or unsupported kinds.
Result<double> angleBetween(const Entity& a, const Entity& b);

/// Coordinates of a vertex entity (mm). InvalidArgument for other kinds.
Result<Vec3> vertexPoint(const Entity& vertex);

/// Length of a linear edge entity (any curve type), mm.
Result<double> edgeLength(const Entity& edge);

/// Area of a face entity, mm^2.
Result<double> faceArea(const Entity& face);

/// Circle parameters of an edge entity whose curve is a circle. InvalidArgument otherwise.
Result<CircleInfo> circleOf(const Entity& edge);

/// Volume, surface area, centre of mass, bounding box, mass (volume * density) and principal moments of a solid.
/// `densityKgPerM3` must be >= 0. InvalidArgument for a null shape or a shape without solids.
Result<BodyProperties> bodyProperties(const Shape& body, double densityKgPerM3 = 0.0);

/// Rows describing one entity on its own (vertex: X/Y/Z; edge: length, circle data; face: area; body: volume, area).
Result<std::vector<MeasureRow>> describe(const Entity& e);

/// Rows for the relation between two entities: minimum distance, closest points (X/Y/Z on A and on B), and angle
/// when the pair supports one. Angle rows appear only when `angleBetween` succeeds.
Result<std::vector<MeasureRow>> between(const Entity& a, const Entity& b);

// ---------------------------------------------------------------------------------------------------------------
// Section analysis (Qt-free model; the viewport consumes RenderRequest).

namespace section {

/// Render request the viewport can consume. Fragments with `clipPlane` dot (x,y,z,1) < 0 are discarded
/// (the kept side is the one the plane normal points to). When `enabled` is false no clipping is applied.
/// Capping: draw back faces with `capColour` where clipped, which reads as a solid cross-section.
struct RenderRequest {
    bool enabled = false;
    std::array<double, 4> clipPlane{0, 0, 1, 0}; ///< (nx, ny, nz, d), unit normal, mm
    std::array<float, 4> capColour{1.0f, 0.55f, 0.1f, 1.0f}; ///< RGBA 0..1
    friend bool operator==(const RenderRequest&, const RenderRequest&) = default;
};

/// Plane by origin + normal + offset, with a flip toggle.
///  * `offset` moves the plane along the *unflipped* normal (mm).
///  * `flipped` reverses which side is kept.
/// Effective plane point P = origin + offset * normal; effective normal N = flipped ? -normal : normal.
class Model {
public:
    /// Errors: InvalidArgument for a zero or non-finite normal.
    Result<void> setPlane(const Vec3& origin, const Vec3& normal);
    void setOffset(double mm) noexcept { offset_ = mm; }
    void flip() noexcept { flipped_ = !flipped_; }
    void setFlipped(bool f) noexcept { flipped_ = f; }
    void setEnabled(bool on) noexcept { enabled_ = on; }
    void setCapColour(std::array<float, 4> rgba) noexcept { cap_ = rgba; }

    const Vec3& origin() const noexcept { return origin_; }
    const Vec3& normal() const noexcept { return normal_; }
    double offset() const noexcept { return offset_; }
    bool flipped() const noexcept { return flipped_; }
    bool enabled() const noexcept { return enabled_; }

    /// Effective normal (after flip) and point (after offset).
    Vec3 effectiveNormal() const noexcept;
    Vec3 effectivePoint() const noexcept;
    /// Signed distance of `p` from the effective plane (positive = kept side).
    double signedDistance(const Vec3& p) const noexcept;

    RenderRequest request() const noexcept;

private:
    Vec3 origin_;
    Vec3 normal_{0, 0, 1};
    double offset_ = 0;
    bool flipped_ = false;
    bool enabled_ = false;
    std::array<float, 4> cap_{1.0f, 0.55f, 0.1f, 1.0f};
};

} // namespace section

} // namespace iw::kernel::measure
