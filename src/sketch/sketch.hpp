// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Sketch data model (ARCHITECTURE section 3, P1a contract "Sketch module"). A Sketch lives on a plane and holds
// entities (Point, Line, Arc, Circle) with stable ids, plus constraint and dimension RECORDS. Records are plain
// data: solving them is the solver's job (src/sketch/solver, module iw_sketch_solver). Dimension values are
// expression strings (evaluated by core/expr). This header is OCCT-free.

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "core/ids.hpp"
#include "core/math.hpp"
#include "core/result.hpp"

namespace iw::sketch {

struct SketchEntityIdTag;
struct SketchConstraintIdTag;
struct SketchDimensionIdTag;
/// Stable, never-reused id of a sketch entity (uint64 underneath).
using SketchEntityId = StrongId<SketchEntityIdTag>;
/// Stable, never-reused id of a constraint record.
using SketchConstraintId = StrongId<SketchConstraintIdTag>;
/// Stable, never-reused id of a dimension record.
using SketchDimensionId = StrongId<SketchDimensionIdTag>;

/// 2D point in sketch coordinates (mm).
struct Point2 {
    double x = 0, y = 0;
    friend constexpr bool operator==(const Point2&, const Point2&) = default;
    constexpr Point2 operator+(const Point2& o) const { return {x + o.x, y + o.y}; }
    constexpr Point2 operator-(const Point2& o) const { return {x - o.x, y - o.y}; }
    constexpr Point2 operator*(double s) const { return {x * s, y * s}; }
};
double distance(Point2 a, Point2 b) noexcept;

/// Sub-point roles addressable by constraints and dimensions.
enum class PointRole : std::uint8_t { Start, End, Center, Mid };

/// Reserved ids of the reference geometry every sketch holds (F46, Fusion parity): the projected ORIGIN point (sketch
/// (0, 0)) and the two sketch axes, construction lines through the origin along u and v. They are fixed in place and
/// never removable. They sit in an id range the allocator never reaches, so they never clash with document ids and
/// never move the id counter.
inline constexpr SketchEntityId kOriginPointId{(std::uint64_t{1} << 60) - 4};
inline constexpr SketchEntityId kXAxisId{(std::uint64_t{1} << 60) - 3};
inline constexpr SketchEntityId kYAxisId{(std::uint64_t{1} << 60) - 2};
/// Half length (mm) of the two axis lines.
inline constexpr double kReferenceAxisHalfLength = 10000.0;
/// True for kOriginPointId, kXAxisId and kYAxisId.
bool isReferenceEntity(SketchEntityId id) noexcept;

/// Sketch plane: origin, unit normal, unit x direction (mm). y direction = normal x xDir (right-handed).
struct SketchPlane {
    Vec3 origin{};
    Vec3 normal{0, 0, 1};
    Vec3 xDir{1, 0, 0};

    Vec3 yDir() const;
    /// Sketch (u, v) -> world (mm).
    Vec3 toWorld(Point2 p) const;
    /// World point -> sketch (u, v); the normal component is discarded.
    Point2 toLocal(const Vec3& p) const;
    /// Validated constructor: normal and xDir must be finite and non-zero, xDir is projected onto the plane
    /// and both are normalised. Errors: InvalidArgument.
    static Result<SketchPlane> make(const Vec3& origin, const Vec3& normal, const Vec3& xDir);
};

// ---- entity geometry ---------------------------------------------------------------------------------------
struct PointGeom {
    Point2 p;
};
struct LineGeom {
    Point2 start, end;
};
/// Counter-clockwise arc about `center` from `startAngle` (rad) over `sweep` (rad, 0 < sweep < 2*pi).
struct ArcGeom {
    Point2 center;
    double radius = 0;
    double startAngle = 0;
    double sweep = 0;
};
struct CircleGeom {
    Point2 center;
    double radius = 0;
};
using EntityGeometry = std::variant<PointGeom, LineGeom, ArcGeom, CircleGeom>;

enum class EntityKind : std::uint8_t { Point, Line, Arc, Circle };

/// A sketch entity. `construction` entities guide the sketch but never form profiles.
struct SketchEntity {
    SketchEntityId id;
    bool construction = false;
    EntityGeometry geometry;

    EntityKind kind() const noexcept;
};

Point2 arcStart(const ArcGeom& a) noexcept;
Point2 arcEnd(const ArcGeom& a) noexcept;
Point2 arcMid(const ArcGeom& a) noexcept;

// ---- constraint & dimension records ------------------------------------------------------------------------
/// A reference from a record to an entity, optionally to one of its sub-points. Valid roles per kind:
/// Point: Start (its position); Line: Start, End, Mid; Arc: Start, End, Center, Mid; Circle: Center.
/// `role == nullopt` references the whole entity (a line, an arc, a circle, a point).
struct SketchRef {
    SketchEntityId entity;
    std::optional<PointRole> role;
    friend bool operator==(const SketchRef&, const SketchRef&) = default;
};

enum class ConstraintKind : std::uint8_t {
    Coincident,
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Tangent,
    Equal,
    Concentric,
    Midpoint,
    Collinear,
    Symmetric,
    Fix,
    /// refs: [point, curve]. The point (entity point or sub-point) lies on the line, circle or arc (whole entity).
    PointOnObject,
};

struct SketchConstraint {
    SketchConstraintId id;
    ConstraintKind kind = ConstraintKind::Coincident;
    std::vector<SketchRef> refs;
};

enum class DimensionKind : std::uint8_t { Distance, DistanceX, DistanceY, Radius, Diameter, Angle };

struct SketchDimension {
    SketchDimensionId id;
    DimensionKind kind = DimensionKind::Distance;
    std::vector<SketchRef> refs;
    /// Unit-aware expression (A1 grammar), e.g. "2 in + 5 mm" or "d1 * 2". Evaluated by the owner, not here.
    std::string expression;
    /// Reference (driven) dimension: measured from the solved geometry, never solved for. Its expression is kept but
    /// ignored by the solver, so a driven dimension can never conflict or change the geometry.
    bool driven = false;
};

/// Editable 2D sketch on a plane. Ids are allocated by the sketch and never reused. All mutators validate their
/// input and return an error (InvalidArgument / NotFound) without changing the sketch on failure.
/// Every sketch starts with the reference geometry (kOriginPointId, kXAxisId, kYAxisId): construction entities that
/// cannot be removed or made non-construction. Records and entity lists always include them; persistence skips them.
class Sketch {
public:
    explicit Sketch(SketchPlane plane);

    const SketchPlane& plane() const noexcept { return plane_; }

    // ---- entities ----
    Result<SketchEntityId> addPoint(Point2 p, bool construction = false);
    /// Errors: InvalidArgument when the two ends coincide (length <= 1e-7 mm) or coordinates are not finite.
    Result<SketchEntityId> addLine(Point2 start, Point2 end, bool construction = false);
    /// Errors: InvalidArgument when radius <= 0 or sweep is not in (0, 2*pi).
    Result<SketchEntityId> addArc(Point2 center, double radius, double startAngle, double sweep,
                                  bool construction = false);
    Result<SketchEntityId> addCircle(Point2 center, double radius, bool construction = false);
    /// Replaces the geometry of an existing entity, keeping its id. The kind must not change. Used by the solver.
    Result<void> setGeometry(SketchEntityId id, EntityGeometry geometry);
    Result<void> setConstruction(SketchEntityId id, bool construction);
    /// Removes the entity and every constraint / dimension that references it. Errors: NotFound; InvalidArgument for
    /// reference geometry (isReferenceEntity), which is never removable.
    Result<void> removeEntity(SketchEntityId id);

    const SketchEntity* findEntity(SketchEntityId id) const noexcept;
    const std::map<SketchEntityId, SketchEntity>& entities() const noexcept { return entities_; }

    /// Position of a point reference (entity point or sub-point). Errors: NotFound, InvalidArgument (bad role).
    Result<Point2> pointOf(const SketchRef& ref) const;

    // ---- records ----
    /// Validates every ref (entity exists, role valid for its kind). Errors: NotFound, InvalidArgument.
    Result<SketchConstraintId> addConstraint(ConstraintKind kind, std::vector<SketchRef> refs);
    Result<SketchDimensionId> addDimension(DimensionKind kind, std::vector<SketchRef> refs, std::string expression,
                                           bool driven = false);
    Result<void> removeConstraint(SketchConstraintId id);
    Result<void> removeDimension(SketchDimensionId id);
    Result<void> setDimensionExpression(SketchDimensionId id, std::string expression);
    /// Turns a dimension into a driven (reference) dimension or back. Errors: NotFound.
    Result<void> setDimensionDriven(SketchDimensionId id, bool driven);
    /// Measured value of a dimension on the current geometry: mm for Distance / DistanceX / DistanceY / Radius /
    /// Diameter, radians for Angle (the unsigned angle between the two line directions, in [0, pi]).
    /// Errors: NotFound (dimension), InvalidArgument (references do not describe a measurable quantity, e.g. a
    /// zero-length line).
    Result<double> measureDimension(SketchDimensionId id) const;

    const std::map<SketchConstraintId, SketchConstraint>& constraints() const noexcept { return constraints_; }

    // ---- persistence: restore records with their ORIGINAL ids ----
    // Used when loading a document. Ids must be unused; the same validation as the add* calls applies. The id
    // counter is raised past each restored id, and restoreNextId sets it explicitly, so ids removed before saving
    // are never handed out again after a reload.

    /// Next id the sketch will allocate (the counter; every id below it is either live or retired).
    std::uint64_t nextId() const noexcept { return nextId_; }
    /// Restores an entity with id `id`. Errors: InvalidArgument (id invalid or in use, bad geometry). Reference ids
    /// are always in use (the constructor holds them), so they cannot be restored.
    Result<void> restoreEntity(SketchEntityId id, EntityGeometry geometry, bool construction = false);
    /// Restores a constraint record. Errors: InvalidArgument (id invalid or in use), NotFound / InvalidArgument (refs).
    Result<void> restoreConstraint(SketchConstraintId id, ConstraintKind kind, std::vector<SketchRef> refs);
    /// Restores a dimension record. Same errors as restoreConstraint.
    Result<void> restoreDimension(SketchDimensionId id, DimensionKind kind, std::vector<SketchRef> refs,
                                  std::string expression, bool driven = false);
    /// Sets the id counter. Errors: InvalidArgument if `next` is not greater than every live id, or would lower the
    /// counter (that would allow reuse of retired ids).
    Result<void> restoreNextId(std::uint64_t next);
    const std::map<SketchDimensionId, SketchDimension>& dimensions() const noexcept { return dimensions_; }

private:
    Result<void> checkRef(const SketchRef& ref) const;
    std::uint64_t allocateId() noexcept { return nextId_++; }
    bool idInUse(std::uint64_t id) const noexcept;

    SketchPlane plane_;
    std::uint64_t nextId_ = 1;
    std::map<SketchEntityId, SketchEntity> entities_;
    std::map<SketchConstraintId, SketchConstraint> constraints_;
    std::map<SketchDimensionId, SketchDimension> dimensions_;
};

} // namespace iw::sketch

template <> struct std::hash<iw::sketch::SketchEntityId> {
    std::size_t operator()(iw::sketch::SketchEntityId id) const noexcept {
        return std::hash<std::uint64_t>{}(id.value());
    }
};
