// SPDX-License-Identifier: GPL-3.0-or-later
#include "sketch/solver/bridge.hpp"

#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>

#include "core/expr.hpp"

namespace iw::sketch {
namespace {

// Dimension-kind ids live above this bit so they never collide with constraint record ids.
constexpr std::uint64_t kDimensionIdBase = std::uint64_t{1} << 62;
constexpr std::uint64_t kMaxEntityValue = (std::uint64_t{1} << 60) - 1; // 4 * value must fit in 62 bits

SolverId entityBase(SketchEntityId id) { return id.value() * 4; }
SolverId constraintSolverId(SketchConstraintId id) { return id.value(); }
SolverId dimensionSolverId(SketchDimensionId id) { return kDimensionIdBase + id.value(); }

constexpr double kRadToDeg = 180.0 / std::numbers::pi;
constexpr double kTwoPi = 2.0 * std::numbers::pi;

/// Maps a sketch reference onto a solver reference. Error text is returned through `err`.
std::optional<SolverRef> mapRef(const Sketch& s, const SketchRef& ref, std::string& err) {
    const SketchEntity* e = s.findEntity(ref.entity);
    if (!e) {
        err = "reference to missing entity " + std::to_string(ref.entity.value());
        return std::nullopt;
    }
    const SolverId base = entityBase(ref.entity);
    switch (e->kind()) {
    case EntityKind::Point:
        return SolverRef{base, SolverRole::Start};
    case EntityKind::Line:
        if (!ref.role || *ref.role == PointRole::Start) return SolverRef{base, SolverRole::Start};
        if (*ref.role == PointRole::End) return SolverRef{base, SolverRole::End};
        if (*ref.role == PointRole::Mid) return SolverRef{base, SolverRole::Mid};
        break;
    case EntityKind::Circle:
        if (!ref.role || *ref.role == PointRole::Center) return SolverRef{base, SolverRole::Center};
        break;
    case EntityKind::Arc:
        if (!ref.role || *ref.role == PointRole::Start) return SolverRef{base, SolverRole::Start};
        if (*ref.role == PointRole::End) return SolverRef{base, SolverRole::End};
        if (*ref.role == PointRole::Center) return SolverRef{base, SolverRole::Center};
        break;
    }
    err = "entity " + std::to_string(ref.entity.value()) + " has no such sub-point in a constraint";
    return std::nullopt;
}

std::optional<SolverConstraintKind> mapConstraintKind(ConstraintKind k) {
    switch (k) {
    case ConstraintKind::Coincident: return SolverConstraintKind::Coincident;
    case ConstraintKind::Horizontal: return SolverConstraintKind::Horizontal;
    case ConstraintKind::Vertical: return SolverConstraintKind::Vertical;
    case ConstraintKind::Parallel: return SolverConstraintKind::Parallel;
    case ConstraintKind::Perpendicular: return SolverConstraintKind::Perpendicular;
    case ConstraintKind::Tangent: return SolverConstraintKind::Tangent;
    case ConstraintKind::Equal: return SolverConstraintKind::Equal;
    case ConstraintKind::Concentric: return SolverConstraintKind::Concentric;
    case ConstraintKind::Midpoint: return SolverConstraintKind::Midpoint;
    case ConstraintKind::Collinear: return SolverConstraintKind::Collinear;
    case ConstraintKind::Symmetric: return SolverConstraintKind::Symmetric;
    case ConstraintKind::Fix: return SolverConstraintKind::Fix;
    case ConstraintKind::PointOnObject: return SolverConstraintKind::PointOnObject;
    }
    return std::nullopt;
}

struct DimensionMapping {
    SolverConstraintKind kind;
    bool angle;  ///< true: value is an angle (solver takes degrees)
};

DimensionMapping mapDimensionKind(DimensionKind k) {
    switch (k) {
    case DimensionKind::Distance: return {SolverConstraintKind::Distance, false};
    case DimensionKind::DistanceX: return {SolverConstraintKind::DistanceX, false};
    case DimensionKind::DistanceY: return {SolverConstraintKind::DistanceY, false};
    case DimensionKind::Radius: return {SolverConstraintKind::Radius, false};
    case DimensionKind::Diameter: return {SolverConstraintKind::Diameter, false};
    case DimensionKind::Angle: return {SolverConstraintKind::Angle, true};
    }
    return {SolverConstraintKind::Distance, false};
}

/// Evaluates a dimension expression to the solver's unit: mm for lengths, degrees for angles. A bare number
/// (unitless) takes the expected unit. Returns the failure text on error.
std::optional<double> evaluateDimension(const SketchDimension& d, const ParameterTable* params, bool angle,
                                        std::string& err) {
    auto parsed = Expression::parse(d.expression);
    if (!parsed) {
        err = "dimension " + std::to_string(d.id.value()) + ": " + parsed.error().message;
        return std::nullopt;
    }
    ParameterLookup lookup = [params](std::string_view name) -> Result<Quantity> {
        if (!params) return makeError(ErrorCode::NotFound, "no parameter table (parameter '" + std::string(name) + "')");
        return params->evaluate(name);
    };
    auto value = parsed->evaluate(lookup);
    if (!value) {
        err = "dimension " + std::to_string(d.id.value()) + ": " + value.error().message;
        return std::nullopt;
    }
    const Quantity q = *value;
    if (angle) {
        if (q.dim == Dimension::Angle) return q.value * kRadToDeg;
        if (q.dim == Dimension::Unitless) return q.value;
    } else {
        if (q.dim == Dimension::Length) return q.value;
        if (q.dim == Dimension::Unitless) return q.value;
    }
    err = "dimension " + std::to_string(d.id.value()) + ": expression '" + d.expression + "' has dimension " +
          dimensionName(q.dim) + ", expected " + (angle ? "angle" : "length");
    return std::nullopt;
}

/// Builds the solver model for `s`. Returns an error message on malformed input.
std::optional<std::string> buildModel(const Sketch& s, const ParameterTable* params, SolverSketch& out) {
    for (const auto& [id, e] : s.entities()) {
        if (id.value() > kMaxEntityValue) return std::string("entity id out of range");
        const SolverId b = entityBase(id);
        std::visit(
            [&](const auto& g) {
                using G = std::decay_t<decltype(g)>;
                if constexpr (std::is_same_v<G, PointGeom>) {
                    out.points.push_back({b, g.p.x, g.p.y});
                } else if constexpr (std::is_same_v<G, LineGeom>) {
                    out.points.push_back({b + 1, g.start.x, g.start.y});
                    out.points.push_back({b + 2, g.end.x, g.end.y});
                    out.lines.push_back({b, b + 1, b + 2});
                } else if constexpr (std::is_same_v<G, CircleGeom>) {
                    out.points.push_back({b + 1, g.center.x, g.center.y});
                    out.circles.push_back({b, b + 1, g.radius});
                } else if constexpr (std::is_same_v<G, ArcGeom>) {
                    const Point2 st = arcStart(g), en = arcEnd(g);
                    out.points.push_back({b + 1, g.center.x, g.center.y});
                    out.points.push_back({b + 2, st.x, st.y});
                    out.points.push_back({b + 3, en.x, en.y});
                    out.arcs.push_back({b, b + 1, b + 2, b + 3});
                }
            },
            e.geometry);
    }

    std::string err;
    for (const auto& [id, c] : s.constraints()) {
        auto kind = mapConstraintKind(c.kind);
        if (!kind) return std::string("unknown constraint kind");
        SolverConstraint sc;
        sc.id = constraintSolverId(id);
        sc.kind = *kind;
        for (const SketchRef& r : c.refs) {
            auto m = mapRef(s, r, err);
            if (!m) return err;
            sc.refs.push_back(*m);
        }
        out.constraints.push_back(std::move(sc));
    }

    for (const auto& [id, d] : s.dimensions()) {
        const DimensionMapping m = mapDimensionKind(d.kind);
        auto value = evaluateDimension(d, params, m.angle, err);
        if (!value) return err;
        SolverConstraint sc;
        sc.id = dimensionSolverId(id);
        sc.kind = m.kind;
        sc.value = *value;
        for (const SketchRef& r : d.refs) {
            auto mr = mapRef(s, r, err);
            if (!mr) return err;
            sc.refs.push_back(*mr);
        }
        out.constraints.push_back(std::move(sc));
    }
    return std::nullopt;
}

/// Maps solver ids in a diagnostic list back to the sketch records.
void mapIds(const std::vector<SolverId>& ids, std::vector<SketchConstraintId>& constraints,
            std::vector<SketchDimensionId>& dimensions) {
    for (SolverId id : ids) {
        if (id >= kDimensionIdBase)
            dimensions.push_back(SketchDimensionId{id - kDimensionIdBase});
        else
            constraints.push_back(SketchConstraintId{id});
    }
}

SketchSolveReport runSolve(const Sketch& s, const ParameterTable* params, std::optional<SolverSketch>& solved) {
    SketchSolveReport report;
    SolverSketch model;
    if (auto bad = buildModel(s, params, model)) {
        report.status = SolverStatus::InvalidInput;
        report.message = std::move(*bad);
        return report;
    }
    SketchSolver solver(std::move(model));
    SolverResult r = solver.solve();
    report.status = r.status;
    report.message = r.message;
    report.dof = r.diagnostics.dof;
    mapIds(r.diagnostics.conflicting, report.conflictingConstraints, report.conflictingDimensions);
    mapIds(r.diagnostics.redundant, report.redundantConstraints, report.redundantDimensions);
    mapIds(r.diagnostics.partiallyRedundant, report.partiallyRedundantConstraints,
           report.partiallyRedundantDimensions);
    if (r.ok()) solved = solver.sketch();
    return report;
}

/// Writes solved coordinates back. Geometry is validated for every entity before any write.
std::optional<std::string> writeBack(Sketch& s, const SolverSketch& solved) {
    std::unordered_map<SolverId, Point2> pts;
    for (const SolverPoint& p : solved.points) pts[p.id] = {p.x, p.y};
    std::unordered_map<SolverId, double> radius;
    for (const SolverCircle& c : solved.circles) radius[c.id] = c.radius;

    std::vector<std::pair<SketchEntityId, EntityGeometry>> updates;
    for (const auto& [id, e] : s.entities()) {
        const SolverId b = entityBase(id);
        EntityGeometry g;
        if (e.kind() == EntityKind::Point) {
            g = PointGeom{pts.at(b)};
        } else if (e.kind() == EntityKind::Line) {
            g = LineGeom{pts.at(b + 1), pts.at(b + 2)};
        } else if (e.kind() == EntityKind::Circle) {
            g = CircleGeom{pts.at(b + 1), radius.at(b)};
        } else {
            ArcGeom a;
            a.center = pts.at(b + 1);
            const Point2 st = pts.at(b + 2), en = pts.at(b + 3);
            a.radius = distance(a.center, st);
            a.startAngle = std::atan2(st.y - a.center.y, st.x - a.center.x);
            const double ae = std::atan2(en.y - a.center.y, en.x - a.center.x);
            double sweep = std::fmod(ae - a.startAngle, kTwoPi);
            if (sweep < 0) sweep += kTwoPi;
            a.sweep = sweep;
            if (!(a.radius > 0) || !(a.sweep > 0)) return std::string("solved arc is degenerate");
            g = a;
        }
        updates.emplace_back(id, std::move(g));
    }
    // Apply. setGeometry keeps ids and kinds; the checks above cover the degenerate cases.
    for (auto& [id, g] : updates) {
        if (auto r = s.setGeometry(id, std::move(g)); !r)
            return "solved geometry rejected: " + r.error().message;
    }
    return std::nullopt;
}

} // namespace

SketchSolveReport solveSketch(Sketch& sketch, const ParameterTable* params) {
    std::optional<SolverSketch> solved;
    SketchSolveReport report = runSolve(sketch, params, solved);
    if (report.ok() && solved) {
        if (auto err = writeBack(sketch, *solved)) {
            report.status = SolverStatus::Failed;
            report.message = std::move(*err);
        }
    }
    return report;
}

SketchSolveReport diagnoseSketch(const Sketch& sketch, const ParameterTable* params) {
    std::optional<SolverSketch> solved;
    return runSolve(sketch, params, solved);
}

} // namespace iw::sketch
