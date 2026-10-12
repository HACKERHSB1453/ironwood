// SPDX-License-Identifier: GPL-3.0-or-later
// Constraint solver for 2D sketches. Adapter over the vendored FreeCAD planegcs library
// (LGPL-2.0-or-later, see third_party/planegcs/README.md).
//
// The input model is defined here, independent of the sketch document types. Mapping the document
// Sketch onto SolverSketch is the job of the sketch feature (B1).
//
// Conventions:
//   - All lengths are mm. Angles are degrees (Angle constraint only).
//   - Every entity id (point, line, circle, arc) lives in one id space and must be unique.
//     Constraint ids live in their own space and must be unique among constraints.
//   - Lines, circles and arcs refer to SolverPoint ids. An arc runs counter-clockwise from `start`
//     to `end` around `center`.
//   - A solve either writes a fully consistent solution back into the sketch, or leaves the sketch
//     untouched and reports the conflicting / redundant constraint ids.
#ifndef IW_SKETCH_SOLVER_SKETCH_SOLVER_HPP
#define IW_SKETCH_SOLVER_SKETCH_SOLVER_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace iw::sketch {

using SolverId = std::uint64_t;

// Sub-point of an entity. Start/End: line or arc endpoints. Center: circle or arc centre.
// Mid: midpoint of a line. For a point entity the role is ignored.
enum class SolverRole : std::uint8_t { Start, End, Center, Mid };

struct SolverRef {
    SolverId entity{};
    SolverRole role = SolverRole::Start;
};

struct SolverPoint {
    SolverId id{};
    double x = 0.0;
    double y = 0.0;
};

struct SolverLine {
    SolverId id{};
    SolverId start{};  // SolverPoint id
    SolverId end{};    // SolverPoint id
};

struct SolverCircle {
    SolverId id{};
    SolverId center{};  // SolverPoint id
    double radius = 0.0;
};

struct SolverArc {
    SolverId id{};
    SolverId center{};  // SolverPoint id
    SolverId start{};   // SolverPoint id
    SolverId end{};     // SolverPoint id
};

enum class SolverConstraintKind : std::uint8_t {
    // refs: 2 points (or sub-points). Mid on either side means the other is the midpoint of a line.
    Coincident,
    // refs: 1 line, or 2 points.
    Horizontal,
    Vertical,
    // refs: 2 lines.
    Parallel,
    Perpendicular,
    // refs: 2 curves (line, circle or arc). Line-line is not a valid tangency.
    Tangent,
    // refs: 2 lines (equal length) or 2 circles/arcs (equal radius).
    Equal,
    // refs: 2 curves (circle or arc). Centres coincide.
    Concentric,
    // refs: point, line. The point is the midpoint of the line.
    Midpoint,
    // refs: 2 lines. Both endpoints of the first lie on the second.
    Collinear,
    // refs: point, point, axis. Axis is a line (mirror) or a point (point reflection).
    Symmetric,
    // refs: 1 entity. Fixes the whole entity at its current position (point: x and y;
    // line: both endpoints; circle: centre and radius; arc: centre, endpoints and radius).
    Fix,
    // refs: 2 points. value: distance (mm).
    Distance,
    // refs: 2 points. value: signed x (or y) difference, second minus first (mm).
    DistanceX,
    DistanceY,
    // refs: 1 circle or arc. value: radius (mm).
    Radius,
    // refs: 1 circle or arc. value: diameter (mm).
    Diameter,
    // refs: 2 lines. value: angle in degrees.
    Angle,
};

struct SolverConstraint {
    SolverId id{};
    SolverConstraintKind kind{SolverConstraintKind::Coincident};
    std::vector<SolverRef> refs;
    double value = 0.0;  // used by Distance, DistanceX/Y, Radius, Diameter, Angle; ignored otherwise
};

struct SolverSketch {
    std::vector<SolverPoint> points;
    std::vector<SolverLine> lines;
    std::vector<SolverCircle> circles;
    std::vector<SolverArc> arcs;
    std::vector<SolverConstraint> constraints;
};

enum class SolverStatus : std::uint8_t {
    Solved,        // a consistent solution was found and written back to the sketch
    Failed,        // no consistent solution: the diagnostics name the offending constraints
    InvalidInput,  // malformed model (unknown ids, wrong number or kind of refs); see message
};

struct SolverDiagnostics {
    // Degrees of freedom: unknown coordinates minus independent constraint equations.
    // 0 means fully constrained. -1 when no diagnosis was run.
    int dof = -1;
    // Constraint ids that conflict with each other (the system has no solution with them all).
    std::vector<SolverId> conflicting;
    // Constraint ids that repeat a constraint already implied by the others.
    std::vector<SolverId> redundant;
    // Constraint ids that are partly redundant (only some of their equations are implied).
    std::vector<SolverId> partiallyRedundant;
};

struct SolverResult {
    SolverStatus status = SolverStatus::Failed;
    SolverDiagnostics diagnostics;
    std::string message;  // human-readable detail for InvalidInput / Failed

    [[nodiscard]] bool ok() const { return status == SolverStatus::Solved; }
};

class SketchSolver {
public:
    explicit SketchSolver(SolverSketch sketch = {});
    ~SketchSolver();
    SketchSolver(const SketchSolver& other);
    SketchSolver& operator=(const SketchSolver& other);
    SketchSolver(SketchSolver&& other) noexcept;
    SketchSolver& operator=(SketchSolver&& other) noexcept;

    // The model. Edit values (e.g. a dimension) here, then call solve(). Geometry is written back
    // by solve() and dragPoint() on success.
    [[nodiscard]] SolverSketch& sketch() { return sketch_; }
    [[nodiscard]] const SolverSketch& sketch() const { return sketch_; }

    // Solves all constraints. On Solved the points and circle/arc radii in sketch() are updated;
    // otherwise the sketch is unchanged. Diagnostics are always filled (for InvalidInput: empty).
    //
    // Performance: the solver keeps its internal model between calls. Changing values (positions,
    // dimension values, radii) and solving again is fast. Adding or removing entities or
    // constraints, or changing their kinds or references, rebuilds the model (slower).
    SolverResult solve();

    // Moves `point` to (x, y) while keeping all constraints. The point ends at the target when the
    // constraints allow it, and otherwise at the nearest consistent position. Solved: the sketch
    // is updated. Failed: the sketch is unchanged. Diagnostics are not computed here (call
    // diagnose() when needed). Consecutive calls on the same point reuse the model, so a drag
    // session is cheap per step.
    SolverResult dragPoint(SolverRef point, double x, double y);

    // Degrees of freedom and conflict/redundancy diagnostics of the current model. Does not change
    // the sketch.
    [[nodiscard]] SolverDiagnostics diagnose();

    // Shorthand for diagnose().dof.
    [[nodiscard]] int dof();

private:
    struct Impl;
    SolverSketch sketch_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace iw::sketch

#endif  // IW_SKETCH_SOLVER_SKETCH_SOLVER_HPP
