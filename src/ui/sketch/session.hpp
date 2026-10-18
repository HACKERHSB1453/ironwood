// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// SketchSession: the Qt-free model/controller behind sketch mode (B3a parts 1-2). It owns one sketch::Sketch and the
// undo history of the edits made to it. Every edit goes through apply(): the edit is run on a copy, the copy is
// solved with the solver bridge (solveSketch), and only a Solved result is pushed as one undoable command. Tools
// turn clicks in sketch coordinates into edits. No widgets, no OCCT: the overlay and the shell translate screen input
// into Point2 values.
//
// Inference (part 2): when enabled, creation tools snap the click to nearby geometry (see SnapKind for the priority
// order) and add the constraints the snap implies. Auto-constraints are tried first; when the solve rejects them the
// edit is repeated without them, so inference never turns a legal click into an error.

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/expr.hpp"
#include "core/params.hpp"
#include "core/result.hpp"
#include "core/undo.hpp"
#include "sketch/profile.hpp"
#include "sketch/sketch.hpp"
#include "sketch/solver/bridge.hpp"

namespace iw::ui {

enum class SketchTool : std::uint8_t {
    Select, Line, Rectangle, CenterRectangle, Circle, Point, Arc, Trim, Offset,
    Dimension   ///< D: pick geometry, then click where the dimension text goes (see clickDimension rules below).
};

/// Solver state of the active sketch as the overlay and the palette show it.
enum class ConstraintState : std::uint8_t {
    Under,        ///< Degrees of freedom left (or nothing drawn yet): curves are blue.
    Fully,        ///< DOF 0 with geometry: curves are black on a light theme, white on a dark one.
    Conflicting,  ///< The solver found conflicting or redundant constraints: those are drawn red.
};

/// A dimension that does not drive the geometry ("reference"/driven): it only reports the measured value. The sketch
/// document has no driven flag yet, so these live in the session (and its undo history) only.
struct DrivenDimension {
    std::uint64_t id = 0;
    sketch::DimensionKind kind = sketch::DimensionKind::Distance;
    std::vector<sketch::SketchRef> refs;
};

/// What the overlay draws for one dimension (driving or driven).
struct DimensionView {
    std::uint64_t id = 0;            ///< SketchDimensionId value, or DrivenDimension::id when `driven`.
    bool driven = false;
    sketch::DimensionKind kind = sketch::DimensionKind::Distance;
    std::vector<sketch::SketchRef> refs;
    std::string expression;          ///< Empty for driven dimensions.
    double value = 0;                ///< mm, or degrees for Angle. NaN when the expression cannot be evaluated.
    std::string text;                ///< "40", "w = 40", "(40)" (driven), "?" ...
    bool conflicting = false;        ///< In the solver's conflicting or redundant list.
};

/// One constraint glyph near its geometry. Screen position = projected `anchor` + `slot` * 14 px to the right.
struct ConstraintGlyph {
    sketch::SketchConstraintId id;
    sketch::ConstraintKind kind = sketch::ConstraintKind::Coincident;
    sketch::Point2 anchor{};
    int slot = 0;                    ///< Index among glyphs sharing the same anchor.
    std::string label;               ///< "H", "V", "//", ... (empty for coincident: drawn as a ring).
    bool conflicting = false;
};

/// Outcome of the Dimension tool's last placement click.
struct DimensionEvent {
    enum class Type : std::uint8_t { Created, OverConstrains } type = Type::Created;
    std::uint64_t id = 0;            ///< Created: the new SketchDimensionId value.
    sketch::DimensionKind kind = sketch::DimensionKind::Distance;
    std::vector<sketch::SketchRef> refs;   ///< OverConstrains: what the user wanted (for addDrivenDimension).
    std::string message;
};

/// What a snap attached to. Priority (highest first): Endpoint, Center, Midpoint, Intersection, Tangent,
/// Perpendicular, OnCurve, then Horizontal/Vertical alignment. Within one priority the nearest candidate wins.
enum class SnapKind : std::uint8_t {
    Endpoint,      ///< Existing line/arc end or point. Constraint: coincident with the new point.
    Center,        ///< Circle or arc centre. Constraint: coincident with the centre.
    Midpoint,      ///< Line midpoint (arc midpoint: no constraint). Constraint: Midpoint on a line.
    Intersection,  ///< Crossing of two curves. Constraint: PointOnObject on both curves.
    Tangent,       ///< Tangent point of a circle/arc seen from the pending start. Constraints: Tangent on the new line
                   ///< and PointOnObject of its end on the circle/arc.
    Perpendicular, ///< Foot of a line perpendicular to an existing line from the pending start. Constraint: Perpendicular.
    OnCurve,       ///< Nearest point on a curve. Constraint: PointOnObject.
    Horizontal,    ///< Same y as an existing point. Dashed guide; the segment becomes exactly horizontal.
    Vertical,      ///< Same x as an existing point. Dashed guide; the segment becomes exactly vertical.
};

/// One snap result. `ref` is the referenced sub-point (Endpoint, Center), the line (Midpoint) or the first curve
/// (Intersection, OnCurve). `other` is the second curve (Intersection) or the target curve (Tangent, Perpendicular).
struct SnapHit {
    SnapKind kind = SnapKind::OnCurve;
    sketch::Point2 point{};
    std::optional<sketch::SketchRef> ref;
    std::optional<sketch::SketchRef> other;
    std::vector<std::pair<sketch::Point2, sketch::Point2>> guides;   ///< Dashed guide lines to draw.
};

struct InferenceSettings {
    bool enabled = true;     ///< Off: drawing is literal, nothing snaps, nothing is constrained automatically.
    double radiusMm = 0.5;   ///< Snap radius in sketch mm (the shell converts pixels with view_math).
};

class SketchSession {
public:
    explicit SketchSession(sketch::SketchPlane plane);
    // The undo commands point at the sketch member: the session must not move.
    SketchSession(const SketchSession&) = delete;
    SketchSession& operator=(const SketchSession&) = delete;
    ~SketchSession();

    const sketch::Sketch& sketch() const noexcept { return sketch_; }
    const sketch::SketchPlane& plane() const noexcept { return sketch_.plane(); }

    // ---- operations: each is one undoable edit; on failure the sketch and the undo stack are unchanged ----
    /// One line. Errors: InvalidArgument (zero length, non-finite).
    Result<sketch::SketchEntityId> addLine(sketch::Point2 a, sketch::Point2 b);
    /// Four lines (bottom, right, top, left) with 4 coincident + 2 horizontal + 2 vertical constraints. The
    /// returned ids are in that order. Errors: InvalidArgument (degenerate corners).
    Result<std::array<sketch::SketchEntityId, 4>> addRect2pt(sketch::Point2 a, sketch::Point2 b);
    /// Rectangle centred on `center` through `corner` (same constraints as addRect2pt).
    Result<std::array<sketch::SketchEntityId, 4>> addRectCenter(sketch::Point2 center, sketch::Point2 corner);
    /// Circle. Errors: InvalidArgument (radius <= 0).
    Result<sketch::SketchEntityId> addCircleCenter(sketch::Point2 center, double radius);
    Result<sketch::SketchEntityId> addPoint(sketch::Point2 p);
    /// Circular arc through three points (start, a point on the arc, end). Errors: InvalidArgument (collinear).
    Result<sketch::SketchEntityId> addArc3(sketch::Point2 a, sketch::Point2 through, sketch::Point2 b);
    /// Flips the construction flag of an entity. Errors: NotFound.
    Result<void> toggleConstruction(sketch::SketchEntityId id);
    /// Trim: removes the piece of the nearest line or arc to `p` that lies between the nearest intersections with
    /// other curves (the whole entity when it has none). The surviving piece that touches the original start keeps
    /// the id; other pieces get new ids. Constraints move to the pieces they still describe; the rest are dropped.
    /// A circle becomes an arc: the piece between the two crossings nearest to `p` is removed (a circle needs two
    /// crossings, or none to be removed whole). The circle's id goes; the arc gets a new id and inherits the centre
    /// constraints, Equal, Radius and Diameter dimensions.
    /// Errors: InvalidArgument (nothing within tolerance, points, a circle with a single crossing), solver errors too.
    Result<void> trimAt(sketch::Point2 p, double pickToleranceMm = 0.5);
    /// Offset of the closed profile loop that contains entity `seed` by `distanceMm`. Positive grows the area the loop
    /// encloses (outward for an outer loop, a larger hole for a hole loop), negative shrinks it. Returns the new
    /// entity ids. Errors: NotFound (not on a closed profile), InvalidArgument (loop contains split or point
    /// entities, zero distance, or the offset collapses an edge).
    Result<std::vector<sketch::SketchEntityId>> offsetLoop(sketch::SketchEntityId seed, double distanceMm);
    /// Offset tool click: offsetLoop on the nearest entity within tolerance, with offsetDistance().
    Result<std::vector<sketch::SketchEntityId>> offsetAt(sketch::Point2 p, double pickToleranceMm = 0.5);

    // ---- inference ----
    void setInference(InferenceSettings s);
    InferenceSettings inference() const noexcept { return inference_; }
    /// Best snap for `p` given the current sketch and pending state. nullopt when inference is off or nothing is near.
    std::optional<SnapHit> snapAt(sketch::Point2 p) const;
    /// Snap of the current cursor (computed by setCursor while inference is on). Drives the glyph and the preview.
    const std::optional<SnapHit>& snapHint() const noexcept { return snapHint_; }
    /// Last used offset distance: the default of the offset dialog, and what offsetAt uses.
    void setOffsetDistance(double mm) noexcept { offsetDistance_ = mm; }
    double offsetDistance() const noexcept { return offsetDistance_; }
    /// The entity an Offset click at `p` would use as seed (nearest within tolerance).
    std::optional<sketch::SketchEntityId> offsetSeedAt(sketch::Point2 p, double pickToleranceMm = 0.5) const;

    // ---- parameters ----
    /// Parameter table dimension expressions are evaluated against (nullptr: literals only). Not owned; must outlive
    /// the session or be reset. Setting it does not re-solve: call resolve() after the table changed.
    void setParameters(const ParameterTable* table) noexcept { params_ = table; }
    const ParameterTable* parameters() const noexcept { return params_; }
    /// Re-solves with the current parameter values. One undo step "Update Parameters" when the geometry moved.
    /// Errors: InvalidArgument (a dimension no longer evaluates or the constraints conflict); the sketch is kept.
    Result<void> resolve();
    // ---- tangent arc from the end of a line (Line tool drag) ----
    /// True when the Line tool has a pending polyline whose end is within `tolMm` of `p`.
    bool canTangentArcAt(sketch::Point2 p, double tolMm) const;
    /// While dragging: the point under the pointer; the preview shows the tangent arc to it. nullopt ends the drag.
    void setTangentDrag(std::optional<sketch::Point2> p) noexcept { tangentDrag_ = p; }
    std::optional<sketch::Point2> tangentDrag() const noexcept { return tangentDrag_; }
    /// Creates the tangent arc from the pending polyline end to `end` (snapped when inference is on), tangent to the
    /// last segment. Errors: InvalidArgument (no tangent continuation, end on the tangent line).
    Result<sketch::SketchEntityId> tangentArcTo(sketch::Point2 end);

    // ---- tools ----
    SketchTool tool() const noexcept { return tool_; }
    /// Switching tool drops any half-placed shape. An open polyline is ended, its segments kept.
    void setTool(SketchTool t);
    /// A click at sketch point `p` with the active tool. Select picks the nearest entity within `pickToleranceMm`.
    /// Line: first click starts a polyline, each further click adds a segment joined to the previous one.
    /// Rectangle / CenterRectangle / Circle: first click places the corner / centre, the second completes the shape.
    /// Arc: start, a point on the arc, end. Point: one click. Trim / Offset: see trimAt / offsetAt.
    Result<void> click(sketch::Point2 p, double pickToleranceMm = 0.5);
    /// Ends the open polyline (double click, Enter). Nothing else changes.
    void endPolyline();
    /// Esc. Drops a half-placed shape and ends an open polyline, then returns to Select. Returns false when there was
    /// nothing to cancel (Select tool, nothing pending): the caller then leaves sketch mode.
    bool cancel();
    /// Last known cursor position in sketch coordinates (drives the rubber-band preview and the snap glyph).
    void setCursor(std::optional<sketch::Point2> p);
    /// Rubber-band geometry of the pending shape (empty when nothing is pending).
    std::vector<sketch::EntityGeometry> preview() const;
    bool hasPending() const noexcept;

    // ---- selection ----
    /// The first selected entity (X toggles its construction flag).
    std::optional<sketch::SketchEntityId> selected() const noexcept {
        if (selection_.empty()) return std::nullopt;
        return selection_.front().entity;
    }
    /// Replaces the selection by one whole entity (nullopt clears it). Also clears a selected glyph.
    void select(std::optional<sketch::SketchEntityId> id);
    /// Everything selected, in click order: whole entities (role nullopt) and points (role set; entity points have
    /// no role).
    const std::vector<sketch::SketchRef>& selection() const noexcept { return selection_; }
    void clearSelection() noexcept;
    /// Replaces the selection by explicit items (whole entities or sub-points). Entities that do not exist are dropped.
    void setSelection(std::vector<sketch::SketchRef> items);
    /// Nearest sub-point (line/arc end, centre, point entity) to `p` within `tolMm`.
    std::optional<sketch::SketchRef> pickPointAt(sketch::Point2 p, double tolMm) const;
    /// Picks the nearest point, else the nearest entity within `tolMm`. `additive` toggles it in the selection;
    /// otherwise it replaces the selection (a miss clears it). Returns the entity of the picked item.
    std::optional<sketch::SketchEntityId> selectAt(sketch::Point2 p, double tolMm, bool additive = false);

    // ---- constraints (the constraint toolbar) ----
    /// Adds the constraint of `kind` described by the selection (see the rules per kind in the .cpp header comment).
    /// One undo step. Errors: InvalidArgument (selection does not fit, or the result over-constrains / conflicts).
    Result<std::vector<sketch::SketchConstraintId>> applyConstraint(sketch::ConstraintKind kind);
    /// Fix toggle on the selected entities: adds Fix where missing; when all are fixed already, removes them.
    Result<void> toggleFix();
    /// Constraint glyphs for the overlay, with the red flag of the solver diagnosis.
    std::vector<ConstraintGlyph> glyphs() const;
    std::optional<sketch::SketchConstraintId> selectedConstraint() const noexcept { return selConstraint_; }
    void selectConstraint(std::optional<sketch::SketchConstraintId> id);
    /// Delete: the selected glyph (constraint or dimension) when there is one, else the selected entities
    /// (with the constraints and dimensions that reference them). One undo step.
    Result<void> deleteSelection();
    Result<void> removeConstraint(sketch::SketchConstraintId id);

    // ---- dimensions ----
    /// What the picks and the mouse position mean for the Dimension tool: a line -> its length; two points -> a
    /// horizontal / vertical / aligned distance by where the mouse is relative to the points' bounding box (above or
    /// below: horizontal, left or right: vertical, otherwise aligned); a circle -> diameter; an arc -> radius; two
    /// lines -> the angle between them. Refs are ordered so that the measured value is positive.
    /// Errors: InvalidArgument (combination not dimensionable).
    struct DimensionPlan {
        sketch::DimensionKind kind = sketch::DimensionKind::Distance;
        std::vector<sketch::SketchRef> refs;
    };
    Result<DimensionPlan> planDimension(const std::vector<sketch::SketchRef>& picks, sketch::Point2 mouse) const;
    /// Measured value of a dimension kind on the current geometry: mm, or degrees for Angle.
    Result<double> measure(sketch::DimensionKind kind, const std::vector<sketch::SketchRef>& refs) const;

    /// Result of addDimension: either the new id, or the news that it would over-constrain (nothing was added).
    struct DimensionTry {
        std::optional<sketch::SketchDimensionId> id;
        bool overConstrains = false;
        std::string message;
    };
    /// Adds a driving dimension; an empty `expression` uses the measured value. A dimension that conflicts with the
    /// rest or repeats what the constraints already imply (the solver reports it redundant) is NOT added and
    /// `overConstrains` is set: offer addDrivenDimension. Errors: InvalidArgument (bad expression or refs).
    Result<DimensionTry> addDimension(sketch::DimensionKind kind, std::vector<sketch::SketchRef> refs,
                                      std::string expression = {});
    /// Adds a driven ("reference") dimension: shown with its measured value, never constrains.
    Result<std::uint64_t> addDrivenDimension(sketch::DimensionKind kind, std::vector<sketch::SketchRef> refs);
    /// Edits the expression (parameters allowed) and re-solves. One undo step. Errors: ParseError / InvalidArgument
    /// (does not evaluate, wrong dimension, or the sketch cannot satisfy it); nothing changes then.
    Result<void> setDimensionExpression(sketch::SketchDimensionId id, std::string expression);
    Result<void> removeDimension(std::uint64_t id, bool driven);
    /// Evaluates `expression` the way a dimension of `kind` does (bare numbers are mm, or degrees for Angle).
    Result<Quantity> evaluateExpression(sketch::DimensionKind kind, const std::string& expression) const;
    std::vector<DimensionView> dimensionViews() const;
    const std::vector<DrivenDimension>& drivenDimensions() const noexcept { return driven_; }
    struct DimensionRef {
        std::uint64_t id = 0;
        bool driven = false;
        friend bool operator==(const DimensionRef&, const DimensionRef&) = default;
    };
    std::optional<DimensionRef> selectedDimension() const noexcept { return selDimension_; }
    void selectDimension(std::optional<DimensionRef> d);
    /// Dimension tool: the geometry picked so far (drawn highlighted).
    const std::vector<sketch::SketchRef>& dimensionPicks() const noexcept { return dimPicks_; }
    /// Set by the Dimension tool's last placement click; cleared by takeDimensionEvent.
    std::optional<DimensionEvent> takeDimensionEvent();

    // ---- state shown by the overlay and palette ----
    ConstraintState constraintState() const;

    // ---- live drag (Select tool) ----
    /// Starts dragging a sub-point or point entity. Errors: NotFound / InvalidArgument.
    Result<void> beginDrag(const sketch::SketchRef& point);
    /// Moves the dragged point to `p` and re-solves (no undo step, no profile or diagnosis refresh: cheap enough for
    /// 60 Hz). When the constraints reject the position, the sketch stays where it was.
    Result<void> dragTo(sketch::Point2 p);
    /// Ends the drag: one undo step "Drag" when the geometry moved, and the derived state is refreshed.
    void endDrag();
    bool dragging() const noexcept { return drag_.has_value(); }

    // ---- derived state (recomputed after every edit and undo/redo) ----
    const std::vector<sketch::SketchProfile>& profiles() const noexcept { return profiles_; }
    /// Solver diagnosis of the current sketch: DOF, conflicts, redundancies.
    const sketch::SketchSolveReport& report() const noexcept { return report_; }

    // ---- history ----
    UndoStack& undoStack() noexcept { return undo_; }
    bool canUndo() const noexcept { return undo_.canUndo(); }
    bool canRedo() const noexcept { return undo_.canRedo(); }
    Result<void> undo();
    Result<void> redo();

private:
    struct DragState {
        sketch::SketchRef point;
        std::shared_ptr<const sketch::Sketch> before;
        std::vector<DrivenDimension> beforeDriven;
        bool moved = false;
    };
    /// Joint at the open end of the polyline: the curve and the role of its far end, the point and the direction of
    /// travel there (used for tangent continuation).
    struct Joint {
        sketch::SketchEntityId entity;
        sketch::PointRole role = sketch::PointRole::End;
        sketch::Point2 p{};
        sketch::Point2 dir{};
    };

    /// Runs `edit` on a copy, solves it, and records the change as one undo step. Returns the edit's error when
    /// it fails, InvalidArgument when the solve does not reach Solved, or the undo stack's error.
    Result<void> apply(std::string text, const std::function<Result<void>(sketch::Sketch&)>& edit);
    /// Pushes `next` (already solved) with the driven list `drivenNext` as one undo step and refreshes.
    Result<void> commit(std::string text, sketch::Sketch next, std::vector<DrivenDimension> drivenNext);
    Result<void> clickDimension(sketch::Point2 p, double tolMm);
    Result<void> trimCircle(sketch::SketchEntityId target, sketch::Point2 p);
    /// Like apply, but `edit` receives autoOn=true first; when that does not solve, it runs again with autoOn=false.
    Result<void> applyAuto(std::string text, const std::function<Result<void>(sketch::Sketch&, bool)>& edit);
    void refresh();
    void clearPending() noexcept;
    std::optional<sketch::SketchEntityId> nearestEntity(sketch::Point2 p, double tolMm) const;
    Result<void> clickLine(sketch::Point2 p, const std::optional<SnapHit>& hit);
    Result<void> clickArc(sketch::Point2 p, const std::optional<SnapHit>& hit);
    sketch::Point2 cursorPoint() const;

    sketch::Sketch sketch_;
    UndoStack undo_;
    std::vector<sketch::SketchProfile> profiles_;
    sketch::SketchSolveReport report_;

    SketchTool tool_ = SketchTool::Select;
    InferenceSettings inference_;
    double offsetDistance_ = 2.0;
    std::optional<SnapHit> snapHint_;
    std::optional<sketch::Point2> lineStart_;          ///< Open polyline: its last point.
    std::optional<SnapHit> lineStartHit_;              ///< Snap of the first polyline point (no joint yet).
    std::optional<Joint> joint_;                       ///< Open polyline: its last segment or tangent arc.
    std::optional<sketch::Point2> firstCorner_;        ///< Rectangle: first corner while pending.
    std::optional<SnapHit> firstCornerHit_;
    std::optional<sketch::Point2> circleCenter_;       ///< Circle: centre while pending.
    std::optional<SnapHit> circleCenterHit_;
    std::optional<sketch::Point2> arcStart_;           ///< Arc: first point while pending.
    std::optional<SnapHit> arcStartHit_;
    std::optional<sketch::Point2> arcMid_;             ///< Arc: second point while pending.
    std::optional<sketch::Point2> tangentDrag_;
    std::optional<sketch::Point2> cursor_;
    std::vector<sketch::SketchRef> selection_;
    std::optional<sketch::SketchConstraintId> selConstraint_;
    std::optional<DimensionRef> selDimension_;
    const ParameterTable* params_ = nullptr;
    std::vector<DrivenDimension> driven_;
    std::uint64_t nextDriven_ = 1;
    std::vector<sketch::SketchRef> dimPicks_;
    std::optional<DimensionEvent> dimEvent_;
    std::optional<DragState> drag_;
};

} // namespace iw::ui
