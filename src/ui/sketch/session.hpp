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
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/result.hpp"
#include "core/undo.hpp"
#include "sketch/profile.hpp"
#include "sketch/sketch.hpp"
#include "sketch/solver/bridge.hpp"

namespace iw::ui {

enum class SketchTool : std::uint8_t { Select, Line, Rectangle, CenterRectangle, Circle, Point, Arc, Trim, Offset };

/// What a snap attached to. Priority (highest first): Endpoint, Center, Midpoint, Intersection, Tangent,
/// Perpendicular, OnCurve, then Horizontal/Vertical alignment. Within one priority the nearest candidate wins.
enum class SnapKind : std::uint8_t {
    Endpoint,      ///< Existing line/arc end or point. Constraint: coincident with the new point.
    Center,        ///< Circle or arc centre. Constraint: coincident with the centre.
    Midpoint,      ///< Line midpoint (arc midpoint: no constraint). Constraint: Midpoint on a line.
    Intersection,  ///< Crossing of two curves. No constraint (there is no point-on-object record).
    Tangent,       ///< Tangent point of a circle/arc seen from the pending start. Constraint: Tangent on the new line.
    Perpendicular, ///< Foot of a line perpendicular to an existing line from the pending start. Constraint: Perpendicular.
    OnCurve,       ///< Nearest point on a curve. No constraint.
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
    bool enabled = false;    ///< Off: drawing is literal, nothing snaps, nothing is constrained automatically.
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
    /// Errors: InvalidArgument (nothing within tolerance, circles or points), KernelFailure-free: solver errors too.
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
    void setOffsetDistance(double mm) noexcept { offsetDistance_ = mm; }
    double offsetDistance() const noexcept { return offsetDistance_; }

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
    std::optional<sketch::SketchEntityId> selected() const noexcept { return selected_; }
    void select(std::optional<sketch::SketchEntityId> id) noexcept { selected_ = id; }
    /// Nearest entity to `p` within `tolMm` (ties: lowest id); selects it, or clears the selection when none.
    std::optional<sketch::SketchEntityId> selectAt(sketch::Point2 p, double tolMm);

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
    std::optional<sketch::SketchEntityId> selected_;
};

} // namespace iw::ui
