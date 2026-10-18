// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QObject>
#include <QPointF>
#include <QString>

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "core/params.hpp"
#include "core/result.hpp"
#include "render/camera.hpp"
#include "sketch/sketch.hpp"
#include "ui/sketch/session.hpp"
#include "ui/sketch/sketch_overlay.hpp"

class QDockWidget;
class QKeyEvent;
class QLineEdit;
class QMainWindow;

namespace iw {
class JobSystem;
}

namespace iw::render {
class Viewport;
}

namespace iw::ui {

class SketchPalette;
class CommandPanel;

/// Sketch mode for the main window: owns every sketch of the session, the active one, the overlay that paints them,
/// the sketch palette dock, and the viewport input while a sketch is active.
///
/// Input (viewport event filter, active sketch only): left click = tool click (Line: click-click, double click or
/// Enter ends the polyline; Rectangle: two corners; Circle: centre then radius; Point; Select picks), L/R/C/P pick a
/// tool, X toggles construction of the selected entity, Esc cancels the tool and a second Esc finishes, Ctrl+Z /
/// Ctrl+Y undo and redo the sketch's own history. Middle/right buttons and Alt+left keep the navigation gestures.
///
/// B3b additions. Select: click a point or curve (Ctrl/Shift adds), click a constraint glyph or a dimension to select it,
/// Delete removes the selection, dragging a point re-solves live. D = Dimension tool: pick geometry, click where the
/// text goes, type an expression in the inline editor (Enter); double click a dimension to edit it. The constraint
/// toolbar of the palette applies a constraint to the selection. Snapping and auto-constraints are on by default.
/// The Offset tool asks for its distance in a command panel (see setPanelHost).
class SketchMode : public QObject {
    Q_OBJECT
public:
    SketchMode(render::Viewport* viewport, QMainWindow* window);
    ~SketchMode() override;

    /// Starts a new sketch on `plane`; the camera animates to look along the plane normal. Finishes a sketch that is
    /// still active first.
    void start(const sketch::SketchPlane& plane, const QString& planeName);
    /// Re-enters an existing sketch (Edit Sketch from the timeline). Entity, constraint and dimension ids are kept.
    /// Finishes a sketch that is still active first. Finish Sketch commits it (sketchFinished with committed=true);
    /// Esc with no tool pending cancels the edit (committed=false) and the caller leaves the document unchanged.
    void startEdit(sketch::Sketch existing, const QString& name);
    /// Finish Sketch: leaves sketch mode. The sketch stays in sessions() (drawn dimmed) for later features.
    void finish();
    /// Faces the active sketch's plane again (the Look At button). No-op when no sketch is active.
    void lookAt();
    bool active() const noexcept { return active_ != nullptr; }
    /// True while the active session edits an existing sketch (startEdit) rather than creating one.
    bool editing() const noexcept { return editing_; }
    SketchSession* activeSession() const noexcept { return active_; }
    const std::vector<std::unique_ptr<SketchSession>>& sessions() const noexcept { return sessions_; }

    void setTool(SketchTool t);
    SketchViewOptions options() const noexcept { return opts_; }
    void setOptions(const SketchViewOptions& o);
    /// Snap and auto-constraints (palette toggle, on by default). Snap radius is 6 px at the current zoom.
    void setSnapEnabled(bool on);
    bool snapEnabled() const noexcept { return snapOn_; }

    /// Parameter table dimension expressions are evaluated against (nullptr: literals only). Not owned. Applies to
    /// every sketch of the session.
    void setParameters(const ParameterTable* table);
    /// Call after the parameter table changed: every sketch is re-solved (one undo step each where geometry moved).
    void parametersChanged();
    /// Job system for the command panel previews. Default: a private single-thread one, created on demand.
    void setJobSystem(JobSystem* jobs) noexcept { jobs_ = jobs; }
    /// Where the Offset command panel goes (the shell's command dock). Default: a right dock owned by sketch mode.
    /// The host shows the widget and must dispose of it once its `finished` signal fired.
    void setPanelHost(std::function<void(QWidget*)> host) { panelHost_ = std::move(host); }
    /// Asked when a new dimension would over-constrain the sketch: true = add it as a driven (reference) dimension.
    /// Default: a message box.
    void setDrivenPrompt(std::function<bool(const QString&)> prompt) { drivenPrompt_ = std::move(prompt); }

    // ---- actions (also reachable from the palette and the keyboard) ----
    void applyConstraint(sketch::ConstraintKind kind);   ///< Constraint toolbar button on the current selection.
    void toggleFix();
    void deleteSelected();                               ///< Delete: selected glyph, dimension or entities.
    /// Opens the offset command panel for the closed profile containing `seed`.
    void beginOffset(sketch::SketchEntityId seed);
    CommandPanel* offsetPanel() const noexcept { return offsetPanel_; }
    /// Opens the inline expression editor on a driving dimension of the active sketch.
    void editDimension(SketchSession::DimensionRef dim);
    QLineEdit* dimensionEditor() const noexcept { return dimEdit_; }
    /// Enter in the editor: applies the expression. On failure the editor stays open with the reason as tooltip.
    bool commitDimensionEditor();
    void closeDimensionEditor();

    /// Esc: cancels the active tool; with no tool left, finishes the sketch. Returns false when no sketch is active.
    bool escape();
    /// Re-syncs the overlay and palette after the active session was edited directly (not through a tool).
    void refresh();

signals:
    void status(const QString& text);
    /// A session left sketch mode. Emitted before an edited session is dropped from sessions(), so the handler can
    /// read its sketch: committed = Finish Sketch (or Esc on a new sketch), false = cancelled edit.
    void sketchFinished(iw::ui::SketchSession* session, bool committed);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    bool handleMouse(QEvent* event);
    /// Shared start of start() / startEdit(): makes `owned` the active session and shows the sketch UI.
    void beginSession(std::unique_ptr<SketchSession> owned, const QString& planeName, const QString& statusText);
    void finishWith(bool committed);
    /// Faces the active sketch's plane (orthographic, target = plane origin, distance and up kept).
    void faceSketchPlane(int ms);
    bool handleKey(const QKeyEvent* key);
    /// Pushes the snap settings (radius from view_math) and the offset distance to the active session.
    void syncSession();
    bool isSketchKey(const QKeyEvent* key) const;
    std::optional<sketch::Point2> toSketch(double x, double y) const;
    void undoRedo(bool redo);
    void report(const Result<void>& r);
    void afterDimensionClick();
    void closeOffsetPanel();
    std::optional<ConstraintGlyph> glyphAt(QPointF px) const;
    std::optional<SketchSession::DimensionRef> dimensionAt(QPointF px) const;

    render::Viewport* vp_;
    std::vector<std::unique_ptr<SketchSession>> sessions_;
    SketchSession* active_ = nullptr;
    /// Camera and projection from before the first sketch of a sketch-mode run; restored when sketch mode ends.
    struct SavedView {
        render::CameraState state;
        render::Projection projection = render::Projection::Perspective;
    };
    std::optional<SavedView> savedView_;
    bool editing_ = false;         ///< The active session re-enters an existing sketch.
    SketchOverlay* overlay_ = nullptr;
    SketchPalette* palette_ = nullptr;
    QDockWidget* dock_ = nullptr;
    SketchViewOptions opts_;
    bool pressConsumed_ = false;   ///< The left press that started a tool click was taken by sketch mode.
    bool tangentPending_ = false;  ///< Press on the end of a Line polyline: a click or a tangent-arc drag.
    QPointF pressPx_;              ///< Where that press happened (widget pixels).
    sketch::Point2 pressSketch_{};
    bool snapOn_ = true;
    const ParameterTable* params_ = nullptr;
    JobSystem* jobs_ = nullptr;
    std::unique_ptr<JobSystem> ownJobs_;
    std::function<void(QWidget*)> panelHost_;
    std::function<bool(const QString&)> drivenPrompt_;
    CommandPanel* offsetPanel_ = nullptr;
    QDockWidget* offsetDock_ = nullptr;
    QLineEdit* dimEdit_ = nullptr;
    std::optional<sketch::SketchDimensionId> dimEditId_;
    QWidget* mainWindow_ = nullptr;
    std::optional<sketch::SketchRef> dragPoint_;   ///< Press on a point with the Select tool: a click or a drag.
    bool draggingPoint_ = false;
};

} // namespace iw::ui
