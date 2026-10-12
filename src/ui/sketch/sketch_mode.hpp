// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QObject>
#include <QPointF>
#include <QString>

#include <memory>
#include <optional>
#include <vector>

#include "core/result.hpp"
#include "sketch/sketch.hpp"
#include "ui/sketch/session.hpp"
#include "ui/sketch/sketch_overlay.hpp"

class QDockWidget;
class QKeyEvent;
class QMainWindow;

namespace iw::render {
class Viewport;
}

namespace iw::ui {

class SketchPalette;

/// Sketch mode for the main window: owns every sketch of the session, the active one, the overlay that paints them,
/// the sketch palette dock, and the viewport input while a sketch is active.
///
/// Input (viewport event filter, active sketch only): left click = tool click (Line: click-click, double click or
/// Enter ends the polyline; Rectangle: two corners; Circle: centre then radius; Point; Select picks), L/R/C/P pick a
/// tool, X toggles construction of the selected entity, Esc cancels the tool and a second Esc finishes, Ctrl+Z /
/// Ctrl+Y undo and redo the sketch's own history. Middle/right buttons and Alt+left keep the navigation gestures.
class SketchMode : public QObject {
    Q_OBJECT
public:
    SketchMode(render::Viewport* viewport, QMainWindow* window);
    ~SketchMode() override;

    /// Starts a new sketch on `plane`; the camera animates to look along the plane normal. Finishes a sketch that is
    /// still active first.
    void start(const sketch::SketchPlane& plane, const QString& planeName);
    /// Finish Sketch: leaves sketch mode. The sketch stays in sessions() (drawn dimmed) for later features.
    void finish();
    bool active() const noexcept { return active_ != nullptr; }
    SketchSession* activeSession() const noexcept { return active_; }
    const std::vector<std::unique_ptr<SketchSession>>& sessions() const noexcept { return sessions_; }

    void setTool(SketchTool t);
    SketchViewOptions options() const noexcept { return opts_; }
    void setOptions(const SketchViewOptions& o);
    /// Snap and auto-constraints (palette toggle). Snap radius is 6 px at the current zoom.
    void setSnapEnabled(bool on);

    /// Esc: cancels the active tool; with no tool left, finishes the sketch. Returns false when no sketch is active.
    bool escape();
    /// Re-syncs the overlay and palette after the active session was edited directly (not through a tool).
    void refresh();

signals:
    void status(const QString& text);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    bool handleMouse(QEvent* event);
    bool handleKey(const QKeyEvent* key);
    /// Pushes the snap settings (radius from view_math) and the offset distance to the active session.
    void syncSession();
    bool isSketchKey(const QKeyEvent* key) const;
    std::optional<sketch::Point2> toSketch(double x, double y) const;
    void undoRedo(bool redo);
    void report(const Result<void>& r);

    render::Viewport* vp_;
    std::vector<std::unique_ptr<SketchSession>> sessions_;
    SketchSession* active_ = nullptr;
    SketchOverlay* overlay_ = nullptr;
    SketchPalette* palette_ = nullptr;
    QDockWidget* dock_ = nullptr;
    SketchViewOptions opts_;
    bool pressConsumed_ = false;   ///< The left press that started a tool click was taken by sketch mode.
    bool tangentPending_ = false;  ///< Press on the end of a Line polyline: a click or a tangent-arc drag.
    QPointF pressPx_;              ///< Where that press happened (widget pixels).
    sketch::Point2 pressSketch_{};
    bool snapOn_ = false;
};

} // namespace iw::ui
