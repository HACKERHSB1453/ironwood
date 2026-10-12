// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/sketch/sketch_mode.hpp"

#include <QDockWidget>
#include <QEvent>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMouseEvent>

#include <cmath>

#include "render/camera.hpp"
#include "render/viewport.hpp"
#include "ui/sketch/sketch_palette.hpp"
#include "ui/sketch/view_math.hpp"

namespace iw::ui {
namespace {

constexpr int kLookAtMs = 250;        ///< Camera animation when a sketch starts.
constexpr double kPickPixels = 6.0;   ///< Entity pick radius for Select.
constexpr double kSnapPixels = 6.0;   ///< Snap radius for points and alignments.
constexpr double kDragPixels = 4.0;   ///< Movement that turns a press on a polyline end into a tangent-arc drag.

bool isLeft(const QMouseEvent* me) { return me->button() == Qt::LeftButton; }
bool altHeld(const QMouseEvent* me) { return me->modifiers().testFlag(Qt::AltModifier); }

} // namespace

SketchMode::SketchMode(render::Viewport* viewport, QMainWindow* window)
    : QObject(window), vp_(viewport) {
    overlay_ = new SketchOverlay(viewport);

    palette_ = new SketchPalette;
    dock_ = new QDockWidget(tr("Sketch"), window);
    dock_->setObjectName("sketchDock");
    dock_->setWidget(palette_);
    window->addDockWidget(Qt::LeftDockWidgetArea, dock_);
    dock_->hide();

    connect(palette_, &SketchPalette::toolRequested, this, [this](SketchTool t) { setTool(t); });
    connect(palette_, &SketchPalette::optionsChanged, this, [this](const SketchViewOptions& o) { setOptions(o); });
    connect(palette_, &SketchPalette::finishRequested, this, [this] { finish(); });
    connect(palette_, &SketchPalette::undoRequested, this, [this] { undoRedo(false); });
    connect(palette_, &SketchPalette::redoRequested, this, [this] { undoRedo(true); });
    connect(palette_, &SketchPalette::snapToggled, this, [this](bool on) { setSnapEnabled(on); });
    connect(palette_, &SketchPalette::offsetDistanceChanged, this, [this](double) { syncSession(); });

    viewport->installEventFilter(this);
    palette_->setOptions(opts_);
}

SketchMode::~SketchMode() = default;

void SketchMode::start(const sketch::SketchPlane& plane, const QString& planeName) {
    if (active_) finish();
    sessions_.push_back(std::make_unique<SketchSession>(plane));
    active_ = sessions_.back().get();

    palette_->setPlaneName(planeName);
    palette_->setTool(SketchTool::Select);
    palette_->setSnapEnabled(snapOn_);
    dock_->show();

    // Look along the plane normal, targeting the plane origin; the distance and up direction are kept.
    render::CameraState pose = vp_->camera().poseForEyeDirection(plane.normal);
    pose.target = plane.origin;
    vp_->animateTo(pose, kLookAtMs);
    vp_->setFocus();

    syncSession();
    refresh();
    emit status(tr("Sketch on %1 plane. L line, R rectangle, C circle, P point, X construction. Esc cancels the tool, "
                   "Esc again or Finish Sketch exits.")
                    .arg(planeName));
}

void SketchMode::finish() {
    if (!active_) return;
    const std::size_t profiles = active_->profiles().size();
    active_ = nullptr;
    dock_->hide();
    refresh();
    emit status(tr("Sketch finished: %1 profile(s)").arg(profiles));
}

void SketchMode::setTool(SketchTool t) {
    if (!active_) return;
    active_->setTool(t);
    syncSession();
    palette_->setTool(t);
    refresh();
}

void SketchMode::setSnapEnabled(bool on) {
    snapOn_ = on;
    palette_->setSnapEnabled(on);
    syncSession();
    refresh();
}

void SketchMode::syncSession() {
    if (!active_) return;
    const double pxPerMm = sketchPixelsPerMm(vp_->camera(), vp_->height());
    active_->setInference(InferenceSettings{snapOn_, kSnapPixels / pxPerMm});
    active_->setOffsetDistance(palette_->offsetDistance());
}

void SketchMode::setOptions(const SketchViewOptions& o) {
    opts_ = o;
    palette_->setOptions(o);
    refresh();
}

bool SketchMode::escape() {
    if (!active_) return false;
    if (active_->cancel()) {
        palette_->setTool(SketchTool::Select);
        refresh();
        return true;
    }
    finish();
    return true;
}

void SketchMode::refresh() {
    std::vector<const SketchSession*> views;
    views.reserve(sessions_.size());
    for (const auto& s : sessions_) views.push_back(s.get());
    overlay_->setContent(std::move(views), active_, opts_);

    if (active_) {
        const sketch::SketchSolveReport& rep = active_->report();
        QString text = tr("DOF %1, %2 profile(s)").arg(rep.dof).arg(active_->profiles().size());
        if (!rep.conflictingConstraints.empty() || !rep.conflictingDimensions.empty())
            text += tr(" - conflicting constraints");
        palette_->setStatus(text);
    }
}

void SketchMode::report(const Result<void>& r) {
    if (!r) emit status(QString::fromStdString(r.error().message));
}

std::optional<sketch::Point2> SketchMode::toSketch(double x, double y) const {
    if (!active_) return std::nullopt;
    return screenToPlane(vp_->camera(), vp_->width(), vp_->height(), active_->plane(), ScreenPoint{x, y});
}

void SketchMode::undoRedo(bool redo) {
    if (!active_) return;
    report(redo ? active_->redo() : active_->undo());
    refresh();
}

bool SketchMode::eventFilter(QObject* watched, QEvent* event) {
    if (watched != vp_) return QObject::eventFilter(watched, event);
    if (event->type() == QEvent::Resize) overlay_->setGeometry(vp_->rect());
    if (!active_) return false;

    switch (event->type()) {
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonDblClick:
    case QEvent::MouseButtonRelease:
    case QEvent::MouseMove:
        return handleMouse(event);
    case QEvent::KeyPress:
        return handleKey(static_cast<QKeyEvent*>(event));
    case QEvent::ShortcutOverride: {
        // Keys sketch mode uses must reach the viewport instead of firing window shortcuts (Esc, S, ...).
        auto* key = static_cast<QKeyEvent*>(event);
        if (isSketchKey(key)) {
            event->accept();
            return true;
        }
        return false;
    }
    default:
        return false;
    }
}

bool SketchMode::handleMouse(QEvent* event) {
    switch (event->type()) {
    case QEvent::MouseButtonPress: {
        auto* me = static_cast<QMouseEvent*>(event);
        if (!isLeft(me) || altHeld(me)) return false;   // middle/right and Alt+left keep navigating
        pressConsumed_ = true;
        syncSession();
        const double pxPerMm = sketchPixelsPerMm(vp_->camera(), vp_->height());
        const auto p = toSketch(me->position().x(), me->position().y());
        if (p && active_->canTangentArcAt(*p, kPickPixels / pxPerMm)) {
            // Press on the end of the polyline: a click starts the next segment, a drag makes a tangent arc.
            tangentPending_ = true;
            pressPx_ = me->position();
            pressSketch_ = *p;
            refresh();
            return true;
        }
        if (p) report(active_->click(*p, kPickPixels / pxPerMm));
        refresh();
        return true;
    }
    case QEvent::MouseButtonDblClick: {
        auto* me = static_cast<QMouseEvent*>(event);
        if (!isLeft(me) || altHeld(me)) return false;
        tangentPending_ = false;
        active_->endPolyline();
        refresh();
        return true;
    }
    case QEvent::MouseButtonRelease: {
        auto* me = static_cast<QMouseEvent*>(event);
        if (!isLeft(me) || !pressConsumed_) return false;
        pressConsumed_ = false;
        if (tangentPending_) {
            tangentPending_ = false;
            if (const auto drag = active_->tangentDrag()) {
                if (auto r = active_->tangentArcTo(*drag); !r) emit status(QString::fromStdString(r.error().message));
            } else {
                report(active_->click(pressSketch_, kPickPixels / sketchPixelsPerMm(vp_->camera(), vp_->height())));
            }
            active_->setTangentDrag(std::nullopt);
            refresh();
        }
        return true;
    }
    case QEvent::MouseMove: {
        auto* me = static_cast<QMouseEvent*>(event);
        const auto here = toSketch(me->position().x(), me->position().y());
        active_->setCursor(here);
        if (tangentPending_ && here) {
            const QPointF d = me->position() - pressPx_;
            if (std::hypot(d.x(), d.y()) > kDragPixels) active_->setTangentDrag(here);
            refresh();
        }
        overlay_->update();   // rubber band follows the cursor; the viewport still gets the move (hover)
        return false;
    }
    default:
        return false;
    }
}

bool SketchMode::isSketchKey(const QKeyEvent* key) const {
    const Qt::KeyboardModifiers mods = key->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier);
    if (mods.testFlag(Qt::ControlModifier)) return key->key() == Qt::Key_Z || key->key() == Qt::Key_Y;
    if (mods != Qt::NoModifier) return false;
    switch (key->key()) {
    case Qt::Key_Escape:
    case Qt::Key_Return:
    case Qt::Key_Enter:
    case Qt::Key_L:
    case Qt::Key_R:
    case Qt::Key_C:
    case Qt::Key_P:
    case Qt::Key_X:
    case Qt::Key_A:
    case Qt::Key_T:
    case Qt::Key_O:
        return true;
    default:
        return false;
    }
}

bool SketchMode::handleKey(const QKeyEvent* key) {
    if (!isSketchKey(key)) return false;
    if (key->modifiers().testFlag(Qt::ControlModifier)) {
        const bool redo = key->key() == Qt::Key_Y || (key->key() == Qt::Key_Z && key->modifiers().testFlag(Qt::ShiftModifier));
        undoRedo(redo);
        return true;
    }
    switch (key->key()) {
    case Qt::Key_Escape:
        return escape();
    case Qt::Key_Return:
    case Qt::Key_Enter:
        active_->endPolyline();
        refresh();
        return true;
    case Qt::Key_L: setTool(SketchTool::Line); return true;
    case Qt::Key_R: setTool(SketchTool::Rectangle); return true;
    case Qt::Key_C: setTool(SketchTool::Circle); return true;
    case Qt::Key_P: setTool(SketchTool::Point); return true;
    case Qt::Key_A: setTool(SketchTool::Arc); return true;
    case Qt::Key_T: setTool(SketchTool::Trim); return true;
    case Qt::Key_O: setTool(SketchTool::Offset); return true;
    case Qt::Key_X: {
        const auto sel = active_->selected();
        if (!sel) {
            emit status(tr("Select an entity first, then press X to toggle construction."));
        } else {
            report(active_->toggleConstruction(*sel));
        }
        refresh();
        return true;
    }
    default:
        return false;
    }
}

} // namespace iw::ui
