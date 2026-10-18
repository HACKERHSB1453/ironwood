// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/sketch/sketch_mode.hpp"

#include <QApplication>
#include <QDockWidget>
#include <QEvent>
#include <QKeyEvent>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPalette>

#include <cmath>
#include <cstdio>

#include "core/job.hpp"
#include "render/camera.hpp"
#include "render/viewport.hpp"
#include "ui/command_dialog.hpp"
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
    : QObject(window), vp_(viewport), mainWindow_(window) {
    opts_.dark = QApplication::palette().color(QPalette::Window).lightness() < 128;
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
    connect(palette_, &SketchPalette::constraintRequested, this,
            [this](sketch::ConstraintKind k) { applyConstraint(k); });
    connect(palette_, &SketchPalette::fixToggleRequested, this, [this] { toggleFix(); });

    viewport->installEventFilter(this);
    palette_->setOptions(opts_);
}

SketchMode::~SketchMode() = default;

void SketchMode::start(const sketch::SketchPlane& plane, const QString& planeName) {
    if (active_) finish();
    closeDimensionEditor();
    sessions_.push_back(std::make_unique<SketchSession>(plane));
    active_ = sessions_.back().get();
    active_->setParameters(params_);

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
    emit status(tr("Sketch on %1 plane. L line, R rectangle, C circle, P point, D dimension, X construction. Esc cancels the tool, "
                   "Esc again or Finish Sketch exits.")
                    .arg(planeName));
}

void SketchMode::finish() {
    if (!active_) return;
    closeDimensionEditor();
    closeOffsetPanel();
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
}

void SketchMode::setParameters(const ParameterTable* table) {
    params_ = table;
    for (auto& s : sessions_) s->setParameters(table);
    refresh();
}

void SketchMode::parametersChanged() {
    for (auto& s : sessions_) {
        if (auto r = s->resolve(); !r && s.get() == active_) emit status(QString::fromStdString(r.error().message));
    }
    refresh();
}

void SketchMode::applyConstraint(sketch::ConstraintKind kind) {
    if (!active_) return;
    if (auto r = active_->applyConstraint(kind); !r) emit status(QString::fromStdString(r.error().message));
    refresh();
}

void SketchMode::toggleFix() {
    if (!active_) return;
    report(active_->toggleFix());
    refresh();
}

void SketchMode::deleteSelected() {
    if (!active_) return;
    report(active_->deleteSelection());
    refresh();
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
        QString detail = QString::fromStdString(rep.message);
        if (detail.isEmpty() && active_->constraintState() == ConstraintState::Conflicting)
            detail = tr("%1 conflicting or redundant constraint(s)")
                         .arg(rep.conflictingConstraints.size() + rep.conflictingDimensions.size() +
                              rep.redundantConstraints.size() + rep.redundantDimensions.size());
        palette_->setConstraintStatus(rep.dof, active_->constraintState(), detail);
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

void SketchMode::closeOffsetPanel() {
    if (offsetDock_) offsetDock_->hide();
    if (offsetPanel_ && !panelHost_) offsetPanel_->deleteLater();
    offsetPanel_ = nullptr;
}

void SketchMode::beginOffset(sketch::SketchEntityId seed) {
    if (!active_) return;
    closeOffsetPanel();
    JobSystem* jobs = jobs_;
    if (!jobs) {
        if (!ownJobs_) ownJobs_ = std::make_unique<JobSystem>(1);
        jobs = ownJobs_.get();
    }
    CommandDialogSpec spec;
    spec.title = tr("Offset");
    InputSpec dist;
    dist.name = "distance";
    dist.label = tr("Distance");
    dist.kind = InputKind::Value;
    dist.dim = Dimension::Length;
    char buf[64];
    std::snprintf(buf, sizeof buf, "%g", active_->offsetDistance());
    dist.defaultText = buf;
    spec.inputs = {dist};
    spec.preview = [](const DialogModel::Snapshot& snap, CancelToken) -> Result<std::string> {
        const auto it = snap.values.find("distance");
        if (it == snap.values.end()) return makeError(ErrorCode::InvalidArgument, "no distance");
        return "Offset by " + formatQuantity(it->second);
    };
    SketchSession* session = active_;
    spec.accept = [this, session, seed](const DialogModel::Snapshot& snap) -> Result<void> {
        const double d = snap.values.at("distance").value;
        session->setOffsetDistance(d);
        auto r = session->offsetLoop(seed, d);
        refresh();
        if (!r) return std::unexpected(r.error());
        return {};
    };
    offsetPanel_ = new CommandPanel(std::move(spec), params_, *jobs);
    offsetPanel_->setObjectName("offsetPanel");
    connect(offsetPanel_, &CommandPanel::finished, this, [this] { closeOffsetPanel(); });
    if (panelHost_) {
        panelHost_(offsetPanel_);
    } else {
        if (!offsetDock_) {
            offsetDock_ = new QDockWidget(tr("Offset"), static_cast<QMainWindow*>(mainWindow_));
            offsetDock_->setObjectName("offsetDock");
            static_cast<QMainWindow*>(mainWindow_)->addDockWidget(Qt::RightDockWidgetArea, offsetDock_);
        }
        if (QWidget* old = offsetDock_->widget()) old->deleteLater();
        offsetDock_->setWidget(offsetPanel_);
        offsetDock_->show();
    }
}

std::optional<ConstraintGlyph> SketchMode::glyphAt(QPointF px) const {
    if (!active_ || !opts_.constraints) return std::nullopt;
    for (const ConstraintGlyph& g : active_->glyphs()) {
        const auto at = glyphScreenPos(vp_->camera(), vp_->width(), vp_->height(), active_->plane(), g);
        if (at && std::hypot(at->x() - px.x(), at->y() - px.y()) <= 9.0) return g;
    }
    return std::nullopt;
}

std::optional<SketchSession::DimensionRef> SketchMode::dimensionAt(QPointF px) const {
    if (!active_ || !opts_.dimensions) return std::nullopt;
    for (const DimensionView& d : active_->dimensionViews()) {
        const auto lay = layoutDimension(vp_->camera(), vp_->width(), vp_->height(), *active_, d);
        if (lay && lay->textRect.adjusted(-3, -3, 3, 3).contains(px)) return SketchSession::DimensionRef{d.id, d.driven};
    }
    return std::nullopt;
}

void SketchMode::closeDimensionEditor() {
    if (dimEdit_) {
        dimEdit_->hide();
        dimEdit_->deleteLater();
        dimEdit_ = nullptr;
    }
    dimEditId_.reset();
    if (vp_) vp_->setFocus();
}

void SketchMode::editDimension(SketchSession::DimensionRef dim) {
    if (!active_) return;
    if (dim.driven) {
        emit status(tr("A reference dimension shows the measured value and cannot be edited."));
        return;
    }
    const sketch::SketchDimensionId id{dim.id};
    const auto it = active_->sketch().dimensions().find(id);
    if (it == active_->sketch().dimensions().end()) return;
    closeDimensionEditor();
    dimEditId_ = id;
    active_->selectDimension(dim);

    QPoint at(40, 40);
    for (const DimensionView& d : active_->dimensionViews()) {
        if (d.driven || d.id != dim.id) continue;
        if (const auto lay = layoutDimension(vp_->camera(), vp_->width(), vp_->height(), *active_, d))
            at = lay->textRect.center().toPoint() - QPoint(60, 12);
    }
    dimEdit_ = new QLineEdit(QString::fromStdString(it->second.expression), vp_);
    dimEdit_->setObjectName("dimensionEditor");
    dimEdit_->setGeometry(at.x(), at.y(), 120, 24);
    dimEdit_->installEventFilter(this);
    connect(dimEdit_, &QLineEdit::returnPressed, this, [this] { commitDimensionEditor(); });
    connect(dimEdit_, &QLineEdit::textChanged, this, [this](const QString& text) {
        if (!active_ || !dimEdit_ || !dimEditId_) return;
        const auto di = active_->sketch().dimensions().find(*dimEditId_);
        if (di == active_->sketch().dimensions().end()) return;
        const auto v = active_->evaluateExpression(di->second.kind, text.toStdString());
        dimEdit_->setStyleSheet(v ? QString() : QStringLiteral("border: 1px solid #eb4034;"));
        dimEdit_->setToolTip(v ? QString() : QString::fromStdString(v.error().message));
    });
    dimEdit_->show();
    dimEdit_->selectAll();
    dimEdit_->setFocus();
    refresh();
}

bool SketchMode::commitDimensionEditor() {
    if (!active_ || !dimEdit_ || !dimEditId_) return false;
    auto r = active_->setDimensionExpression(*dimEditId_, dimEdit_->text().toStdString());
    if (!r) {
        dimEdit_->setStyleSheet(QStringLiteral("border: 1px solid #eb4034;"));
        dimEdit_->setToolTip(QString::fromStdString(r.error().message));
        emit status(QString::fromStdString(r.error().message));
        return false;
    }
    closeDimensionEditor();
    refresh();
    return true;
}

void SketchMode::afterDimensionClick() {
    if (!active_) return;
    const auto ev = active_->takeDimensionEvent();
    if (!ev) return;
    if (ev->type == DimensionEvent::Type::Created) {
        editDimension({ev->id, false});   // Fusion-style: the editor opens on the new dimension
        return;
    }
    const QString question = tr("This dimension would over-constrain the sketch (%1). Make it a driven (reference) "
                                "dimension instead?")
                                 .arg(QString::fromStdString(ev->message));
    const bool yes = drivenPrompt_ ? drivenPrompt_(question)
                                   : QMessageBox::question(vp_, tr("Over-constrained"), question) == QMessageBox::Yes;
    if (!yes) {
        emit status(tr("Dimension not added: it would over-constrain the sketch."));
        return;
    }
    if (auto r = active_->addDrivenDimension(ev->kind, ev->refs); !r) emit status(QString::fromStdString(r.error().message));
}

bool SketchMode::eventFilter(QObject* watched, QEvent* event) {
    if (dimEdit_ && watched == dimEdit_ && event->type() == QEvent::KeyPress &&
        static_cast<QKeyEvent*>(event)->key() == Qt::Key_Escape) {
        closeDimensionEditor();
        return true;
    }
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
        if (dimEdit_) commitDimensionEditor();
        if (active_->tool() == SketchTool::Select) {
            // Glyphs and dimension texts first, then a press on a point (click or drag), then a plain pick.
            if (const auto g = glyphAt(me->position())) {
                active_->selectConstraint(g->id);
                refresh();
                return true;
            }
            if (const auto d = dimensionAt(me->position())) {
                active_->selectDimension(d);
                refresh();
                return true;
            }
            const bool additive = me->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier);
            if (p && !additive) {
                if (const auto pt = active_->pickPointAt(*p, kPickPixels / pxPerMm)) {
                    dragPoint_ = pt;
                    draggingPoint_ = false;
                    pressPx_ = me->position();
                    pressSketch_ = *p;
                    return true;
                }
            }
            if (p && additive) {
                active_->selectAt(*p, kPickPixels / pxPerMm, true);
                refresh();
                return true;
            }
        }
        if (p && active_->canTangentArcAt(*p, kPickPixels / pxPerMm)) {
            // Press on the end of the polyline: a click starts the next segment, a drag makes a tangent arc.
            tangentPending_ = true;
            pressPx_ = me->position();
            pressSketch_ = *p;
            refresh();
            return true;
        }
        if (p && active_->tool() == SketchTool::Offset) {
            const auto seed = active_->offsetSeedAt(*p, kPickPixels / pxPerMm);
            if (seed) beginOffset(*seed);
            else emit status(tr("Nothing to offset at that point."));
            refresh();
            return true;
        }
        if (p) report(active_->click(*p, kPickPixels / pxPerMm));
        if (active_->tool() == SketchTool::Dimension) afterDimensionClick();
        refresh();
        return true;
    }
    case QEvent::MouseButtonDblClick: {
        auto* me = static_cast<QMouseEvent*>(event);
        if (!isLeft(me) || altHeld(me)) return false;
        if (active_->tool() == SketchTool::Select) {
            if (const auto d = dimensionAt(me->position())) {
                pressConsumed_ = true;
                editDimension(*d);
                return true;
            }
        }
        tangentPending_ = false;
        active_->endPolyline();
        refresh();
        return true;
    }
    case QEvent::MouseButtonRelease: {
        auto* me = static_cast<QMouseEvent*>(event);
        if (!isLeft(me) || !pressConsumed_) return false;
        pressConsumed_ = false;
        if (dragPoint_) {
            const auto pt = *dragPoint_;
            dragPoint_.reset();
            if (draggingPoint_) {
                draggingPoint_ = false;
                active_->endDrag();
            } else {
                // No movement: a plain pick of that point.
                active_->selectAt(pressSketch_, kPickPixels / sketchPixelsPerMm(vp_->camera(), vp_->height()));
                (void)pt;
            }
            refresh();
            return true;
        }
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
        if (dragPoint_ && here) {
            const QPointF d = me->position() - pressPx_;
            if (!draggingPoint_ && std::hypot(d.x(), d.y()) > kDragPixels) {
                draggingPoint_ = active_->beginDrag(*dragPoint_).has_value();
            }
            if (draggingPoint_) (void)active_->dragTo(*here);
            refresh();
            return true;
        }
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
    case Qt::Key_D:
    case Qt::Key_Delete:
    case Qt::Key_Backspace:
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
    case Qt::Key_D: setTool(SketchTool::Dimension); return true;
    case Qt::Key_Delete:
    case Qt::Key_Backspace:
        deleteSelected();
        return true;
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
