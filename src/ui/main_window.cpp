// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/main_window.hpp"

#include <QAction>
#include <QActionGroup>
#include <QCoreApplication>
#include <QEventLoop>
#include <QDockWidget>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QSettings>
#include <QStatusBar>
#include <QThread>
#include <QTimer>
#include <QToolBar>
#include <QTreeWidget>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>

#include "features/construction_features.hpp"
#include "features/sketch_feature.hpp"
#include "kernel/ops.hpp"
#include "render/demo_scene.hpp"
#include "render/viewport.hpp"
#include "ui/command_actions.hpp"
#include "ui/command_dialog.hpp"
#include "ui/default_commands.hpp"
#include "ui/icons.hpp"
#include "ui/manipulators.hpp"
#include "ui/marking_menu.hpp"
#include "ui/parameters_dialog.hpp"
#include "ui/ribbon.hpp"
#include "ui/sbox.hpp"
#include "ui/sketch/plane_chooser.hpp"
#include "ui/sketch/session.hpp"
#include "ui/sketch/sketch_mode.hpp"
#include "ui/timeline/feature_info.hpp"
#include "ui/timeline/timeline_widget.hpp"

namespace iw::ui {
namespace {

constexpr int kNodeRole = Qt::UserRole + 1;   ///< Assembly node index (int), imported trees only.
constexpr int kBodyRole = Qt::UserRole + 2;   ///< BodyId value (qulonglong) of a body item; absent on groups.
constexpr int kPollIntervalMs = 30;
constexpr int kEvalPollIntervalMs = 8;
constexpr std::uint64_t kImportIdBase = 1ULL << 40;   ///< Synthetic body ids of imported leaves.
const char* const kOpenFilter =
    "CAD files (*.step *.stp *.stl);;STEP (*.step *.stp);;STL (*.stl);;All files (*)";
const char* const kDocFilter = "Ironwood document (*.iwd);;All files (*)";

BodyId importBodyId(std::size_t i) { return BodyId{kImportIdBase + static_cast<std::uint64_t>(i)}; }

QString messageOf(const recompute::FeatureStatus& s) {
    QString out;
    if (s.state == recompute::FeatureStatus::State::Failed) out = QString::fromStdString(s.error.message);
    for (const recompute::Diagnostic& d : s.diagnostics) {
        if (d.severity == recompute::Diagnostic::Severity::Info) continue;
        if (!out.isEmpty()) out += QLatin1Char('\n');
        out += QString::fromStdString(d.message);
    }
    return out;
}

/// Under-constrained warning of a sketch (SketchFeature publishes warning="underconstrained" while DOF > 0). Empty when
/// the sketch is fully constrained or the status carries no such attribute.
QString underconstrainedMessage(const recompute::FeatureStatus& s) {
    const auto warning = s.attributes.find("warning");
    if (warning == s.attributes.end() || warning->second != "underconstrained") return {};
    const auto dof = s.attributes.find("dof");
    const QString count = dof != s.attributes.end() ? QString::fromStdString(dof->second) : QStringLiteral("?");
    return QCoreApplication::translate("MainWindow", "Sketch is not fully constrained (%1 degrees of freedom)").arg(count);
}

/// The icon with the timeline's yellow warning badge ('!') at its top-right corner.
QIcon withWarningBadge(const QIcon& base) {
    QPixmap pm = base.pixmap(16, 16);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF badge(8.0, 0.0, 8.0, 8.0);
    p.setPen(QPen(QColor("#7d6608"), 1));
    p.setBrush(QColor("#f1c40f"));
    p.drawEllipse(badge);
    QFont f = p.font();
    f.setPixelSize(7);
    f.setBold(true);
    p.setFont(f);
    p.setPen(QColor("#202020"));
    p.drawText(badge, Qt::AlignCenter, QStringLiteral("!"));
    return QIcon(pm);
}

} // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle(defaultTitle());
    resize(1280, 800);

    auto* browser = new QDockWidget(tr("Browser"), this);
    browser->setObjectName("browserDock");
    buildBrowser();
    browser->setWidget(tree_);
    addDockWidget(Qt::LeftDockWidgetArea, browser);

    timelineDock_ = new QDockWidget(tr("Timeline"), this);
    timelineDock_->setObjectName("timelineDock");
    timeline_ = new timeline::TimelineWidget(timelineDock_);
    timelineDock_->setWidget(timeline_);
    addDockWidget(Qt::BottomDockWidgetArea, timelineDock_);
    connect(timeline_, &timeline::TimelineWidget::rollbackPreview, this, [this](std::size_t v) {
        previewMarker_ = v;
        requestEvaluation();
    });
    connect(timeline_, &timeline::TimelineWidget::rollbackCommitted, this, [this](std::size_t v) {
        previewMarker_.reset();
        if (!model_) return;
        const std::size_t size = model_->timeline().size();
        if (v == model_->timeline().rollbackMarker()) return;
        if (auto r = model_->setRollbackMarker(v >= size ? std::nullopt : std::optional<std::size_t>(v)); !r)
            setStatus(tr("Rollback failed: %1").arg(QString::fromStdString(r.error().message)));
        requestEvaluation();
    });
    connect(timeline_, &timeline::TimelineWidget::editRequested, this, [this](FeatureId id) {
        std::optional<std::string> kind;
        if (model_)
            if (auto idx = model_->timeline().indexOf(id)) kind = std::string(model_->timeline().at(*idx)->kind());
        if (!kind) return;
        if (*kind == "sketch") {
            editSketch(id);
            return;
        }
        if (auto r = startFeature(*kind, id); !r) setStatus(tr("Cannot edit: %1").arg(QString::fromStdString(r.error().message)));
    });
    connect(timeline_, &timeline::TimelineWidget::contextRequested, this,
            [this](FeatureId id, QPoint pos) { showTimelineMenu(id, pos); });
    connect(timeline_, &timeline::TimelineWidget::hoverChanged, this,
            [this](std::optional<FeatureId> id) { highlightFeature(id); });

    viewport_ = new render::Viewport(this);
    setCentralWidget(viewport_);

    status_ = new QLabel(this);
    statusBar()->addWidget(status_, 1);
    progressBar_ = new QProgressBar(this);
    progressBar_->setMaximumWidth(220);
    progressBar_->setTextVisible(false);
    progressBar_->hide();
    statusBar()->addPermanentWidget(progressBar_);

    pollTimer_ = new QTimer(this);
    pollTimer_->setInterval(kPollIntervalMs);
    connect(pollTimer_, &QTimer::timeout, this, [this] { pollImport(); });

    evalTimer_ = new QTimer(this);
    evalTimer_->setInterval(kEvalPollIntervalMs);
    connect(evalTimer_, &QTimer::timeout, this, [this] { pollEvaluation(); });

    connect(viewport_, &render::Viewport::hoverChanged, this, [this](const render::PickResult& r) {
        // While a dialog waits for a selection the hint stays first; what is under the cursor follows it.
        if (pickHint_.isEmpty()) status_->setText(describePick(r));
        else status_->setText(r.bodyIndex ? pickHint_ + QStringLiteral(" - ") + describePick(r) : pickHint_);
    });
    connect(viewport_, &render::Viewport::picked, this, [this](const render::PickResult& r) {
        status_->setText(pickHint_.isEmpty() ? describePick(r) : pickHint_);
        routePick(r);
    });

    attachModel(std::make_unique<features::Document>());
    buildActions();
    setupCommands();
    dialogs_ = std::make_unique<FeatureDialogs>(*this);
    showDemoScene();
    requestEvaluation();
}

MainWindow::~MainWindow() {
    cancelImport();   // the JobSystem member waits for running jobs; ask them to stop first
    if (evalJob_) evalJob_->cancel();
}

QString MainWindow::defaultTitle() { return tr("Ironwood - Untitled"); }

QString MainWindow::describePick(const render::PickResult& r) {
    if (!r.bodyIndex) return tr("Nothing under cursor");
    QString s = tr("Body %1").arg(*r.bodyIndex + 1);
    if (r.edgeIndex) s += tr(" - Edge %1").arg(*r.edgeIndex);
    else if (r.faceIndex) s += tr(" - Face %1").arg(*r.faceIndex);
    if (r.point)
        s += tr("   (%1, %2, %3) mm").arg(r.point->x, 0, 'f', 2).arg(r.point->y, 0, 'f', 2).arg(r.point->z, 0, 'f', 2);
    return s;
}

void MainWindow::showDemoScene() {
    demo_ = true;
    std::vector<render::SceneBody> bodies(2);
    bodies[0].id = BodyId{1};
    bodies[1].id = BodyId{2};
    bodies[0].mesh = render::makeBoxMesh({-60, 0, -40}, {60, 40, 40});
    bodies[1].mesh = render::makeBoxMesh({-20, 40, -20}, {20, 90, 20});
    sceneIds_ = {bodies[0].id, bodies[1].id};
    viewport_->setScene(std::move(bodies));
    rebuildBrowser();
}

void MainWindow::setStatus(const QString& text) { status_->setText(text); }

void MainWindow::setPickHint(const QString& hint) {
    pickHint_ = hint;
    if (!hint.isEmpty()) status_->setText(hint);
}

void MainWindow::routePick(const render::PickResult& r) {
    if (pickHandler_) {
        auto handler = pickHandler_;   // the handler may clear itself
        handler(r);
        return;
    }
    recordPick(r);
    selectTreeForBody(r.bodyIndex);
}

void MainWindow::recordPick(const render::PickResult& r) {
    if (!r.bodyIndex) {
        picks_.clear();
        return;
    }
    const auto body = bodyAtIndex(*r.bodyIndex);
    const auto shape = shapeOfScene(*r.bodyIndex);
    if (!body || !shape) return;
    kernel::measure::Entity e;
    e.body = *shape;
    if (r.edgeIndex) {
        e.kind = kernel::measure::EntityKind::Edge;
        e.index = *r.edgeIndex;
    } else if (r.faceIndex) {
        e.kind = kernel::measure::EntityKind::Face;
        e.index = *r.faceIndex;
    } else {
        e.kind = kernel::measure::EntityKind::Body;
    }
    picks_.push_back(std::move(e));
    if (picks_.size() > 2) picks_.erase(picks_.begin());
}

std::optional<kernel::Shape> MainWindow::shapeOfBody(BodyId body) const {
    auto it = shown_.out.eval.bodies.find(body);
    if (it != shown_.out.eval.bodies.end()) return it->second;
    if (!imported_) return std::nullopt;
    for (std::size_t i = 0; i < imported_->leaves.size(); ++i) {
        if (importBodyId(i) != body) continue;
        const ImportedLeaf& leaf = imported_->leaves[i];
        const kernel::Shape& local = imported_->assembly.nodes[leaf.node].shape;
        // Placed in world space, so measurements match the viewport. Sub-shape indices are unchanged by the transform.
        auto placed = kernel::transform(local, leaf.world);
        return placed ? placed->shape : local;
    }
    return std::nullopt;
}

std::optional<kernel::Shape> MainWindow::shapeOfScene(std::uint32_t index) const {
    if (index >= sceneIds_.size()) return std::nullopt;
    return shapeOfBody(sceneIds_[index]);
}

std::optional<kernel::Shape> MainWindow::activeBodyShape() const {
    if (!picks_.empty()) return picks_.back().body;
    if (sceneIds_.size() == 1) return shapeOfScene(0);
    return std::nullopt;
}

void MainWindow::applySectionRequest(const kernel::measure::section::RenderRequest& request) {
    sectionRequest_ = request;
    render::SectionRequest r;   // same fields, field by field
    r.enabled = request.enabled;
    r.clipPlane = request.clipPlane;
    r.capColour = request.capColour;
    viewport_->setSectionRequest(r);
    setStatus(request.enabled ? tr("Section analysis on") : tr("Section analysis off"));
}

void MainWindow::setOverlayFaces(std::vector<render::OverlayFace> faces) {
    viewport_->setOverlayFaces(std::move(faces));
}

void MainWindow::startSketch(const sketch::SketchPlane& plane, const QString& name) {
    if (!sketch_) return;
    // start() finishes a sketch that is still active (its finish commits it) before starting the new one.
    sketch_->start(plane, name);
}

void MainWindow::editSketch(FeatureId id) {
    if (!sketch_ || !model_) return;
    auto idx = model_->timeline().indexOf(id);
    if (!idx) return;
    const std::string kind = std::string(model_->timeline().at(*idx)->kind());
    if (kind != "sketch") {
        setStatus(tr("Cannot edit: not a sketch"));
        return;
    }
    auto sk = model_->sketchForEdit(id);
    if (!sk) {
        setStatus(tr("Cannot edit sketch: %1").arg(QString::fromStdString(sk.error().message)));
        return;
    }
    // startEdit finishes a sketch that is still active first (its finish is handled before editingSketch_ changes).
    sketch_->startEdit(std::move(*sk), tr("Sketch %1").arg(static_cast<qulonglong>(id.value())));
    editingSketch_ = id;
}

QString MainWindow::bodyTitle(BodyId body) const {
    std::size_t k = 0;
    for (const auto& [id, shape] : shown_.out.eval.bodies) {
        (void)shape;
        ++k;
        if (id == body) return tr("Body %1").arg(k);
    }
    return tr("Body ?");
}

std::optional<BodyId> MainWindow::bodyAtIndex(std::uint32_t index) const {
    if (index < sceneIds_.size()) return sceneIds_[index];
    return std::nullopt;
}

std::optional<std::size_t> MainWindow::sceneIndexOf(BodyId body) const {
    for (std::size_t i = 0; i < sceneIds_.size(); ++i)
        if (sceneIds_[i] == body) return i;
    return std::nullopt;
}

void MainWindow::buildBrowser() {
    tree_ = new QTreeWidget;
    tree_->setObjectName("browserTree");
    tree_->setHeaderLabels({tr("Name")});
    tree_->header()->setStretchLastSection(true);
    tree_->setSelectionMode(QAbstractItemView::SingleSelection);
    connect(tree_, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* cur, QTreeWidgetItem*) {
        if (syncing_) return;
        render::PickResult r;
        if (auto body = bodyOfItem(cur))
            if (auto idx = sceneIndexOf(*body)) r.bodyIndex = static_cast<std::uint32_t>(*idx);
        viewport_->setSelection(r);
    });
    // Check state propagates to children through ItemIsAutoTristate, so leaves always reflect their own box.
    connect(tree_, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int column) {
        if (syncing_ || column != 0) return;
        auto body = bodyOfItem(item);
        if (!body) return;
        const bool visible = item->checkState(0) == Qt::Checked;
        if (visible) hidden_.erase(*body);
        else hidden_.insert(*body);
        if (auto idx = sceneIndexOf(*body)) viewport_->setBodyVisible(*idx, visible);
    });
}

std::optional<BodyId> MainWindow::bodyOfItem(const QTreeWidgetItem* item) const {
    if (!item || !item->data(0, kBodyRole).isValid()) return std::nullopt;
    return BodyId{item->data(0, kBodyRole).toULongLong()};
}

void MainWindow::selectTreeForBody(std::optional<std::uint32_t> body) {
    syncing_ = true;
    QTreeWidgetItem* item = nullptr;
    if (body && *body < sceneIds_.size()) {
        auto it = bodyItems_.find(sceneIds_[*body].value());
        if (it != bodyItems_.end()) item = it->second;
    }
    tree_->setCurrentItem(item);
    if (item) tree_->scrollToItem(item);
    syncing_ = false;
}

// ---- document -----------------------------------------------------------------------------------------------

void MainWindow::attachModel(std::unique_ptr<features::Document> doc) {
    // Parameter dialogs follow the document they were opened on: release them before it is replaced.
    for (ParametersDialog* dialog : findChildren<ParametersDialog*>()) dialog->detach();
    model_ = std::move(doc);
    cmdCtx_.doc = model_.get();
    picks_.clear();
    model_->addListener([this] {
        updateTitle();
        syncParameters();
        requestEvaluation();
    });
    paramsHash_ = parameterHash();
    // The sketches of this document evaluate their dimensions against its parameter table (stable address: the
    // document owns it and undo edits it in place).
    if (sketch_) sketch_->setParameters(&model_->parameters());
    updateTitle();
}

std::uint64_t MainWindow::parameterHash() const {
    if (!model_) return 0;
    std::vector<std::string> names;
    for (const Parameter& p : model_->parameters().parameters()) names.push_back(p.name);
    return model_->parameters().hashOfUsed(names);
}

void MainWindow::syncParameters() {
    if (!model_ || !sketch_) return;
    const std::uint64_t h = parameterHash();
    if (h == paramsHash_) return;
    paramsHash_ = h;
    sketch_->parametersChanged();
}

void MainWindow::updateTitle() {
    const QString name = docPath_.isEmpty() ? QStringLiteral("Untitled") : QFileInfo(docPath_).completeBaseName();
    QString title = tr("Ironwood - %1").arg(name);
    if (model_ && !model_->isClean()) title += QStringLiteral(" *");
    setWindowTitle(title);
}

void MainWindow::newDocument() {
    attachModel(std::make_unique<features::Document>());
    hidden_.clear();
    imported_.reset();
    docPath_.clear();
    demo_ = false;
    previewMarker_.reset();
    requestEvaluation();
    setStatus(tr("New document"));
}

Result<void> MainWindow::openDocument(const QString& path) {
    auto fresh = std::make_unique<features::Document>();
    if (auto r = fresh->load(std::filesystem::path(path.toStdString())); !r) return std::unexpected(r.error());
    attachModel(std::move(fresh));
    hidden_.clear();
    imported_.reset();
    docPath_ = path;
    demo_ = false;
    previewMarker_.reset();
    requestEvaluation();
    updateTitle();
    return {};
}

Result<void> MainWindow::saveDocument(const QString& path) {
    if (!model_) return makeError(ErrorCode::InvalidArgument, "no document");
    const std::filesystem::path p(path.toStdString());
    if (auto r = model_->save(p); !r) return r;
    docPath_ = path;
    model_->setName(p.stem().string());
    updateTitle();
    return {};
}

bool MainWindow::modelCanUndo() const { return model_ && model_->canUndo(); }
bool MainWindow::modelCanRedo() const { return model_ && model_->canRedo(); }

void MainWindow::modelUndo() {
    if (!model_) return;
    if (auto r = model_->undo(); !r) setStatus(tr("Undo failed: %1").arg(QString::fromStdString(r.error().message)));
}

void MainWindow::modelRedo() {
    if (!model_) return;
    if (auto r = model_->redo(); !r) setStatus(tr("Redo failed: %1").arg(QString::fromStdString(r.error().message)));
}

Result<void> MainWindow::startFeature(const std::string& kind, std::optional<FeatureId> editing) {
    if (!dialogs_) return makeError(ErrorCode::Internal, "dialogs not ready");
    auto r = dialogs_->start(kind, editing);
    if (!r) setStatus(QString::fromStdString(r.error().message));
    return r;
}

Result<void> MainWindow::commitFeature(std::shared_ptr<const recompute::Feature> feature, bool editing) {
    if (!model_) return makeError(ErrorCode::InvalidArgument, "no document");
    if (editing) {
        // Construction features have typed edits (they keep their id and their place in the timeline).
        if (auto p = std::dynamic_pointer_cast<const features::ConstructionPlaneFeature>(feature))
            return model_->editConstructionPlane(p->id(), p->params());
        if (auto a = std::dynamic_pointer_cast<const features::ConstructionAxisFeature>(feature))
            return model_->editConstructionAxis(a->id(), a->params());
        if (auto pt = std::dynamic_pointer_cast<const features::ConstructionPointFeature>(feature))
            return model_->editConstructionPoint(pt->id(), pt->params());
        return model_->editFeature(std::move(feature));
    }
    // A new feature goes to the end: a rolled-back marker moves to the end first (the dialogs refuse otherwise).
    if (model_->timeline().rollbackMarker() < model_->timeline().size()) {
        if (auto r = model_->setRollbackMarker(std::nullopt); !r) return r;
    }
    return model_->addFeature(std::move(feature));
}

void MainWindow::commitSketch(SketchSession* session) {
    if (!model_ || !session) return;
    // Every sketch holds the reference origin and axes, so only user entities decide whether it is empty.
    const auto& entities = session->sketch().entities();
    const bool hasUserEntity = std::any_of(entities.begin(), entities.end(),
                                           [](const auto& kv) { return !sketch::isReferenceEntity(kv.first); });
    if (!hasUserEntity) {
        setStatus(tr("Empty sketch: nothing added to the timeline"));
        return;
    }
    auto sk = session->toSketch();
    if (!sk) {
        setStatus(tr("Sketch not added: %1").arg(QString::fromStdString(sk.error().message)));
        return;
    }
    features::SketchFeatureParams p{std::move(*sk), std::nullopt};
    auto feature = std::make_shared<features::SketchFeature>(model_->allocateFeatureId(), p);
    if (auto r = commitFeature(feature, false); !r)
        setStatus(tr("Sketch not added: %1").arg(QString::fromStdString(r.error().message)));
}

void MainWindow::commitSketchEdit(FeatureId id, SketchSession* session) {
    if (!model_ || !session) return;
    if (!session->canUndo()) {   // nothing was changed in the session: keep the document (and its undo stack) as is
        setStatus(tr("Sketch unchanged"));
        return;
    }
    auto sk = session->toSketch();
    if (!sk) {
        setStatus(tr("Sketch not changed: %1").arg(QString::fromStdString(sk.error().message)));
        return;
    }
    if (auto r = model_->replaceSketch(id, std::move(*sk)); !r)
        setStatus(tr("Sketch not changed: %1").arg(QString::fromStdString(r.error().message)));
}

void MainWindow::requestEvaluation() {
    if (!model_) return;
    recompute::Timeline tl = model_->timeline();
    if (previewMarker_) tl.setRollbackMarker(*previewMarker_);
    ParameterTable params = model_->parameters();
    auto evaluator = evaluator_;
    evalJob_ = evalLatest_.submit(Priority::Normal, [tl = std::move(tl), params = std::move(params), evaluator](CancelToken token) mutable
                                                       -> Result<EvalJob> {
        auto out = timeline::evaluateAndTessellate(tl, params, *evaluator, token);
        if (!out) return std::unexpected(out.error());
        return EvalJob{std::move(tl), std::move(*out)};
    });
    evalTimer_->start();
}

void MainWindow::pollEvaluation() {
    if (!evalJob_ || !evalJob_->ready()) return;
    Result<EvalJob> res = evalJob_->wait();   // ready: returns immediately
    evalJob_.reset();
    evalTimer_->stop();
    if (!res) {
        if (res.error().code != ErrorCode::Cancelled)
            setStatus(tr("Evaluation failed: %1").arg(QString::fromStdString(res.error().message)));
        return;
    }
    applyEvaluation(std::move(*res));
}

bool MainWindow::settle(int timeoutMs) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (evalJob_ && std::chrono::steady_clock::now() < deadline) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        pollEvaluation();
        QThread::msleep(2);
    }
    return !evalJob_;
}

void MainWindow::applyEvaluation(EvalJob job) {
    if (job.timeline.size() > 0) demo_ = false;
    shown_ = std::move(job);
    applyScene();
    rebuildTimeline();
    rebuildBrowser();

    const auto& out = shown_.out;
    int failed = 0;
    QString firstFailure;
    for (std::size_t i = 0; i < out.eval.statuses.size() && i < shown_.timeline.size(); ++i) {
        const auto& s = out.eval.statuses[i];
        if (s.state != recompute::FeatureStatus::State::Failed) continue;
        ++failed;
        if (firstFailure.isEmpty())
            firstFailure = tr("%1: %2").arg(QString::fromStdString(timeline::displayNames(shown_.timeline)[i]))
                               .arg(QString::fromStdString(s.error.message));
    }
    if (failed > 0) {
        setStatus(tr("%1 feature(s) failed. %2").arg(failed).arg(firstFailure));
    } else {
        setStatus(tr("Evaluated %1 feature(s): %2 computed, %3 reused, %4 ms")
                      .arg(shown_.timeline.size())
                      .arg(out.eval.computedCount)
                      .arg(out.eval.reusedCount)
                      .arg(static_cast<int>(std::lround(out.evalMs + out.tessellateMs))));
    }
}

void MainWindow::applyScene() {
    if (demo_) return;
    std::vector<render::SceneBody> bodies;
    sceneIds_.clear();
    for (const auto& [id, mesh] : shown_.out.meshes) {
        render::SceneBody b;
        b.id = id;
        b.mesh = mesh;
        b.visible = !hidden_.contains(id);
        bodies.push_back(std::move(b));
        sceneIds_.push_back(id);
    }
    if (imported_) {
        for (std::size_t i = 0; i < imported_->leaves.size(); ++i) {
            const ImportedLeaf& leaf = imported_->leaves[i];
            render::SceneBody b;
            b.id = importBodyId(i);
            b.mesh = leaf.mesh;
            b.transform = leaf.world;
            b.color = leaf.color;
            b.visible = !hidden_.contains(b.id);
            bodies.push_back(std::move(b));
            sceneIds_.push_back(importBodyId(i));
        }
    }
    viewport_->setScene(std::move(bodies));
}

void MainWindow::rebuildTimeline() {
    const std::vector<std::string> names = timeline::displayNames(shown_.timeline);
    // Labels follow the live document, so a rename shows at once; the statuses still belong to the last evaluation.
    const bool live = model_ != nullptr;
    const std::vector<std::string> liveNames = live ? timeline::displayNames(model_->timeline()) : std::vector<std::string>{};

    std::vector<timeline::TimelineEntry> entries;
    for (std::size_t i = 0; i < shown_.timeline.size(); ++i) {
        const recompute::Feature& f = *shown_.timeline.at(i);
        timeline::TimelineEntry e;
        e.id = f.id();
        e.kind = std::string(f.kind());
        std::string label = names[i];
        if (live)
            if (auto li = model_->timeline().indexOf(f.id())) label = liveNames[*li];
        e.label = QString::fromStdString(label);
        e.family = timeline::iconFamilyOf(f.kind());
        e.icon = timeline::timelineIconName(f.kind());
        e.suppressed = shown_.timeline.isSuppressed(f.id());
        if (i < shown_.out.eval.statuses.size()) {
            const auto& s = shown_.out.eval.statuses[i];
            e.state = s.state;
            e.message = messageOf(s);
            for (const auto& d : s.diagnostics)
                if (d.severity == recompute::Diagnostic::Severity::Warning) e.warning = true;
            const QString underconstrained = underconstrainedMessage(s);
            if (!underconstrained.isEmpty()) {
                e.warning = true;
                e.message = e.message.isEmpty() ? underconstrained : underconstrained + QLatin1Char('\n') + e.message;
            }
        }
        if (e.state == recompute::FeatureStatus::State::Failed) e.warning = false;
        entries.push_back(std::move(e));
    }
    timeline_->setEntries(std::move(entries), shown_.timeline.rollbackMarker());
}

void MainWindow::highlightFeature(std::optional<FeatureId> id) {
    render::PickResult r;
    if (id) {
        if (auto idx = shown_.timeline.indexOf(*id)) {
            if (auto body = timeline::producedBody(*shown_.timeline.at(*idx)))
                if (auto scene = sceneIndexOf(*body)) r.bodyIndex = static_cast<std::uint32_t>(*scene);
        }
    }
    viewport_->setSelection(r);
}

void MainWindow::showTimelineMenu(FeatureId id, QPoint globalPos) {
    if (!model_) return;
    auto idx = model_->timeline().indexOf(id);
    if (!idx) return;
    const std::string kind = std::string(model_->timeline().at(*idx)->kind());
    const bool suppressed = model_->timeline().isSuppressed(id);

    QMenu menu(this);
    QAction* edit = menu.addAction(kind == "sketch" ? tr("Edit Sketch") : tr("Edit..."));
    QAction* sup = menu.addAction(suppressed ? tr("Unsuppress") : tr("Suppress"));
    QAction* del = menu.addAction(tr("Delete"));
    QAction* ren = menu.addAction(tr("Rename..."));
    QAction* chosen = menu.exec(globalPos);
    if (!chosen) return;
    if (chosen == edit) runTimelineAction(id, TimelineAction::Edit);
    else if (chosen == sup) runTimelineAction(id, TimelineAction::Suppress);
    else if (chosen == del) runTimelineAction(id, TimelineAction::Delete);
    else if (chosen == ren) runTimelineAction(id, TimelineAction::Rename);
}

void MainWindow::runTimelineAction(FeatureId id, TimelineAction action) {
    if (!model_) return;
    auto idx = model_->timeline().indexOf(id);
    if (!idx) return;
    const std::string kind = std::string(model_->timeline().at(*idx)->kind());
    const bool suppressed = model_->timeline().isSuppressed(id);
    switch (action) {
    case TimelineAction::Edit:
        if (kind == "sketch") {
            editSketch(id);
        } else if (auto r = startFeature(kind, id); !r) {
            setStatus(tr("Cannot edit: %1").arg(QString::fromStdString(r.error().message)));
        }
        break;
    case TimelineAction::Suppress: setFeatureSuppressed(id, !suppressed); break;
    case TimelineAction::Delete: deleteFeature(id); break;
    case TimelineAction::Rename: renameFeature(id); break;
    }
}

void MainWindow::renameFeature(FeatureId id) {
    if (!model_) return;
    bool ok = false;
    const QString text = QInputDialog::getText(this, tr("Rename feature"), tr("Name"), QLineEdit::Normal,
                                               QString(), &ok);
    if (!ok || text.trimmed().isEmpty()) return;
    renameFeatureTo(id, text.trimmed().toStdString());
}

void MainWindow::setFeatureSuppressed(FeatureId id, bool suppressed) {
    if (!model_) return;
    if (auto r = model_->setSuppressed(id, suppressed); !r)
        setStatus(tr("Suppress failed: %1").arg(QString::fromStdString(r.error().message)));
}

void MainWindow::renameFeatureTo(FeatureId id, std::string name) {
    if (!model_ || name.empty()) return;
    if (auto r = model_->renameFeature(id, std::move(name)); !r) {
        setStatus(tr("Rename failed: %1").arg(QString::fromStdString(r.error().message)));
        return;
    }
    rebuildTimeline();
}

void MainWindow::deleteFeature(FeatureId id) {
    if (!model_) return;
    if (auto r = model_->deleteFeature(id); !r)
        setStatus(tr("Delete failed: %1").arg(QString::fromStdString(r.error().message)));
}

// ---- import -------------------------------------------------------------------------------------------------

bool MainWindow::openFile(const QString& path) {
    const std::filesystem::path p(path.toStdString());
    if (!isSupportedCadFile(p)) {
        status_->setText(tr("Unsupported file type: %1").arg(QFileInfo(path).fileName()));
        return false;
    }
    cancelImport();
    progress_ = std::make_shared<ImportProgress>();
    importName_ = QFileInfo(path).fileName();
    auto progress = progress_;
    job_ = jobs_.submit(Priority::Normal, [p, progress](CancelToken token) {
        return loadModel(p, token, *progress);
    });
    setImportUi(true);
    status_->setText(tr("Importing %1...").arg(importName_));
    pollTimer_->start();
    return true;
}

void MainWindow::cancelImport() {
    if (!job_) return;
    job_->cancel();
    job_.reset();
    progress_.reset();
    pollTimer_->stop();
    setImportUi(false);
    status_->setText(tr("Import cancelled"));
}

void MainWindow::setImportUi(bool active) {
    progressBar_->setVisible(active);
    if (active) {
        progressBar_->setRange(0, 0);   // busy until the total is known
        progressBar_->setValue(0);
    }
    if (cancelAction_) cancelAction_->setEnabled(active);
}

void MainWindow::pollImport() {
    if (!job_ || !progress_) {
        pollTimer_->stop();
        return;
    }
    const auto total = progress_->total.load();
    if (progress_->stage.load() == 1 && total > 0) {
        progressBar_->setRange(0, static_cast<int>(total));
        progressBar_->setValue(static_cast<int>(progress_->done.load()));
        status_->setText(tr("Tessellating %1: %2 / %3").arg(importName_).arg(progress_->done.load()).arg(total));
    }
    if (!job_->ready()) return;   // never block the GUI thread on the job

    const Result<LoadedModel>& result = job_->wait();   // ready: returns immediately
    LoadedModel model;
    const bool ok = static_cast<bool>(result);
    if (ok) model = *result;
    const Error error = ok ? Error{} : result.error();
    job_.reset();
    progress_.reset();
    pollTimer_->stop();
    setImportUi(false);

    if (!ok) {
        status_->setText(error.code == ErrorCode::Cancelled ? tr("Import cancelled")
                                                            : tr("Import failed: %1").arg(QString::fromStdString(error.message)));
        return;
    }
    lastReport_ = tr("Imported %1 bodies from %2: import %3 ms, tessellate %4 ms")
                      .arg(model.leaves.size())
                      .arg(model.sourceName)
                      .arg(static_cast<int>(std::lround(model.importMs)))
                      .arg(static_cast<int>(std::lround(model.tessellateMs)));
    status_->setText(lastReport_);
    setWindowTitle(tr("Ironwood - %1").arg(model.sourceName));
    applyModel(std::move(model));
}

void MainWindow::applyModel(LoadedModel model) {
    imported_ = std::move(model);
    demo_ = false;
    applyScene();
    rebuildBrowser();
    viewport_->viewFit(0);
}

// ---- browser ------------------------------------------------------------------------------------------------

void MainWindow::rebuildBrowser() {
    syncing_ = true;
    tree_->clear();
    bodyItems_.clear();
    importRoot_ = nullptr;
    if (demo_) {
        syncing_ = false;
        return;
    }

    // Browser icons from the icon set, drawn for the palette (the same theme rule as the ribbon).
    const IconTheme theme = iconThemeFor(palette());
    auto browserIcon = [theme](const char* name) { return icon(name, 16, theme); };
    auto group = [this, &browserIcon](const QString& text) {
        auto* item = new QTreeWidgetItem(tree_, QStringList{text});
        item->setIcon(0, browserIcon("browser_folder"));
        return item;
    };
    auto leaf = [&browserIcon](QTreeWidgetItem* parent, const QString& text, const char* iconName) {
        auto* item = new QTreeWidgetItem(parent, QStringList{text});
        item->setFlags(item->flags() & ~Qt::ItemIsUserCheckable);
        item->setIcon(0, browserIcon(iconName));
        return item;
    };

    QTreeWidgetItem* origin = group(tr("Origin"));
    for (const char* n : {"XY plane", "YZ plane", "ZX plane"}) leaf(origin, QString::fromLatin1(n), "browser_plane");
    for (const char* n : {"X axis", "Y axis", "Z axis"}) leaf(origin, QString::fromLatin1(n), "browser_axis");

    std::vector<std::string> names = timeline::displayNames(shown_.timeline);
    QTreeWidgetItem* sketches = group(tr("Sketches"));
    for (std::size_t i = 0; i < shown_.timeline.size(); ++i) {
        if (std::string(shown_.timeline.at(i)->kind()) != "sketch") continue;
        QTreeWidgetItem* item = leaf(sketches, QString::fromStdString(names[i]), "browser_sketch");
        // Under-constrained sketch: a small warning marker on the item, the same text as its timeline hover.
        const QString underconstrained = i < shown_.out.eval.statuses.size() ? underconstrainedMessage(shown_.out.eval.statuses[i]) : QString{};
        if (!underconstrained.isEmpty()) {
            item->setIcon(0, withWarningBadge(item->icon(0)));
            item->setToolTip(0, underconstrained);
        }
    }

    QTreeWidgetItem* bodies = group(tr("Bodies"));
    std::size_t k = 0;
    for (const auto& [id, mesh] : shown_.out.meshes) {
        (void)mesh;
        ++k;
        auto* item = new QTreeWidgetItem(bodies, QStringList{tr("Body %1").arg(k)});
        item->setIcon(0, browserIcon("browser_body"));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setData(0, kBodyRole, static_cast<qulonglong>(id.value()));
        item->setCheckState(0, hidden_.contains(id) ? Qt::Unchecked : Qt::Checked);
        bodyItems_[id.value()] = item;
    }

    QTreeWidgetItem* construction = group(tr("Construction"));
    for (std::size_t i = 0; i < shown_.timeline.size(); ++i) {
        const std::string kind(shown_.timeline.at(i)->kind());
        if (kind == "plane") leaf(construction, QString::fromStdString(names[i]), "browser_plane");
        else if (kind == "axis") leaf(construction, QString::fromStdString(names[i]), "browser_axis");
        else if (kind == "point") leaf(construction, QString::fromStdString(names[i]), "browser_origin");
    }

    if (imported_) {
        // The assembly's own roots are top-level items of the browser (as before the document existed).
        const kernel::ImportedAssembly& assembly = imported_->assembly;
        std::vector<int> bodyOfNode(assembly.nodes.size(), -1);
        for (std::size_t i = 0; i < imported_->leaves.size(); ++i)
            bodyOfNode[imported_->leaves[i].node] = static_cast<int>(i);
        auto addNode = [&](auto&& self, std::uint32_t idx, QTreeWidgetItem* parent) -> void {
            const kernel::AssemblyNode& n = assembly.nodes[idx];
            auto* item = parent ? new QTreeWidgetItem(parent) : new QTreeWidgetItem(tree_);
            item->setText(0, n.name.empty() ? tr("(unnamed %1)").arg(idx) : QString::fromStdString(n.name));
            item->setData(0, kNodeRole, static_cast<int>(idx));
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            if (bodyOfNode[idx] >= 0) {
                const BodyId id = importBodyId(static_cast<std::size_t>(bodyOfNode[idx]));
                item->setData(0, kBodyRole, static_cast<qulonglong>(id.value()));
                bodyItems_[id.value()] = item;
                item->setCheckState(0, hidden_.contains(id) ? Qt::Unchecked : Qt::Checked);
            }
            if (!n.children.empty()) item->setFlags(item->flags() | Qt::ItemIsAutoTristate);
            for (std::uint32_t c : n.children) self(self, c, item);
        };
        for (std::uint32_t r : assembly.roots) addNode(addNode, r, nullptr);
    }
    tree_->expandAll();
    syncing_ = false;
}

// ---- actions ------------------------------------------------------------------------------------------------

void MainWindow::buildActions() {
    auto* file = menuBar()->addMenu(tr("&File"));
    auto* fileNew = new QAction(tr("&New"), this);
    fileNew->setShortcut(QKeySequence::New);
    connect(fileNew, &QAction::triggered, this, [this] { newDocument(); });
    auto* fileOpen = new QAction(tr("&Open document..."), this);
    fileOpen->setShortcut(QKeySequence::Open);
    connect(fileOpen, &QAction::triggered, this, [this] {
        const QString dir = QFileDialog::getExistingDirectory(this, tr("Open document"));
        if (dir.isEmpty()) return;
        if (auto r = openDocument(dir); !r) setStatus(tr("Open failed: %1").arg(QString::fromStdString(r.error().message)));
        else setStatus(tr("Opened %1").arg(QFileInfo(dir).fileName()));
    });
    auto* fileSave = new QAction(tr("&Save"), this);
    fileSave->setShortcut(QKeySequence::Save);
    connect(fileSave, &QAction::triggered, this, [this] {
        if (!hasDocumentPath()) {
            QAction* saveAs = findChild<QAction*>("fileSaveAs");
            if (saveAs) saveAs->trigger();
            return;
        }
        if (auto r = saveDocument(docPath_); !r) setStatus(tr("Save failed: %1").arg(QString::fromStdString(r.error().message)));
        else setStatus(tr("Saved %1").arg(docPath_));
    });
    auto* fileSaveAs = new QAction(tr("Save &As..."), this);
    fileSaveAs->setObjectName("fileSaveAs");
    fileSaveAs->setShortcut(QKeySequence::SaveAs);
    connect(fileSaveAs, &QAction::triggered, this, [this] {
        const QString path = QFileDialog::getSaveFileName(this, tr("Save document"), QString(), tr(kDocFilter));
        if (path.isEmpty()) return;
        if (auto r = saveDocument(path); !r) setStatus(tr("Save failed: %1").arg(QString::fromStdString(r.error().message)));
        else setStatus(tr("Saved %1").arg(path));
    });

    auto* open = new QAction(tr("&Import..."), this);
    open->setShortcut(QKeySequence(QStringLiteral("Ctrl+I")));
    open->setToolTip(tr("Import a STEP or STL file as separate bodies (Ctrl+I)"));
    connect(open, &QAction::triggered, this, [this] { chooseAndOpenFile(); });
    openAction_ = open;
    cancelAction_ = new QAction(tr("&Cancel import"), this);
    cancelAction_->setShortcut(Qt::Key_Escape);
    cancelAction_->setEnabled(false);
    connect(cancelAction_, &QAction::triggered, this, [this] { cancelImport(); });
    addAction(cancelAction_);
    file->addAction(fileNew);
    file->addAction(fileOpen);
    file->addAction(fileSave);
    file->addAction(fileSaveAs);
    file->addSeparator();
    file->addAction(open);
    file->addAction(cancelAction_);

    auto* view = menuBar()->addMenu(tr("&View"));
    auto* bar = addToolBar(tr("View"));
    bar->setObjectName("viewToolbar");

    auto* home = new QAction(tr("&Home"), this);
    home->setShortcut(Qt::Key_Home);
    home->setToolTip(tr("Home view (Home)"));
    connect(home, &QAction::triggered, this, [this] { viewport_->viewHome(); });
    auto* fit = new QAction(tr("&Fit"), this);
    fit->setToolTip(tr("Fit all (F)"));
    connect(fit, &QAction::triggered, this, [this] { viewport_->viewFit(); });

    auto* ortho = new QAction(tr("&Orthographic"), this);
    ortho->setCheckable(true);
    ortho->setToolTip(tr("Toggle orthographic / perspective projection"));
    connect(ortho, &QAction::toggled, this, [this](bool on) {
        viewport_->setProjectionMode(on ? render::Projection::Orthographic : render::Projection::Perspective);
    });

    auto* cube = new QAction(tr("View&Cube"), this);
    cube->setCheckable(true);
    cube->setChecked(viewport_->viewCubeVisible());
    connect(cube, &QAction::toggled, this, [this](bool on) { viewport_->setViewCubeVisible(on); });

    auto* upGroup = new QActionGroup(this);
    auto* yUp = new QAction(tr("&Y up"), upGroup);
    auto* zUp = new QAction(tr("&Z up"), upGroup);
    yUp->setCheckable(true);
    zUp->setCheckable(true);
    yUp->setChecked(viewport_->upAxis() == render::UpAxis::Y);
    zUp->setChecked(viewport_->upAxis() == render::UpAxis::Z);
    connect(yUp, &QAction::triggered, this, [this] { viewport_->setUpAxis(render::UpAxis::Y); });
    connect(zUp, &QAction::triggered, this, [this] { viewport_->setUpAxis(render::UpAxis::Z); });

    view->addAction(home);
    view->addAction(fit);
    view->addSeparator();
    view->addAction(ortho);
    view->addAction(cube);
    auto* up = view->addMenu(tr("Up &axis"));
    up->addAction(yUp);
    up->addAction(zUp);

    auto* nav = view->addMenu(tr("&Navigation"));
    auto* navGroup = new QActionGroup(this);
    using render::NavPreset;
    for (NavPreset p : {NavPreset::Fusion, NavPreset::Inventor, NavPreset::SolidWorks, NavPreset::Alias,
                        NavPreset::Tinkercad}) {
        auto* a = new QAction(QString::fromUtf8(render::navMappingFor(p).name.data(),
                                                static_cast<qsizetype>(render::navMappingFor(p).name.size())),
                              navGroup);
        a->setCheckable(true);
        a->setChecked(p == viewport_->navPreset());
        connect(a, &QAction::triggered, this, [this, p] { viewport_->setNavPreset(p); });
        nav->addAction(a);
    }

    bar->addAction(home);
    bar->addAction(fit);
    bar->addSeparator();
    bar->addAction(ortho);
    bar->addAction(cube);
    bar->addSeparator();
    bar->addAction(yUp);
    bar->addAction(zUp);
}

void MainWindow::chooseAndOpenFile() {
    const QString path = QFileDialog::getOpenFileName(this, tr("Import"), QString(), tr(kOpenFilter));
    if (!path.isEmpty()) openFile(path);
}

void MainWindow::setupCommands() {
    cmdCtx_.parent = this;
    cmdCtx_.doc = model_.get();
    cmdCtx_.jobs = &jobs_;
    cmdCtx_.importFile = [this] { chooseAndOpenFile(); };
    cmdCtx_.showPanel = [this](QWidget* panel) { showCommandPanel(panel); };
    cmdCtx_.openCommandSearch = [this] { openCommandSearch(); };

    if (auto r = registerDefaultCommands(commands_); !r)
        status_->setText(tr("Command setup failed: %1").arg(QString::fromStdString(r.error().message)));
    // B6 Inspect: Measure, Properties, Section Analysis. Providers read the shell's picks and bodies.
    inspect_.selection = [this] { return picks_; };
    inspect_.activeBody = [this] { return activeBodyShape(); };
    inspect_.sectionSink = [this](const kernel::measure::section::RenderRequest& r) { applySectionRequest(r); };
    if (auto r = inspect::registerCommands(commands_, inspect_); !r)
        status_->setText(tr("Inspect setup failed: %1").arg(QString::fromStdString(r.error().message)));
    QSettings settings;
    commands_.loadShortcuts(settings);
    pinned_.clear();
    for (const QString& id : settings.value(QStringLiteral("toolbar/pinned")).toStringList())
        if (commands_.find(id.toStdString())) pinned_.push_back(id.toStdString());

    actions_ = new CommandActions(commands_, cmdCtx_, this);
    for (const Command& c : commands_.all()) addAction(actions_->action(c.id));
    connect(actions_, &CommandActions::failed, this, [this](const QString& m) { status_->setText(m); });

    auto* edit = menuBar()->addMenu(tr("&Edit"));
    edit->addAction(actions_->action("undo"));
    edit->addAction(actions_->action("redo"));
    auto* modify = menuBar()->addMenu(tr("&Modify"));
    modify->addAction(actions_->action("parameters"));

    auto* ribbonBar = addToolBar(tr("Ribbon"));
    ribbonBar->setObjectName("ribbonToolbar");
    ribbonBar->setMovable(false);
    ribbon_ = new RibbonBar(commands_, *actions_, this);
    ribbonBar->addWidget(ribbon_);
    ribbon_->setPinned(pinned_);

    palette_ = new CommandPalette(commands_, this);
    palette_->setRecent([&] {
        std::vector<std::string> ids;
        for (const QString& id : settings.value(QStringLiteral("commands/recent")).toStringList())
            if (commands_.find(id.toStdString())) ids.push_back(id.toStdString());
        return ids;
    }());
    palette_->hide();
    connect(palette_, &CommandPalette::commandChosen, this, [this](const QString& id) {
        QSettings s;
        QStringList recent;
        for (const std::string& r : palette_->recent()) recent << QString::fromStdString(r);
        s.setValue(QStringLiteral("commands/recent"), recent);
        actions_->run(id.toStdString());
    });
    connect(palette_, &CommandPalette::pinRequested, this,
            [this](const QString& id) { pinCommand(id.toStdString()); });

    marking_ = new MarkingMenu(commands_, cmdCtx_, viewport_);
    marking_->hide();
    connect(marking_, &MarkingMenu::commandChosen, this,
            [this](const QString& id) { actions_->run(id.toStdString()); });

    overlay_ = new ManipulatorOverlay(viewport_);
    overlay_->setGeometry(viewport_->rect());
    viewport_->installEventFilter(this);

    sketch_ = new SketchMode(viewport_, this);
    sketch_->setJobSystem(&jobs_);
    sketch_->setPanelHost([this](QWidget* panel) { showCommandPanel(panel); });
    sketch_->setParameters(&model_->parameters());
    paramsHash_ = parameterHash();
    connect(sketch_, &SketchMode::status, this, [this](const QString& text) { status_->setText(text); });
    // Finishing a sketch (Finish Sketch, Esc) leaves sketch mode: a new sketch becomes a feature, an edited one replaces
    // the sketch of its feature. A cancelled edit changes nothing.
    connect(sketch_, &SketchMode::sketchFinished, this, [this](SketchSession* done, bool committed) {
        const std::optional<FeatureId> edited = editingSketch_;
        editingSketch_.reset();
        if (edited) {
            if (committed) commitSketchEdit(*edited, done);
        } else if (committed) {
            commitSketch(done);
        }
    });

    panelDock_ = new QDockWidget(tr("Command"), this);
    panelDock_->setObjectName("commandDock");
    addDockWidget(Qt::RightDockWidgetArea, panelDock_);
    panelDock_->hide();

    // Enablement depends on document state (undo history, parameters): re-evaluate a few times a second.
    refreshTimer_ = new QTimer(this);
    refreshTimer_->setInterval(250);
    connect(refreshTimer_, &QTimer::timeout, this, [this] { actions_->refresh(); });
    refreshTimer_->start();
    actions_->refresh();
}

void MainWindow::pinCommand(const std::string& id) {
    if (!commands_.find(id)) return;
    if (std::find(pinned_.begin(), pinned_.end(), id) == pinned_.end()) pinned_.push_back(id);
    ribbon_->setPinned(pinned_);
    QSettings settings;
    QStringList ids;
    for (const std::string& p : pinned_) ids << QString::fromStdString(p);
    settings.setValue(QStringLiteral("toolbar/pinned"), ids);
    status_->setText(tr("Pinned %1 to the toolbar").arg(QString::fromStdString(id)));
}

void MainWindow::showCommandPanel(QWidget* panel) {
    if (!panel) return;
    if (QWidget* old = panelDock_->widget()) old->deleteLater();
    panelDock_->setWidget(panel);
    if (auto* p = qobject_cast<CommandPanel*>(panel))
        connect(p, &CommandPanel::finished, this, [this] { panelDock_->hide(); });
    panelDock_->show();
}

void MainWindow::chooseSketchPlane() {
    if (!sketch_) return;
    auto* chooser = new SketchPlaneChooser(this);
    connect(chooser, &SketchPlaneChooser::planeChosen, this, [this](int choice) {
        const auto plane = static_cast<PlaneChoice>(choice);
        panelDock_->hide();
        startSketch(originPlane(plane), planeName(plane));
    });
    connect(chooser, &SketchPlaneChooser::cancelled, this, [this] { panelDock_->hide(); });
    showCommandPanel(chooser);
}

void MainWindow::openCommandSearch() {
    const int w = std::min(480, width() - 40);
    palette_->setGeometry((width() - w) / 2, 80, w, 360);
    palette_->open();
}

void MainWindow::showMarkingMenu(QPoint viewportPos) {
    marking_->popup(viewportPos);
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event) {
    if (watched == viewport_) {
        switch (event->type()) {
        case QEvent::Resize:
            if (overlay_) overlay_->setGeometry(viewport_->rect());
            break;
        case QEvent::KeyPress: {
            // A dialog that waits for a selection: Escape clears the selection, or cancels the dialog when it is empty.
            auto* ke = static_cast<QKeyEvent*>(event);
            if (ke->key() == Qt::Key_Escape && pickHandler_) {
                if (auto* panel = qobject_cast<CommandPanel*>(panelDock_->widget())) {
                    panel->escape();
                    return true;
                }
            }
            break;
        }
        case QEvent::MouseButtonPress: {
            auto* me = static_cast<QMouseEvent*>(event);
            if (me->button() == Qt::RightButton) {
                rightPressed_ = true;
                rightPressPos_ = me->position().toPoint();
            }
            break;
        }
        case QEvent::MouseMove: {
            auto* me = static_cast<QMouseEvent*>(event);
            // A right drag orbits; only a right click (no movement) opens the marking menu.
            if (rightPressed_ && (me->position().toPoint() - rightPressPos_).manhattanLength() > 4)
                rightPressed_ = false;
            break;
        }
        case QEvent::MouseButtonRelease: {
            auto* me = static_cast<QMouseEvent*>(event);
            if (me->button() == Qt::RightButton && rightPressed_) {
                rightPressed_ = false;
                if (pickHandler_) {
                    // A dialog waits for picks: a right click is "done" (OK when it is valid), not the marking menu.
                    if (auto* panel = qobject_cast<CommandPanel*>(panelDock_->widget())) panel->doneWithSelection();
                } else {
                    showMarkingMenu(me->position().toPoint());
                }
            }
            break;
        }
        default:
            break;
        }
    }
    return QMainWindow::eventFilter(watched, event);
}

} // namespace iw::ui
