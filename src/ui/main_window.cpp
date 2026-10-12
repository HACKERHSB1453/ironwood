// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/main_window.hpp"

#include <QAction>
#include <QActionGroup>
#include <QDockWidget>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QHeaderView>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMouseEvent>
#include <QProgressBar>
#include <QSettings>
#include <QStatusBar>
#include <QTimer>
#include <QToolBar>
#include <QTreeWidget>

#include <algorithm>
#include <cmath>
#include <filesystem>

#include "render/demo_scene.hpp"
#include "render/viewport.hpp"
#include "ui/command_actions.hpp"
#include "ui/command_dialog.hpp"
#include "ui/default_commands.hpp"
#include "ui/manipulators.hpp"
#include "ui/marking_menu.hpp"
#include "ui/ribbon.hpp"
#include "ui/sbox.hpp"
#include "ui/sketch/plane_chooser.hpp"
#include "ui/sketch/sketch_mode.hpp"

namespace iw::ui {
namespace {
constexpr int kNodeRole = Qt::UserRole + 1;  ///< Assembly node index (int).
constexpr int kBodyRole = Qt::UserRole + 2;  ///< Viewport body index, -1 for grouping nodes.
constexpr int kPollIntervalMs = 30;
const char* const kOpenFilter =
    "CAD files (*.step *.stp *.stl);;STEP (*.step *.stp);;STL (*.stl);;All files (*)";
} // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle(defaultTitle());
    resize(1280, 800);

    auto* browser = new QDockWidget(tr("Browser"), this);
    browser->setObjectName("browserDock");
    buildBrowser();
    browser->setWidget(tree_);
    addDockWidget(Qt::LeftDockWidgetArea, browser);

    auto* timeline = new QDockWidget(tr("Timeline"), this);
    timeline->setObjectName("timelineDock");
    timeline->setWidget(new QLabel(tr("Timeline placeholder"), timeline));
    addDockWidget(Qt::BottomDockWidgetArea, timeline);

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

    connect(viewport_, &render::Viewport::hoverChanged, this,
            [this](const render::PickResult& r) { status_->setText(describePick(r)); });
    connect(viewport_, &render::Viewport::picked, this, [this](const render::PickResult& r) {
        status_->setText(describePick(r));
        selectTreeForBody(r.bodyIndex);
    });

    buildActions();
    setupCommands();
    showDemoScene();
}

MainWindow::~MainWindow() {
    cancelImport();   // the JobSystem member waits for running jobs; ask them to stop first
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
    std::vector<render::SceneBody> bodies(2);
    bodies[0].id = BodyId{1};
    bodies[1].id = BodyId{2};
    bodies[0].mesh = render::makeBoxMesh({-60, 0, -40}, {60, 40, 40});
    bodies[1].mesh = render::makeBoxMesh({-20, 40, -20}, {20, 90, 20});
    viewport_->setScene(std::move(bodies));
    bodyItems_.clear();
    tree_->clear();
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
        const int body = bodyOf(cur);
        if (body >= 0) r.bodyIndex = static_cast<std::uint32_t>(body);
        viewport_->setSelection(r);
    });
    // Check state propagates to children through ItemIsAutoTristate, so leaves always reflect their own box.
    connect(tree_, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int column) {
        if (syncing_ || column != 0) return;
        const int body = bodyOf(item);
        if (body >= 0) viewport_->setBodyVisible(static_cast<std::size_t>(body), item->checkState(0) == Qt::Checked);
    });
}

int MainWindow::bodyOf(const QTreeWidgetItem* item) const {
    return item ? item->data(0, kBodyRole).toInt() : -1;
}

void MainWindow::selectTreeForBody(std::optional<std::uint32_t> body) {
    syncing_ = true;
    if (body && *body < bodyItems_.size()) {
        tree_->setCurrentItem(bodyItems_[*body]);
        tree_->scrollToItem(bodyItems_[*body]);
    } else {
        tree_->setCurrentItem(nullptr);
    }
    syncing_ = false;
}

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
    syncing_ = true;
    tree_->clear();
    bodyItems_.assign(model.leaves.size(), nullptr);

    std::vector<int> bodyOfNode(model.assembly.nodes.size(), -1);
    for (std::size_t i = 0; i < model.leaves.size(); ++i) bodyOfNode[model.leaves[i].node] = static_cast<int>(i);

    // Recursive over the assembly tree (depth is the assembly nesting, small in practice).
    auto addNode = [&](auto&& self, std::uint32_t idx, QTreeWidgetItem* parent) -> void {
        const kernel::AssemblyNode& n = model.assembly.nodes[idx];
        auto* item = parent ? new QTreeWidgetItem(parent) : new QTreeWidgetItem(tree_);
        item->setText(0, n.name.empty() ? tr("(unnamed %1)").arg(idx) : QString::fromStdString(n.name));
        item->setData(0, kNodeRole, static_cast<int>(idx));
        item->setData(0, kBodyRole, bodyOfNode[idx]);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        if (bodyOfNode[idx] >= 0) bodyItems_[static_cast<std::size_t>(bodyOfNode[idx])] = item;
        item->setCheckState(0, Qt::Checked);
        if (!n.children.empty()) item->setFlags(item->flags() | Qt::ItemIsAutoTristate);
        for (std::uint32_t c : n.children) self(self, c, item);
    };
    for (std::uint32_t r : model.assembly.roots) addNode(addNode, r, nullptr);
    tree_->expandAll();

    std::vector<render::SceneBody> bodies;
    bodies.reserve(model.leaves.size());
    for (std::size_t i = 0; i < model.leaves.size(); ++i) {
        const ImportedLeaf& leaf = model.leaves[i];
        render::SceneBody b;
        b.id = BodyId{static_cast<std::uint64_t>(i + 1)};
        b.mesh = leaf.mesh;
        b.transform = leaf.world;
        b.color = leaf.color;
        bodies.push_back(std::move(b));
    }
    viewport_->setScene(std::move(bodies));
    viewport_->viewFit(0);
    syncing_ = false;
}

void MainWindow::buildActions() {
    auto* file = menuBar()->addMenu(tr("&File"));
    auto* open = new QAction(tr("&Open..."), this);
    open->setShortcut(QKeySequence::Open);
    open->setToolTip(tr("Open a STEP or STL file (Ctrl+O)"));
    connect(open, &QAction::triggered, this, [this] { chooseAndOpenFile(); });
    openAction_ = open;
    cancelAction_ = new QAction(tr("&Cancel import"), this);
    cancelAction_->setShortcut(Qt::Key_Escape);
    cancelAction_->setEnabled(false);
    connect(cancelAction_, &QAction::triggered, this, [this] { cancelImport(); });
    addAction(cancelAction_);
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
    const QString path = QFileDialog::getOpenFileName(this, tr("Open"), QString(), tr(kOpenFilter));
    if (!path.isEmpty()) openFile(path);
}

void MainWindow::setupCommands() {
    cmdCtx_.parent = this;
    cmdCtx_.doc = &doc_;
    cmdCtx_.jobs = &jobs_;
    cmdCtx_.importFile = [this] { chooseAndOpenFile(); };
    cmdCtx_.showPanel = [this](QWidget* panel) { showCommandPanel(panel); };
    cmdCtx_.openCommandSearch = [this] { openCommandSearch(); };

    if (auto r = registerDefaultCommands(commands_); !r)
        status_->setText(tr("Command setup failed: %1").arg(QString::fromStdString(r.error().message)));
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
    connect(sketch_, &SketchMode::status, this, [this](const QString& text) { status_->setText(text); });

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
        sketch_->start(originPlane(plane), planeName(plane));
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
                showMarkingMenu(me->position().toPoint());
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
