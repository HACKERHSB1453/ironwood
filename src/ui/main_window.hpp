// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QMainWindow>
#include <QPoint>
#include <QString>

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include "core/job.hpp"
#include "ui/command.hpp"
#include "ui/doc_state.hpp"
#include "ui/import_job.hpp"

namespace iw::render {
class Viewport;
struct PickResult;
}

class QAction;
class QDockWidget;
class QLabel;
class QProgressBar;
class QTimer;
class QTreeWidget;
class QTreeWidgetItem;

namespace iw::ui {
class CommandActions;
class CommandPalette;
class ManipulatorOverlay;
class MarkingMenu;
class RibbonBar;
class SketchMode;
}

namespace iw::ui {

/// Application shell: File/View menus, toolbar (Home/Fit/projection/up axis/ViewCube/navigation preset), Browser
/// dock (left: assembly tree with visibility checkboxes), Timeline dock (bottom, placeholder), central Viewport and
/// a status bar (pick/hover result, import progress and timing). Opening a file (Ctrl+O or a path argument) runs
/// the kernel import and tessellation on the JobSystem; the UI polls for completion and never waits. Esc cancels.
/// Until a file is opened, a built-in demo scene is shown. All UI strings go through tr(). GUI thread only.
///
/// Commands: every operation is a registry command (ui::CommandRegistry). The ribbon (SOLID/... tabs, Fusion
/// layout), the Edit/Modify menus, shortcuts, the S-box (S) and the right-click marking menu all read the registry.
/// Right-docked command dialogs go into the Command dock; manipulators are drawn over the viewport.
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    CommandRegistry& commands() noexcept { return commands_; }
    DocumentState& document() noexcept { return doc_; }
    CommandActions* actions() const noexcept { return actions_; }
    RibbonBar* ribbon() const noexcept { return ribbon_; }
    CommandPalette* commandSearch() const noexcept { return palette_; }
    MarkingMenu* markingMenu() const noexcept { return marking_; }
    ManipulatorOverlay* manipulators() const noexcept { return overlay_; }
    QDockWidget* commandDock() const noexcept { return panelDock_; }
    /// Docks a command dialog on the right, replacing the previous one (takes ownership of `panel`).
    void showCommandPanel(QWidget* panel);
    /// Sketch mode (create/finish sketches, sketch palette, overlay).
    SketchMode* sketchMode() const noexcept { return sketch_; }
    /// "Create Sketch": docks the plane chooser; the chosen origin plane starts a sketch.
    void chooseSketchPlane();
    /// Opens the S-box (also bound to S).
    void openCommandSearch();
    /// Opens the marking menu centred on a viewport-local point.
    void showMarkingMenu(QPoint viewportPos);

    render::Viewport* viewport() const noexcept { return viewport_; }
    /// Assembly tree: top-level items are the roots; each item's checkbox shows/hides its leaf bodies.
    QTreeWidget* browser() const noexcept { return tree_; }
    /// Window title used for an untitled document.
    static QString defaultTitle();
    /// Replaces the viewport scene with the built-in demo bodies (two boxes).
    void showDemoScene();
    /// Text shown in the status bar for a pick result ("Body 1 / Face 3 ...").
    static QString describePick(const render::PickResult& r);

    /// Starts importing a .step/.stp/.stl file in the background. A running import is cancelled and replaced.
    /// Returns false (and reports in the status bar) for unsupported types; load errors are reported later.
    bool openFile(const QString& path);
    /// True from openFile until the import finishes, fails or is cancelled.
    bool importing() const noexcept { return job_.has_value(); }
    /// Cancels the running import (bound to Esc). No-op when idle.
    void cancelImport();
    /// Summary of the last completed import ("Imported 3 bodies: import 12 ms, tessellate 40 ms"), empty if none.
    QString lastImportReport() const { return lastReport_; }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void buildActions();
    void setupCommands();
    void chooseAndOpenFile();
    void pinCommand(const std::string& id);
    void buildBrowser();
    void pollImport();
    void applyModel(LoadedModel model);
    void selectTreeForBody(std::optional<std::uint32_t> body);
    void setImportUi(bool active);
    int bodyOf(const QTreeWidgetItem* item) const;

    iw::JobSystem jobs_;
    std::optional<iw::JobHandle<LoadedModel>> job_;
    std::shared_ptr<ImportProgress> progress_;
    QString importName_;
    QString lastReport_;

    render::Viewport* viewport_ = nullptr;
    QLabel* status_ = nullptr;
    QProgressBar* progressBar_ = nullptr;
    QTimer* pollTimer_ = nullptr;
    QTreeWidget* tree_ = nullptr;
    QAction* openAction_ = nullptr;
    QAction* cancelAction_ = nullptr;
    std::vector<QTreeWidgetItem*> bodyItems_; ///< Body index -> tree item.
    bool syncing_ = false;                    ///< Guards tree <-> viewport feedback loops.

    DocumentState doc_;
    CommandRegistry commands_;
    CommandContext cmdCtx_;
    CommandActions* actions_ = nullptr;
    RibbonBar* ribbon_ = nullptr;
    CommandPalette* palette_ = nullptr;
    MarkingMenu* marking_ = nullptr;
    ManipulatorOverlay* overlay_ = nullptr;
    SketchMode* sketch_ = nullptr;
    QDockWidget* panelDock_ = nullptr;
    QTimer* refreshTimer_ = nullptr;
    bool rightPressed_ = false;               ///< Right button went down on the viewport and has not moved.
    QPoint rightPressPos_;
    std::vector<std::string> pinned_;         ///< Commands pinned to the quick-access row.
};

} // namespace iw::ui
