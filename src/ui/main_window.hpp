// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QMainWindow>
#include <QPoint>
#include <QString>

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "core/job.hpp"
#include "features/document.hpp"
#include "kernel/measure.hpp"
#include "ui/command.hpp"
#include "ui/import_job.hpp"
#include "ui/inspect/inspect_commands.hpp"
#include "ui/timeline/doc_eval.hpp"
#include "ui/timeline/feature_dialogs.hpp"
#include "sketch/sketch.hpp"

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
class SketchSession;
namespace timeline {
class TimelineWidget;
}
}

namespace iw::ui {

/// Application shell. It owns the Ironwood document (features::Document): File menu for new / open / save / save as of
/// the native document and for importing STEP and STL files. Every change of the document (commands, undo, rollback,
/// sketch finish) triggers a background evaluation on the JobSystem (LatestWins: a newer request cancels the older one);
/// the result (bodies, tessellated meshes, statuses, naming) is applied on the GUI thread when it is ready, and the
/// timeline dock, the browser tree and the viewport are rebuilt from it. The UI never waits for a job.
///
/// Panels: Browser (left: Origin, Sketches, Bodies with visibility checkboxes, Construction, and any imported
/// assembly), Timeline (bottom), Command (right, command dialogs), Sketch (left, sketch mode). Status bar: pick and
/// hover text, evaluation summary, errors. GUI thread only.
class MainWindow : public QMainWindow, public FeatureHost {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    CommandRegistry& commands() noexcept { return commands_; }
    CommandActions* actions() const noexcept { return actions_; }
    RibbonBar* ribbon() const noexcept { return ribbon_; }
    CommandPalette* commandSearch() const noexcept { return palette_; }
    MarkingMenu* markingMenu() const noexcept { return marking_; }
    ManipulatorOverlay* manipulators() const noexcept override { return overlay_; }
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

    render::Viewport* viewport() const noexcept override { return viewport_; }
    /// Browser tree (Origin, Sketches, Bodies, Construction, and the imported assembly).
    QTreeWidget* browser() const noexcept { return tree_; }
    /// Timeline strip (bottom dock).
    timeline::TimelineWidget* timelineWidget() const noexcept { return timeline_; }

    /// Window title used for an untitled document.
    static QString defaultTitle();
    /// Replaces the viewport scene with the built-in demo bodies (two boxes).
    void showDemoScene();
    /// Text shown in the status bar for a pick result ("Body 1 / Face 3 ...").
    static QString describePick(const render::PickResult& r);

    // ---- document ----
    /// The document (never nullptr after construction).
    features::Document* model() override { return model_.get(); }
    /// Starts a new empty document (discards the current one; undo history is discarded too).
    void newDocument();
    /// Loads a native document directory. On failure the current document is kept.
    Result<void> openDocument(const QString& path);
    /// Saves the native document directory (creates it). Sets the document path on success.
    Result<void> saveDocument(const QString& path);
    /// True when the document has been saved to or loaded from a path.
    bool hasDocumentPath() const noexcept { return !docPath_.isEmpty(); }
    /// Undo / redo of the document: feature edits, rollback, sketch finish, parameter edits (one stack).
    bool modelCanUndo() const;
    bool modelCanRedo() const;
    void modelUndo();
    void modelRedo();
    /// Starts a sketch on `plane` (the sketch becomes a SketchFeature when it finishes).
    void startSketch(const sketch::SketchPlane& plane, const QString& planeName);
    /// Edit Sketch (timeline double-click / menu): re-enters sketch mode with the sketch feature `id`. Finish Sketch
    /// replaces its sketch as one undo step; Esc discards the edit. Errors are reported in the status bar.
    void editSketch(FeatureId id);
    /// Actions of the timeline's right-click menu. Edit is "Edit Sketch" for a sketch, "Edit..." otherwise.
    enum class TimelineAction { Edit, Suppress, Delete, Rename };
    /// Runs one timeline menu action on feature `id` (what the right-click menu does after the user picks an item).
    /// Tests call it directly; the menu is only a front end for it.
    void runTimelineAction(FeatureId id, TimelineAction action);
    /// Routes a viewport pick as the viewport's picked signal does (to the open dialog, or the browser selection).
    void routePick(const render::PickResult& r);
    /// Suppresses / unsuppresses, deletes, or renames a timeline feature. Same as the timeline menu. A rename is an
    /// undoable document command (Document::renameFeature) and is saved with the document.
    void setFeatureSuppressed(FeatureId id, bool suppressed);
    void deleteFeature(FeatureId id);
    void renameFeatureTo(FeatureId id, std::string name);
    /// Starts a modelling command dialog ("extrude", "fillet", "chamfer", "hole", "revolve", "move", "plane").
    Result<void> startFeature(const std::string& kind, std::optional<FeatureId> editing = std::nullopt);
    /// Re-evaluates the document in the background (also called after every change).
    void requestEvaluation();
    /// Waits (processing events) until the latest evaluation has been applied, or the timeout. Returns true when idle.
    /// For tests and scripts; the GUI itself never waits.
    bool settle(int timeoutMs);

    // ---- inspect (B6) ----
    /// Viewport picks the Measure command works on (the last two; a click on nothing clears them).
    const std::vector<kernel::measure::Entity>& picks() const noexcept { return picks_; }
    /// Last section render request published by the Section Analysis command (empty before the first one).
    std::optional<kernel::measure::section::RenderRequest> sectionRequest() const noexcept { return sectionRequest_; }

    // ---- import (STEP / STL, kept as separate bodies of the scene) ----
    /// Starts importing a .step/.stp/.stl file in the background. A running import is cancelled and replaced.
    /// Returns false (and reports in the status bar) for unsupported types; load errors are reported later.
    bool openFile(const QString& path);
    /// True from openFile until the import finishes, fails or is cancelled.
    bool importing() const noexcept { return job_.has_value(); }
    /// Cancels the running import (bound to Esc). No-op when idle.
    void cancelImport();
    /// Summary of the last completed import ("Imported 3 bodies: import 12 ms, tessellate 40 ms"), empty if none.
    QString lastImportReport() const { return lastReport_; }

    // ---- FeatureHost ----
    const recompute::Timeline& shownTimeline() const override { return shown_.timeline; }
    /// Evaluation as shown (the statuses, bodies and naming the timeline and picks refer to).
    const timeline::EvalOutput& shownEvaluation() const override { return shown_.out; }
    std::optional<BodyId> bodyAtIndex(std::uint32_t index) const override;
    JobSystem& jobs() override { return jobs_; }
    void showPanel(QWidget* panel) override { showCommandPanel(panel); }
    void setPickHandler(std::function<void(const render::PickResult&)> handler) override { pickHandler_ = std::move(handler); }
    void setStatus(const QString& text) override;
    void setPickHint(const QString& hint) override;
    Result<void> commitFeature(std::shared_ptr<const recompute::Feature> feature, bool editing) override;
    QString bodyTitle(BodyId body) const override;
    void setOverlayFaces(std::vector<render::OverlayFace> faces) override;

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    /// One evaluation: the timeline it ran on (marker included) and its result.
    struct EvalJob {
        recompute::Timeline timeline;
        timeline::EvalOutput out;
    };

    void buildActions();
    void setupCommands();
    void buildDocking();
    void chooseAndOpenFile();
    void pinCommand(const std::string& id);
    void buildBrowser();
    void pollImport();
    void applyModel(LoadedModel model);
    void pollEvaluation();
    void applyEvaluation(EvalJob job);
    void applyScene();
    void rebuildTimeline();
    void rebuildBrowser();
    /// Keeps the sketch dimensions on the document's parameter table; re-solves sketches when a parameter changed.
    void syncParameters();
    std::uint64_t parameterHash() const;
    void highlightFeature(std::optional<FeatureId> id);
    void showTimelineMenu(FeatureId id, QPoint globalPos);
    void renameFeature(FeatureId id);
    void attachModel(std::unique_ptr<features::Document> doc);
    /// Records a viewport pick for Measure (ignored while a dialog routes picks).
    void recordPick(const render::PickResult& r);
    /// Kernel shape of a body (feature result, or an imported leaf placed in world space).
    std::optional<kernel::Shape> shapeOfBody(BodyId body) const;
    std::optional<kernel::Shape> shapeOfScene(std::uint32_t index) const;
    /// Body Properties shows: the last picked body, else the only body of the scene.
    std::optional<kernel::Shape> activeBodyShape() const;
    /// Section Analysis hook: stores the request; the viewport connection is a TODO (see the .cpp).
    void applySectionRequest(const kernel::measure::section::RenderRequest& request);
    void updateTitle();
    void commitSketch(SketchSession* session);
    /// Finish of an edit started by editSketch: replaces the sketch of `id` (one undo step, skipped when nothing changed).
    void commitSketchEdit(FeatureId id, SketchSession* session);
    void selectTreeForBody(std::optional<std::uint32_t> body);
    void setImportUi(bool active);
    std::optional<BodyId> bodyOfItem(const QTreeWidgetItem* item) const;
    std::optional<std::size_t> sceneIndexOf(BodyId body) const;

    iw::JobSystem jobs_;
    std::optional<iw::JobHandle<LoadedModel>> job_;
    std::shared_ptr<ImportProgress> progress_;
    QString importName_;
    QString lastReport_;
    std::optional<LoadedModel> imported_;     ///< Last completed import (its tree and leaves).

    // ---- document and evaluation ----
    std::unique_ptr<features::Document> model_;
    std::shared_ptr<recompute::Evaluator> evaluator_ = std::make_shared<recompute::Evaluator>();
    LatestWins<EvalJob> evalLatest_{jobs_};
    std::optional<JobHandle<EvalJob>> evalJob_;
    EvalJob shown_;                           ///< Last applied evaluation.
    std::optional<std::size_t> previewMarker_; ///< Provisional rollback marker during a drag.
    std::set<BodyId> hidden_;                  ///< Bodies hidden with the browser checkboxes.
    std::vector<BodyId> sceneIds_;             ///< Body of each scene index.
    std::unique_ptr<FeatureDialogs> dialogs_;
    std::function<void(const render::PickResult&)> pickHandler_;
    QString pickHint_;   ///< Hint of the open dialog's active selection (kept in the status bar while hovering).
    std::optional<FeatureId> editingSketch_;   ///< The sketch feature whose sketch edit is open (none: new sketch).
    QString docPath_;
    bool demo_ = true;                         ///< The built-in demo bodies are shown until a document or import exists.

    render::Viewport* viewport_ = nullptr;
    QLabel* status_ = nullptr;
    QProgressBar* progressBar_ = nullptr;
    QTimer* pollTimer_ = nullptr;
    QTimer* evalTimer_ = nullptr;
    timeline::TimelineWidget* timeline_ = nullptr;
    QDockWidget* timelineDock_ = nullptr;
    QTreeWidget* tree_ = nullptr;
    QAction* openAction_ = nullptr;
    QAction* cancelAction_ = nullptr;
    QTreeWidgetItem* importRoot_ = nullptr;
    std::map<std::uint64_t, QTreeWidgetItem*> bodyItems_; ///< BodyId value -> browser item.
    bool syncing_ = false;                    ///< Guards tree <-> viewport feedback loops.
    CommandRegistry commands_;
    CommandContext cmdCtx_;
    inspect::InspectContext inspect_;          ///< Providers of Measure / Properties and the section sink.
    std::vector<kernel::measure::Entity> picks_;
    std::optional<kernel::measure::section::RenderRequest> sectionRequest_;
    std::uint64_t paramsHash_ = 0;             ///< Parameter values the sketches were last solved against.
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
