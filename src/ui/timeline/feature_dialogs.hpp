// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Command dialogs of the modelling features (Extrude, Fillet, Chamfer, Hole, Revolve, Move, Offset Plane). Each is a
// B4 CommandPanel built from a CommandDialogSpec: its fields, a live preview (evaluated off the GUI thread on a copy of
// the timeline) and an accept function that adds the feature, or replaces it when the dialog edits an existing one.
// Viewport picks fill the selection fields: an edge / face pick is translated to a naming::ElementRef through the
// NamingMap of the last evaluation.

#include <QString>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/ids.hpp"
#include "core/job.hpp"
#include "features/document.hpp"
#include "recompute/feature.hpp"
#include "render/viewport.hpp"
#include "recompute/timeline.hpp"
#include "ui/timeline/doc_eval.hpp"

class QWidget;

namespace iw::render {
struct PickResult;
}

namespace iw::ui {
class CommandPanel;
class ManipulatorOverlay;

/// What the dialogs need from the shell. Implemented by MainWindow.
class FeatureHost {
public:
    virtual ~FeatureHost() = default;
    /// The document; nullptr when none is open.
    virtual features::Document* model() = 0;
    /// Timeline and evaluation currently shown (what picks and previews refer to).
    virtual const recompute::Timeline& shownTimeline() const = 0;
    virtual const timeline::EvalOutput& shownEvaluation() const = 0;
    /// Body of the viewport scene at `index`, if any.
    virtual std::optional<BodyId> bodyAtIndex(std::uint32_t index) const = 0;
    virtual JobSystem& jobs() = 0;
    /// Docks a command panel on the right (takes ownership).
    virtual void showPanel(QWidget* panel) = 0;
    /// Routes the next viewport picks to `handler` (empty = picks select as usual).
    virtual void setPickHandler(std::function<void(const render::PickResult&)> handler) = 0;
    virtual void setStatus(const QString& text) = 0;
    /// Adds `feature` at the end of the timeline, or replaces the feature with the same id when `editing`. Undoable.
    virtual Result<void> commitFeature(std::shared_ptr<const recompute::Feature> feature, bool editing) = 0;
    /// Name shown for a body ("Body 1", ...).
    virtual QString bodyTitle(BodyId body) const = 0;
    /// Translucent viewport overlays (the finished sketch profiles while Extrude / Revolve is open). Empty clears.
    virtual void setOverlayFaces(std::vector<render::OverlayFace> faces) = 0;
    /// The viewport (camera for projecting manipulators) and the manipulator overlay drawn over it.
    virtual render::Viewport* viewport() const = 0;
    virtual ManipulatorOverlay* manipulators() const = 0;
};

/// Opens and drives the modelling command dialogs for one shell.
class FeatureDialogs {
public:
    explicit FeatureDialogs(FeatureHost& host);
    ~FeatureDialogs();

    /// Starts the dialog of `kind` ("extrude", "fillet", "chamfer", "hole", "revolve", "move", "plane"). With `editing`
    /// the dialog is prefilled from that feature and OK replaces it. Returns an error (also shown in the status bar)
    /// when the dialog cannot start (no document, rollback marker not at the end, unknown kind, feature of another kind).
    Result<void> start(const std::string& kind, std::optional<FeatureId> editing = std::nullopt);

private:
    struct Impl;
    FeatureHost& host_;
    std::shared_ptr<ParameterTable> params_;   ///< Parameter snapshot the open panel validates against.
    std::unique_ptr<Impl> impl_;
};

} // namespace iw::ui
