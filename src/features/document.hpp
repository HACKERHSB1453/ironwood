// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Qt-free document model (B1c): timeline, parameter table, evaluator, feature registry, native save/load and an undo
// stack of document commands. The UI timeline and browser bind to this class through change listeners.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "core/cancel.hpp"
#include "core/ids.hpp"
#include "core/params.hpp"
#include "core/result.hpp"
#include "core/undo.hpp"
#include "features/construction_features.hpp"
#include "features/sketch_feature.hpp"
#include "io/document_io.hpp"
#include "kernel/shape.hpp"
#include "naming/naming_map.hpp"
#include "recompute/evaluator.hpp"
#include "recompute/feature.hpp"
#include "recompute/timeline.hpp"

namespace iw::features {

/// Result of Document::evaluate().
struct DocumentEvaluation {
    /// One status per timeline feature, in timeline order. Features after the rollback marker are reported as Skipped
    /// (or Suppressed) with an Info diagnostic and are not evaluated.
    std::vector<recompute::FeatureStatus> statuses;
    std::map<BodyId, kernel::Shape> bodies;
    std::map<BodyId, naming::NamingMap> namings;
    /// Construction geometry (plane / axis / point features) at the end of the active timeline.
    std::map<FeatureId, recompute::ConstructionGeometry> construction;
    /// Solved sketches of the sketch features that solved successfully.
    std::map<FeatureId, SketchSolution> sketches;
    std::size_t computedCount = 0;
    std::size_t reusedCount = 0;
};

class Document {
public:
    using Listener = std::function<void()>;
    using ListenerId = std::uint64_t;

    Document();
    ~Document();
    Document(const Document&) = delete;
    Document& operator=(const Document&) = delete;

    // ---- read access ----
    const std::string& name() const noexcept { return data_.name; }
    void setName(std::string name);
    const recompute::Timeline& timeline() const noexcept { return data_.timeline; }
    const ParameterTable& parameters() const noexcept { return data_.parameters; }
    const recompute::FeatureRegistry& registry() const noexcept { return registry_; }

    /// Allocates a fresh feature / body id (never reused). Feature and body ids share one counter.
    FeatureId allocateFeatureId();
    BodyId allocateBodyId();

    // ---- undoable commands. On error the document is unchanged and nothing is recorded. ----
    /// Inserts `feature` at `position` (default: end). Errors: InvalidArgument (duplicate id, bad position).
    Result<void> addFeature(std::shared_ptr<const recompute::Feature> feature, std::optional<std::size_t> position = std::nullopt);
    /// Replaces the feature with the same id (parameter edit). Errors: NotFound. Prefer the typed edit calls above.
    Result<void> editFeature(std::shared_ptr<const recompute::Feature> feature);
    /// Errors: NotFound.
    Result<void> setSuppressed(FeatureId id, bool suppressed);
    /// Removes the feature. Errors: NotFound.
    Result<void> deleteFeature(FeatureId id);
    /// Rollback marker: number of active features, or nullopt for "at end". Errors: InvalidArgument (out of range).
    Result<void> setRollbackMarker(std::optional<std::size_t> activeCount);
    /// Feature names. Every feature has one: addFeature assigns "<Kind> <n>" (Fusion-style, numbered per kind and never
    /// reused) when the feature has none. Errors: NotFound, InvalidArgument (empty name).
    Result<void> renameFeature(FeatureId id, std::string name);

    // ---- parameters. Each call is one undoable command; it also rewrites the feature parameters and sketch dimensions
    // that reference the parameter, so the parameter table and the timeline never disagree. ----
    /// Adds a parameter. Errors: as ParameterTable::add.
    Result<void> addParameter(std::string name, std::string expression, std::string unit = {}, std::string comment = {});
    /// Replaces a parameter expression. Errors: as ParameterTable::setExpression.
    Result<void> setParameterExpression(const std::string& name, std::string expression);
    /// Errors: as ParameterTable::setUnit / setComment / setFavourite (NotFound, InvalidArgument).
    Result<void> setParameterUnit(const std::string& name, std::string unit);
    Result<void> setParameterComment(const std::string& name, std::string comment);
    Result<void> setParameterFavourite(const std::string& name, bool favourite);
    /// Renames a parameter and rewrites every expression that references it: other parameters, feature parameters
    /// (extrude distances, hole sizes, ...) and sketch dimensions. Evaluated values do not change. Errors: NotFound
    /// (from), InvalidArgument (bad or taken new name, cycle), ParseError.
    Result<void> renameParameter(const std::string& from, const std::string& to);
    /// Removes a parameter. Refused while a parameter or a feature (incl. its sketch dimensions) references it:
    /// InvalidArgument whose message lists the users. Errors: NotFound, InvalidArgument.
    Result<void> removeParameter(const std::string& name);
    /// Replaces the whole parameter table in one command (for dialogs that edit a copy). Feature references are not
    /// checked here; an unknown name fails at evaluation like any other.
    Result<void> replaceAllParameters(ParameterTable table);

    // ---- construction features (plane / axis / point): replaces the feature with the same id. Errors: NotFound,
    // InvalidArgument (the feature is not of that kind). ----
    Result<void> editConstructionPlane(FeatureId id, ConstructionPlaneParams params);
    Result<void> editConstructionAxis(FeatureId id, ConstructionAxisParams params);
    Result<void> editConstructionPoint(FeatureId id, ConstructionPointParams params);

    // ---- sketches ----
    /// Copy of the sketch definition of a sketch feature (unsolved, as stored), for re-entering sketch mode.
    /// Errors: NotFound, InvalidArgument (not a sketch feature).
    Result<sketch::Sketch> sketchForEdit(FeatureId id) const;
    /// Replaces the sketch of a sketch feature (attachment and name are kept) as one undoable command.
    /// Errors: NotFound, InvalidArgument (not a sketch feature).
    Result<void> replaceSketch(FeatureId id, sketch::Sketch sk);

    Result<void> undo();
    Result<void> redo();
    bool canUndo() const noexcept { return undo_.canUndo(); }
    bool canRedo() const noexcept { return undo_.canRedo(); }
    std::string undoText() const { return undo_.undoText(); }
    std::string redoText() const { return undo_.redoText(); }
    bool isClean() const noexcept { return undo_.isClean(); }

    // ---- evaluation ----
    /// Evaluates the active timeline. Sketch references of Extrude / Revolve are resolved here: each sketch feature is
    /// solved against the state before it, and the dependent feature receives the solved sketch. Errors: Cancelled.
    Result<DocumentEvaluation> evaluate(CancelToken cancel = {});

    // ---- persistence ----
    /// Writes the native document directory (io::saveDocument). On success the document is marked clean.
    Result<void> save(const std::filesystem::path& path);
    /// Replaces the content with a loaded document; clears undo history. Errors: as io::loadDocument.
    Result<void> load(const std::filesystem::path& path);

    // ---- change notification (plain callbacks, called on the owning thread after each successful change) ----
    ListenerId addListener(Listener listener);
    void removeListener(ListenerId id);

private:
    struct Ops;   // command helpers; defined in document.cpp
    friend struct Ops;
    void notify();
    /// Copy of `f` (same kind, id and parameters) carrying `name`. Errors: as FeatureRegistry::create.
    Result<std::shared_ptr<const recompute::Feature>> withName(const recompute::Feature& f, std::string name) const;
    /// Returns `f` with every expression that references `from` rewritten to `to`, or nullptr when nothing changes.
    Result<std::shared_ptr<const recompute::Feature>> renamedReferences(const recompute::Feature& f, const std::string& from,
                                                                        const std::string& to) const;
    /// Assigns default names to unnamed features and raises the per-kind counters past existing names (after load).
    void completeNames();
    /// Feature by id on the timeline, or nullptr.
    std::shared_ptr<const recompute::Feature> featureById(FeatureId id) const;
    /// Names of features whose parameters reference `name`, in timeline order.
    std::vector<std::string> featureUsers(const std::string& name) const;

    io::Document data_;
    recompute::FeatureRegistry registry_;
    recompute::Evaluator evaluator_;
    UndoStack undo_;
    std::map<ListenerId, Listener> listeners_;
    ListenerId nextListener_ = 1;
};

} // namespace iw::features
