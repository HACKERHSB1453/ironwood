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
    /// Replaces the feature with the same id (parameter edit). Errors: NotFound.
    Result<void> editFeature(std::shared_ptr<const recompute::Feature> feature);
    /// Errors: NotFound.
    Result<void> setSuppressed(FeatureId id, bool suppressed);
    /// Removes the feature. Errors: NotFound.
    Result<void> deleteFeature(FeatureId id);
    /// Rollback marker: number of active features, or nullopt for "at end". Errors: InvalidArgument (out of range).
    Result<void> setRollbackMarker(std::optional<std::size_t> activeCount);
    /// Adds a parameter. Errors: as ParameterTable::add.
    Result<void> addParameter(std::string name, std::string expression, std::string unit = {}, std::string comment = {});
    /// Replaces a parameter expression. Errors: as ParameterTable::setExpression.
    Result<void> setParameterExpression(const std::string& name, std::string expression);

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

    io::Document data_;
    recompute::FeatureRegistry registry_;
    recompute::Evaluator evaluator_;
    UndoStack undo_;
    std::map<ListenerId, Listener> listeners_;
    ListenerId nextListener_ = 1;
};

} // namespace iw::features
