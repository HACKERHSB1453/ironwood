// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <memory>
#include <vector>

#include "core/cancel.hpp"
#include "core/params.hpp"
#include "recompute/timeline.hpp"

namespace iw::recompute {

/// Per-feature outcome of an evaluation.
struct FeatureStatus {
    FeatureId id;
    enum class State { Ok, Suppressed, Failed, Skipped } state = State::Ok;
    bool reused = false;      ///< Served from the prefix cache (not recomputed).
    Error error;              ///< Valid when state == Failed.
    std::vector<Diagnostic> diagnostics;
    /// Construction geometry published by this feature (B1 Plane / Axis / Point), when Ok.
    std::optional<ConstructionGeometry> construction;
    /// Non-geometric attributes published by the feature (FeatureResult::attributes), when Ok.
    std::map<std::string, std::string> attributes;
};

/// Model state at the evaluated timeline position.
struct EvalResult {
    std::map<BodyId, kernel::Shape> bodies;
    std::map<BodyId, naming::NamingMap> namings;
    std::vector<FeatureStatus> statuses;   ///< One per timeline feature up to the marker, in order.
    std::size_t computedCount = 0;         ///< Features actually computed this call.
    std::size_t reusedCount = 0;           ///< Features served from cache.
};

/// Timeline evaluator with prefix reuse (ARCHITECTURE section 4).
///
/// Cache: for each position k the evaluator stores the state after feature k, keyed by a *stamp chain*:
///   stamp_k = combine(stamp_{k-1}, featureId, paramHash, paramTableHash(used), suppressed-flag).
/// evaluate() walks features in order and reuses the longest prefix whose chain is unchanged; editing
/// feature N recomputes N..marker only. Marker moves and undo/redo are therefore cache hits.
/// Features with disjoint read/write body sets MAY be computed in parallel later; the P0 contract is
/// sequential evaluation with identical observable results.
///
/// Thread-safety: one evaluate() at a time per Evaluator (internally synchronised: concurrent calls
/// serialise). The Timeline is read at call start; mutate a copy for async use.
/// A feature failure does not abort: it is recorded (Failed), later features see the previous state and
/// are evaluated anyway unless they read a body the failed feature should have written (-> Skipped).
class Evaluator {
public:
    Evaluator();
    ~Evaluator();
    Evaluator(const Evaluator&) = delete;
    Evaluator& operator=(const Evaluator&) = delete;

    /// Evaluates features [0, timeline.rollbackMarker()) without a parameter table. Errors: Cancelled (partial cache
    /// kept).
    Result<EvalResult> evaluate(const Timeline& timeline, CancelToken cancel = {});

    /// Same, with the document parameter table visible to features (EvalContext::params). Each feature's stamp
    /// includes params->hashOfUsed(feature.parameterReferences()), so a parameter edit recomputes exactly the
    /// features that use it and everything after them.
    Result<EvalResult> evaluate(const Timeline& timeline, const ParameterTable* params, CancelToken cancel = {});

    /// Drops all cached prefixes.
    void clearCache();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace iw::recompute
