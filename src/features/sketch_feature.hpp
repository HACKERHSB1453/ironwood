// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// SketchFeature (B1c): a timeline feature that owns a sketch. It produces no bodies. Its compute() resolves the plane
// (its own, or a planar face / construction plane it is attached to), solves the constraints against the document
// parameter table and detects the profiles. Later Extrude / Revolve features (SketchReferenceSource) and sketch-placed
// Holes reference the solved sketch by feature id. The solved sketch reaches them through the recompute sketch channel
// (FeatureResult::sketch -> EvalContext::sketch, F30), so it is solved once per change and kept in the evaluator's
// prefix cache; each SketchFeature additionally memoises its recent solves (see SketchFeature::solve).

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "features/construction_features.hpp"
#include "features/feature_common.hpp"
#include "recompute/feature.hpp"
#include "sketch/profile.hpp"
#include "sketch/sketch.hpp"
#include "sketch/solver/bridge.hpp"

namespace iw::features {

/// Sketch feature parameters. `sketch` carries entities, constraint and dimension records. Its own plane is used when
/// `attach` is empty; otherwise the plane is re-derived from the attachment at every compute, so a change upstream
/// (a dimension of the body the face belongs to, a construction plane's offset) moves the sketch.
struct SketchFeatureParams {
    sketch::Sketch sketch;
    std::optional<PlaneRef> attach;
};

/// Result of solving a sketch feature at its timeline position.
struct SketchSolution {
    sketch::Sketch sketch;                        ///< Solved geometry on the resolved plane (ids preserved).
    std::vector<sketch::SketchProfile> profiles;  ///< detectProfiles(sketch), sorted by name.
    sketch::SketchSolveReport report;             ///< DOF and redundancy diagnostics.
    std::vector<recompute::Diagnostic> diagnostics;
    /// Measured value of every driven (reference) dimension on the solved geometry (mm, or rad for angles). A driven
    /// dimension that cannot be measured is omitted and reported as a Warning in `diagnostics`.
    std::map<sketch::SketchDimensionId, double> drivenValues;
};

/// The solved sketch a SketchFeature publishes on the recompute sketch channel (FeatureResult::sketch).
struct SolvedSketchOutput final : recompute::SketchOutput {
    explicit SolvedSketchOutput(SketchSolution s) : solution(std::move(s)) {}
    SketchSolution solution;
};

class SketchFeature final : public recompute::Feature {
public:
    SketchFeature(FeatureId id, SketchFeatureParams params);

    /// Parses `paramsJson()` output. Errors: ParseError, InvalidArgument.
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);

    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "sketch"; }
    std::vector<naming::ElementRef> explicitRefs() const override;
    std::vector<std::string> parameterReferences() const override;
    std::vector<BodyId> readsBodies() const override;
    std::vector<BodyId> writesBodies() const override { return {}; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    /// Publishes dof / profile-count attributes, and one "driven.<dimension id>" attribute per measured driven
    /// dimension (its value, decimal, mm or rad). The solved sketch is published on FeatureResult::sketch as a
    /// SolvedSketchOutput. Errors: the solve errors of solve().
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;

    /// Resolves the plane at `ctx`, solves against ctx.params and detects profiles. Failures: InvalidArgument with the
    /// solver message (InvalidInput), or for a Failed solve the conflicting constraint / dimension ids. Cancelled.
    ///
    /// Successful solves are memoised per feature instance (its sketch is immutable, so the instance stands for the
    /// sketch content hash), keyed by the resolved plane and ParameterTable::hashOfUsed(parameterReferences()): an
    /// evaluation after an unrelated edit (a parameter the sketch does not use, an upstream change that leaves the
    /// attachment plane where it was, undo / redo) does not run the solver again. Thread-safe.
    Result<SketchSolution> solve(const recompute::EvalContext& ctx) const;

    const SketchFeatureParams& params() const noexcept { return params_; }

private:
    struct SolveCache;
    /// solve(), shared with the cache (and published as is when resolving the plane added no diagnostics).
    Result<std::shared_ptr<const SolvedSketchOutput>> solveShared(const recompute::EvalContext& ctx) const;

    FeatureId id_;
    SketchFeatureParams params_;
    std::uint64_t paramHash_ = 0;         ///< paramHash(), computed once (the parameters are immutable).
    std::vector<std::string> paramRefs_;  ///< parameterReferences(), computed once.
    std::shared_ptr<SolveCache> cache_;   ///< Memoised solves (internally synchronised).
};

/// Solved sketch of sketch feature `sketch` as published before this timeline position (EvalContext::sketch), or
/// nullptr when it published none (not a sketch feature, suppressed, failed, or not earlier on the timeline).
const SketchSolution* solvedSketchAt(const recompute::EvalContext& ctx, FeatureId sketch);

/// Number of times SketchFeature::solve has run the constraint solver in this process (cache misses). Test and
/// profiling hook.
std::uint64_t sketchSolverRuns() noexcept;

/// Feature id of the sketch an Extrude or Revolve takes its profile from, when `f` is such a feature with a
/// SketchReferenceSource, or that a Hole placed from sketch points uses (HoleSketchPlacement); nullopt otherwise.
std::optional<FeatureId> sketchReferenceOf(const recompute::Feature& f);

/// Copy of an Extrude / Revolve feature whose sketch reference is replaced by an embedded copy of `solved` (the
/// solved sketch of the referenced feature); for a sketch-placed Hole, `solved` is stored in
/// HoleSketchPlacement::solved. Returns nullptr when `f` does not reference a sketch. Not needed with
/// features::Document (the recompute sketch channel delivers the solved sketch); kept for callers that still resolve
/// references in a pre-pass. An embedded copy takes precedence over the channel.
std::shared_ptr<recompute::Feature> embedSketchReference(const recompute::Feature& f, const sketch::Sketch& solved);

/// Registers kind "sketch" with `registry`.
void registerSketchFeature(recompute::FeatureRegistry& registry);

} // namespace iw::features
