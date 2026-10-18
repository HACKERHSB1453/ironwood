// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// SketchFeature (B1c): a timeline feature that owns a sketch. It produces no bodies. Its compute() resolves the plane
// (its own, or a planar face / construction plane it is attached to), solves the constraints against the document
// parameter table and detects the profiles. Later Extrude and Revolve features reference the solved sketch by feature
// id and profile name (SketchReferenceSource); the Document resolves those references before evaluation.

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

class SketchFeature final : public recompute::Feature {
public:
    SketchFeature(FeatureId id, SketchFeatureParams params) : id_(id), params_(std::move(params)) {}

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
    /// dimension (its value, decimal, mm or rad). Errors: the solve errors of solve().
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;

    /// Resolves the plane at `ctx`, solves against ctx.params and detects profiles. Failures: InvalidArgument with the
    /// solver message (InvalidInput), or for a Failed solve the conflicting constraint / dimension ids. Cancelled.
    Result<SketchSolution> solve(const recompute::EvalContext& ctx) const;

    const SketchFeatureParams& params() const noexcept { return params_; }

private:
    FeatureId id_;
    SketchFeatureParams params_;
};

/// Feature id of the sketch an Extrude or Revolve takes its profile from, when `f` is such a feature with a
/// SketchReferenceSource; nullopt otherwise.
std::optional<FeatureId> sketchReferenceOf(const recompute::Feature& f);

/// Copy of an Extrude / Revolve feature whose sketch reference is replaced by an embedded copy of `solved` (the
/// solved sketch of the referenced feature). Returns nullptr when `f` does not reference a sketch.
std::shared_ptr<recompute::Feature> embedSketchReference(const recompute::Feature& f, const sketch::Sketch& solved);

/// Registers kind "sketch" with `registry`.
void registerSketchFeature(recompute::FeatureRegistry& registry);

} // namespace iw::features
