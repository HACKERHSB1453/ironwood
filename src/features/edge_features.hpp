// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "core/ids.hpp"
#include "features/feature_common.hpp"
#include "naming/element_name.hpp"
#include "recompute/feature.hpp"

namespace iw::features {

/// Constant-radius fillet of a set of edges of `body` (B1). `radius` is a length expression. With `tangentChain` the
/// selection is first expanded to every edge tangent-continuous with it (Fusion's "tangent chain" option).
struct FilletParams {
    BodyId body;
    std::vector<naming::ElementRef> edges;
    std::string radius = "1 mm";
    bool tangentChain = true;
};

class FilletFeature final : public recompute::Feature {
public:
    FilletFeature(FeatureId id, FilletParams params) : id_(id), params_(std::move(params)) {}
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);

    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "fillet"; }
    std::vector<naming::ElementRef> explicitRefs() const override { return params_.edges; }
    std::vector<std::string> parameterReferences() const override;
    std::vector<BodyId> readsBodies() const override { return {params_.body}; }
    std::vector<BodyId> writesBodies() const override { return {params_.body}; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;
    const FilletParams& params() const noexcept { return params_; }

private:
    FeatureId id_;
    FilletParams params_;
};

/// Chamfer of a set of edges of `body` (B1): equal distance, two distances, or distance + angle.
struct ChamferParams {
    enum class Mode { Equal, TwoDistances, DistanceAngle };
    BodyId body;
    std::vector<naming::ElementRef> edges;
    Mode mode = Mode::Equal;
    std::string distance = "1 mm";
    std::string distance2 = "1 mm";
    std::string angle = "45 deg";
    bool tangentChain = true;
};

class ChamferFeature final : public recompute::Feature {
public:
    ChamferFeature(FeatureId id, ChamferParams params) : id_(id), params_(std::move(params)) {}
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);

    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "chamfer"; }
    std::vector<naming::ElementRef> explicitRefs() const override { return params_.edges; }
    std::vector<std::string> parameterReferences() const override;
    std::vector<BodyId> readsBodies() const override { return {params_.body}; }
    std::vector<BodyId> writesBodies() const override { return {params_.body}; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;
    const ChamferParams& params() const noexcept { return params_; }

private:
    FeatureId id_;
    ChamferParams params_;
};

void registerEdgeFeatures(recompute::FeatureRegistry& registry);

} // namespace iw::features
