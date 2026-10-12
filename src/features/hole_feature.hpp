// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "core/ids.hpp"
#include "features/feature_common.hpp"
#include "kernel/ops.hpp"
#include "naming/element_name.hpp"
#include "recompute/feature.hpp"

namespace iw::features {

/// Hole (B1): simple, counterbore or countersink hole cut into `body`, entering through a planar face.
///  * Placement: the entry point is origin + u * X + v * Y of the face's plane (kernel::planeFromFace); the hole runs
///    into the material along -normal.
///  * Depth: `depth` from the entry plane, or `through` (depth ignored). Drill point: `drillAngle` (full included angle).
///  * `thread` is a cosmetic thread designation such as "M6x1.0" (or empty). It produces NO geometry; it is published
///    as the feature result attribute "thread" (FeatureStatus::attributes).
struct HoleParams {
    BodyId body;
    naming::ElementRef face;
    std::string u = "0 mm";
    std::string v = "0 mm";
    kernel::HoleKind kind = kernel::HoleKind::Simple;
    std::string diameter = "6 mm";
    std::string depth = "10 mm";
    bool through = false;
    std::string drillAngle = "118 deg";
    std::string counterboreDiameter = "10 mm";
    std::string counterboreDepth = "3 mm";
    std::string countersinkDiameter = "12 mm";
    std::string countersinkAngle = "90 deg";
    std::string thread;
};

class HoleFeature final : public recompute::Feature {
public:
    HoleFeature(FeatureId id, HoleParams params) : id_(id), params_(std::move(params)) {}
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);

    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "hole"; }
    std::vector<naming::ElementRef> explicitRefs() const override { return {params_.face}; }
    std::vector<std::string> parameterReferences() const override;
    std::vector<BodyId> readsBodies() const override { return {params_.body}; }
    std::vector<BodyId> writesBodies() const override { return {params_.body}; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;
    const HoleParams& params() const noexcept { return params_; }

private:
    FeatureId id_;
    HoleParams params_;
};

void registerHoleFeature(recompute::FeatureRegistry& registry);

} // namespace iw::features
