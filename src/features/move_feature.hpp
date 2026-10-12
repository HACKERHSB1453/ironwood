// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "core/ids.hpp"
#include "core/math.hpp"
#include "recompute/feature.hpp"

namespace iw::features {

/// Move / Copy body (B1). The rotation (angle about the axis through `axisOrigin` along `axisDirection`, applied first)
/// is followed by the translation `translate` (three length expressions, world X/Y/Z mm). Angle and translation are
/// expressions. Move (copy = false) replaces `body`; Copy leaves `body` alone and creates `newBody`.
struct MoveParams {
    BodyId body;
    bool copy = false;
    BodyId newBody;
    std::array<std::string, 3> translate{"0 mm", "0 mm", "0 mm"};
    std::string angle = "0 deg";
    Vec3 axisOrigin{0, 0, 0};
    Vec3 axisDirection{0, 0, 1};
};

class MoveFeature final : public recompute::Feature {
public:
    MoveFeature(FeatureId id, MoveParams params) : id_(id), params_(std::move(params)) {}
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);

    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "move"; }
    std::vector<std::string> parameterReferences() const override;
    std::vector<BodyId> readsBodies() const override { return {params_.body}; }
    std::vector<BodyId> writesBodies() const override;
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;
    const MoveParams& params() const noexcept { return params_; }

private:
    FeatureId id_;
    MoveParams params_;
};

void registerMoveFeature(recompute::FeatureRegistry& registry);

} // namespace iw::features
