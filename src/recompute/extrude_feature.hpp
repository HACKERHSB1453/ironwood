// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <vector>

#include "core/math.hpp"
#include "recompute/feature.hpp"

namespace iw::recompute {

/// The P0 proof feature: extrudes a polygon profile (2D points on a plane) along a vector.
///  * NewBody: produces body `body` (must not be relied upon to pre-exist; an existing body with that id is replaced).
///  * Join:    fuses the extruded solid into the existing body `body` (reads and writes it; implicit dependency).
struct ExtrudeParams {
    enum class Operation { NewBody, Join };

    Vec3 planeOrigin;                  ///< mm
    Vec3 planeNormal{0, 0, 1};         ///< non-zero; normalised internally
    Vec3 planeXDir{1, 0, 0};           ///< non-zero, not parallel to the normal; orthogonalised internally
    std::vector<std::array<double, 2>> points; ///< >= 3 polygon vertices in plane coordinates (mm)
    Vec3 vector{0, 0, 10};             ///< extrusion vector (mm), non-zero
    Operation operation = Operation::NewBody;
    BodyId body;                       ///< created body (NewBody) or target body (Join)
};

class ExtrudeFeature final : public Feature {
public:
    ExtrudeFeature(FeatureId id, ExtrudeParams params) : id_(id), params_(std::move(params)) {}

    /// Parses `paramsJson()` output. Errors: ParseError / InvalidArgument.
    static Result<std::shared_ptr<Feature>> fromJson(FeatureId id, std::string_view json);

    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "extrude"; }
    std::vector<BodyId> readsBodies() const override;
    std::vector<BodyId> writesBodies() const override { return {params_.body}; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    Result<FeatureResult> compute(const EvalContext& ctx) const override;

    const ExtrudeParams& params() const noexcept { return params_; }

private:
    FeatureId id_;
    ExtrudeParams params_;
};

/// Registers kind "extrude" with `registry`.
void registerExtrudeFeature(FeatureRegistry& registry);

} // namespace iw::recompute
