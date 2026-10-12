// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "features/extrude_feature.hpp"
#include "features/feature_common.hpp"
#include "naming/element_name.hpp"
#include "recompute/feature.hpp"
#include "sketch/sketch.hpp"

namespace iw::features {

/// Revolve axis: a straight sketch line, a construction axis (a feature of kind "axis"), or a straight edge of the
/// target body (resolved by name in that body).
struct RevolveAxis {
    enum class Kind { SketchLine, Construction, Edge };
    Kind kind = Kind::SketchLine;
    sketch::SketchEntityId line; ///< SketchLine: id of a LineGeom entity of the sketch source.
    FeatureId construction;      ///< Construction: id of the axis feature.
    naming::ElementRef edge;     ///< Edge: reference to a linear edge of the target body.
};

/// Revolve parameters (B1).
///  * Extent: OneSide sweeps `angle` from the profile; TwoSides sweeps `angle` forwards and `angle2` backwards;
///    Symmetric sweeps `angle` in total, half on each side; Full sweeps 360 degrees (`angle` ignored).
///  * Angles are angle expressions; operation as for extrude (NewBody / Join / Cut / Intersect with `body`).
struct RevolveParams {
    enum class Extent { OneSide, TwoSides, Symmetric, Full };
    std::variant<PolygonSource, SketchSource, SketchReferenceSource> source;
    RevolveAxis axis;
    Extent extent = Extent::Full;
    std::string angle = "360 deg";
    std::string angle2 = "0 deg";
    common::BodyOp operation = common::BodyOp::NewBody;
    BodyId body;
};

class RevolveFeature final : public recompute::Feature {
public:
    RevolveFeature(FeatureId id, RevolveParams params) : id_(id), params_(std::move(params)) {}

    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);

    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "revolve"; }
    std::vector<naming::ElementRef> explicitRefs() const override;
    std::vector<std::string> parameterReferences() const override;
    std::vector<BodyId> readsBodies() const override;
    std::vector<BodyId> writesBodies() const override { return {params_.body}; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;

    const RevolveParams& params() const noexcept { return params_; }

private:
    FeatureId id_;
    RevolveParams params_;
};

void registerRevolveFeature(recompute::FeatureRegistry& registry);

} // namespace iw::features
