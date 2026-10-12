// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "core/math.hpp"
#include "recompute/feature.hpp"
#include "sketch/sketch.hpp"

namespace iw::features {

/// P0 profile: a polygon in plane coordinates with a numeric extrusion vector. Serialised exactly as in P0 (no
/// "source" key), so P0 documents keep loading.
struct PolygonSource {
    Vec3 planeOrigin;                          ///< mm
    Vec3 planeNormal{0, 0, 1};                 ///< non-zero; normalised internally
    Vec3 planeXDir{1, 0, 0};                   ///< non-zero, not parallel to the normal; orthogonalised internally
    std::vector<std::array<double, 2>> points; ///< >= 3 polygon vertices in plane coordinates (mm)
    Vec3 vector{0, 0, 10};                     ///< extrusion vector (mm), non-zero
};

/// B1 profile: a region of a sketch, selected by its detectProfiles() name ("e<id>.<sub>+...").
struct SketchSource {
    sketch::Sketch sketch; ///< Plane, entities (stable ids), constraint and dimension records.
    std::string profile;   ///< SketchProfile::name.
};

/// B1c profile taken from an upstream SketchFeature: stores the sketch feature id and the profile name, not a copy of
/// the sketch. The Document replaces it with an embedded SketchSource (solved sketch) before evaluation; evaluated
/// directly it fails with NotFound.
struct SketchReferenceSource {
    FeatureId sketch; ///< Id of the SketchFeature (must precede the referencing feature on the timeline).
    std::string profile;
};

/// Extrude feature parameters.
///  * Operation: NewBody creates `body`; Join / Cut / Intersect combine the extruded solid with the existing `body`
///    (fuse / subtract / common), which the feature reads and writes (implicit order dependency).
///  * Extent (sketch profiles only): OneSide extrudes `distance` along the sketch normal (negative = backwards);
///    TwoSides extrudes `distance` forwards and `distance2` backwards; Symmetric extrudes `distance` in total,
///    half on each side.
///  * `distance`, `distance2` are length expressions; `taper` is an angle expression (non-zero taper is not yet
///    implemented: ErrorCode::NotImplemented). Expressions are evaluated at compute time against the parameter table.
struct ExtrudeParams {
    enum class Operation { NewBody, Join, Cut, Intersect };
    enum class Extent { OneSide, TwoSides, Symmetric };

    std::variant<PolygonSource, SketchSource, SketchReferenceSource> source;
    Extent extent = Extent::OneSide;
    std::string distance = "10 mm";
    std::string distance2 = "0 mm";
    std::string taper = "0 deg";
    Operation operation = Operation::NewBody;
    BodyId body;
};

class ExtrudeFeature final : public recompute::Feature {
public:
    ExtrudeFeature(FeatureId id, ExtrudeParams params) : id_(id), params_(std::move(params)) {}

    /// Parses `paramsJson()` output (both the P0 polygon form and the sketch form). Errors: ParseError, InvalidArgument.
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);

    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "extrude"; }
    std::vector<naming::ElementRef> explicitRefs() const override { return {}; }
    std::vector<std::string> parameterReferences() const override;
    std::vector<BodyId> readsBodies() const override;
    std::vector<BodyId> writesBodies() const override { return {params_.body}; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;

    const ExtrudeParams& params() const noexcept { return params_; }

private:
    FeatureId id_;
    ExtrudeParams params_;
};

/// Registers kind "extrude" with `registry`.
void registerExtrudeFeature(recompute::FeatureRegistry& registry);

/// Registers every feature kind shipped by this module (currently "extrude").
void registerFeatures(recompute::FeatureRegistry& registry);

} // namespace iw::features
