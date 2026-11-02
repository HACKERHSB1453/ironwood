// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/ids.hpp"
#include "features/feature_common.hpp"
#include "kernel/ops.hpp"
#include "naming/element_name.hpp"
#include "recompute/feature.hpp"
#include "sketch/sketch.hpp"

namespace iw::features {

/// Placement "from sketch" (F26, Fusion: Hole -> From Sketch, multiple holes): one hole per referenced sketch point of an
/// upstream SketchFeature, all cut by the one feature. Points are bound by their stable SketchEntityId, so editing the
/// sketch (moving a point, changing a dimension) moves the holes.
///  * Accepted entities: Point entities (construction or not, including the sketch origin kOriginPointId) and the centre
///    of a Circle or Arc entity. Any other entity kind fails the feature (InvalidArgument).
///  * A referenced entity that no longer exists in the solved sketch is skipped with a Warning diagnostic naming its id;
///    when no referenced point is left the feature fails (NotFound).
///  * Entry point: the point on the sketch plane. Depth (or "through") is measured from the sketch plane. Like every
///    kernel::hole, the tool also clears max(1 mm, diameter) on the entry side (it assumes the entry lies on a surface),
///    so a sketch plane INSIDE the body also removes that much material above the entry (follow-up: kernel clearance option).
///  * Axis: the sketch normal n. The drill direction is decided once per feature from the body's bounding box relative
///    to the sketch plane: if the body lies entirely on the +n side the holes run along +n, otherwise (body entirely on
///    the -n side, or the plane cuts through the body) along -n, so a sketch on a top face (outward normal) drills
///    down into the part, like face placement. `flip` reverses the decided direction.
///  * The SketchFeature must precede the hole on the timeline. compute() reads its solved sketch from the recompute
///    sketch channel (EvalContext::sketch, F30), like Extrude / Revolve sketch references; when it published none the
///    feature fails with NotFound. `solved` is an optional embedded copy (set by pre-pass callers through
///    embedSketchReference) that takes precedence over the channel; it is never serialised.
struct HoleSketchPlacement {
    FeatureId sketch;
    std::vector<sketch::SketchEntityId> points;
    bool flip = false;
    std::optional<sketch::Sketch> solved;
};

/// Hole (B1): simple, counterbore or countersink hole cut into `body`, entering through a planar face.
///  * Placement: the entry point is origin + u * X + v * Y of the face's plane (kernel::planeFromFace); the hole runs
///    into the material along -normal. When `sketchPoints` is set, the holes are placed from sketch points instead
///    (see HoleSketchPlacement) and `face`, `u`, `v` are ignored.
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
    std::optional<HoleSketchPlacement> sketchPoints;
};

class HoleFeature final : public recompute::Feature {
public:
    HoleFeature(FeatureId id, HoleParams params) : id_(id), params_(std::move(params)) {}
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);

    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "hole"; }
    std::vector<naming::ElementRef> explicitRefs() const override {
        if (params_.sketchPoints) return {};
        return {params_.face};
    }
    std::vector<std::string> parameterReferences() const override;
    std::vector<BodyId> readsBodies() const override { return {params_.body}; }
    std::vector<BodyId> writesBodies() const override { return {params_.body}; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;
    const HoleParams& params() const noexcept { return params_; }

private:
    Result<recompute::FeatureResult> computeFromSketch(const recompute::EvalContext& ctx, const kernel::HoleSpec& spec) const;

    FeatureId id_;
    HoleParams params_;
};

void registerHoleFeature(recompute::FeatureRegistry& registry);

} // namespace iw::features
