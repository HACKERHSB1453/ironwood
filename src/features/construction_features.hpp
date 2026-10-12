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
#include "features/feature_common.hpp"
#include "naming/element_name.hpp"
#include "recompute/feature.hpp"

namespace iw::features {

/// A plane used as input of a construction feature: a planar face of a body, or a construction plane feature.
struct PlaneRef {
    bool isFace = true;
    BodyId body;                  ///< isFace: the body that owns the face.
    naming::ElementRef face;      ///< isFace: reference to a planar face.
    FeatureId construction;       ///< !isFace: id of a Plane construction feature.
};

/// Construction plane (B1), produces no bodies; its plane is published as ConstructionGeometry (Kind::Plane).
///  * Offset:       `base` moved by `distance` (expression) along its normal.
///  * MidPlane:     midway between two parallel planes `a` and `b`.
///  * ThreePoints:  through three points (each a triple of length expressions), normal = (b-a) x (c-a).
///  * AngleAxis:    `base` orientation rotated by `angle` about the construction axis `axis`; the plane passes through
///                  the axis origin. [VERIFY Fusion's exact anchor for the angle plane]
struct ConstructionPlaneParams {
    enum class Kind { Offset, MidPlane, ThreePoints, AngleAxis };
    Kind kind = Kind::Offset;
    PlaneRef base;   ///< Offset, AngleAxis
    PlaneRef other;  ///< MidPlane
    std::string distance = "10 mm";
    std::array<std::array<std::string, 3>, 3> points{};
    FeatureId axis;  ///< AngleAxis: id of an Axis construction feature.
    std::string angle = "45 deg";
};

/// Construction axis (B1): through a cylinder/cone face, through two points (expressions), or along a linear edge.
struct ConstructionAxisParams {
    enum class Kind { Cylinder, TwoPoints, Edge };
    Kind kind = Kind::TwoPoints;
    BodyId body;
    naming::ElementRef ref; ///< Cylinder: face; Edge: edge.
    std::array<std::string, 3> a{"0 mm", "0 mm", "0 mm"};
    std::array<std::string, 3> b{"0 mm", "0 mm", "10 mm"};
};

/// Construction point (B1): a vertex, or the centre of a circular edge (centre of mass of a full circle; partial arcs
/// are NOT validated and give the arc's centroid [limitation]).
struct ConstructionPointParams {
    enum class Kind { Vertex, CircleCentre };
    Kind kind = Kind::Vertex;
    BodyId body;
    naming::ElementRef ref;
};

class ConstructionPlaneFeature final : public recompute::Feature {
public:
    ConstructionPlaneFeature(FeatureId id, ConstructionPlaneParams p) : id_(id), params_(std::move(p)) {}
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);
    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "plane"; }
    std::vector<naming::ElementRef> explicitRefs() const override;
    std::vector<std::string> parameterReferences() const override;
    std::vector<BodyId> readsBodies() const override;
    std::vector<BodyId> writesBodies() const override { return {}; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;

private:
    FeatureId id_;
    ConstructionPlaneParams params_;
};

class ConstructionAxisFeature final : public recompute::Feature {
public:
    ConstructionAxisFeature(FeatureId id, ConstructionAxisParams p) : id_(id), params_(std::move(p)) {}
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);
    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "axis"; }
    std::vector<naming::ElementRef> explicitRefs() const override;
    std::vector<std::string> parameterReferences() const override;
    std::vector<BodyId> readsBodies() const override;
    std::vector<BodyId> writesBodies() const override { return {}; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;

private:
    FeatureId id_;
    ConstructionAxisParams params_;
};

class ConstructionPointFeature final : public recompute::Feature {
public:
    ConstructionPointFeature(FeatureId id, ConstructionPointParams p) : id_(id), params_(std::move(p)) {}
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);
    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "point"; }
    std::vector<naming::ElementRef> explicitRefs() const override { return {params_.ref}; }
    std::vector<BodyId> readsBodies() const override { return {params_.body}; }
    std::vector<BodyId> writesBodies() const override { return {}; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;

private:
    FeatureId id_;
    ConstructionPointParams params_;
};

void registerConstructionFeatures(recompute::FeatureRegistry& registry);

} // namespace iw::features
