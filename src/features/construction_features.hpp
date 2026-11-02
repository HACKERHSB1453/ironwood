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
///  * Tangent (F26): tangent to the cylindrical or conical face `base` (must be a face reference, `isFace`) along the
///                  surface ruling at `angle` (expression) around the face's axis. Angle convention (see
///                  kernel::tangentPlaneOnFace): axis k = the surface axis flipped so its largest component is
///                  positive; reference direction r = the world axis least aligned with k, projected perpendicular to
///                  k; the ruling is at radial direction cos(angle) r + sin(angle) (k x r), i.e. counter-clockwise
///                  about k from r. E.g. for a Z-axis cylinder 0 deg touches the +X side, 90 deg the +Y side. The
///                  normal points away from the axis; xDir runs along the ruling.
struct ConstructionPlaneParams {
    enum class Kind { Offset, MidPlane, ThreePoints, AngleAxis, Tangent };
    Kind kind = Kind::Offset;
    PlaneRef base;   ///< Offset, AngleAxis; Tangent: the cylindrical/conical face (isFace = true)
    PlaneRef other;  ///< MidPlane
    std::string distance = "10 mm";
    std::array<std::array<std::string, 3>, 3> points{};
    FeatureId axis;  ///< AngleAxis: id of an Axis construction feature.
    std::string angle = "45 deg"; ///< AngleAxis, Tangent
};

/// Construction axis (B1): through a cylinder/cone face, through two points (expressions), along a linear edge, or
/// (F26) TwoPlanes: the intersection line of `planeA` and `planeB` (planar faces or Plane construction features).
/// TwoPlanes: direction = normalised(nA x nB), origin = the line's point closest to the world origin; parallel planes
/// fail with InvalidArgument ("planes are parallel").
struct ConstructionAxisParams {
    enum class Kind { Cylinder, TwoPoints, Edge, TwoPlanes };
    Kind kind = Kind::TwoPoints;
    BodyId body;
    naming::ElementRef ref; ///< Cylinder: face; Edge: edge.
    std::array<std::string, 3> a{"0 mm", "0 mm", "0 mm"};
    std::array<std::string, 3> b{"0 mm", "0 mm", "10 mm"};
    PlaneRef planeA; ///< TwoPlanes
    PlaneRef planeB; ///< TwoPlanes
};

/// Construction point (B1):
///  * Vertex:       the vertex `ref`.
///  * CircleCentre: the centre of the circular edge `ref` (full circle or arc: the circle's centre, not the arc's
///                  centroid). Any non-circular edge fails with InvalidArgument (F26).
///  * EdgePlane (F26): intersection of the edge `ref` (line, arc, B-spline, ...) with `plane` (a planar face or a Plane
///                  construction feature; the plane is infinite, the edge is bounded). No intersection, or the edge
///                  lying in the plane, fails with InvalidArgument. With several intersections the one nearest to the
///                  edge's start (along the edge, honouring its orientation) is used and a Warning diagnostic reports
///                  the count.
struct ConstructionPointParams {
    enum class Kind { Vertex, CircleCentre, EdgePlane };
    Kind kind = Kind::Vertex;
    BodyId body;
    naming::ElementRef ref;
    PlaneRef plane; ///< EdgePlane
};

class ConstructionPlaneFeature final : public recompute::Feature {
public:
    ConstructionPlaneFeature(FeatureId id, ConstructionPlaneParams p) : id_(id), params_(std::move(p)) {}
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);
    FeatureId id() const override { return id_; }
    /// Parameters as last set (edit them through Document::editConstruction*).
    const ConstructionPlaneParams& params() const noexcept { return params_; }
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
    /// Parameters as last set (edit them through Document::editConstruction*).
    const ConstructionAxisParams& params() const noexcept { return params_; }
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
    /// Parameters as last set (edit them through Document::editConstruction*).
    const ConstructionPointParams& params() const noexcept { return params_; }
    std::string_view kind() const override { return "point"; }
    std::vector<naming::ElementRef> explicitRefs() const override;
    std::vector<BodyId> readsBodies() const override;
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
