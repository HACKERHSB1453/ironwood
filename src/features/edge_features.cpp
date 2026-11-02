// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/edge_features.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <utility>

#include <nlohmann/json.hpp>

#include "core/expr.hpp"
#include "features/feature_json.hpp"
#include "kernel/ops.hpp"
#include "naming/naming_map.hpp"

namespace iw::features {
namespace {

using json = nlohmann::json;
using common::resolveRef;
using naming::ElementType;
using recompute::Diagnostic;
using recompute::EvalContext;
using recompute::FeatureResult;

/// Element references of an edge operation (JSON array under "edges").
Result<std::vector<naming::ElementRef>> edgesFromJson(const json& j) {
    auto it = j.find("edges");
    if (it == j.end() || !it->is_array() || it->empty()) return makeError(ErrorCode::ParseError, "edge feature: 'edges' must be a non-empty array");
    std::vector<naming::ElementRef> out;
    for (const json& e : *it) {
        json wrapper{{"e", e}};
        auto r = common::refFromJson(wrapper, "e");
        if (!r) return std::unexpected(r.error());
        if (r->type != ElementType::Edge) return makeError(ErrorCode::ParseError, "edge feature: reference is not an edge");
        out.push_back(*r);
    }
    return out;
}

json edgesToJson(const std::vector<naming::ElementRef>& edges) {
    json arr = json::array();
    for (const auto& e : edges) arr.push_back(common::refToJson(e));
    return arr;
}

/// Resolves the references to unique edge indices of the body at this position; optionally expands tangent chains.
/// `anchor` receives the index of the FIRST reference (a name-stable choice, used for the sibling frame).
Result<std::vector<std::uint32_t>> selectEdges(const EvalContext& ctx, BodyId body, const std::vector<naming::ElementRef>& refs,
                                               bool chain, std::vector<Diagnostic>& diags, std::uint32_t& anchor) {
    std::vector<std::uint32_t> idx;
    for (const auto& r : refs) {
        auto i = resolveRef(ctx, body, r, ElementType::Edge, diags);
        if (!i) return std::unexpected(i.error());
        idx.push_back(*i);
    }
    anchor = idx.front();
    std::sort(idx.begin(), idx.end());
    idx.erase(std::unique(idx.begin(), idx.end()), idx.end());
    if (!chain) return idx;
    auto expanded = kernel::tangentChain(*ctx.body(body), idx, ctx.cancel);
    if (!expanded) return std::unexpected(expanded.error());
    return *expanded;
}

/// Unit component of `v` perpendicular to unit `x`, or nullopt when `v` is (nearly) parallel to `x` or zero.
std::optional<Vec3> perpendicularPart(const Vec3& v, const Vec3& x) {
    const Vec3 p = v - x * dot(v, x);
    const double len = length(p);
    if (!(len > 1e-6 * std::max(1.0, length(v)))) return std::nullopt;
    return p * (1.0 / len);
}

/// F42: sibling frame attached to the anchor edge of the selection, so that sibling ordinals of the result do not change
/// when the body is moved OR rotated. Origin = the edge's start point, x = its start tangent (both from the edge's own
/// parameterisation, kernel::EdgeGeometry), z = the part of an adjacent face's normal (or axis) perpendicular to x; the
/// adjacent faces are tried in NAME order so the choice does not depend on kernel indices. Curved edges without a usable
/// face fall back to the circle axis / end tangent. nullopt (world keys) when no frame can be built (degenerate edge).
std::optional<naming::SiblingFrame> edgeFrame(const kernel::Shape& shape, const naming::NamingMap* names, std::uint32_t edge) {
    const kernel::EdgeGeometry g = shape.edgeGeometry(edge);
    if (g.degenerate || length(g.startTangent) < 0.5) return std::nullopt;
    const Vec3 x = g.startTangent;
    std::vector<std::pair<std::optional<naming::ElementName>, std::uint32_t>> faces;
    for (std::uint32_t f : shape.edgeFaces(edge)) faces.push_back({names ? names->nameOf(ElementType::Face, f) : std::nullopt, f});
    std::sort(faces.begin(), faces.end());
    std::optional<Vec3> z;
    for (const auto& [name, f] : faces) {
        const kernel::FaceGeometry fg = shape.faceGeometry(f);
        if (fg.kind == kernel::FaceGeometry::Kind::Plane) z = perpendicularPart(fg.normal, x);
        else if (fg.kind != kernel::FaceGeometry::Kind::Other) z = perpendicularPart(fg.direction, x);
        if (z) break;
    }
    if (!z && g.kind == kernel::EdgeGeometry::Kind::Circle) z = perpendicularPart(g.direction, x);
    if (!z) z = perpendicularPart(g.endTangent, x);
    if (!z) return std::nullopt;
    naming::SiblingFrame frame;
    frame.origin = g.start;
    frame.xAxis = x;
    frame.zAxis = *z;
    frame.yAxis = cross(*z, x);
    return frame;
}

/// Names the result of a fillet / chamfer of `body` (as common::modifyBody for a one-operand operation), with sibling
/// ordinals ranked by topological neighbours first and then in the anchor edge's frame (F42).
Result<FeatureResult> finishEdgeOp(FeatureId feature, BodyId body, const EvalContext& ctx, const kernel::ShapeResult& result,
                                   std::string_view opTag, std::uint32_t anchor, std::vector<Diagnostic> diags) {
    const kernel::Shape* input = ctx.body(body);
    const naming::NamingMap* inputNaming = ctx.naming(body);
    if (!input || !inputNaming) return makeError(ErrorCode::NotFound, "body " + std::to_string(body.value()) + " does not exist at this position");
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "feature cancelled");
    kernel::ShapeHistory own; // operand 0 only (the body); other operands carry no names here
    for (const auto& [in, out] : result.history.generated)
        if (in.operand == 0) own.generated.insert({in, out});
    for (const auto& [in, out] : result.history.modified)
        if (in.operand == 0) own.modified.insert({in, out});
    for (const auto& [in, out] : result.history.unchanged)
        if (in.operand == 0) own.unchanged.insert({in, out});
    for (const auto& in : result.history.deleted)
        if (in.operand == 0) own.deleted.insert(in);
    const auto frame = edgeFrame(*input, inputNaming, anchor);
    auto names = naming::deriveNamingMap(feature, opTag, result.shape, {inputNaming}, own, frame ? &*frame : nullptr, UINT32_MAX,
                                         /*topologicalSiblings=*/true);
    if (!names) return std::unexpected(names.error());
    FeatureResult out;
    out.diagnostics = std::move(diags);
    out.bodies.emplace(body, result.shape);
    out.namings.emplace(body, std::move(*names));
    return out;
}

Result<double> positiveLength(const EvalContext& ctx, const std::string& text, const char* what) {
    auto v = common::lengthOf(ctx.params, text, what);
    if (!v) return std::unexpected(v.error());
    if (!(*v > 0)) return makeError(ErrorCode::InvalidArgument, std::string(what) + " must be > 0");
    return *v;
}

std::vector<std::string> mergedRefs(const std::vector<const std::string*>& texts) {
    std::vector<std::string> out;
    for (const std::string* t : texts) {
        auto ex = Expression::parse(*t);
        if (ex) for (auto& r : ex->references()) out.push_back(r);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::uint64_t hashOf(std::string_view tag, const std::string& json) {
    common::Fnv f;
    f.bytes(tag);
    f.bytes(json);
    return f.h;
}

} // namespace

// ---- fillet ----------------------------------------------------------------------------------------------------

std::vector<std::string> FilletFeature::parameterReferences() const { return mergedRefs({&params_.radius}); }

std::uint64_t FilletFeature::paramHash() const { return hashOf("fillet/v1:", paramsJson()); }

std::string FilletFeature::paramsJson() const {
    json j{{"body", params_.body.value()}, {"edges", edgesToJson(params_.edges)}, {"radius", params_.radius},
           {"tangent_chain", params_.tangentChain}};
    return j.dump();
}

Result<std::shared_ptr<recompute::Feature>> FilletFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "fillet: params are not a JSON object");
    FilletParams p;
    auto edges = edgesFromJson(j);
    if (!edges) return std::unexpected(edges.error());
    p.edges = std::move(*edges);
    auto radius = common::readString(j, "radius");
    if (!radius) return std::unexpected(radius.error());
    if (auto c = common::checkExpression(*radius, "radius"); !c) return std::unexpected(c.error());
    p.radius = *radius;
    p.tangentChain = j.value("tangent_chain", true);
    auto body = j.find("body");
    if (body == j.end() || !body->is_number_unsigned()) return makeError(ErrorCode::ParseError, "fillet: 'body' must be an unsigned id");
    p.body = BodyId{body->get<std::uint64_t>()};
    return std::shared_ptr<recompute::Feature>(std::make_shared<FilletFeature>(id, std::move(p)));
}

Result<FeatureResult> FilletFeature::compute(const EvalContext& ctx) const {
    if (!params_.body.valid()) return makeError(ErrorCode::InvalidArgument, "fillet: invalid body id");
    if (params_.edges.empty()) return makeError(ErrorCode::InvalidArgument, "fillet: no edges selected");
    if (!ctx.body(params_.body)) return makeError(ErrorCode::NotFound, "fillet: body does not exist at this position");
    auto radius = positiveLength(ctx, params_.radius, "fillet radius");
    if (!radius) return std::unexpected(radius.error());
    std::vector<Diagnostic> diags;
    std::uint32_t anchor = 0;
    auto idx = selectEdges(ctx, params_.body, params_.edges, params_.tangentChain, diags, anchor);
    if (!idx) return std::unexpected(idx.error());
    auto result = kernel::filletEdges(*ctx.body(params_.body), *idx, *radius, ctx.cancel);
    if (!result) return std::unexpected(result.error());
    return finishEdgeOp(id_, params_.body, ctx, *result, "fillet", anchor, std::move(diags));
}

// ---- chamfer ---------------------------------------------------------------------------------------------------

namespace {
const char* modeName(ChamferParams::Mode m) {
    switch (m) {
    case ChamferParams::Mode::Equal: return "equal";
    case ChamferParams::Mode::TwoDistances: return "two_distances";
    case ChamferParams::Mode::DistanceAngle: return "distance_angle";
    }
    return "equal";
}
} // namespace

std::vector<std::string> ChamferFeature::parameterReferences() const {
    return mergedRefs({&params_.distance, &params_.distance2, &params_.angle});
}

std::uint64_t ChamferFeature::paramHash() const { return hashOf("chamfer/v1:", paramsJson()); }

std::string ChamferFeature::paramsJson() const {
    json j{{"body", params_.body.value()}, {"edges", edgesToJson(params_.edges)}, {"mode", modeName(params_.mode)},
           {"distance", params_.distance}, {"distance2", params_.distance2}, {"angle", params_.angle},
           {"tangent_chain", params_.tangentChain}};
    return j.dump();
}

Result<std::shared_ptr<recompute::Feature>> ChamferFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "chamfer: params are not a JSON object");
    ChamferParams p;
    auto edges = edgesFromJson(j);
    if (!edges) return std::unexpected(edges.error());
    p.edges = std::move(*edges);
    auto mode = common::readString(j, "mode", std::string("equal"));
    if (!mode) return std::unexpected(mode.error());
    if (*mode == "equal") p.mode = ChamferParams::Mode::Equal;
    else if (*mode == "two_distances") p.mode = ChamferParams::Mode::TwoDistances;
    else if (*mode == "distance_angle") p.mode = ChamferParams::Mode::DistanceAngle;
    else return makeError(ErrorCode::ParseError, "chamfer: unknown mode '" + *mode + "'");
    auto d1 = common::readString(j, "distance");
    if (!d1) return std::unexpected(d1.error());
    auto d2 = common::readString(j, "distance2", std::string("1 mm"));
    if (!d2) return std::unexpected(d2.error());
    auto an = common::readString(j, "angle", std::string("45 deg"));
    if (!an) return std::unexpected(an.error());
    for (const auto& [t, key] : {std::pair{*d1, "distance"}, std::pair{*d2, "distance2"}, std::pair{*an, "angle"}})
        if (auto c = common::checkExpression(t, key); !c) return std::unexpected(c.error());
    p.distance = *d1;
    p.distance2 = *d2;
    p.angle = *an;
    p.tangentChain = j.value("tangent_chain", true);
    auto body = j.find("body");
    if (body == j.end() || !body->is_number_unsigned()) return makeError(ErrorCode::ParseError, "chamfer: 'body' must be an unsigned id");
    p.body = BodyId{body->get<std::uint64_t>()};
    return std::shared_ptr<recompute::Feature>(std::make_shared<ChamferFeature>(id, std::move(p)));
}

Result<FeatureResult> ChamferFeature::compute(const EvalContext& ctx) const {
    if (!params_.body.valid()) return makeError(ErrorCode::InvalidArgument, "chamfer: invalid body id");
    if (params_.edges.empty()) return makeError(ErrorCode::InvalidArgument, "chamfer: no edges selected");
    if (!ctx.body(params_.body)) return makeError(ErrorCode::NotFound, "chamfer: body does not exist at this position");
    std::vector<Diagnostic> diags;
    std::uint32_t anchor = 0;
    auto idx = selectEdges(ctx, params_.body, params_.edges, params_.tangentChain, diags, anchor);
    if (!idx) return std::unexpected(idx.error());
    const kernel::Shape& shape = *ctx.body(params_.body);

    Result<kernel::ShapeResult> result = makeError(ErrorCode::Internal, "chamfer not computed");
    switch (params_.mode) {
    case ChamferParams::Mode::Equal: {
        auto d = positiveLength(ctx, params_.distance, "chamfer distance");
        if (!d) return std::unexpected(d.error());
        result = kernel::chamferEdges(shape, *idx, *d, *d, ctx.cancel);
        break;
    }
    case ChamferParams::Mode::TwoDistances: {
        auto d1 = positiveLength(ctx, params_.distance, "chamfer distance");
        if (!d1) return std::unexpected(d1.error());
        auto d2 = positiveLength(ctx, params_.distance2, "chamfer distance2");
        if (!d2) return std::unexpected(d2.error());
        result = kernel::chamferEdges(shape, *idx, *d1, *d2, ctx.cancel);
        break;
    }
    case ChamferParams::Mode::DistanceAngle: {
        auto d = positiveLength(ctx, params_.distance, "chamfer distance");
        if (!d) return std::unexpected(d.error());
        auto a = common::angleOf(ctx.params, params_.angle, "chamfer angle");
        if (!a) return std::unexpected(a.error());
        result = kernel::chamferEdgesByAngle(shape, *idx, *d, *a, ctx.cancel);
        break;
    }
    }
    if (!result) return std::unexpected(result.error());
    return finishEdgeOp(id_, params_.body, ctx, *result, "chamfer", anchor, std::move(diags));
}

void registerEdgeFeatures(recompute::FeatureRegistry& registry) {
    registry.add("fillet", [](FeatureId id, std::string_view p) { return FilletFeature::fromJson(id, p); });
    registry.add("chamfer", [](FeatureId id, std::string_view p) { return ChamferFeature::fromJson(id, p); });
}

} // namespace iw::features
