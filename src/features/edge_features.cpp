// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/edge_features.hpp"

#include <algorithm>
#include <utility>

#include <nlohmann/json.hpp>

#include "core/expr.hpp"
#include "features/feature_json.hpp"
#include "kernel/ops.hpp"

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
Result<std::vector<std::uint32_t>> selectEdges(const EvalContext& ctx, BodyId body, const std::vector<naming::ElementRef>& refs,
                                               bool chain, std::vector<Diagnostic>& diags) {
    std::vector<std::uint32_t> idx;
    for (const auto& r : refs) {
        auto i = resolveRef(ctx, body, r, ElementType::Edge, diags);
        if (!i) return std::unexpected(i.error());
        idx.push_back(*i);
    }
    std::sort(idx.begin(), idx.end());
    idx.erase(std::unique(idx.begin(), idx.end()), idx.end());
    if (!chain) return idx;
    auto expanded = kernel::tangentChain(*ctx.body(body), idx, ctx.cancel);
    if (!expanded) return std::unexpected(expanded.error());
    return *expanded;
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
    auto idx = selectEdges(ctx, params_.body, params_.edges, params_.tangentChain, diags);
    if (!idx) return std::unexpected(idx.error());
    auto result = kernel::filletEdges(*ctx.body(params_.body), *idx, *radius, ctx.cancel);
    if (!result) return std::unexpected(result.error());
    return common::modifyBody(id_, params_.body, ctx, *result, "fillet", std::move(diags));
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
    auto idx = selectEdges(ctx, params_.body, params_.edges, params_.tangentChain, diags);
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
    return common::modifyBody(id_, params_.body, ctx, *result, "chamfer", std::move(diags));
}

void registerEdgeFeatures(recompute::FeatureRegistry& registry) {
    registry.add("fillet", [](FeatureId id, std::string_view p) { return FilletFeature::fromJson(id, p); });
    registry.add("chamfer", [](FeatureId id, std::string_view p) { return ChamferFeature::fromJson(id, p); });
}

} // namespace iw::features
