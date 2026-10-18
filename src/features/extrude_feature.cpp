// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/extrude_feature.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <numbers>
#include <map>
#include <optional>
#include <utility>

#include <nlohmann/json.hpp>

#include "core/expr.hpp"
#include "core/params.hpp"
#include "features/construction_features.hpp"
#include "features/edge_features.hpp"
#include "features/feature_common.hpp"
#include "features/hole_feature.hpp"
#include "features/move_feature.hpp"
#include "features/revolve_feature.hpp"
#include "features/feature_json.hpp"
#include "features/sketch_feature.hpp"
#include "features/sketch_json.hpp"
#include "kernel/internal/profile_support.hpp"
#include "kernel/ops.hpp"
#include "kernel/taper.hpp"
#include "naming/naming_map.hpp"
#include "sketch/profile.hpp"

namespace iw::features {
using namespace common;
namespace {

using json = nlohmann::json;
using recompute::Diagnostic;
using recompute::EvalContext;
using recompute::Feature;
using recompute::FeatureRegistry;
using recompute::FeatureResult;
using naming::ElementName;
using naming::ElementType;
using naming::NamingMap;

const char* operationName(ExtrudeParams::Operation o) {
    switch (o) {
    case ExtrudeParams::Operation::NewBody: return "new_body";
    case ExtrudeParams::Operation::Join: return "join";
    case ExtrudeParams::Operation::Cut: return "cut";
    case ExtrudeParams::Operation::Intersect: return "intersect";
    }
    return "new_body";
}

const char* extentName(ExtrudeParams::Extent e) {
    switch (e) {
    case ExtrudeParams::Extent::OneSide: return "one_side";
    case ExtrudeParams::Extent::TwoSides: return "two_sides";
    case ExtrudeParams::Extent::Symmetric: return "symmetric";
    }
    return "one_side";
}

Result<ExtrudeParams::Operation> operationFrom(const std::string& s) {
    if (s == "new_body") return ExtrudeParams::Operation::NewBody;
    if (s == "join") return ExtrudeParams::Operation::Join;
    if (s == "cut") return ExtrudeParams::Operation::Cut;
    if (s == "intersect") return ExtrudeParams::Operation::Intersect;
    return makeError(ErrorCode::ParseError, "extrude: unknown operation '" + s + "'");
}

Result<ExtrudeParams::Extent> extentFrom(const std::string& s) {
    if (s == "one_side") return ExtrudeParams::Extent::OneSide;
    if (s == "two_sides") return ExtrudeParams::Extent::TwoSides;
    if (s == "symmetric") return ExtrudeParams::Extent::Symmetric;
    return makeError(ErrorCode::ParseError, "extrude: unknown extent '" + s + "'");
}

} // namespace

std::vector<std::string> ExtrudeFeature::parameterReferences() const {
    std::vector<std::string> out;
    if (!std::holds_alternative<PolygonSource>(params_.source)) {
        for (const std::string* text : {&params_.distance, &params_.distance2, &params_.taper}) {
            auto r = referencesOf(*text);
            out.insert(out.end(), r.begin(), r.end());
        }
    }
    if (const auto* sk = std::get_if<SketchSource>(&params_.source)) {
        // Sketch dimensions are expressions too; editing one changes the sketch the feature extrudes.
        for (const auto& [id, dim] : sk->sketch.dimensions()) {
            auto r = referencesOf(dim.expression);
            out.insert(out.end(), r.begin(), r.end());
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::vector<BodyId> ExtrudeFeature::readsBodies() const {
    if (params_.operation != ExtrudeParams::Operation::NewBody) return {params_.body};
    return {};
}

std::uint64_t ExtrudeFeature::paramHash() const {
    // Hashes the serialised parameters: every field (including sketch geometry and ids) is covered exactly.
    Fnv f;
    f.bytes("extrude/v1:");
    f.bytes(paramsJson());
    return f.h;
}

std::string ExtrudeFeature::paramsJson() const {
    json j = json::object();
    auto v3 = [](const Vec3& v) { return json::array({v.x, v.y, v.z}); };
    if (const auto* poly = std::get_if<PolygonSource>(&params_.source)) {
        // P0 layout, unchanged: no "source" key.
        j["origin"] = v3(poly->planeOrigin);
        j["normal"] = v3(poly->planeNormal);
        j["xdir"] = v3(poly->planeXDir);
        json pts = json::array();
        for (const auto& p : poly->points) pts.push_back(json::array({p[0], p[1]}));
        j["points"] = std::move(pts);
        j["vector"] = v3(poly->vector);
    } else if (const auto* ref = std::get_if<SketchReferenceSource>(&params_.source)) {
        j["source"] = "sketch_ref";
        j["sketch_feature"] = ref->sketch.value();
        j["profile"] = ref->profile;
        j["extent"] = extentName(params_.extent);
        j["distance"] = params_.distance;
        j["distance2"] = params_.distance2;
        j["taper"] = params_.taper;
    } else {
        const auto& sk = std::get<SketchSource>(params_.source);
        j["source"] = "sketch";
        auto sj = sketchToJson(sk.sketch);
        j["sketch"] = sj ? json::parse(*sj, nullptr, false) : json::object();
        j["profile"] = sk.profile;
        j["extent"] = extentName(params_.extent);
        j["distance"] = params_.distance;
        j["distance2"] = params_.distance2;
        j["taper"] = params_.taper;
    }
    j["operation"] = operationName(params_.operation);
    j["body"] = params_.body.value();
    return j.dump();
}

Result<std::shared_ptr<Feature>> ExtrudeFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "extrude: params are not a JSON object");
    ExtrudeParams p;

    // P0 documents have no "source" key and are polygon profiles.
    std::string source = "polygon";
    if (auto it = j.find("source"); it != j.end()) {
        if (!it->is_string()) return makeError(ErrorCode::ParseError, "extrude: 'source' must be a string");
        source = it->get<std::string>();
    }
    if (source == "sketch" || source == "sketch_ref") {
        const bool isRef = source == "sketch_ref";
        std::optional<sketch::Sketch> embedded;
        FeatureId refId;
        if (isRef) {
            auto sf = j.find("sketch_feature");
            if (sf == j.end() || !sf->is_number_unsigned()) return makeError(ErrorCode::ParseError, "extrude: 'sketch_feature' must be an unsigned id");
            refId = FeatureId{sf->get<std::uint64_t>()};
        } else {
            auto skIt = j.find("sketch");
            if (skIt == j.end()) return makeError(ErrorCode::ParseError, "extrude: sketch source without 'sketch'");
            auto sk = sketchFromJson(skIt->dump());
            if (!sk) return std::unexpected(sk.error());
            embedded = std::move(*sk);
        }
        auto profile = readString(j, "profile");
        if (!profile) return std::unexpected(profile.error());
        auto extent = readString(j, "extent", std::string("one_side"));
        if (!extent) return std::unexpected(extent.error());
        auto ext = extentFrom(*extent);
        if (!ext) return std::unexpected(ext.error());
        auto d1 = readString(j, "distance");
        if (!d1) return std::unexpected(d1.error());
        auto d2 = readString(j, "distance2", std::string("0 mm"));
        if (!d2) return std::unexpected(d2.error());
        auto tp = readString(j, "taper", std::string("0 deg"));
        if (!tp) return std::unexpected(tp.error());
        for (const auto& [text2, key] : {std::pair{*d1, "distance"}, std::pair{*d2, "distance2"}, std::pair{*tp, "taper"}}) {
            if (auto c = checkExpression(text2, key); !c) return std::unexpected(c.error());
        }
        if (isRef) p.source = SketchReferenceSource{refId, *profile};
        else p.source = SketchSource{std::move(*embedded), *profile};
        p.extent = *ext;
        p.distance = *d1;
        p.distance2 = *d2;
        p.taper = *tp;
    } else if (source == "polygon") {
        PolygonSource poly;
        auto grab = [&](const char* key, Vec3& out) -> Result<void> {
            auto v = readVec(j, key);
            if (!v) return std::unexpected(v.error());
            out = {(*v)[0], (*v)[1], (*v)[2]};
            return {};
        };
        if (auto r = grab("origin", poly.planeOrigin); !r) return std::unexpected(r.error());
        if (auto r = grab("normal", poly.planeNormal); !r) return std::unexpected(r.error());
        if (auto r = grab("xdir", poly.planeXDir); !r) return std::unexpected(r.error());
        if (auto r = grab("vector", poly.vector); !r) return std::unexpected(r.error());
        auto pts = j.find("points");
        if (pts == j.end() || !pts->is_array()) return makeError(ErrorCode::ParseError, "extrude: 'points' must be an array");
        for (const json& e : *pts) {
            if (!e.is_array() || e.size() != 2 || !e[0].is_number() || !e[1].is_number())
                return makeError(ErrorCode::ParseError, "extrude: each point must be [x, y]");
            poly.points.push_back({e[0].get<double>(), e[1].get<double>()});
        }
        p.source = std::move(poly);
    } else {
        return makeError(ErrorCode::ParseError, "extrude: unknown source '" + source + "'");
    }

    auto op = readString(j, "operation");
    if (!op) return std::unexpected(op.error());
    auto operation = operationFrom(*op);
    if (!operation) return std::unexpected(operation.error());
    p.operation = *operation;
    auto body = j.find("body");
    if (body == j.end() || !body->is_number_unsigned()) return makeError(ErrorCode::ParseError, "extrude: 'body' must be an unsigned id");
    p.body = BodyId{body->get<std::uint64_t>()};
    return std::shared_ptr<Feature>(std::make_shared<ExtrudeFeature>(id, std::move(p)));
}

namespace {

struct Extrusion {
    kernel::Shape shape;
    NamingMap naming;
};

/// One prism of the profile face along `vector`: plain when `taper` is zero, otherwise a draft prism.
Result<Extrusion> extrudeOne(FeatureId feature, const kernel::Shape& face, const NamingMap& pnaming, const Vec3& vector,
                             double taper, const char* opTag, const CancelToken& cancel) {
    auto ext = taper == 0 ? kernel::extrudeProfile(face, vector, cancel) : kernel::extrudeProfileTapered(face, vector, taper, cancel);
    if (!ext) return std::unexpected(ext.error());
    auto nm = naming::deriveNamingMap(feature, opTag, ext->shape, {&pnaming}, ext->history);
    if (!nm) return std::unexpected(nm.error());
    return Extrusion{std::move(ext->shape), std::move(*nm)};
}

/// The extrusion of the profile: one prism, or for a tapered two-sided extent one draft prism per side fused together
/// (both sides start at the sketch plane and lean inward by the same taper).
Result<Extrusion> extrudeSides(FeatureId feature, const kernel::Shape& face, const NamingMap& pnaming,
                               const std::vector<Vec3>& vectors, double taper, const CancelToken& cancel) {
    if (vectors.size() == 1) return extrudeOne(feature, face, pnaming, vectors[0], taper, "extrude", cancel);
    // The two sides carry distinct op tags so that their side faces (same profile edges) get distinct names.
    auto a = extrudeOne(feature, face, pnaming, vectors[0], taper, "extrude", cancel);
    if (!a) return std::unexpected(a.error());
    auto b = extrudeOne(feature, face, pnaming, vectors[1], taper, "extrude_back", cancel);
    if (!b) return std::unexpected(b.error());
    auto fused = kernel::fuse(a->shape, b->shape, cancel);
    if (!fused) return std::unexpected(fused.error());
    if (fused->shape.isNull() || fused->shape.subShapeCount(kernel::SubShapeKind::Face) == 0)
        return makeError(ErrorCode::KernelFailure, "extrude: tapered sides did not fuse into a solid");
    auto nm = naming::deriveNamingMap(feature, "fuse", fused->shape, {&a->naming, &b->naming}, fused->history);
    if (!nm) return std::unexpected(nm.error());
    return Extrusion{std::move(fused->shape), std::move(*nm)};
}

/// Shared tail: extrude the planar face, then apply the operation to the target body.
Result<FeatureResult> finishExtrude(FeatureId feature, const ExtrudeParams& p, const EvalContext& ctx, const kernel::Shape& face,
                                    const NamingMap& pnaming, const std::vector<Vec3>& vectors, double taper,
                                    std::vector<Diagnostic> diags) {
    const bool combine = p.operation != ExtrudeParams::Operation::NewBody;
    const kernel::Shape* target = nullptr;
    const NamingMap* targetNaming = nullptr;
    if (combine) {
        target = ctx.body(p.body);
        if (!target) return makeError(ErrorCode::NotFound, "extrude: target body does not exist at this position");
        targetNaming = ctx.naming(p.body);
        if (!targetNaming) return makeError(ErrorCode::Internal, "extrude: target has no naming map");
    }
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "extrude cancelled");

    auto ext = extrudeSides(feature, face, pnaming, vectors, taper, ctx.cancel);
    if (!ext) return std::unexpected(ext.error());

    FeatureResult out;
    out.diagnostics = std::move(diags);
    if (!combine) {
        out.bodies.emplace(p.body, std::move(ext->shape));
        out.namings.emplace(p.body, std::move(ext->naming));
        return out;
    }

    Result<kernel::ShapeResult> combined = makeError(ErrorCode::Internal, "not combined");
    const char* opTag = "fuse";
    switch (p.operation) {
    case ExtrudeParams::Operation::Join: combined = kernel::fuse(*target, ext->shape, ctx.cancel); opTag = "fuse"; break;
    case ExtrudeParams::Operation::Cut: combined = kernel::cut(*target, ext->shape, ctx.cancel); opTag = "cut"; break;
    case ExtrudeParams::Operation::Intersect: combined = kernel::common(*target, ext->shape, ctx.cancel); opTag = "common"; break;
    case ExtrudeParams::Operation::NewBody: break;
    }
    if (!combined) return std::unexpected(combined.error());
    if (combined->shape.isNull() || combined->shape.subShapeCount(kernel::SubShapeKind::Face) == 0)
        return makeError(ErrorCode::KernelFailure, std::string("extrude: ") + operationName(p.operation) + " produced no solid");
    auto combinedNaming = naming::deriveNamingMap(feature, opTag, combined->shape, {targetNaming, &ext->naming}, combined->history);
    if (!combinedNaming) return std::unexpected(combinedNaming.error());
    out.bodies.emplace(p.body, std::move(combined->shape));
    out.namings.emplace(p.body, std::move(*combinedNaming));
    return out;
}

} // namespace

Result<FeatureResult> ExtrudeFeature::compute(const EvalContext& ctx) const {
    const ExtrudeParams& p = params_;
    if (!p.body.valid()) return makeError(ErrorCode::InvalidArgument, "extrude: invalid body id");

    if (const auto* poly = std::get_if<PolygonSource>(&p.source)) {
        if (poly->points.size() < 3) return makeError(ErrorCode::InvalidArgument, "extrude: profile needs >= 3 points");
        if (!finite(poly->planeOrigin) || !finite(poly->planeNormal) || !finite(poly->planeXDir) || !finite(poly->vector))
            return makeError(ErrorCode::InvalidArgument, "extrude: non-finite parameter");
        if (length(poly->planeNormal) < 1e-12 || length(poly->planeXDir) < 1e-12)
            return makeError(ErrorCode::InvalidArgument, "extrude: plane normal/xdir must be non-zero");
        const Vec3 n = normalized(poly->planeNormal);
        Vec3 xd = poly->planeXDir - n * dot(poly->planeXDir, n);
        if (length(xd) < 1e-9) return makeError(ErrorCode::InvalidArgument, "extrude: xdir is parallel to the normal");
        xd = normalized(xd);
        const Vec3 yd = cross(n, xd);
        std::vector<Vec3> pts;
        pts.reserve(poly->points.size());
        for (const auto& q : poly->points) pts.push_back(poly->planeOrigin + xd * q[0] + yd * q[1]);
        if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "extrude cancelled");
        auto face = kernel::makePolygonFace(pts);
        if (!face) return std::unexpected(face.error());
        auto pnaming = polygonProfileNaming(id_, *face);
        if (!pnaming) return std::unexpected(pnaming.error());
        return finishExtrude(id_, p, ctx, *face, *pnaming, {poly->vector}, 0.0, {});
    }

    if (std::holds_alternative<SketchReferenceSource>(p.source))
        return makeError(ErrorCode::NotFound, "extrude: sketch feature " + std::to_string(std::get<SketchReferenceSource>(p.source).sketch.value()) +
                                                  " has no solved profile at this position (suppressed, failed, or not earlier on the timeline)");
    const auto& src = std::get<SketchSource>(p.source);
    auto d1 = lengthOf(ctx.params, p.distance, "distance");
    if (!d1) return std::unexpected(d1.error());
    double d2 = 0;
    if (p.extent == ExtrudeParams::Extent::TwoSides) {
        auto v = lengthOf(ctx.params, p.distance2, "distance2");
        if (!v) return std::unexpected(v.error());
        d2 = *v;
    }
    auto taper = angleOf(ctx.params, p.taper, "taper");
    if (!taper) return std::unexpected(taper.error());

    auto profiles = sketch::detectProfiles(src.sketch);
    if (!profiles) return std::unexpected(profiles.error());
    const sketch::SketchProfile* prof = nullptr;
    for (const auto& pr : *profiles)
        if (pr.name == src.profile) prof = &pr;
    if (!prof) return makeError(ErrorCode::NotFound, "extrude: sketch has no profile '" + src.profile + "'");

    // Start the profile on the side the extent requires; the sweep vector then spans the whole extent.
    // Untapered extents sweep one vector (the plane moves to the far end for TwoSides/Symmetric). A tapered two-sided
    // extent keeps the plane and sweeps each side as its own draft prism: forwards by `d1` (Symmetric: d/2) and
    // backwards by `d2` (Symmetric: d/2).
    const bool tapered = std::abs(*taper) > 1e-12;
    sketch::SketchPlane plane = src.sketch.plane();
    const Vec3 n = plane.normal;
    std::vector<Vec3> vectors;
    switch (p.extent) {
    case ExtrudeParams::Extent::OneSide:
        vectors = {n * *d1};
        break;
    case ExtrudeParams::Extent::TwoSides:
        if (tapered) {
            vectors = {n * *d1, n * -d2};
        } else {
            plane.origin = plane.origin - n * d2;
            vectors = {n * (*d1 + d2)};
        }
        break;
    case ExtrudeParams::Extent::Symmetric:
        if (tapered) {
            vectors = {n * (*d1 * 0.5), n * (*d1 * -0.5)};
        } else {
            plane.origin = plane.origin - n * (*d1 * 0.5);
            vectors = {n * *d1};
        }
        break;
    }
    if (tapered) {
        std::erase_if(vectors, [](const Vec3& v) { return length(v) < 1e-12; });
        if (vectors.empty()) return makeError(ErrorCode::InvalidArgument, "extrude: tapered extrude needs a non-zero extent");
    }
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "extrude cancelled");

    auto face = sketch::profileFace(plane, *prof);
    if (!face) return std::unexpected(face.error());
    std::vector<Diagnostic> diags;
    auto pnaming = sketchProfileNaming(id_, *face, *prof, plane, diags);
    if (!pnaming) return std::unexpected(pnaming.error());
    return finishExtrude(id_, p, ctx, *face, *pnaming, vectors, *taper, std::move(diags));
}

void registerExtrudeFeature(FeatureRegistry& registry) {
    registry.add("extrude", [](FeatureId id, std::string_view params) { return ExtrudeFeature::fromJson(id, params); });
}

void registerFeatures(FeatureRegistry& registry) {
    registerExtrudeFeature(registry);
    registerRevolveFeature(registry);
    registerEdgeFeatures(registry);
    registerHoleFeature(registry);
    registerMoveFeature(registry);
    registerConstructionFeatures(registry);
    registerSketchFeature(registry);
}

} // namespace iw::features
