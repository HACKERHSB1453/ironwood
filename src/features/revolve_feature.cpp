// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/revolve_feature.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include <nlohmann/json.hpp>

#include "core/expr.hpp"
#include "features/feature_json.hpp"
#include "features/sketch_json.hpp"
#include "kernel/internal/profile_support.hpp"
#include "kernel/ops.hpp"
#include "naming/naming_map.hpp"
#include "sketch/profile.hpp"

namespace iw::features {
namespace {

using json = nlohmann::json;
using common::BodyOp;
using recompute::Diagnostic;
using recompute::EvalContext;
using recompute::FeatureResult;
using naming::ElementType;

const char* extentName(RevolveParams::Extent e) {
    switch (e) {
    case RevolveParams::Extent::OneSide: return "one_side";
    case RevolveParams::Extent::TwoSides: return "two_sides";
    case RevolveParams::Extent::Symmetric: return "symmetric";
    case RevolveParams::Extent::Full: return "full";
    }
    return "full";
}

Result<RevolveParams::Extent> extentFrom(const std::string& s) {
    if (s == "one_side") return RevolveParams::Extent::OneSide;
    if (s == "two_sides") return RevolveParams::Extent::TwoSides;
    if (s == "symmetric") return RevolveParams::Extent::Symmetric;
    if (s == "full") return RevolveParams::Extent::Full;
    return makeError(ErrorCode::ParseError, "revolve: unknown extent '" + s + "'");
}

const char* axisKindName(RevolveAxis::Kind k) {
    switch (k) {
    case RevolveAxis::Kind::SketchLine: return "sketch_line";
    case RevolveAxis::Kind::Construction: return "construction";
    case RevolveAxis::Kind::Edge: return "edge";
    }
    return "sketch_line";
}

/// Sketch expression references (dimensions) plus the given texts.
std::vector<std::string> referencesFor(const std::vector<const std::string*>& texts, const sketch::Sketch* sk) {
    std::vector<std::string> out;
    auto add = [&](const std::string& text) {
        auto ex = Expression::parse(text);
        if (ex) for (auto& r : ex->references()) out.push_back(r);
    };
    for (const std::string* t : texts) add(*t);
    if (sk)
        for (const auto& [id, dim] : sk->dimensions()) add(dim.expression);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

/// Profile source JSON, the same layout as ExtrudeFeature (P0 polygon form has no "source" key).
void sourceToJson(json& j, const std::variant<PolygonSource, SketchSource, SketchReferenceSource>& src) {
    auto v3 = [](const Vec3& v) { return json::array({v.x, v.y, v.z}); };
    if (const auto* poly = std::get_if<PolygonSource>(&src)) {
        j["source"] = "polygon";
        j["origin"] = v3(poly->planeOrigin);
        j["normal"] = v3(poly->planeNormal);
        j["xdir"] = v3(poly->planeXDir);
        json pts = json::array();
        for (const auto& p : poly->points) pts.push_back(json::array({p[0], p[1]}));
        j["points"] = std::move(pts);
    } else if (const auto* ref = std::get_if<SketchReferenceSource>(&src)) {
        j["source"] = "sketch_ref";
        j["sketch_feature"] = ref->sketch.value();
        j["profile"] = ref->profile;
    } else {
        const auto& sk = std::get<SketchSource>(src);
        j["source"] = "sketch";
        auto sj = sketchToJson(sk.sketch);
        j["sketch"] = sj ? json::parse(*sj, nullptr, false) : json::object();
        j["profile"] = sk.profile;
    }
}

Result<std::variant<PolygonSource, SketchSource, SketchReferenceSource>> sourceFromJson(const json& j) {
    auto srcIt = j.find("source");
    const std::string source = srcIt != j.end() && srcIt->is_string() ? srcIt->get<std::string>() : "polygon";
    if (source == "sketch_ref") {
        auto sf = j.find("sketch_feature");
        if (sf == j.end() || !sf->is_number_unsigned()) return makeError(ErrorCode::ParseError, "revolve: 'sketch_feature' must be an unsigned id");
        auto profile = common::readString(j, "profile");
        if (!profile) return std::unexpected(profile.error());
        return std::variant<PolygonSource, SketchSource, SketchReferenceSource>(
            SketchReferenceSource{FeatureId{sf->get<std::uint64_t>()}, *profile});
    }
    if (source == "sketch") {
        auto skIt = j.find("sketch");
        if (skIt == j.end()) return makeError(ErrorCode::ParseError, "revolve: sketch source without 'sketch'");
        auto sk = sketchFromJson(skIt->dump());
        if (!sk) return std::unexpected(sk.error());
        auto profile = common::readString(j, "profile");
        if (!profile) return std::unexpected(profile.error());
        return std::variant<PolygonSource, SketchSource, SketchReferenceSource>(SketchSource{std::move(*sk), *profile});
    }
    if (source != "polygon") return makeError(ErrorCode::ParseError, "revolve: unknown source '" + source + "'");
    PolygonSource poly;
    auto vec = [&](const char* key) -> Result<Vec3> {
        auto v = common::readVec(j, key);
        if (!v) return std::unexpected(v.error());
        return Vec3{(*v)[0], (*v)[1], (*v)[2]};
    };
    auto o = vec("origin"); if (!o) return std::unexpected(o.error());
    auto n = vec("normal"); if (!n) return std::unexpected(n.error());
    auto x = vec("xdir"); if (!x) return std::unexpected(x.error());
    poly.planeOrigin = *o;
    poly.planeNormal = *n;
    poly.planeXDir = *x;
    auto pts = j.find("points");
    if (pts == j.end() || !pts->is_array()) return makeError(ErrorCode::ParseError, "revolve: 'points' must be an array");
    for (const json& e : *pts) {
        if (!e.is_array() || e.size() != 2 || !e[0].is_number() || !e[1].is_number())
            return makeError(ErrorCode::ParseError, "revolve: each point must be [x, y]");
        poly.points.push_back({e[0].get<double>(), e[1].get<double>()});
    }
    return std::variant<PolygonSource, SketchSource, SketchReferenceSource>(std::move(poly));
}

/// Profile face and its naming, built exactly as ExtrudeFeature builds it, on the plane of the source.
struct Profile {
    kernel::Shape face;
    naming::NamingMap naming;
};

Result<Profile> buildProfile(FeatureId id, const std::variant<PolygonSource, SketchSource, SketchReferenceSource>& src, std::vector<Diagnostic>& diags) {
    if (const auto* poly = std::get_if<PolygonSource>(&src)) {
        if (poly->points.size() < 3) return makeError(ErrorCode::InvalidArgument, "revolve: profile needs >= 3 points");
        if (!common::finite(poly->planeOrigin) || !common::finite(poly->planeNormal) || !common::finite(poly->planeXDir))
            return makeError(ErrorCode::InvalidArgument, "revolve: non-finite plane");
        if (length(poly->planeNormal) < 1e-12 || length(poly->planeXDir) < 1e-12)
            return makeError(ErrorCode::InvalidArgument, "revolve: plane normal/xdir must be non-zero");
        const Vec3 n = common::normalized(poly->planeNormal);
        Vec3 xd = poly->planeXDir - n * dot(poly->planeXDir, n);
        if (length(xd) < 1e-9) return makeError(ErrorCode::InvalidArgument, "revolve: xdir is parallel to the normal");
        xd = common::normalized(xd);
        const Vec3 yd = cross(n, xd);
        std::vector<Vec3> pts;
        for (const auto& q : poly->points) pts.push_back(poly->planeOrigin + xd * q[0] + yd * q[1]);
        auto face = kernel::makePolygonFace(pts);
        if (!face) return std::unexpected(face.error());
        auto names = common::polygonProfileNaming(id, *face);
        if (!names) return std::unexpected(names.error());
        return Profile{*face, std::move(*names)};
    }
    if (std::holds_alternative<SketchReferenceSource>(src))
        return makeError(ErrorCode::NotFound, "revolve: sketch feature " + std::to_string(std::get<SketchReferenceSource>(src).sketch.value()) +
                                                  " has no solved profile at this position (suppressed, failed, or not earlier on the timeline)");
    const auto& sk = std::get<SketchSource>(src);
    auto profiles = sketch::detectProfiles(sk.sketch);
    if (!profiles) return std::unexpected(profiles.error());
    const sketch::SketchProfile* prof = nullptr;
    for (const auto& pr : *profiles)
        if (pr.name == sk.profile) prof = &pr;
    if (!prof) return makeError(ErrorCode::NotFound, "revolve: sketch has no profile '" + sk.profile + "'");
    auto face = sketch::profileFace(sk.sketch.plane(), *prof);
    if (!face) return std::unexpected(face.error());
    auto names = common::sketchProfileNaming(id, *face, *prof, sk.sketch.plane(), diags);
    if (!names) return std::unexpected(names.error());
    return Profile{*face, std::move(*names)};
}

} // namespace

std::vector<naming::ElementRef> RevolveFeature::explicitRefs() const {
    if (params_.axis.kind == RevolveAxis::Kind::Edge) return {params_.axis.edge};
    return {};
}

std::vector<std::string> RevolveFeature::parameterReferences() const {
    const sketch::Sketch* sk = nullptr;
    if (const auto* s = std::get_if<SketchSource>(&params_.source)) sk = &s->sketch;
    return referencesFor({&params_.angle, &params_.angle2}, sk);
}

std::vector<BodyId> RevolveFeature::readsBodies() const {
    if (params_.operation != BodyOp::NewBody || params_.axis.kind == RevolveAxis::Kind::Edge) return {params_.body};
    return {};
}

std::uint64_t RevolveFeature::paramHash() const {
    common::Fnv f;
    f.bytes("revolve/v1:");
    f.bytes(paramsJson());
    return f.h;
}

std::string RevolveFeature::paramsJson() const {
    json j = json::object();
    sourceToJson(j, params_.source);
    json ax = json::object();
    ax["kind"] = axisKindName(params_.axis.kind);
    switch (params_.axis.kind) {
    case RevolveAxis::Kind::SketchLine: ax["entity"] = params_.axis.line.value(); break;
    case RevolveAxis::Kind::Construction: ax["feature"] = params_.axis.construction.value(); break;
    case RevolveAxis::Kind::Edge: ax["edge"] = common::refToJson(params_.axis.edge); break;
    }
    j["axis"] = std::move(ax);
    j["extent"] = extentName(params_.extent);
    j["angle"] = params_.angle;
    j["angle2"] = params_.angle2;
    j["operation"] = common::bodyOpName(params_.operation);
    j["body"] = params_.body.value();
    return j.dump();
}

Result<std::shared_ptr<recompute::Feature>> RevolveFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "revolve: params are not a JSON object");
    RevolveParams p;
    auto src = sourceFromJson(j);
    if (!src) return std::unexpected(src.error());
    p.source = std::move(*src);

    auto axIt = j.find("axis");
    if (axIt == j.end() || !axIt->is_object()) return makeError(ErrorCode::ParseError, "revolve: missing 'axis'");
    auto kind = common::readString(*axIt, "kind");
    if (!kind) return std::unexpected(kind.error());
    if (*kind == "sketch_line") {
        p.axis.kind = RevolveAxis::Kind::SketchLine;
        if (!axIt->contains("entity") || !(*axIt)["entity"].is_number_unsigned()) return makeError(ErrorCode::ParseError, "revolve: axis 'entity' missing");
        p.axis.line = sketch::SketchEntityId{(*axIt)["entity"].get<std::uint64_t>()};
    } else if (*kind == "construction") {
        p.axis.kind = RevolveAxis::Kind::Construction;
        if (!axIt->contains("feature") || !(*axIt)["feature"].is_number_unsigned()) return makeError(ErrorCode::ParseError, "revolve: axis 'feature' missing");
        p.axis.construction = FeatureId{(*axIt)["feature"].get<std::uint64_t>()};
    } else if (*kind == "edge") {
        p.axis.kind = RevolveAxis::Kind::Edge;
        auto ref = common::refFromJson(*axIt, "edge");
        if (!ref) return std::unexpected(ref.error());
        p.axis.edge = *ref;
    } else {
        return makeError(ErrorCode::ParseError, "revolve: unknown axis kind '" + *kind + "'");
    }

    auto ext = common::readString(j, "extent", std::string("full"));
    if (!ext) return std::unexpected(ext.error());
    auto e = extentFrom(*ext);
    if (!e) return std::unexpected(e.error());
    p.extent = *e;
    auto a1 = common::readString(j, "angle", std::string("360 deg"));
    if (!a1) return std::unexpected(a1.error());
    auto a2 = common::readString(j, "angle2", std::string("0 deg"));
    if (!a2) return std::unexpected(a2.error());
    for (const auto& [text2, key] : {std::pair{*a1, "angle"}, std::pair{*a2, "angle2"}})
        if (auto c = common::checkExpression(text2, key); !c) return std::unexpected(c.error());
    p.angle = *a1;
    p.angle2 = *a2;

    auto op = common::readString(j, "operation");
    if (!op) return std::unexpected(op.error());
    auto operation = common::bodyOpFrom(*op);
    if (!operation) return std::unexpected(operation.error());
    p.operation = *operation;
    auto body = j.find("body");
    if (body == j.end() || !body->is_number_unsigned()) return makeError(ErrorCode::ParseError, "revolve: 'body' must be an unsigned id");
    p.body = BodyId{body->get<std::uint64_t>()};
    return std::shared_ptr<recompute::Feature>(std::make_shared<RevolveFeature>(id, std::move(p)));
}

Result<FeatureResult> RevolveFeature::compute(const EvalContext& ctx) const {
    const RevolveParams& p = params_;
    if (!p.body.valid()) return makeError(ErrorCode::InvalidArgument, "revolve: invalid body id");
    if (std::holds_alternative<SketchReferenceSource>(p.source))
        return makeError(ErrorCode::NotFound, "revolve: sketch feature " + std::to_string(std::get<SketchReferenceSource>(p.source).sketch.value()) +
                                                  " has no solved profile at this position (suppressed, failed, or not earlier on the timeline)");
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "revolve cancelled");

    // Angles: total sweep and the rotation that places the profile at the start of the sweep.
    double total = 2.0 * std::numbers::pi, offset = 0.0;
    if (p.extent != RevolveParams::Extent::Full) {
        auto a1 = common::angleOf(ctx.params, p.angle, "angle");
        if (!a1) return std::unexpected(a1.error());
        switch (p.extent) {
        case RevolveParams::Extent::OneSide: total = *a1; break;
        case RevolveParams::Extent::Symmetric: total = *a1; offset = -*a1 * 0.5; break;
        case RevolveParams::Extent::TwoSides: {
            auto a2 = common::angleOf(ctx.params, p.angle2, "angle2");
            if (!a2) return std::unexpected(a2.error());
            total = *a1 + *a2;
            offset = -*a2;
            break;
        }
        case RevolveParams::Extent::Full: break;
        }
    }
    if (!(total > 1e-9) || total > 2.0 * std::numbers::pi + 1e-9)
        return makeError(ErrorCode::InvalidArgument, "revolve: total angle must be in (0, 360 deg]");
    total = std::min(total, 2.0 * std::numbers::pi);

    // Axis in world coordinates.
    std::vector<Diagnostic> diags;
    kernel::Axis axis;
    switch (p.axis.kind) {
    case RevolveAxis::Kind::SketchLine: {
        const auto* sk = std::get_if<SketchSource>(&p.source);
        if (!sk) return makeError(ErrorCode::InvalidArgument, "revolve: a sketch line axis needs a sketch profile");
        const sketch::SketchEntity* ent = sk->sketch.findEntity(p.axis.line);
        if (!ent) return makeError(ErrorCode::NotFound, "revolve: axis sketch line does not exist");
        const auto* line = std::get_if<sketch::LineGeom>(&ent->geometry);
        if (!line) return makeError(ErrorCode::InvalidArgument, "revolve: axis entity is not a line");
        auto a = kernel::axisFromPoints(sk->sketch.plane().toWorld(line->start), sk->sketch.plane().toWorld(line->end));
        if (!a) return std::unexpected(a.error());
        axis = *a;
        break;
    }
    case RevolveAxis::Kind::Construction: {
        const recompute::ConstructionGeometry* g = ctx.construction(p.axis.construction);
        if (!g || g->kind != recompute::ConstructionGeometry::Kind::Axis)
            return makeError(ErrorCode::NotFound, "revolve: axis feature " + std::to_string(p.axis.construction.value()) + " is not an axis at this position");
        axis = kernel::Axis{g->origin, g->direction};
        break;
    }
    case RevolveAxis::Kind::Edge: {
        auto idx = common::resolveRef(ctx, p.body, p.axis.edge, ElementType::Edge, diags);
        if (!idx) return std::unexpected(idx.error());
        auto a = kernel::axisFromEdge(*ctx.body(p.body), *idx);
        if (!a) return std::unexpected(a.error());
        axis = kernel::Axis{a->origin, a->direction};
        break;
    }
    }

    auto profile = buildProfile(id_, p.source, diags);
    if (!profile) return std::unexpected(profile.error());
    kernel::Shape face = profile->face;
    if (std::abs(offset) > 1e-15) {
        auto moved = kernel::transform(face, common::rotationAbout(axis, offset), ctx.cancel);
        if (!moved) return std::unexpected(moved.error());
        face = moved->shape;
    }
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "revolve cancelled");
    auto tool = kernel::revolve(face, axis, total, ctx.cancel);
    if (!tool) return std::unexpected(tool.error());
    return common::applyBodyOp(id_, p.operation, p.body, ctx, *tool, profile->naming, "revolve", std::move(diags));
}

void registerRevolveFeature(recompute::FeatureRegistry& registry) {
    registry.add("revolve", [](FeatureId id, std::string_view params) { return RevolveFeature::fromJson(id, params); });
}

} // namespace iw::features
