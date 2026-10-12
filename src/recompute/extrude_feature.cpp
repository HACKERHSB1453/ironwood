// SPDX-License-Identifier: GPL-3.0-or-later
#include "recompute/extrude_feature.hpp"

#include <bit>
#include <cmath>

#include <nlohmann/json.hpp>

#include "kernel/internal/profile_support.hpp"
#include "kernel/ops.hpp"
#include "naming/naming_map.hpp"

namespace iw::recompute {
namespace {

using json = nlohmann::json;

struct Fnv {
    std::uint64_t h = 0xcbf29ce484222325ULL;
    void u64(std::uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            h ^= (v >> (8 * i)) & 0xffU;
            h *= 0x100000001b3ULL;
        }
    }
    void f64(double d) { u64(std::bit_cast<std::uint64_t>(d + 0.0)); } // +0.0 folds -0.0 into 0.0
    void vec(const Vec3& v) { f64(v.x); f64(v.y); f64(v.z); }
};

Vec3 normalized(const Vec3& v) { return v * (1.0 / length(v)); }

bool finite(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

const char* opName(ExtrudeParams::Operation o) { return o == ExtrudeParams::Operation::Join ? "join" : "new_body"; }

/// Gives every element of a profile shape a stable base name (the profile has no earlier history).
Result<naming::NamingMap> profileNaming(FeatureId feature, const kernel::Shape& profile) {
    naming::NamingMap map;
    const auto nf = profile.subShapeCount(kernel::SubShapeKind::Face);
    const auto ne = profile.subShapeCount(kernel::SubShapeKind::Edge);
    const auto nv = profile.subShapeCount(kernel::SubShapeKind::Vertex);
    map.resize(nf, ne, nv);
    auto fill = [&](naming::ElementType t, std::uint32_t n, const char* op) -> Result<void> {
        for (std::uint32_t i = 0; i < n; ++i) {
            naming::ElementName name;
            name.feature = feature;
            name.op = op;
            name.type = t;
            name.ordinal = i;
            if (auto r = map.set(t, i, std::move(name)); !r) return std::unexpected(r.error());
        }
        return {};
    };
    if (auto r = fill(naming::ElementType::Face, nf, "profile.face"); !r) return std::unexpected(r.error());
    if (auto r = fill(naming::ElementType::Edge, ne, "profile.edge"); !r) return std::unexpected(r.error());
    if (auto r = fill(naming::ElementType::Vertex, nv, "profile.vertex"); !r) return std::unexpected(r.error());
    return map;
}

Result<std::array<double, 3>> readVec(const json& obj, const char* key) {
    auto it = obj.find(key);
    if (it == obj.end() || !it->is_array() || it->size() != 3)
        return makeError(ErrorCode::ParseError, std::string("extrude: '") + key + "' must be a 3-number array");
    std::array<double, 3> out{};
    for (std::size_t i = 0; i < 3; ++i) {
        if (!(*it)[i].is_number()) return makeError(ErrorCode::ParseError, std::string("extrude: '") + key + "' has a non-number");
        out[i] = (*it)[i].get<double>();
    }
    return out;
}

} // namespace

std::vector<BodyId> ExtrudeFeature::readsBodies() const {
    if (params_.operation == ExtrudeParams::Operation::Join) return {params_.body};
    return {};
}

std::uint64_t ExtrudeFeature::paramHash() const {
    Fnv f;
    f.u64(0x6578747275646531ULL); // "extrude1"
    f.vec(params_.planeOrigin);
    f.vec(params_.planeNormal);
    f.vec(params_.planeXDir);
    f.u64(params_.points.size());
    for (const auto& p : params_.points) { f.f64(p[0]); f.f64(p[1]); }
    f.vec(params_.vector);
    f.u64(static_cast<std::uint64_t>(params_.operation));
    f.u64(params_.body.value());
    return f.h;
}

std::string ExtrudeFeature::paramsJson() const {
    json j = json::object();
    auto v3 = [](const Vec3& v) { return json::array({v.x, v.y, v.z}); };
    j["origin"] = v3(params_.planeOrigin);
    j["normal"] = v3(params_.planeNormal);
    j["xdir"] = v3(params_.planeXDir);
    json pts = json::array();
    for (const auto& p : params_.points) pts.push_back(json::array({p[0], p[1]}));
    j["points"] = std::move(pts);
    j["vector"] = v3(params_.vector);
    j["operation"] = opName(params_.operation);
    j["body"] = params_.body.value();
    return j.dump();
}

Result<std::shared_ptr<Feature>> ExtrudeFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "extrude: params are not a JSON object");
    ExtrudeParams p;
    auto grab = [&](const char* key, Vec3& out) -> Result<void> {
        auto v = readVec(j, key);
        if (!v) return std::unexpected(v.error());
        out = {(*v)[0], (*v)[1], (*v)[2]};
        return {};
    };
    if (auto r = grab("origin", p.planeOrigin); !r) return std::unexpected(r.error());
    if (auto r = grab("normal", p.planeNormal); !r) return std::unexpected(r.error());
    if (auto r = grab("xdir", p.planeXDir); !r) return std::unexpected(r.error());
    if (auto r = grab("vector", p.vector); !r) return std::unexpected(r.error());
    auto pts = j.find("points");
    if (pts == j.end() || !pts->is_array()) return makeError(ErrorCode::ParseError, "extrude: 'points' must be an array");
    for (const json& e : *pts) {
        if (!e.is_array() || e.size() != 2 || !e[0].is_number() || !e[1].is_number())
            return makeError(ErrorCode::ParseError, "extrude: each point must be [x, y]");
        p.points.push_back({e[0].get<double>(), e[1].get<double>()});
    }
    auto op = j.find("operation");
    if (op == j.end() || !op->is_string()) return makeError(ErrorCode::ParseError, "extrude: missing 'operation'");
    const std::string opText = op->get<std::string>();
    if (opText == "new_body") p.operation = ExtrudeParams::Operation::NewBody;
    else if (opText == "join") p.operation = ExtrudeParams::Operation::Join;
    else return makeError(ErrorCode::ParseError, "extrude: unknown operation '" + opText + "'");
    auto body = j.find("body");
    if (body == j.end() || !body->is_number_unsigned()) return makeError(ErrorCode::ParseError, "extrude: 'body' must be an unsigned id");
    p.body = BodyId{body->get<std::uint64_t>()};
    return std::shared_ptr<Feature>(std::make_shared<ExtrudeFeature>(id, std::move(p)));
}

Result<FeatureResult> ExtrudeFeature::compute(const EvalContext& ctx) const {
    const ExtrudeParams& p = params_;
    if (!p.body.valid()) return makeError(ErrorCode::InvalidArgument, "extrude: invalid body id");
    if (p.points.size() < 3) return makeError(ErrorCode::InvalidArgument, "extrude: profile needs >= 3 points");
    if (!finite(p.planeOrigin) || !finite(p.planeNormal) || !finite(p.planeXDir) || !finite(p.vector))
        return makeError(ErrorCode::InvalidArgument, "extrude: non-finite parameter");
    if (length(p.planeNormal) < 1e-12 || length(p.planeXDir) < 1e-12)
        return makeError(ErrorCode::InvalidArgument, "extrude: plane normal/xdir must be non-zero");
    const Vec3 n = normalized(p.planeNormal);
    Vec3 xd = p.planeXDir - n * dot(p.planeXDir, n);
    if (length(xd) < 1e-9) return makeError(ErrorCode::InvalidArgument, "extrude: xdir is parallel to the normal");
    xd = normalized(xd);
    const Vec3 yd = cross(n, xd);

    std::vector<Vec3> pts;
    pts.reserve(p.points.size());
    for (const auto& q : p.points) pts.push_back(p.planeOrigin + xd * q[0] + yd * q[1]);

    const bool join = p.operation == ExtrudeParams::Operation::Join;
    const kernel::Shape* target = nullptr;
    const naming::NamingMap* targetNaming = nullptr;
    if (join) {
        target = ctx.body(p.body);
        if (!target) return makeError(ErrorCode::NotFound, "extrude: join target body does not exist at this position");
        targetNaming = ctx.naming(p.body);
        if (!targetNaming) return makeError(ErrorCode::Internal, "extrude: join target has no naming map");
    }
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "extrude cancelled");

    auto face = kernel::makePolygonFace(pts);
    if (!face) return std::unexpected(face.error());
    auto pnaming = profileNaming(id_, *face);
    if (!pnaming) return std::unexpected(pnaming.error());
    auto ext = kernel::extrudeProfile(*face, p.vector, ctx.cancel);
    if (!ext) return std::unexpected(ext.error());
    auto extNaming = naming::deriveNamingMap(id_, "extrude", ext->shape, {&*pnaming}, ext->history);
    if (!extNaming) return std::unexpected(extNaming.error());

    FeatureResult out;
    if (!join) {
        out.bodies.emplace(p.body, std::move(ext->shape));
        out.namings.emplace(p.body, std::move(*extNaming));
        return out;
    }
    auto fused = kernel::fuse(*target, ext->shape, ctx.cancel);
    if (!fused) return std::unexpected(fused.error());
    auto fusedNaming = naming::deriveNamingMap(id_, "fuse", fused->shape, {targetNaming, &*extNaming}, fused->history);
    if (!fusedNaming) return std::unexpected(fusedNaming.error());
    out.bodies.emplace(p.body, std::move(fused->shape));
    out.namings.emplace(p.body, std::move(*fusedNaming));
    return out;
}

void registerExtrudeFeature(FeatureRegistry& registry) {
    registry.add("extrude", [](FeatureId id, std::string_view params) { return ExtrudeFeature::fromJson(id, params); });
}

} // namespace iw::recompute
