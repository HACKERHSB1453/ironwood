// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/construction_features.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
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
using recompute::ConstructionGeometry;
using recompute::Diagnostic;
using recompute::EvalContext;
using recompute::FeatureResult;

Result<std::array<std::string, 3>> triple(const json& j, const char* key) {
    auto it = j.find(key);
    if (it == j.end() || !it->is_array() || it->size() != 3) return makeError(ErrorCode::ParseError, std::string("'") + key + "' must be 3 expressions");
    std::array<std::string, 3> out;
    for (std::size_t i = 0; i < 3; ++i) {
        if (!(*it)[i].is_string()) return makeError(ErrorCode::ParseError, std::string("'") + key + "' entries must be strings");
        out[i] = (*it)[i].get<std::string>();
        if (auto c = common::checkExpression(out[i], key); !c) return std::unexpected(c.error());
    }
    return out;
}

json triples(const std::array<std::string, 3>& t) { return json::array({t[0], t[1], t[2]}); }

/// Evaluates a triple of length expressions to a point (mm).
Result<Vec3> pointOf(const EvalContext& ctx, const std::array<std::string, 3>& t, const char* what) {
    Vec3 p;
    double* comps[] = {&p.x, &p.y, &p.z};
    for (std::size_t i = 0; i < 3; ++i) {
        auto v = common::lengthOf(ctx.params, t[i], what);
        if (!v) return std::unexpected(v.error());
        *comps[i] = *v;
    }
    return p;
}

/// Rodrigues rotation of a direction vector (no translation).
Vec3 rotateVector(const Vec3& v, const Vec3& unitAxis, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return v * c + cross(unitAxis, v) * s + unitAxis * (dot(unitAxis, v) * (1.0 - c));
}

json planeRefToJson(const PlaneRef& r) {
    if (r.isFace) return json{{"body", r.body.value()}, {"face", common::refToJson(r.face)}};
    return json{{"feature", r.construction.value()}};
}

Result<PlaneRef> planeRefFromJson(const json& j, const char* key) {
    auto it = j.find(key);
    if (it == j.end() || !it->is_object()) return makeError(ErrorCode::ParseError, std::string("missing plane reference '") + key + "'");
    PlaneRef r;
    if (it->contains("feature")) {
        if (!(*it)["feature"].is_number_unsigned()) return makeError(ErrorCode::ParseError, "plane feature id must be unsigned");
        r.isFace = false;
        r.construction = FeatureId{(*it)["feature"].get<std::uint64_t>()};
        return r;
    }
    if (!it->contains("body") || !(*it)["body"].is_number_unsigned()) return makeError(ErrorCode::ParseError, "plane face needs 'body'");
    r.body = BodyId{(*it)["body"].get<std::uint64_t>()};
    auto face = common::refFromJson(*it, "face");
    if (!face) return std::unexpected(face.error());
    if (face->type != ElementType::Face) return makeError(ErrorCode::ParseError, "plane reference is not a face");
    r.face = *face;
    return r;
}

/// Plane of a PlaneRef at this timeline position.
Result<kernel::Plane> resolvePlane(const EvalContext& ctx, const PlaneRef& r, std::vector<Diagnostic>& diags) {
    if (!r.isFace) {
        const ConstructionGeometry* g = ctx.construction(r.construction);
        if (!g || g->kind != ConstructionGeometry::Kind::Plane)
            return makeError(ErrorCode::NotFound, "plane feature " + std::to_string(r.construction.value()) + " is not a plane at this position");
        return kernel::Plane{g->origin, g->normal, g->xDir};
    }
    auto idx = resolveRef(ctx, r.body, r.face, ElementType::Face, diags);
    if (!idx) return std::unexpected(idx.error());
    auto plane = kernel::planeFromFace(*ctx.body(r.body), *idx);
    if (!plane) return makeError(ErrorCode::InvalidArgument, "construction plane: the referenced face is not planar");
    return *plane;
}

std::uint64_t hashJson(std::string_view tag, const std::string& text) {
    common::Fnv f;
    f.bytes(tag);
    f.bytes(text);
    return f.h;
}

FeatureResult geometryOnly(ConstructionGeometry g, std::vector<Diagnostic> diags) {
    FeatureResult out;
    out.constructionGeometry = g;
    out.diagnostics = std::move(diags);
    return out;
}

} // namespace

// ---- plane -----------------------------------------------------------------------------------------------------

std::vector<naming::ElementRef> ConstructionPlaneFeature::explicitRefs() const {
    std::vector<naming::ElementRef> out;
    for (const PlaneRef* r : {&params_.base, &params_.other})
        if (r->isFace) out.push_back(r->face);
    return out;
}

std::vector<BodyId> ConstructionPlaneFeature::readsBodies() const {
    std::vector<BodyId> out;
    for (const PlaneRef* r : {&params_.base, &params_.other})
        if (r->isFace) out.push_back(r->body);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::vector<std::string> ConstructionPlaneFeature::parameterReferences() const {
    std::vector<std::string> out;
    std::vector<std::string> texts{params_.distance, params_.angle};
    for (const auto& row : params_.points) texts.insert(texts.end(), row.begin(), row.end());
    for (const std::string& t : texts) {
        auto ex = Expression::parse(t);
        if (ex) for (auto& r : ex->references()) out.push_back(r);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::string ConstructionPlaneFeature::paramsJson() const {
    const auto& p = params_;
    json j{{"id", id_.value()}};
    switch (p.kind) {
    case ConstructionPlaneParams::Kind::Offset:
        j["kind"] = "offset";
        j["base"] = planeRefToJson(p.base);
        j["distance"] = p.distance;
        break;
    case ConstructionPlaneParams::Kind::MidPlane:
        j["kind"] = "midplane";
        j["base"] = planeRefToJson(p.base);
        j["other"] = planeRefToJson(p.other);
        break;
    case ConstructionPlaneParams::Kind::ThreePoints:
        j["kind"] = "three_points";
        j["points"] = json::array({triples(p.points[0]), triples(p.points[1]), triples(p.points[2])});
        break;
    case ConstructionPlaneParams::Kind::AngleAxis:
        j["kind"] = "angle_axis";
        j["base"] = planeRefToJson(p.base);
        j["axis"] = p.axis.value();
        j["angle"] = p.angle;
        break;
    }
    return j.dump();
}

std::uint64_t ConstructionPlaneFeature::paramHash() const { return hashJson("plane/v1:", paramsJson()); }

Result<std::shared_ptr<recompute::Feature>> ConstructionPlaneFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "plane: params are not a JSON object");
    ConstructionPlaneParams p;
    auto kind = common::readString(j, "kind");
    if (!kind) return std::unexpected(kind.error());
    if (*kind == "offset") {
        p.kind = ConstructionPlaneParams::Kind::Offset;
        auto base = planeRefFromJson(j, "base");
        if (!base) return std::unexpected(base.error());
        p.base = *base;
        auto d = common::readString(j, "distance");
        if (!d) return std::unexpected(d.error());
        if (auto c = common::checkExpression(*d, "distance"); !c) return std::unexpected(c.error());
        p.distance = *d;
    } else if (*kind == "midplane") {
        p.kind = ConstructionPlaneParams::Kind::MidPlane;
        auto a = planeRefFromJson(j, "base");
        if (!a) return std::unexpected(a.error());
        auto b = planeRefFromJson(j, "other");
        if (!b) return std::unexpected(b.error());
        p.base = *a;
        p.other = *b;
    } else if (*kind == "three_points") {
        p.kind = ConstructionPlaneParams::Kind::ThreePoints;
        auto pts = j.find("points");
        if (pts == j.end() || !pts->is_array() || pts->size() != 3) return makeError(ErrorCode::ParseError, "plane: 'points' needs 3 points");
        for (std::size_t i = 0; i < 3; ++i) {
            auto t = triple((*pts)[i], "point");
            if (!t) return std::unexpected(t.error());
            p.points[i] = *t;
        }
    } else if (*kind == "angle_axis") {
        p.kind = ConstructionPlaneParams::Kind::AngleAxis;
        auto base = planeRefFromJson(j, "base");
        if (!base) return std::unexpected(base.error());
        p.base = *base;
        if (!j.contains("axis") || !j["axis"].is_number_unsigned()) return makeError(ErrorCode::ParseError, "plane: 'axis' must be a feature id");
        p.axis = FeatureId{j["axis"].get<std::uint64_t>()};
        auto a = common::readString(j, "angle");
        if (!a) return std::unexpected(a.error());
        if (auto c = common::checkExpression(*a, "angle"); !c) return std::unexpected(c.error());
        p.angle = *a;
    } else {
        return makeError(ErrorCode::ParseError, "plane: unknown kind '" + *kind + "'");
    }
    return std::shared_ptr<recompute::Feature>(std::make_shared<ConstructionPlaneFeature>(id, std::move(p)));
}

Result<FeatureResult> ConstructionPlaneFeature::compute(const EvalContext& ctx) const {
    const auto& p = params_;
    std::vector<Diagnostic> diags;
    ConstructionGeometry g;
    g.kind = ConstructionGeometry::Kind::Plane;
    switch (p.kind) {
    case ConstructionPlaneParams::Kind::Offset: {
        auto base = resolvePlane(ctx, p.base, diags);
        if (!base) return std::unexpected(base.error());
        auto d = common::lengthOf(ctx.params, p.distance, "plane offset");
        if (!d) return std::unexpected(d.error());
        g.origin = base->origin + base->normal * *d;
        g.normal = base->normal;
        g.xDir = base->xDir;
        break;
    }
    case ConstructionPlaneParams::Kind::MidPlane: {
        auto a = resolvePlane(ctx, p.base, diags);
        if (!a) return std::unexpected(a.error());
        auto b = resolvePlane(ctx, p.other, diags);
        if (!b) return std::unexpected(b.error());
        if (std::abs(std::abs(dot(a->normal, b->normal)) - 1.0) > 1e-9)
            return makeError(ErrorCode::InvalidArgument, "midplane: the two planes are not parallel");
        const double gap = dot(b->origin - a->origin, a->normal) * 0.5;
        g.origin = a->origin + a->normal * gap;
        g.normal = a->normal;
        g.xDir = a->xDir;
        break;
    }
    case ConstructionPlaneParams::Kind::ThreePoints: {
        Vec3 pts[3];
        for (std::size_t i = 0; i < 3; ++i) {
            auto v = pointOf(ctx, p.points[i], "plane point");
            if (!v) return std::unexpected(v.error());
            pts[i] = *v;
        }
        auto plane = kernel::planeFromThreePoints(pts[0], pts[1], pts[2]);
        if (!plane) return std::unexpected(plane.error());
        g.origin = plane->origin;
        g.normal = plane->normal;
        g.xDir = plane->xDir;
        break;
    }
    case ConstructionPlaneParams::Kind::AngleAxis: {
        auto base = resolvePlane(ctx, p.base, diags);
        if (!base) return std::unexpected(base.error());
        const ConstructionGeometry* ax = ctx.construction(p.axis);
        if (!ax || ax->kind != ConstructionGeometry::Kind::Axis) return makeError(ErrorCode::NotFound, "plane: angle axis feature is not an axis at this position");
        auto angle = common::angleOf(ctx.params, p.angle, "plane angle");
        if (!angle) return std::unexpected(angle.error());
        const Vec3 k = common::normalized(ax->direction);
        g.origin = ax->origin;
        g.normal = common::normalized(rotateVector(base->normal, k, *angle));
        g.xDir = common::normalized(rotateVector(base->xDir, k, *angle));
        break;
    }
    }
    if (!common::finite(g.origin) || !common::finite(g.normal)) return makeError(ErrorCode::KernelFailure, "plane: non-finite result");
    return geometryOnly(g, std::move(diags));
}

// ---- axis ------------------------------------------------------------------------------------------------------

std::vector<naming::ElementRef> ConstructionAxisFeature::explicitRefs() const {
    if (params_.kind == ConstructionAxisParams::Kind::TwoPoints) return {};
    return {params_.ref};
}

std::vector<BodyId> ConstructionAxisFeature::readsBodies() const {
    if (params_.kind == ConstructionAxisParams::Kind::TwoPoints) return {};
    return {params_.body};
}

std::vector<std::string> ConstructionAxisFeature::parameterReferences() const {
    std::vector<std::string> out;
    for (const auto* arr : {&params_.a, &params_.b})
        for (const std::string& t : *arr) {
            auto ex = Expression::parse(t);
            if (ex) for (auto& r : ex->references()) out.push_back(r);
        }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::string ConstructionAxisFeature::paramsJson() const {
    const auto& p = params_;
    json j{{"id", id_.value()}};
    switch (p.kind) {
    case ConstructionAxisParams::Kind::Cylinder: j["kind"] = "cylinder"; break;
    case ConstructionAxisParams::Kind::TwoPoints: j["kind"] = "two_points"; break;
    case ConstructionAxisParams::Kind::Edge: j["kind"] = "edge"; break;
    }
    j["body"] = p.body.value();
    if (p.kind != ConstructionAxisParams::Kind::TwoPoints) j["ref"] = common::refToJson(p.ref);
    if (p.kind == ConstructionAxisParams::Kind::TwoPoints) {
        j["a"] = triples(p.a);
        j["b"] = triples(p.b);
    }
    return j.dump();
}

std::uint64_t ConstructionAxisFeature::paramHash() const { return hashJson("axis/v1:", paramsJson()); }

Result<std::shared_ptr<recompute::Feature>> ConstructionAxisFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "axis: params are not a JSON object");
    ConstructionAxisParams p;
    auto kind = common::readString(j, "kind");
    if (!kind) return std::unexpected(kind.error());
    if (*kind == "two_points") {
        p.kind = ConstructionAxisParams::Kind::TwoPoints;
        auto a = triple(j, "a");
        if (!a) return std::unexpected(a.error());
        auto b = triple(j, "b");
        if (!b) return std::unexpected(b.error());
        p.a = *a;
        p.b = *b;
    } else if (*kind == "cylinder" || *kind == "edge") {
        p.kind = *kind == "cylinder" ? ConstructionAxisParams::Kind::Cylinder : ConstructionAxisParams::Kind::Edge;
        auto ref = common::refFromJson(j, "ref");
        if (!ref) return std::unexpected(ref.error());
        const ElementType want = p.kind == ConstructionAxisParams::Kind::Edge ? ElementType::Edge : ElementType::Face;
        if (ref->type != want) return makeError(ErrorCode::ParseError, "axis: reference has the wrong element type");
        p.ref = *ref;
    } else {
        return makeError(ErrorCode::ParseError, "axis: unknown kind '" + *kind + "'");
    }
    if (p.kind != ConstructionAxisParams::Kind::TwoPoints) {
        if (!j.contains("body") || !j["body"].is_number_unsigned()) return makeError(ErrorCode::ParseError, "axis: 'body' must be an unsigned id");
        p.body = BodyId{j["body"].get<std::uint64_t>()};
    }
    return std::shared_ptr<recompute::Feature>(std::make_shared<ConstructionAxisFeature>(id, std::move(p)));
}

Result<FeatureResult> ConstructionAxisFeature::compute(const EvalContext& ctx) const {
    const auto& p = params_;
    std::vector<Diagnostic> diags;
    ConstructionGeometry g;
    g.kind = ConstructionGeometry::Kind::Axis;
    switch (p.kind) {
    case ConstructionAxisParams::Kind::TwoPoints: {
        auto a = pointOf(ctx, p.a, "axis point");
        if (!a) return std::unexpected(a.error());
        auto b = pointOf(ctx, p.b, "axis point");
        if (!b) return std::unexpected(b.error());
        auto axis = kernel::axisFromPoints(*a, *b);
        if (!axis) return std::unexpected(axis.error());
        g.origin = axis->origin;
        g.direction = axis->direction;
        break;
    }
    case ConstructionAxisParams::Kind::Cylinder: {
        auto idx = resolveRef(ctx, p.body, p.ref, ElementType::Face, diags);
        if (!idx) return std::unexpected(idx.error());
        const kernel::FaceGeometry fg = ctx.body(p.body)->faceGeometry(*idx);
        if (fg.kind != kernel::FaceGeometry::Kind::Cylinder && fg.kind != kernel::FaceGeometry::Kind::Cone)
            return makeError(ErrorCode::InvalidArgument, "axis: the referenced face is not cylindrical or conical");
        g.origin = fg.point;
        g.direction = fg.direction;
        break;
    }
    case ConstructionAxisParams::Kind::Edge: {
        auto idx = resolveRef(ctx, p.body, p.ref, ElementType::Edge, diags);
        if (!idx) return std::unexpected(idx.error());
        auto axis = kernel::axisFromEdge(*ctx.body(p.body), *idx);
        if (!axis) return std::unexpected(axis.error());
        g.origin = axis->origin;
        g.direction = axis->direction;
        break;
    }
    }
    if (!common::finite(g.origin) || !common::finite(g.direction)) return makeError(ErrorCode::KernelFailure, "axis: non-finite result");
    return geometryOnly(g, std::move(diags));
}

// ---- point -----------------------------------------------------------------------------------------------------

std::uint64_t ConstructionPointFeature::paramHash() const { return hashJson("point/v1:", paramsJson()); }

std::string ConstructionPointFeature::paramsJson() const {
    json j{{"id", id_.value()},
           {"kind", params_.kind == ConstructionPointParams::Kind::Vertex ? "vertex" : "circle_centre"},
           {"body", params_.body.value()},
           {"ref", common::refToJson(params_.ref)}};
    return j.dump();
}

Result<std::shared_ptr<recompute::Feature>> ConstructionPointFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "point: params are not a JSON object");
    ConstructionPointParams p;
    auto kind = common::readString(j, "kind");
    if (!kind) return std::unexpected(kind.error());
    if (*kind == "vertex") p.kind = ConstructionPointParams::Kind::Vertex;
    else if (*kind == "circle_centre") p.kind = ConstructionPointParams::Kind::CircleCentre;
    else return makeError(ErrorCode::ParseError, "point: unknown kind '" + *kind + "'");
    if (!j.contains("body") || !j["body"].is_number_unsigned()) return makeError(ErrorCode::ParseError, "point: 'body' must be an unsigned id");
    p.body = BodyId{j["body"].get<std::uint64_t>()};
    auto ref = common::refFromJson(j, "ref");
    if (!ref) return std::unexpected(ref.error());
    const ElementType want = p.kind == ConstructionPointParams::Kind::Vertex ? ElementType::Vertex : ElementType::Edge;
    if (ref->type != want) return makeError(ErrorCode::ParseError, "point: reference has the wrong element type");
    p.ref = *ref;
    return std::shared_ptr<recompute::Feature>(std::make_shared<ConstructionPointFeature>(id, std::move(p)));
}

Result<FeatureResult> ConstructionPointFeature::compute(const EvalContext& ctx) const {
    const auto& p = params_;
    if (!ctx.body(p.body)) return makeError(ErrorCode::NotFound, "point: body does not exist at this position");
    std::vector<Diagnostic> diags;
    ConstructionGeometry g;
    g.kind = ConstructionGeometry::Kind::Point;
    if (p.kind == ConstructionPointParams::Kind::Vertex) {
        auto idx = resolveRef(ctx, p.body, p.ref, ElementType::Vertex, diags);
        if (!idx) return std::unexpected(idx.error());
        const auto k = ctx.body(p.body)->subShapeKey(kernel::SubShapeKind::Vertex, *idx);
        g.origin = {k[0], k[1], k[2]};
    } else {
        auto idx = resolveRef(ctx, p.body, p.ref, ElementType::Edge, diags);
        if (!idx) return std::unexpected(idx.error());
        const auto k = ctx.body(p.body)->subShapeKey(kernel::SubShapeKind::Edge, *idx);
        g.origin = {k[0], k[1], k[2]};
    }
    return geometryOnly(g, std::move(diags));
}

void registerConstructionFeatures(recompute::FeatureRegistry& registry) {
    registry.add("plane", [](FeatureId id, std::string_view p) { return ConstructionPlaneFeature::fromJson(id, p); });
    registry.add("axis", [](FeatureId id, std::string_view p) { return ConstructionAxisFeature::fromJson(id, p); });
    registry.add("point", [](FeatureId id, std::string_view p) { return ConstructionPointFeature::fromJson(id, p); });
}

} // namespace iw::features
