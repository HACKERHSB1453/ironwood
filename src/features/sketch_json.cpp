// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/sketch_json.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

namespace iw::features {
namespace {

using json = nlohmann::json;
using sketch::ConstraintKind;
using sketch::DimensionKind;
using sketch::Point2;
using sketch::PointRole;
using sketch::Sketch;
using sketch::SketchRef;

Result<double> num(const json& o, const char* key) {
    auto it = o.find(key);
    if (it == o.end() || !it->is_number()) return makeError(ErrorCode::ParseError, std::string("sketch: '") + key + "' must be a number");
    return it->get<double>();
}

Result<Point2> pt(const json& o, const char* key) {
    auto it = o.find(key);
    if (it == o.end() || !it->is_array() || it->size() != 2 || !(*it)[0].is_number() || !(*it)[1].is_number())
        return makeError(ErrorCode::ParseError, std::string("sketch: '") + key + "' must be [x, y]");
    return Point2{(*it)[0].get<double>(), (*it)[1].get<double>()};
}

Result<const json*> arr(const json& o, const char* key) {
    auto it = o.find(key);
    if (it == o.end() || !it->is_array()) return makeError(ErrorCode::ParseError, std::string("sketch: '") + key + "' must be an array");
    return &*it;
}

json vec3(const Vec3& v) { return json::array({v.x, v.y, v.z}); }
json pt2(Point2 p) { return json::array({p.x, p.y}); }

const char* roleName(PointRole r) {
    switch (r) {
    case PointRole::Start: return "start";
    case PointRole::End: return "end";
    case PointRole::Center: return "center";
    case PointRole::Mid: return "mid";
    }
    return "start";
}

Result<PointRole> roleFrom(const std::string& s) {
    if (s == "start") return PointRole::Start;
    if (s == "end") return PointRole::End;
    if (s == "center") return PointRole::Center;
    if (s == "mid") return PointRole::Mid;
    return makeError(ErrorCode::ParseError, "sketch: unknown point role '" + s + "'");
}

const char* constraintName(ConstraintKind k) {
    switch (k) {
    case ConstraintKind::Coincident: return "coincident";
    case ConstraintKind::Horizontal: return "horizontal";
    case ConstraintKind::Vertical: return "vertical";
    case ConstraintKind::Parallel: return "parallel";
    case ConstraintKind::Perpendicular: return "perpendicular";
    case ConstraintKind::Tangent: return "tangent";
    case ConstraintKind::Equal: return "equal";
    case ConstraintKind::Concentric: return "concentric";
    case ConstraintKind::Midpoint: return "midpoint";
    case ConstraintKind::Collinear: return "collinear";
    case ConstraintKind::Symmetric: return "symmetric";
    case ConstraintKind::Fix: return "fix";
    }
    return "coincident";
}

Result<ConstraintKind> constraintFrom(const std::string& s) {
    static const ConstraintKind all[] = {ConstraintKind::Coincident, ConstraintKind::Horizontal, ConstraintKind::Vertical,
                                         ConstraintKind::Parallel,   ConstraintKind::Perpendicular, ConstraintKind::Tangent,
                                         ConstraintKind::Equal,      ConstraintKind::Concentric, ConstraintKind::Midpoint,
                                         ConstraintKind::Collinear,  ConstraintKind::Symmetric,  ConstraintKind::Fix};
    for (ConstraintKind k : all)
        if (s == constraintName(k)) return k;
    return makeError(ErrorCode::ParseError, "sketch: unknown constraint kind '" + s + "'");
}

const char* dimensionName(DimensionKind k) {
    switch (k) {
    case DimensionKind::Distance: return "distance";
    case DimensionKind::DistanceX: return "distance_x";
    case DimensionKind::DistanceY: return "distance_y";
    case DimensionKind::Radius: return "radius";
    case DimensionKind::Diameter: return "diameter";
    case DimensionKind::Angle: return "angle";
    }
    return "distance";
}

Result<DimensionKind> dimensionFrom(const std::string& s) {
    static const DimensionKind all[] = {DimensionKind::Distance, DimensionKind::DistanceX, DimensionKind::DistanceY,
                                        DimensionKind::Radius,   DimensionKind::Diameter,  DimensionKind::Angle};
    for (DimensionKind k : all)
        if (s == dimensionName(k)) return k;
    return makeError(ErrorCode::ParseError, "sketch: unknown dimension kind '" + s + "'");
}

json refsToJson(const std::vector<SketchRef>& refs) {
    json out = json::array();
    for (const SketchRef& r : refs) {
        json e = json::object();
        e["entity"] = r.entity.value();
        if (r.role) e["role"] = roleName(*r.role);
        out.push_back(std::move(e));
    }
    return out;
}

Result<std::vector<SketchRef>> refsFromJson(const json& a) {
    std::vector<SketchRef> refs;
    for (const json& e : a) {
        if (!e.is_object() || !e.contains("entity") || !e["entity"].is_number_unsigned())
            return makeError(ErrorCode::ParseError, "sketch: ref needs an unsigned 'entity'");
        SketchRef r;
        r.entity = sketch::SketchEntityId{e["entity"].get<std::uint64_t>()};
        if (e.contains("role")) {
            if (!e["role"].is_string()) return makeError(ErrorCode::ParseError, "sketch: ref role must be a string");
            auto role = roleFrom(e["role"].get<std::string>());
            if (!role) return std::unexpected(role.error());
            r.role = *role;
        }
        refs.push_back(r);
    }
    return refs;
}

} // namespace

Result<json> toJson(const sketch::Sketch& sk) {
    json out = json::object();
    const auto& pl = sk.plane();
    out["plane"] = json::object({{"origin", vec3(pl.origin)}, {"normal", vec3(pl.normal)}, {"xdir", vec3(pl.xDir)}});

    // The next id is observed through a throwaway copy: allocate one id, then drop it.
    Sketch probe = sk;
    auto tmp = probe.addPoint({0, 0});
    if (!tmp) return std::unexpected(tmp.error());
    out["nextId"] = tmp->value();

    json entities = json::array();
    for (const auto& [id, e] : sk.entities()) {
        json j = json::object();
        j["id"] = id.value();
        j["construction"] = e.construction;
        std::visit(
            [&](const auto& g) {
                using G = std::decay_t<decltype(g)>;
                if constexpr (std::is_same_v<G, sketch::PointGeom>) {
                    j["type"] = "point";
                    j["p"] = pt2(g.p);
                } else if constexpr (std::is_same_v<G, sketch::LineGeom>) {
                    j["type"] = "line";
                    j["start"] = pt2(g.start);
                    j["end"] = pt2(g.end);
                } else if constexpr (std::is_same_v<G, sketch::ArcGeom>) {
                    j["type"] = "arc";
                    j["center"] = pt2(g.center);
                    j["radius"] = g.radius;
                    j["start_angle"] = g.startAngle;
                    j["sweep"] = g.sweep;
                } else {
                    j["type"] = "circle";
                    j["center"] = pt2(g.center);
                    j["radius"] = g.radius;
                }
            },
            e.geometry);
        entities.push_back(std::move(j));
    }
    out["entities"] = std::move(entities);

    json constraints = json::array();
    for (const auto& [id, c] : sk.constraints()) {
        json j = json::object();
        j["id"] = id.value();
        j["kind"] = constraintName(c.kind);
        j["refs"] = refsToJson(c.refs);
        constraints.push_back(std::move(j));
    }
    out["constraints"] = std::move(constraints);

    json dimensions = json::array();
    for (const auto& [id, d] : sk.dimensions()) {
        json j = json::object();
        j["id"] = id.value();
        j["kind"] = dimensionName(d.kind);
        j["refs"] = refsToJson(d.refs);
        j["expression"] = d.expression;
        dimensions.push_back(std::move(j));
    }
    out["dimensions"] = std::move(dimensions);
    return out;
}

Result<Sketch> fromJson(const json& j) {
    if (!j.is_object()) return makeError(ErrorCode::ParseError, "sketch: not a JSON object");
    auto plane = j.find("plane");
    if (plane == j.end() || !plane->is_object()) return makeError(ErrorCode::ParseError, "sketch: 'plane' missing");
    auto v3 = [&](const char* key) -> Result<Vec3> {
        auto it = plane->find(key);
        if (it == plane->end() || !it->is_array() || it->size() != 3)
            return makeError(ErrorCode::ParseError, std::string("sketch plane: '") + key + "' must be [x, y, z]");
        for (const json& c : *it)
            if (!c.is_number()) return makeError(ErrorCode::ParseError, std::string("sketch plane: '") + key + "' has a non-number");
        return Vec3{(*it)[0].get<double>(), (*it)[1].get<double>(), (*it)[2].get<double>()};
    };
    auto origin = v3("origin"), normal = v3("normal"), xdir = v3("xdir");
    if (!origin) return std::unexpected(origin.error());
    if (!normal) return std::unexpected(normal.error());
    if (!xdir) return std::unexpected(xdir.error());
    auto sp = sketch::SketchPlane::make(*origin, *normal, *xdir);
    if (!sp) return std::unexpected(sp.error());
    Sketch sk(*sp);

    // Every record keeps its persisted id (Sketch::restore*). Entities first, so records can refer to them.
    auto ents = arr(j, "entities");
    if (!ents) return std::unexpected(ents.error());
    for (const json& e : **ents) {
        if (!e.is_object() || !e.contains("id") || !e["id"].is_number_unsigned())
            return makeError(ErrorCode::ParseError, "sketch: entity id missing");
        const sketch::SketchEntityId id{e["id"].get<std::uint64_t>()};
        const bool construction = e.value("construction", false);
        const std::string type = e.value("type", std::string{});
        Result<void> restored;
        if (type == "point") {
            auto p = pt(e, "p");
            if (!p) return std::unexpected(p.error());
            restored = sk.restoreEntity(id, sketch::PointGeom{*p}, construction);
        } else if (type == "line") {
            auto a = pt(e, "start"), b = pt(e, "end");
            if (!a) return std::unexpected(a.error());
            if (!b) return std::unexpected(b.error());
            restored = sk.restoreEntity(id, sketch::LineGeom{*a, *b}, construction);
        } else if (type == "arc") {
            auto c = pt(e, "center");
            auto rad = num(e, "radius"), sa = num(e, "start_angle"), sw = num(e, "sweep");
            if (!c) return std::unexpected(c.error());
            if (!rad) return std::unexpected(rad.error());
            if (!sa) return std::unexpected(sa.error());
            if (!sw) return std::unexpected(sw.error());
            restored = sk.restoreEntity(id, sketch::ArcGeom{*c, *rad, *sa, *sw}, construction);
        } else if (type == "circle") {
            auto c = pt(e, "center");
            auto rad = num(e, "radius");
            if (!c) return std::unexpected(c.error());
            if (!rad) return std::unexpected(rad.error());
            restored = sk.restoreEntity(id, sketch::CircleGeom{*c, *rad}, construction);
        } else {
            return makeError(ErrorCode::ParseError, "sketch: unknown entity type '" + type + "'");
        }
        if (!restored) return makeError(ErrorCode::ParseError, "sketch: " + restored.error().message);
    }

    auto cons = arr(j, "constraints");
    if (!cons) return std::unexpected(cons.error());
    for (const json& c : **cons) {
        if (!c.is_object() || !c.contains("id") || !c["id"].is_number_unsigned() || !c.contains("refs") || !c["refs"].is_array())
            return makeError(ErrorCode::ParseError, "sketch: malformed constraint");
        auto kind = constraintFrom(c.value("kind", std::string{}));
        if (!kind) return std::unexpected(kind.error());
        auto refs = refsFromJson(c["refs"]);
        if (!refs) return std::unexpected(refs.error());
        auto restored = sk.restoreConstraint(sketch::SketchConstraintId{c["id"].get<std::uint64_t>()}, *kind, *refs);
        if (!restored) return makeError(ErrorCode::ParseError, "sketch: " + restored.error().message);
    }

    auto dims = arr(j, "dimensions");
    if (!dims) return std::unexpected(dims.error());
    for (const json& d : **dims) {
        if (!d.is_object() || !d.contains("id") || !d["id"].is_number_unsigned() || !d.contains("refs") || !d["refs"].is_array() ||
            !d.contains("expression") || !d["expression"].is_string())
            return makeError(ErrorCode::ParseError, "sketch: malformed dimension");
        auto kind = dimensionFrom(d.value("kind", std::string{}));
        if (!kind) return std::unexpected(kind.error());
        auto refs = refsFromJson(d["refs"]);
        if (!refs) return std::unexpected(refs.error());
        auto restored = sk.restoreDimension(sketch::SketchDimensionId{d["id"].get<std::uint64_t>()}, *kind, *refs,
                                            d["expression"].get<std::string>());
        if (!restored) return makeError(ErrorCode::ParseError, "sketch: " + restored.error().message);
    }

    auto nextIdIt = j.find("nextId");
    if (nextIdIt == j.end() || !nextIdIt->is_number_unsigned()) return makeError(ErrorCode::ParseError, "sketch: 'nextId' missing");
    if (auto r = sk.restoreNextId(nextIdIt->get<std::uint64_t>()); !r)
        return makeError(ErrorCode::ParseError, "sketch: " + r.error().message);
    return sk;
}

Result<std::string> sketchToJson(const sketch::Sketch& sk) {
    auto j = toJson(sk);
    if (!j) return std::unexpected(j.error());
    return j->dump();
}

Result<Sketch> sketchFromJson(std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded()) return makeError(ErrorCode::ParseError, "sketch: not valid JSON");
    return fromJson(j);
}

} // namespace iw::features
