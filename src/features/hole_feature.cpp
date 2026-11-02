// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/hole_feature.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <cmath>
#include <map>
#include <optional>
#include <numbers>
#include <type_traits>
#include <utility>
#include <variant>

#include <nlohmann/json.hpp>

#include "core/expr.hpp"
#include "features/feature_json.hpp"
#include "features/sketch_feature.hpp"
#include "kernel/naming_support.hpp"
#include "kernel/ops.hpp"
#include "naming/naming_map.hpp"

namespace iw::features {
namespace {

using json = nlohmann::json;
using recompute::Diagnostic;
using recompute::EvalContext;
using recompute::FeatureResult;
using naming::ElementType;

const char* kindName(kernel::HoleKind k) {
    switch (k) {
    case kernel::HoleKind::Simple: return "simple";
    case kernel::HoleKind::Counterbore: return "counterbore";
    case kernel::HoleKind::Countersink: return "countersink";
    }
    return "simple";
}

/// Cosmetic thread designation: "M<d>" optionally followed by "x<pitch>" (e.g. "M6", "M6x1.0", "M10x1.5").
bool validThread(const std::string& s) {
    if (s.size() < 2 || s[0] != 'M') return false;
    std::size_t i = 1;
    auto digits = [&](bool allowDot) {
        std::size_t start = i;
        while (i < s.size() && (std::isdigit(static_cast<unsigned char>(s[i])) || (allowDot && s[i] == '.' && i > start))) ++i;
        return i > start && std::isdigit(static_cast<unsigned char>(s[i - 1]));
    };
    if (!digits(true)) return false;
    if (i == s.size()) return true;
    if (s[i] != 'x' && s[i] != 'X') return false;
    ++i;
    return digits(true) && i == s.size();
}

/// Names of the hole tool (operand 1 of the cut), by ROLE. The tool is a revolved polygon; each profile segment has a
/// role ("wall", "floor", "drill_cone", "cbore_wall", ...) that does not depend on any dimension, so the faces and edges
/// the tool contributes to the body keep their names when the diameter, depth or position of the hole is edited.
///   profile face -> "holetool.profile"; profile edge -> "holetool.<role>"; profile vertex -> "holetool.v_<role of the
///   segment it starts>"; the tool's own elements then derive from those through the revolve history (ops "holetool.<...>").
/// Ops "hole.*" are what the cut itself creates where the tool meets the body (rim edges and vertices), named from the
/// body face and the tool face they lie between.
/// `tag` is "holetool" for a face-placed hole and "holetool.e<sketch entity id>" for each hole placed from a sketch point,
/// so the holes of one multi-hole feature never share names.
Result<naming::NamingMap> holeToolNaming(FeatureId id, const kernel::HoleDetail& hd, const naming::SiblingFrame& frame,
                                         const std::string& tag = "holetool") {
    const auto& pts = hd.profile.points;
    const auto& roles = hd.profile.segmentRoles;
    const std::size_t n = pts.size();
    naming::NamingMap prof;
    prof.resize(hd.profileFace.subShapeCount(kernel::SubShapeKind::Face), hd.profileFace.subShapeCount(kernel::SubShapeKind::Edge),
                hd.profileFace.subShapeCount(kernel::SubShapeKind::Vertex));
    auto make = [&](ElementType t, const std::string& op) {
        naming::ElementName nm;
        nm.feature = id;
        nm.type = t;
        nm.op = op;
        return nm;
    };
    for (std::uint32_t i = 0; i < prof.size(ElementType::Face); ++i)
        if (auto r = prof.set(ElementType::Face, i, make(ElementType::Face, tag + ".profile")); !r) return std::unexpected(r.error());
    auto close = [](const Vec3& a, const Vec3& b) {
        return length(a - b) <= 1e-6 * (1.0 + std::max(length(a), length(b)));
    };
    for (std::uint32_t i = 0; i < prof.size(ElementType::Edge); ++i) {
        const auto k = kernel::stableSubShapeKey(hd.profileFace, kernel::SubShapeKind::Edge, i);
        bool found = false;
        for (std::size_t s = 0; s < n && !found; ++s) {
            const Vec3 mid = (pts[s] + pts[(s + 1) % n]) * 0.5;
            if (!close(mid, Vec3{k[0], k[1], k[2]})) continue;
            found = true;
            if (auto r = prof.set(ElementType::Edge, i, make(ElementType::Edge, tag + "." + roles[s])); !r) return std::unexpected(r.error());
        }
        if (!found) return makeError(ErrorCode::Internal, "hole: tool profile edge does not match a profile segment");
    }
    for (std::uint32_t i = 0; i < prof.size(ElementType::Vertex); ++i) {
        const auto k = hd.profileFace.subShapeKey(kernel::SubShapeKind::Vertex, i);
        bool found = false;
        for (std::size_t s = 0; s < n && !found; ++s) {
            if (!close(pts[s], Vec3{k[0], k[1], k[2]})) continue;
            found = true;
            if (auto r = prof.set(ElementType::Vertex, i, make(ElementType::Vertex, tag + ".v_" + roles[s])); !r) return std::unexpected(r.error());
        }
        if (!found) return makeError(ErrorCode::Internal, "hole: tool profile vertex does not match a profile point");
    }
    return naming::deriveNamingMap(id, tag, hd.tool.shape, {&prof}, hd.tool.history, &frame);
}

} // namespace

std::vector<std::string> HoleFeature::parameterReferences() const {
    std::vector<std::string> out;
    for (const std::string* t : {&params_.u, &params_.v, &params_.diameter, &params_.depth, &params_.drillAngle, &params_.counterboreDiameter,
                                 &params_.counterboreDepth, &params_.countersinkDiameter, &params_.countersinkAngle}) {
        auto ex = Expression::parse(*t);
        if (ex) for (auto& r : ex->references()) out.push_back(r);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::uint64_t HoleFeature::paramHash() const {
    common::Fnv f;
    f.bytes("hole/v1:");
    f.bytes(paramsJson());
    // The embedded solved sketch (sketch placement) is not serialised but decides the geometry: hash the plane and the
    // position of every referenced entity, so editing the sketch invalidates the cached result.
    if (params_.sketchPoints && params_.sketchPoints->solved) {
        const sketch::Sketch& sk = *params_.sketchPoints->solved;
        const sketch::SketchPlane& pl = sk.plane();
        json h = json::array({pl.origin.x, pl.origin.y, pl.origin.z, pl.normal.x, pl.normal.y, pl.normal.z, pl.xDir.x, pl.xDir.y,
                              pl.xDir.z});
        for (sketch::SketchEntityId e : params_.sketchPoints->points) {
            const sketch::SketchEntity* ent = sk.findEntity(e);
            if (!ent) {
                h.push_back("missing");
                continue;
            }
            h.push_back(static_cast<int>(ent->kind()));
            std::visit(
                [&](const auto& g) {
                    using G = std::decay_t<decltype(g)>;
                    if constexpr (std::is_same_v<G, sketch::PointGeom>) {
                        h.push_back(g.p.x);
                        h.push_back(g.p.y);
                    } else if constexpr (std::is_same_v<G, sketch::LineGeom>) {
                        h.push_back(g.start.x);
                        h.push_back(g.start.y);
                    } else {
                        h.push_back(g.center.x);
                        h.push_back(g.center.y);
                    }
                },
                ent->geometry);
        }
        f.bytes(":solved:");
        f.bytes(h.dump());
    }
    return f.h;
}

std::string HoleFeature::paramsJson() const {
    const HoleParams& p = params_;
    json j{{"body", p.body.value()}, {"u", p.u}, {"v", p.v}, {"kind", kindName(p.kind)},
           {"diameter", p.diameter}, {"depth", p.depth}, {"through", p.through}, {"drill_angle", p.drillAngle},
           {"cb_diameter", p.counterboreDiameter}, {"cb_depth", p.counterboreDepth}, {"cs_diameter", p.countersinkDiameter},
           {"cs_angle", p.countersinkAngle}, {"thread", p.thread}};
    if (p.sketchPoints) {
        // Sketch placement: no face reference. The face form keeps exactly its earlier keys (no "placement" key).
        j["placement"] = "sketch_points";
        j["sketch_feature"] = p.sketchPoints->sketch.value();
        json pts = json::array();
        for (sketch::SketchEntityId e : p.sketchPoints->points) pts.push_back(e.value());
        j["points"] = std::move(pts);
        j["flip"] = p.sketchPoints->flip;
    } else {
        j["face"] = common::refToJson(p.face);
    }
    return j.dump();
}

Result<std::shared_ptr<recompute::Feature>> HoleFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "hole: params are not a JSON object");
    HoleParams p;
    auto placement = common::readString(j, "placement", std::string("face"));
    if (!placement) return std::unexpected(placement.error());
    if (*placement == "sketch_points") {
        HoleSketchPlacement sp;
        auto sf = j.find("sketch_feature");
        if (sf == j.end() || !sf->is_number_unsigned()) return makeError(ErrorCode::ParseError, "hole: 'sketch_feature' must be an unsigned id");
        sp.sketch = FeatureId{sf->get<std::uint64_t>()};
        auto pts = j.find("points");
        if (pts == j.end() || !pts->is_array() || pts->empty())
            return makeError(ErrorCode::ParseError, "hole: 'points' must be a non-empty array of sketch entity ids");
        for (const json& e : *pts) {
            if (!e.is_number_unsigned()) return makeError(ErrorCode::ParseError, "hole: sketch point ids must be unsigned integers");
            sp.points.push_back(sketch::SketchEntityId{e.get<std::uint64_t>()});
        }
        auto flip = j.find("flip");
        if (flip != j.end() && !flip->is_boolean()) return makeError(ErrorCode::ParseError, "hole: 'flip' must be a boolean");
        sp.flip = flip != j.end() && flip->get<bool>();
        p.sketchPoints = std::move(sp);
    } else if (*placement == "face") {
        auto face = common::refFromJson(j, "face");
        if (!face) return std::unexpected(face.error());
        if (face->type != ElementType::Face) return makeError(ErrorCode::ParseError, "hole: placement reference is not a face");
        p.face = *face;
    } else {
        return makeError(ErrorCode::ParseError, "hole: unknown placement '" + *placement + "'");
    }

    struct Field { std::string* dst; const char* key; const char* fallback; };
    const Field fields[] = {{&p.u, "u", "0 mm"}, {&p.v, "v", "0 mm"}, {&p.diameter, "diameter", "6 mm"}, {&p.depth, "depth", "10 mm"},
                            {&p.drillAngle, "drill_angle", "118 deg"}, {&p.counterboreDiameter, "cb_diameter", "10 mm"},
                            {&p.counterboreDepth, "cb_depth", "3 mm"}, {&p.countersinkDiameter, "cs_diameter", "12 mm"},
                            {&p.countersinkAngle, "cs_angle", "90 deg"}};
    for (const Field& f : fields) {
        auto v = common::readString(j, f.key, std::string(f.fallback));
        if (!v) return std::unexpected(v.error());
        if (auto c = common::checkExpression(*v, f.key); !c) return std::unexpected(c.error());
        *f.dst = *v;
    }
    auto kind = common::readString(j, "kind", std::string("simple"));
    if (!kind) return std::unexpected(kind.error());
    if (*kind == "simple") p.kind = kernel::HoleKind::Simple;
    else if (*kind == "counterbore") p.kind = kernel::HoleKind::Counterbore;
    else if (*kind == "countersink") p.kind = kernel::HoleKind::Countersink;
    else return makeError(ErrorCode::ParseError, "hole: unknown kind '" + *kind + "'");
    p.through = j.value("through", false);
    p.thread = j.value("thread", std::string());
    auto body = j.find("body");
    if (body == j.end() || !body->is_number_unsigned()) return makeError(ErrorCode::ParseError, "hole: 'body' must be an unsigned id");
    p.body = BodyId{body->get<std::uint64_t>()};
    return std::shared_ptr<recompute::Feature>(std::make_shared<HoleFeature>(id, std::move(p)));
}

Result<FeatureResult> HoleFeature::compute(const EvalContext& ctx) const {
    const HoleParams& p = params_;
    if (!p.body.valid()) return makeError(ErrorCode::InvalidArgument, "hole: invalid body id");
    const kernel::Shape* body = ctx.body(p.body);
    if (!body) return makeError(ErrorCode::NotFound, "hole: body does not exist at this position");
    if (!p.thread.empty() && !validThread(p.thread))
        return makeError(ErrorCode::InvalidArgument, "hole: thread '" + p.thread + "' is not a designation such as M6 or M6x1.0");

    kernel::HoleSpec spec;
    spec.kind = p.kind;
    auto dia = common::lengthOf(ctx.params, p.diameter, "hole diameter");
    if (!dia) return std::unexpected(dia.error());
    spec.diameter = *dia;
    spec.through = p.through;
    if (!p.through) {
        auto d = common::lengthOf(ctx.params, p.depth, "hole depth");
        if (!d) return std::unexpected(d.error());
        spec.depth = *d;
    }
    auto drill = common::angleOf(ctx.params, p.drillAngle, "drill point angle");
    if (!drill) return std::unexpected(drill.error());
    spec.drillPointAngle = *drill;
    if (p.kind == kernel::HoleKind::Counterbore) {
        auto cd = common::lengthOf(ctx.params, p.counterboreDiameter, "counterbore diameter");
        if (!cd) return std::unexpected(cd.error());
        auto cdep = common::lengthOf(ctx.params, p.counterboreDepth, "counterbore depth");
        if (!cdep) return std::unexpected(cdep.error());
        spec.counterboreDiameter = *cd;
        spec.counterboreDepth = *cdep;
    } else if (p.kind == kernel::HoleKind::Countersink) {
        auto cd = common::lengthOf(ctx.params, p.countersinkDiameter, "countersink diameter");
        if (!cd) return std::unexpected(cd.error());
        auto ca = common::angleOf(ctx.params, p.countersinkAngle, "countersink angle");
        if (!ca) return std::unexpected(ca.error());
        spec.countersinkDiameter = *cd;
        spec.countersinkAngle = *ca;
    }
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "hole cancelled");
    if (p.sketchPoints) {
        auto out = computeFromSketch(ctx, spec);
        if (out && !p.thread.empty()) out->attributes["thread"] = p.thread;
        return out;
    }

    std::vector<Diagnostic> diags;
    auto faceIdx = common::resolveRef(ctx, p.body, p.face, ElementType::Face, diags);
    if (!faceIdx) return std::unexpected(faceIdx.error());
    auto plane = kernel::planeFromFace(*body, *faceIdx);
    if (!plane) return makeError(ErrorCode::InvalidArgument, "hole: the placement face is not planar");

    auto u = common::lengthOf(ctx.params, p.u, "hole placement u");
    if (!u) return std::unexpected(u.error());
    auto v = common::lengthOf(ctx.params, p.v, "hole placement v");
    if (!v) return std::unexpected(v.error());
    const Vec3 yDir = cross(plane->normal, plane->xDir);
    const Vec3 entry = plane->origin + plane->xDir * *u + yDir * *v;

    const kernel::Axis placement{entry, plane->normal * -1.0};
    auto detail = kernel::holeWithTool(*body, placement, spec, ctx.cancel);
    if (!detail) return std::unexpected(detail.error());
    // Sibling ranks are taken in the hole's own frame (origin = entry point, z = drill direction, x = the face plane's x), so
    // translating or rotating the hole together with its body cannot reorder siblings.
    naming::SiblingFrame frame;
    frame.origin = entry;
    frame.zAxis = placement.direction;
    frame.xAxis = plane->xDir;
    frame.yAxis = cross(frame.zAxis, frame.xAxis);
    auto toolNames = holeToolNaming(id_, *detail, frame);
    if (!toolNames) return std::unexpected(toolNames.error());
    auto out = common::modifyBody(id_, p.body, ctx, detail->result, "hole", std::move(diags), &*toolNames, &frame);
    if (!out) return std::unexpected(out.error());
    if (!p.thread.empty()) out->attributes["thread"] = p.thread;
    return out;
}

Result<FeatureResult> HoleFeature::computeFromSketch(const EvalContext& ctx, const kernel::HoleSpec& spec) const {
    const HoleParams& p = params_;
    const HoleSketchPlacement& sp = *p.sketchPoints;
    const std::string sketchName = "sketch feature " + std::to_string(sp.sketch.value());
    // An embedded copy (pre-pass callers) wins; otherwise the solved sketch published on the recompute sketch channel.
    const sketch::Sketch* solved = sp.solved ? &*sp.solved : nullptr;
    if (!solved)
        if (const SketchSolution* sol = solvedSketchAt(ctx, sp.sketch)) solved = &sol->sketch;
    if (!solved)
        return makeError(ErrorCode::NotFound, "hole: " + sketchName + " has no solved sketch at this position (it must precede the hole and solve)");
    if (sp.points.empty()) return makeError(ErrorCode::InvalidArgument, "hole: no sketch points selected");
    const sketch::Sketch& sk = *solved;
    const sketch::SketchPlane& plane = sk.plane();

    std::vector<Diagnostic> diags;
    struct Site {
        sketch::SketchEntityId id;
        Vec3 entry;
    };
    std::vector<Site> sites;
    std::vector<sketch::SketchEntityId> seen;
    for (sketch::SketchEntityId e : sp.points) {
        if (std::find(seen.begin(), seen.end(), e) != seen.end()) continue; // a point selected twice drills once
        seen.push_back(e);
        const std::string pointName = "e" + std::to_string(e.value());
        const sketch::SketchEntity* ent = sk.findEntity(e);
        if (!ent) {
            diags.push_back({Diagnostic::Severity::Warning,
                             "hole: sketch point " + pointName + " no longer exists in " + sketchName + "; its hole is skipped", std::nullopt});
            continue;
        }
        sketch::Point2 at;
        if (const auto* pt = std::get_if<sketch::PointGeom>(&ent->geometry)) at = pt->p;
        else if (const auto* c = std::get_if<sketch::CircleGeom>(&ent->geometry)) at = c->center;
        else if (const auto* a = std::get_if<sketch::ArcGeom>(&ent->geometry)) at = a->center;
        else
            return makeError(ErrorCode::InvalidArgument,
                             "hole: sketch entity " + pointName + " of " + sketchName + " is a line; select sketch points or circle / arc centres");
        sites.push_back({e, plane.toWorld(at)});
    }
    if (sites.empty()) return makeError(ErrorCode::NotFound, "hole: none of the selected sketch points exists in " + sketchName + " any more");

    // Drill direction (see HoleSketchPlacement): +n only when the whole body lies on the +n side of the sketch plane.
    const kernel::Shape* body = ctx.body(p.body);
    const naming::NamingMap* bodyNames = ctx.naming(p.body);
    if (!body || !bodyNames) return makeError(ErrorCode::NotFound, "hole: body does not exist at this position");
    const Vec3 n = common::normalized(plane.normal);
    const Box3 bb = body->boundingBox();
    double sMin = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 8; ++i) {
        const Vec3 corner{(i & 1) ? bb.max.x : bb.min.x, (i & 2) ? bb.max.y : bb.min.y, (i & 4) ? bb.max.z : bb.min.z};
        sMin = std::min(sMin, dot(corner - plane.origin, n));
    }
    const double tol = 1e-6 * (1.0 + length(bb.max - bb.min));
    Vec3 dir = sMin >= -tol ? n : n * -1.0;
    if (sp.flip) dir = dir * -1.0;

    // One kernel hole per point, applied in selection order to the evolving body. Each hole is named in its own frame and
    // with its own tags ("hole.e<id>", "holetool.e<id>"), so the names follow the sketch point, not its position or rank.
    std::map<BodyId, kernel::Shape> bodies{{p.body, *body}};
    std::map<BodyId, naming::NamingMap> namings{{p.body, *bodyNames}};
    FeatureResult last;
    // A hole whose point has moved off the part removes no material. That is not an error (restoring the point brings the
    // hole back), but it must never be silent: warn per missing hole, like Fusion's "does not intersect" feedback [VERIFY].
    auto volumeOf = [](const kernel::Shape& s) -> std::optional<double> {
        auto m = kernel::massProperties(s);
        return m ? std::optional<double>(m->volume) : std::nullopt;
    };
    std::optional<double> volBefore = volumeOf(bodies.at(p.body));
    for (const Site& site : sites) {
        if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "hole cancelled");
        const std::string pointName = "e" + std::to_string(site.id.value());
        const kernel::Axis placement{site.entry, dir};
        auto detail = kernel::holeWithTool(bodies.at(p.body), placement, spec, ctx.cancel);
        if (!detail) return makeError(detail.error().code, detail.error().message + " (at sketch point " + pointName + ")");
        const std::optional<double> volAfter = volumeOf(detail->result.shape);
        if (volBefore && volAfter && *volBefore - *volAfter <= 1e-9 * (1.0 + std::abs(*volBefore))) {
            diags.push_back({Diagnostic::Severity::Warning,
                             "hole: the hole at sketch point " + pointName + " does not intersect the body; it removes no material",
                             std::nullopt});
        }
        volBefore = volAfter;
        naming::SiblingFrame frame;
        frame.origin = site.entry;
        frame.zAxis = dir;
        frame.xAxis = common::normalized(plane.xDir);
        frame.yAxis = cross(frame.zAxis, frame.xAxis);
        auto toolNames = holeToolNaming(id_, *detail, frame, "holetool." + pointName);
        if (!toolNames) return std::unexpected(toolNames.error());
        const EvalContext step{bodies, namings, ctx.cancel, ctx.params, ctx.constructions};
        auto out = common::modifyBody(id_, p.body, step, detail->result, "hole." + pointName, {}, &*toolNames, &frame);
        if (!out) return std::unexpected(out.error());
        bodies.insert_or_assign(p.body, out->bodies.at(p.body));
        namings.insert_or_assign(p.body, out->namings.at(p.body));
        last = std::move(*out);
    }
    last.diagnostics = std::move(diags);
    last.attributes["holes"] = std::to_string(sites.size());
    return last;
}

void registerHoleFeature(recompute::FeatureRegistry& registry) {
    registry.add("hole", [](FeatureId id, std::string_view p) { return HoleFeature::fromJson(id, p); });
}

} // namespace iw::features
