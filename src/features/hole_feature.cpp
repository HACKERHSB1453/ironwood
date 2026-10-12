// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/hole_feature.hpp"

#include <algorithm>
#include <cctype>
#include <numbers>
#include <utility>

#include <nlohmann/json.hpp>

#include "core/expr.hpp"
#include "features/feature_json.hpp"
#include "kernel/ops.hpp"

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
    return f.h;
}

std::string HoleFeature::paramsJson() const {
    const HoleParams& p = params_;
    json j{{"body", p.body.value()}, {"face", common::refToJson(p.face)}, {"u", p.u}, {"v", p.v}, {"kind", kindName(p.kind)},
           {"diameter", p.diameter}, {"depth", p.depth}, {"through", p.through}, {"drill_angle", p.drillAngle},
           {"cb_diameter", p.counterboreDiameter}, {"cb_depth", p.counterboreDepth}, {"cs_diameter", p.countersinkDiameter},
           {"cs_angle", p.countersinkAngle}, {"thread", p.thread}};
    return j.dump();
}

Result<std::shared_ptr<recompute::Feature>> HoleFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "hole: params are not a JSON object");
    HoleParams p;
    auto face = common::refFromJson(j, "face");
    if (!face) return std::unexpected(face.error());
    if (face->type != ElementType::Face) return makeError(ErrorCode::ParseError, "hole: placement reference is not a face");
    p.face = *face;

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

    const kernel::Axis placement{entry, plane->normal * -1.0};
    auto result = kernel::hole(*body, placement, spec, ctx.cancel);
    if (!result) return std::unexpected(result.error());
    auto out = common::modifyBody(id_, p.body, ctx, *result, "hole", std::move(diags));
    if (!out) return std::unexpected(out.error());
    if (!p.thread.empty()) out->attributes["thread"] = p.thread;
    return out;
}

void registerHoleFeature(recompute::FeatureRegistry& registry) {
    registry.add("hole", [](FeatureId id, std::string_view p) { return HoleFeature::fromJson(id, p); });
}

} // namespace iw::features
