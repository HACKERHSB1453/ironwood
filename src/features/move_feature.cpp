// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/move_feature.hpp"

#include <algorithm>
#include <utility>

#include <nlohmann/json.hpp>

#include "core/expr.hpp"
#include "features/feature_common.hpp"
#include "features/feature_json.hpp"
#include "kernel/ops.hpp"
#include "naming/naming_map.hpp"

namespace iw::features {
namespace {

using json = nlohmann::json;
using recompute::Diagnostic;
using recompute::EvalContext;
using recompute::FeatureResult;

Result<Vec3> vec3From(const json& j, const char* key) {
    auto v = common::readVec(j, key);
    if (!v) return std::unexpected(v.error());
    return Vec3{(*v)[0], (*v)[1], (*v)[2]};
}

json v3(const Vec3& v) { return json::array({v.x, v.y, v.z}); }

} // namespace

std::vector<std::string> MoveFeature::parameterReferences() const {
    std::vector<std::string> out;
    for (const std::string* t : {&params_.translate[0], &params_.translate[1], &params_.translate[2], &params_.angle}) {
        auto ex = Expression::parse(*t);
        if (ex) for (auto& r : ex->references()) out.push_back(r);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::vector<BodyId> MoveFeature::writesBodies() const {
    if (params_.copy) return {params_.newBody};
    return {params_.body};
}

std::uint64_t MoveFeature::paramHash() const {
    common::Fnv f;
    f.bytes("move/v1:");
    f.bytes(paramsJson());
    return f.h;
}

std::string MoveFeature::paramsJson() const {
    json j{{"body", params_.body.value()}, {"copy", params_.copy}, {"new_body", params_.newBody.value()},
           {"translate", json::array({params_.translate[0], params_.translate[1], params_.translate[2]})},
           {"angle", params_.angle}, {"axis_origin", v3(params_.axisOrigin)}, {"axis_direction", v3(params_.axisDirection)}};
    return j.dump();
}

Result<std::shared_ptr<recompute::Feature>> MoveFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "move: params are not a JSON object");
    MoveParams p;
    auto body = j.find("body");
    if (body == j.end() || !body->is_number_unsigned()) return makeError(ErrorCode::ParseError, "move: 'body' must be an unsigned id");
    p.body = BodyId{body->get<std::uint64_t>()};
    p.copy = j.value("copy", false);
    p.newBody = BodyId{j.value("new_body", std::uint64_t{0})};
    auto tr = j.find("translate");
    if (tr == j.end() || !tr->is_array() || tr->size() != 3) return makeError(ErrorCode::ParseError, "move: 'translate' must be 3 expressions");
    for (std::size_t i = 0; i < 3; ++i) {
        if (!(*tr)[i].is_string()) return makeError(ErrorCode::ParseError, "move: 'translate' entries must be strings");
        p.translate[i] = (*tr)[i].get<std::string>();
        if (auto c = common::checkExpression(p.translate[i], "translate"); !c) return std::unexpected(c.error());
    }
    auto angle = common::readString(j, "angle", std::string("0 deg"));
    if (!angle) return std::unexpected(angle.error());
    if (auto c = common::checkExpression(*angle, "angle"); !c) return std::unexpected(c.error());
    p.angle = *angle;
    auto o = vec3From(j, "axis_origin");
    if (!o) return std::unexpected(o.error());
    auto d = vec3From(j, "axis_direction");
    if (!d) return std::unexpected(d.error());
    p.axisOrigin = *o;
    p.axisDirection = *d;
    return std::shared_ptr<recompute::Feature>(std::make_shared<MoveFeature>(id, std::move(p)));
}

Result<FeatureResult> MoveFeature::compute(const EvalContext& ctx) const {
    const MoveParams& p = params_;
    if (!p.body.valid()) return makeError(ErrorCode::InvalidArgument, "move: invalid body id");
    const kernel::Shape* in = ctx.body(p.body);
    const naming::NamingMap* inNames = ctx.naming(p.body);
    if (!in || !inNames) return makeError(ErrorCode::NotFound, "move: body does not exist at this position");
    if (p.copy && (!p.newBody.valid() || p.newBody == p.body))
        return makeError(ErrorCode::InvalidArgument, "copy: needs a new body id different from the source body");
    if (p.copy && ctx.body(p.newBody)) return makeError(ErrorCode::InvalidArgument, "copy: the new body already exists");
    if (!common::finite(p.axisOrigin) || !common::finite(p.axisDirection) || length(p.axisDirection) < 1e-12)
        return makeError(ErrorCode::InvalidArgument, "move: rotation axis needs a finite origin and a non-zero direction");

    auto angle = common::angleOf(ctx.params, p.angle, "rotation angle");
    if (!angle) return std::unexpected(angle.error());
    Vec3 t{0, 0, 0};
    double* comps[] = {&t.x, &t.y, &t.z};
    for (std::size_t i = 0; i < 3; ++i) {
        auto v = common::lengthOf(ctx.params, p.translate[i], "translation");
        if (!v) return std::unexpected(v.error());
        *comps[i] = *v;
    }

    const kernel::Axis axis{p.axisOrigin, common::normalized(p.axisDirection)};
    Transform m = common::rotationAbout(axis, *angle);
    m.m[3] += t.x;
    m.m[7] += t.y;
    m.m[11] += t.z;
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "move cancelled");
    auto moved = kernel::transform(*in, m, ctx.cancel);
    if (!moved) return std::unexpected(moved.error());

    auto names = naming::deriveNamingMap(id_, "move", moved->shape, {inNames}, moved->history);
    if (!names) return std::unexpected(names.error());
    FeatureResult out;
    if (p.copy) {
        out.bodies.emplace(p.newBody, std::move(moved->shape));
        out.namings.emplace(p.newBody, std::move(*names));
    } else {
        out.bodies.emplace(p.body, std::move(moved->shape));
        out.namings.emplace(p.body, std::move(*names));
    }
    return out;
}

void registerMoveFeature(recompute::FeatureRegistry& registry) {
    registry.add("move", [](FeatureId id, std::string_view p) { return MoveFeature::fromJson(id, p); });
}

} // namespace iw::features
