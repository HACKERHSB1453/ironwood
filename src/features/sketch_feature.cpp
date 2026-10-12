// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/sketch_feature.hpp"

#include <algorithm>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "features/extrude_feature.hpp"
#include "features/feature_json.hpp"
#include "features/revolve_feature.hpp"
#include "features/sketch_json.hpp"
#include "kernel/ops.hpp"

namespace iw::features {
namespace {

using json = nlohmann::json;
using recompute::Diagnostic;
using recompute::EvalContext;
using recompute::Feature;
using recompute::FeatureRegistry;
using recompute::FeatureResult;
using naming::ElementType;

json planeRefToJson(const PlaneRef& r) {
    if (r.isFace) return json{{"body", r.body.value()}, {"face", common::refToJson(r.face)}};
    return json{{"feature", r.construction.value()}};
}

Result<PlaneRef> planeRefFromJson(const json& j) {
    if (!j.is_object()) return makeError(ErrorCode::ParseError, "sketch: 'attach' must be an object");
    PlaneRef r;
    if (j.contains("feature")) {
        if (!j["feature"].is_number_unsigned()) return makeError(ErrorCode::ParseError, "sketch: attach feature id must be unsigned");
        r.isFace = false;
        r.construction = FeatureId{j["feature"].get<std::uint64_t>()};
        return r;
    }
    if (!j.contains("body") || !j["body"].is_number_unsigned()) return makeError(ErrorCode::ParseError, "sketch: attach face needs 'body'");
    r.body = BodyId{j["body"].get<std::uint64_t>()};
    auto face = common::refFromJson(j, "face");
    if (!face) return std::unexpected(face.error());
    if (face->type != ElementType::Face) return makeError(ErrorCode::ParseError, "sketch: attach reference is not a face");
    r.face = *face;
    return r;
}

/// Plane of a PlaneRef at this timeline position: a planar face of a body, or a Plane construction feature.
Result<sketch::SketchPlane> resolvePlane(const EvalContext& ctx, const PlaneRef& r, std::vector<Diagnostic>& diags) {
    kernel::Plane plane;
    if (!r.isFace) {
        const recompute::ConstructionGeometry* g = ctx.construction(r.construction);
        if (!g || g->kind != recompute::ConstructionGeometry::Kind::Plane)
            return makeError(ErrorCode::NotFound, "sketch attachment: plane feature " + std::to_string(r.construction.value()) + " is not a plane at this position");
        plane = kernel::Plane{g->origin, g->normal, g->xDir};
    } else {
        auto idx = common::resolveRef(ctx, r.body, r.face, ElementType::Face, diags);
        if (!idx) return std::unexpected(idx.error());
        auto p = kernel::planeFromFace(*ctx.body(r.body), *idx);
        if (!p) return makeError(ErrorCode::InvalidArgument, "sketch attachment: the referenced face is not planar");
        plane = *p;
    }
    return sketch::SketchPlane::make(plane.origin, plane.normal, plane.xDir);
}

/// Copy of `src` on `plane`, with every record keeping its id.
Result<sketch::Sketch> rebase(const sketch::Sketch& src, const sketch::SketchPlane& plane) {
    sketch::Sketch out(plane);
    for (const auto& [id, e] : src.entities())
        if (auto r = out.restoreEntity(id, e.geometry, e.construction); !r) return std::unexpected(r.error());
    for (const auto& [id, c] : src.constraints())
        if (auto r = out.restoreConstraint(id, c.kind, c.refs); !r) return std::unexpected(r.error());
    for (const auto& [id, d] : src.dimensions())
        if (auto r = out.restoreDimension(id, d.kind, d.refs, d.expression); !r) return std::unexpected(r.error());
    if (auto r = out.restoreNextId(src.nextId()); !r) return std::unexpected(r.error());
    return out;
}

std::string joinIds(const std::vector<std::uint64_t>& ids) {
    std::string out;
    for (std::uint64_t v : ids) {
        if (!out.empty()) out += ", ";
        out += std::to_string(v);
    }
    return out;
}

/// Message of a Failed solve: names every conflicting constraint and dimension record id.
std::string failureMessage(const sketch::SketchSolveReport& rep) {
    std::vector<std::uint64_t> cs, ds;
    for (auto c : rep.conflictingConstraints) cs.push_back(c.value());
    for (auto d : rep.conflictingDimensions) ds.push_back(d.value());
    if (cs.empty() && ds.empty()) return "sketch does not solve" + (rep.message.empty() ? std::string() : ": " + rep.message);
    std::string msg = "over-constrained sketch: conflicting";
    if (!cs.empty()) msg += " constraint ids " + joinIds(cs);
    if (!cs.empty() && !ds.empty()) msg += ";";
    if (!ds.empty()) msg += " dimension ids " + joinIds(ds);
    return msg;
}

std::uint64_t hashOf(std::string_view tag, const std::string& text) {
    common::Fnv f;
    f.bytes(tag);
    f.bytes(text);
    return f.h;
}

} // namespace

// ---- SketchFeature ----------------------------------------------------------------------------------------------

std::vector<naming::ElementRef> SketchFeature::explicitRefs() const {
    if (params_.attach && params_.attach->isFace) return {params_.attach->face};
    return {};
}

std::vector<std::string> SketchFeature::parameterReferences() const {
    std::vector<std::string> out;
    for (const auto& [id, dim] : params_.sketch.dimensions()) {
        auto r = common::referencesOf(dim.expression);
        out.insert(out.end(), r.begin(), r.end());
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::vector<BodyId> SketchFeature::readsBodies() const {
    if (params_.attach && params_.attach->isFace) return {params_.attach->body};
    return {};
}

std::uint64_t SketchFeature::paramHash() const { return hashOf("sketch/v1:", paramsJson()); }

std::string SketchFeature::paramsJson() const {
    json j = json::object();
    auto sj = sketchToJson(params_.sketch);
    j["sketch"] = sj ? json::parse(*sj, nullptr, false) : json::object();
    j["attach"] = params_.attach ? planeRefToJson(*params_.attach) : json(nullptr);
    return j.dump();
}

Result<std::shared_ptr<Feature>> SketchFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "sketch: params are not a JSON object");
    auto skIt = j.find("sketch");
    if (skIt == j.end()) return makeError(ErrorCode::ParseError, "sketch: missing 'sketch'");
    auto sk = sketchFromJson(skIt->dump());
    if (!sk) return std::unexpected(sk.error());
    SketchFeatureParams p{std::move(*sk), std::nullopt};
    if (auto at = j.find("attach"); at != j.end() && !at->is_null()) {
        auto r = planeRefFromJson(*at);
        if (!r) return std::unexpected(r.error());
        p.attach = std::move(*r);
    }
    return std::shared_ptr<Feature>(std::make_shared<SketchFeature>(id, std::move(p)));
}

Result<SketchSolution> SketchFeature::solve(const EvalContext& ctx) const {
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "sketch cancelled");
    std::vector<Diagnostic> diags;

    Result<sketch::Sketch> work = makeError(ErrorCode::Internal, "sketch not built");
    if (params_.attach) {
        auto plane = resolvePlane(ctx, *params_.attach, diags);
        if (!plane) return std::unexpected(plane.error());
        work = rebase(params_.sketch, *plane);
    } else {
        work = params_.sketch; // copy: the solver writes back into the working sketch only
    }
    if (!work) return std::unexpected(work.error());

    const sketch::SketchSolveReport rep = sketch::solveSketch(*work, ctx.params);
    if (rep.status == sketch::SolverStatus::InvalidInput)
        return makeError(ErrorCode::InvalidArgument, "sketch: " + rep.message);
    if (rep.status == sketch::SolverStatus::Failed) return makeError(ErrorCode::InvalidArgument, failureMessage(rep));
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "sketch cancelled");

    for (auto c : rep.redundantConstraints)
        diags.push_back({Diagnostic::Severity::Warning, "constraint " + std::to_string(c.value()) + " is redundant", std::nullopt});
    for (auto d : rep.redundantDimensions)
        diags.push_back({Diagnostic::Severity::Warning, "dimension " + std::to_string(d.value()) + " is redundant", std::nullopt});

    auto profiles = sketch::detectProfiles(*work);
    if (!profiles) return std::unexpected(profiles.error());
    return SketchSolution{std::move(*work), std::move(*profiles), rep, std::move(diags)};
}

Result<FeatureResult> SketchFeature::compute(const EvalContext& ctx) const {
    auto sol = solve(ctx);
    if (!sol) return std::unexpected(sol.error());
    FeatureResult out;
    out.diagnostics = std::move(sol->diagnostics);
    out.attributes["dof"] = std::to_string(sol->report.dof);
    out.attributes["profiles"] = std::to_string(sol->profiles.size());
    return out;
}

// ---- references and registration -------------------------------------------------------------------------------

std::optional<FeatureId> sketchReferenceOf(const Feature& f) {
    if (const auto* e = dynamic_cast<const ExtrudeFeature*>(&f)) {
        if (const auto* ref = std::get_if<SketchReferenceSource>(&e->params().source)) return ref->sketch;
    } else if (const auto* r = dynamic_cast<const RevolveFeature*>(&f)) {
        if (const auto* ref = std::get_if<SketchReferenceSource>(&r->params().source)) return ref->sketch;
    }
    return std::nullopt;
}

std::shared_ptr<Feature> embedSketchReference(const Feature& f, const sketch::Sketch& solved) {
    if (const auto* e = dynamic_cast<const ExtrudeFeature*>(&f)) {
        if (const auto* ref = std::get_if<SketchReferenceSource>(&e->params().source)) {
            ExtrudeParams p = e->params();
            p.source = SketchSource{solved, ref->profile};
            return std::make_shared<ExtrudeFeature>(e->id(), std::move(p));
        }
    } else if (const auto* r = dynamic_cast<const RevolveFeature*>(&f)) {
        if (const auto* ref = std::get_if<SketchReferenceSource>(&r->params().source)) {
            RevolveParams p = r->params();
            p.source = SketchSource{solved, ref->profile};
            return std::make_shared<RevolveFeature>(r->id(), std::move(p));
        }
    }
    return nullptr;
}

void registerSketchFeature(FeatureRegistry& registry) {
    registry.add("sketch", [](FeatureId id, std::string_view params) { return SketchFeature::fromJson(id, params); });
}

} // namespace iw::features
