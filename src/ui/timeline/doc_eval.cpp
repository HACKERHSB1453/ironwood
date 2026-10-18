// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/timeline/doc_eval.hpp"

#include <chrono>
#include <memory>

#include "features/sketch_feature.hpp"
#include "kernel/ops.hpp"

namespace iw::ui::timeline {
namespace {

using recompute::Feature;
using recompute::FeatureStatus;

double elapsedMs(std::chrono::steady_clock::time_point since) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - since).count();
}

} // namespace

Result<features::DocumentEvaluation> evaluateTimeline(const recompute::Timeline& tl, const ParameterTable& params,
                                                      recompute::Evaluator& evaluator, CancelToken cancel) {
    const std::size_t marker = tl.rollbackMarker();
    features::DocumentEvaluation out;

    // Pass 1 (as features::Document::evaluate): solve each active sketch against the state before it and embed the
    // solved sketch into the features that reference it.
    recompute::Timeline resolved;
    for (std::size_t k = 0; k < tl.size(); ++k) {
        const std::shared_ptr<const Feature>& f = tl.at(k);
        const bool suppressed = tl.isSuppressed(f->id());
        const bool active = k < marker && !suppressed;
        if (active) {
            if (const auto* sf = dynamic_cast<const features::SketchFeature*>(f.get())) {
                auto before = evaluator.evaluate(resolved, &params, cancel);
                if (!before) return std::unexpected(before.error());
                std::map<FeatureId, recompute::ConstructionGeometry> cons;
                for (const auto& s : before->statuses)
                    if (s.state == FeatureStatus::State::Ok && s.construction) cons.emplace(s.id, *s.construction);
                recompute::EvalContext ctx{before->bodies, before->namings, cancel, &params, &cons};
                auto sol = sf->solve(ctx);
                if (!sol) {
                    if (sol.error().code == ErrorCode::Cancelled) return std::unexpected(sol.error());
                    // A failed solve is reported by the sketch's own status; dependants see no profile.
                } else {
                    out.sketches.insert_or_assign(f->id(), std::move(*sol));
                }
            }
        }
        std::shared_ptr<const Feature> entry = f;
        if (active) {
            if (auto ref = features::sketchReferenceOf(*f)) {
                auto it = out.sketches.find(*ref);
                if (it != out.sketches.end())
                    if (auto emb = features::embedSketchReference(*f, it->second.sketch)) entry = std::move(emb);
            }
        }
        resolved.append(std::move(entry));
        if (suppressed) resolved.setSuppressed(f->id(), true);
    }
    if (marker < tl.size()) resolved.setRollbackMarker(marker);

    // Pass 2: the evaluator (prefix reuse through its cache).
    auto res = evaluator.evaluate(resolved, &params, cancel);
    if (!res) return std::unexpected(res.error());
    out.statuses = std::move(res->statuses);
    out.bodies = std::move(res->bodies);
    out.namings = std::move(res->namings);
    out.computedCount = res->computedCount;
    out.reusedCount = res->reusedCount;
    for (const FeatureStatus& s : out.statuses)
        if (s.state == FeatureStatus::State::Ok && s.construction) out.construction.emplace(s.id, *s.construction);

    // Features after the rollback marker are not evaluated; still one status per feature.
    for (std::size_t k = marker; k < tl.size(); ++k) {
        FeatureStatus s;
        s.id = tl.at(k)->id();
        const bool suppressed = tl.isSuppressed(s.id);
        s.state = suppressed ? FeatureStatus::State::Suppressed : FeatureStatus::State::Skipped;
        if (!suppressed)
            s.diagnostics.push_back({recompute::Diagnostic::Severity::Info, "not evaluated: after the rollback marker", std::nullopt});
        out.statuses.push_back(std::move(s));
    }
    return out;
}

Result<EvalOutput> evaluateAndTessellate(const recompute::Timeline& tl, const ParameterTable& params,
                                         recompute::Evaluator& evaluator, CancelToken cancel) {
    EvalOutput out;
    const auto t0 = std::chrono::steady_clock::now();
    auto eval = evaluateTimeline(tl, params, evaluator, cancel);
    if (!eval) return std::unexpected(eval.error());
    out.eval = std::move(*eval);
    out.evalMs = elapsedMs(t0);

    const auto t1 = std::chrono::steady_clock::now();
    const tess::TessParams tp{};
    for (const auto& [id, shape] : out.eval.bodies) {
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "evaluation cancelled");
        auto mesh = tess::tessellate(shape, tp, cancel);
        if (!mesh) return std::unexpected(mesh.error());
        out.meshes.emplace(id, std::make_shared<const tess::MeshData>(std::move(*mesh)));
    }
    out.tessellateMs = elapsedMs(t1);
    return out;
}

std::optional<double> volumeOfBody(const features::DocumentEvaluation& eval, BodyId body) {
    auto it = eval.bodies.find(body);
    if (it == eval.bodies.end()) return std::nullopt;
    auto mp = kernel::massProperties(it->second);
    if (!mp) return std::nullopt;
    return mp->volume;
}

} // namespace iw::ui::timeline
