// SPDX-License-Identifier: GPL-3.0-or-later
#include "fuzz/scenario.hpp"

#include <algorithm>
#include <set>

#include "recompute/evaluator.hpp"
#include "recompute/feature.hpp"

namespace iw::fuzz {
namespace {

std::vector<FeatureId> idsOf(const recompute::Timeline& t) {
    std::vector<FeatureId> ids;
    for (std::size_t i = 0; i < t.size(); ++i) ids.push_back(t.at(i)->id());
    return ids;
}

/// A feature from a random registered generator (weighted), id `id`, for insertion at `position`. The generator sees
/// the model state at that position, so it can name elements (edges to fillet, faces to drill) that exist there.
std::shared_ptr<const recompute::Feature> makeFeature(Rng& rng, FeatureId id, const recompute::Timeline& t,
                                                      std::size_t position, const std::vector<const FeatureGenerator*>& gens,
                                                      recompute::Evaluator& ev, std::string_view prefer = {}) {
    if (gens.empty()) return nullptr;
    recompute::Timeline prefix = t;
    prefix.setRollbackMarker(std::min(position, t.size()));
    ModelState state;
    if (auto r = ev.evaluate(prefix)) state = ModelState::from(*r);
    std::vector<BodyId> bodies;
    for (const auto& [b, shape] : state.bodies) bodies.push_back(b);
    const GenContext ctx{t, std::move(bodies), state};

    if (!prefer.empty()) {
        for (const auto* g : gens)
            if (g->kind() == prefer)
                if (auto f = g->generate(rng, id, ctx)) return f;
    }
    std::vector<const FeatureGenerator*> left = gens;
    while (!left.empty()) {
        int total = 0;
        for (const auto* g : left) total += std::max(1, g->weight());
        int roll = rng.intIn(0, total - 1);
        std::size_t chosen = 0;
        for (std::size_t i = 0; i < left.size(); ++i) {
            roll -= std::max(1, left[i]->weight());
            if (roll < 0) { chosen = i; break; }
        }
        if (auto f = left[chosen]->generate(rng, id, ctx)) return f;
        left.erase(left.begin() + static_cast<std::ptrdiff_t>(chosen));
    }
    return nullptr;
}

const char* stepName(StepKind k) {
    switch (k) {
    case StepKind::Insert: return "insert";
    case StepKind::EditDimension: return "edit";
    case StepKind::Remove: return "remove";
    case StepKind::Suppress: return "suppress";
    case StepKind::Unsuppress: return "unsuppress";
    case StepKind::Move: return "move";
    }
    return "?";
}

std::string idText(FeatureId id) { return "F" + std::to_string(id.value()); }

std::string featureText(const recompute::Feature& f) {
    return std::string(f.kind()) + " " + idText(f.id()) + " " + f.paramsJson();
}

} // namespace

Scenario generateScenario(std::uint64_t seed, const GeneratorRegistry& registry, const ScenarioConfig& config) {
    Scenario sc;
    sc.seed = seed;
    const auto gens = registry.all();
    if (gens.empty()) return sc;
    Rng rng(seed);
    const int maxFeatures = std::max(1, config.maxFeatures);

    recompute::Timeline tl;
    recompute::Evaluator ev;
    std::uint64_t nextId = 1;
    const int nBase = config.holeFocus ? std::max(3, rng.intIn(3, maxFeatures)) : rng.intIn(1, maxFeatures);
    static const char* kFocus[] = {"extrude", "hole", "fillet"};
    for (int i = 0; i < nBase; ++i) {
        std::string_view prefer;
        if (config.holeFocus && i < 3) prefer = kFocus[i];
        auto f = makeFeature(rng, FeatureId{nextId}, tl, tl.size(), gens, ev, prefer);
        if (!f) break;
        ++nextId;
        tl.append(f);
        sc.base.push_back(f);
    }

    const int nSteps = rng.intIn(1, std::max(1, config.maxSteps));
    for (int s = 0; s < nSteps; ++s) {
        for (int attempt = 0; attempt < 8; ++attempt) {
            Step st;
            const std::size_t size = tl.size();
            const std::uint64_t roll = rng.below(100);
            if (size == 0) break;
            if (roll < 38) {
                if (size >= static_cast<std::size_t>(maxFeatures)) continue;
                st.kind = StepKind::Insert;
                st.position = rng.below(size + 1);
                st.feature = makeFeature(rng, FeatureId{nextId}, tl, st.position, gens, ev, rng.chance(0.5) ? "extrude" : "");
                if (!st.feature) continue;
            } else if (roll < 56) {
                st.kind = StepKind::EditDimension;
                const auto& f = tl.at(rng.below(size));
                const FeatureGenerator* g = registry.find(f->kind());
                if (!g) continue;
                st.feature = g->editDimension(rng, *f);
                if (!st.feature) continue;
                st.target = f->id();
            } else if (roll < 66) {
                st.kind = StepKind::Remove;
                st.target = tl.at(rng.below(size))->id();
            } else if (roll < 74) {
                std::vector<FeatureId> cand;
                for (FeatureId id : idsOf(tl))
                    if (!tl.isSuppressed(id)) cand.push_back(id);
                if (cand.empty()) continue;
                st.kind = StepKind::Suppress;
                st.target = cand[rng.below(cand.size())];
            } else if (roll < 84) {
                std::vector<FeatureId> cand;
                for (FeatureId id : idsOf(tl))
                    if (tl.isSuppressed(id)) cand.push_back(id);
                if (cand.empty()) continue;
                st.kind = StepKind::Unsuppress;
                st.target = cand[rng.below(cand.size())];
            } else {
                st.kind = StepKind::Move;
                st.target = tl.at(rng.below(size))->id();
                st.position = rng.below(size);
            }
            if (st.kind == StepKind::Insert) nextId += 1;
            if (auto r = applyStep(tl, st); !r) continue;
            sc.steps.push_back(std::move(st));
            break;
        }
    }
    return sc;
}

Result<void> applyStep(recompute::Timeline& tl, const Step& step) {
    switch (step.kind) {
    case StepKind::Insert:
        if (!step.feature) return makeError(ErrorCode::InvalidArgument, "insert step without a feature");
        tl.insert(std::min(step.position, tl.size()), step.feature);
        return {};
    case StepKind::EditDimension:
        if (!step.feature || step.feature->id() != step.target)
            return makeError(ErrorCode::InvalidArgument, "edit step payload id differs from its target");
        return tl.replace(step.feature);
    case StepKind::Remove:
        return tl.remove(step.target);
    case StepKind::Suppress:
    case StepKind::Unsuppress:
        if (!tl.indexOf(step.target)) return makeError(ErrorCode::NotFound, "suppress target not in timeline");
        tl.setSuppressed(step.target, step.kind == StepKind::Suppress);
        return {};
    case StepKind::Move: {
        auto idx = tl.indexOf(step.target);
        if (!idx) return makeError(ErrorCode::NotFound, "move target not in timeline");
        auto f = tl.at(*idx);
        if (auto r = tl.remove(step.target); !r) return r;
        tl.insert(std::min(step.position, tl.size()), f);
        return {};
    }
    }
    return makeError(ErrorCode::Internal, "unknown step kind");
}

std::string describe(const Scenario& sc) {
    std::string out = "scenario seed=" + std::to_string(sc.seed) + "\n";
    out += "base (" + std::to_string(sc.base.size()) + " features):\n";
    for (std::size_t i = 0; i < sc.base.size(); ++i)
        out += "  [" + std::to_string(i) + "] " + featureText(*sc.base[i]) + "\n";
    out += "steps (" + std::to_string(sc.steps.size()) + "):\n";
    for (std::size_t i = 0; i < sc.steps.size(); ++i) {
        const Step& s = sc.steps[i];
        out += "  #" + std::to_string(i) + " " + stepName(s.kind);
        if (s.kind != StepKind::Insert) out += " " + idText(s.target);
        if (s.kind == StepKind::Insert || s.kind == StepKind::Move) out += " to index " + std::to_string(s.position);
        if (s.feature) out += ": " + featureText(*s.feature);
        out += "\n";
    }
    return out;
}

Scenario shrinkScenario(const Scenario& scenario, const std::function<bool(const Scenario&)>& keepsFailing,
                        int maxRuns) {
    Scenario cur = scenario;
    int runs = 0;
    bool progress = true;
    while (progress && runs < maxRuns) {
        progress = false;
        for (std::size_t i = cur.steps.size(); i-- > 0 && runs < maxRuns;) {
            Scenario c = cur;
            c.steps.erase(c.steps.begin() + static_cast<std::ptrdiff_t>(i));
            ++runs;
            if (keepsFailing(c)) {
                cur = std::move(c);
                progress = true;
            }
        }
        for (std::size_t i = cur.base.size(); i-- > 0 && runs < maxRuns;) {
            Scenario c = cur;
            c.base.erase(c.base.begin() + static_cast<std::ptrdiff_t>(i));
            ++runs;
            if (keepsFailing(c)) {
                cur = std::move(c);
                progress = true;
            }
        }
    }
    return cur;
}

} // namespace iw::fuzz
