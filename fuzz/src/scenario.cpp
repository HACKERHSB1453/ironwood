// SPDX-License-Identifier: GPL-3.0-or-later
#include "fuzz/scenario.hpp"

#include <algorithm>
#include <set>

#include "recompute/feature.hpp"

namespace iw::fuzz {
namespace {

std::vector<BodyId> bodiesOf(const recompute::Timeline& t) {
    std::set<BodyId> s;
    for (std::size_t i = 0; i < t.size(); ++i)
        for (BodyId b : t.at(i)->writesBodies()) s.insert(b);
    return {s.begin(), s.end()};
}

std::vector<FeatureId> idsOf(const recompute::Timeline& t) {
    std::vector<FeatureId> ids;
    for (std::size_t i = 0; i < t.size(); ++i) ids.push_back(t.at(i)->id());
    return ids;
}

/// A feature from a random registered generator, id `id`; tries every generator from a random start.
std::shared_ptr<const recompute::Feature> makeFeature(Rng& rng, FeatureId id, const recompute::Timeline& t,
                                                      const std::vector<const FeatureGenerator*>& gens) {
    if (gens.empty()) return nullptr;
    const GenContext ctx{t, bodiesOf(t)};
    const std::size_t start = rng.below(gens.size());
    for (std::size_t k = 0; k < gens.size(); ++k)
        if (auto f = gens[(start + k) % gens.size()]->generate(rng, id, ctx)) return f;
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
    std::uint64_t nextId = 1;
    const int nBase = rng.intIn(1, maxFeatures);
    for (int i = 0; i < nBase; ++i) {
        auto f = makeFeature(rng, FeatureId{nextId}, tl, gens);
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
            if (roll < 30) {
                if (size >= static_cast<std::size_t>(maxFeatures)) continue;
                st.kind = StepKind::Insert;
                st.position = rng.below(size + 1);
                st.feature = makeFeature(rng, FeatureId{nextId}, tl, gens);
                if (!st.feature) continue;
            } else if (roll < 50) {
                st.kind = StepKind::EditDimension;
                const auto& f = tl.at(rng.below(size));
                const FeatureGenerator* g = registry.find(f->kind());
                if (!g) continue;
                st.feature = g->editDimension(rng, *f);
                if (!st.feature) continue;
                st.target = f->id();
            } else if (roll < 62) {
                st.kind = StepKind::Remove;
                st.target = tl.at(rng.below(size))->id();
            } else if (roll < 72) {
                std::vector<FeatureId> cand;
                for (FeatureId id : idsOf(tl))
                    if (!tl.isSuppressed(id)) cand.push_back(id);
                if (cand.empty()) continue;
                st.kind = StepKind::Suppress;
                st.target = cand[rng.below(cand.size())];
            } else if (roll < 82) {
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
