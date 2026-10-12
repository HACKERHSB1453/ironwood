// SPDX-License-Identifier: GPL-3.0-or-later
#include "fuzz/runner.hpp"

#include <algorithm>
#include <chrono>
#include <optional>
#include <set>

#include <nlohmann/json.hpp>

#include "recompute/evaluator.hpp"
#include "recompute/feature.hpp"
#include "recompute/timeline.hpp"

namespace iw::fuzz {
namespace {

using recompute::Evaluator;
using recompute::EvalResult;
using recompute::FeatureStatus;
using recompute::Timeline;
using naming::ElementType;

constexpr std::array<ElementType, 3> kTypes{ElementType::Face, ElementType::Edge, ElementType::Vertex};

/// One captured reference: a name the owner feature assigned to an element of its own result, plus the element's
/// geometry before the edit.
struct Captured {
    naming::ElementRef ref;
    BodyId body;
    GeomSig sig;
    std::size_t pos = 0; ///< Owner position in the pre-edit timeline.
};

GeomSig sigOf(const kernel::Shape& shape, ElementType t, std::uint32_t index) {
    return GeomSig{t, shape.subShapeKey(naming::toSubShapeKind(t), index)};
}

const char* qualityName(naming::ResolveQuality q) {
    switch (q) {
    case naming::ResolveQuality::Exact: return "Exact";
    case naming::ResolveQuality::HistoryDerived: return "HistoryDerived";
    case naming::ResolveQuality::GeometricFallback: return "GeometricFallback";
    }
    return "?";
}

std::string nameText(const naming::ElementRef& ref) {
    return "F" + std::to_string(ref.feature.value()) + " " + ref.name.toString();
}

/// Timeline copy evaluated up to (not including) `upto`.
Result<EvalResult> evaluatePrefix(Evaluator& ev, const Timeline& t, std::size_t upto) {
    Timeline c = t;
    c.setRollbackMarker(upto);
    return ev.evaluate(c);
}

/// True when features [0, upto] are identical (id, kind, parameters, suppression) in both timelines. The geometry
/// of an element of feature k depends only on that prefix, so an identical prefix means an unchanged result.
bool prefixIdentical(const Timeline& a, const Timeline& b, std::size_t upto) {
    if (a.size() <= upto || b.size() <= upto) return false;
    for (std::size_t k = 0; k <= upto; ++k) {
        const auto& fa = a.at(k);
        const auto& fb = b.at(k);
        if (fa->id() != fb->id() || fa->kind() != fb->kind() || fa->paramHash() != fb->paramHash()) return false;
        if (a.isSuppressed(fa->id()) != b.isSuppressed(fb->id())) return false;
    }
    return true;
}

std::optional<std::size_t> positionOf(const Timeline& t, FeatureId id) {
    for (std::size_t k = 0; k < t.size(); ++k)
        if (t.at(k)->id() == id) return k;
    return std::nullopt;
}

bool anyMatch(const std::vector<GeomSig>& candidates, const GeomSig& got) {
    return std::any_of(candidates.begin(), candidates.end(), [&](const GeomSig& c) { return sameGeometry(c, got); });
}

std::string candidatesText(const std::vector<GeomSig>& candidates) {
    std::string out;
    const std::size_t shown = std::min<std::size_t>(candidates.size(), 3);
    for (std::size_t i = 0; i < shown; ++i) out += (i ? ", " : "") + describe(candidates[i]);
    if (candidates.size() > shown) out += ", ... (" + std::to_string(candidates.size()) + " candidates)";
    return out;
}

/// Checks one edit (or the baseline when before == after). Captures refs on `before`, resolves them on `after`.
void checkStep(Evaluator& ev, const Timeline& before, const Timeline& after, int stepIndex,
               const GeneratorRegistry& reg, const ResolveFn& resolve, ScenarioOutcome& out) {
    auto fail = [&](std::string category, std::string detail) {
        out.failures.push_back({std::move(category), stepIndex, std::move(detail)});
    };

    auto preFull = ev.evaluate(before);
    auto postFull = ev.evaluate(after);
    if (!preFull || !postFull) {
        fail("internal_evaluation_error", !preFull ? preFull.error().message : postFull.error().message);
        return;
    }
    for (const auto& st : postFull->statuses) {
        if (st.state == FeatureStatus::State::Failed && st.error.code == ErrorCode::Internal) {
            ++out.counts.internalFeatureErrors;
            out.internalErrors.push_back("F" + std::to_string(st.id.value()) + ": " + st.error.message);
        }
    }

    // 1. Capture every name the surviving features assign, with the geometry it names.
    std::vector<Captured> caps;
    for (std::size_t p = 0; p < before.size(); ++p) {
        if (preFull->statuses[p].state != FeatureStatus::State::Ok) continue;
        const auto& f = before.at(p);
        auto st = evaluatePrefix(ev, before, p + 1);
        if (!st) continue;
        for (BodyId b : f->writesBodies()) {
            auto shapeIt = st->bodies.find(b);
            auto nameIt = st->namings.find(b);
            if (shapeIt == st->bodies.end() || nameIt == st->namings.end()) continue;
            const kernel::Shape& shape = shapeIt->second;
            for (ElementType t : kTypes) {
                const auto count = shape.subShapeCount(naming::toSubShapeKind(t));
                if (nameIt->second.size(t) != count) {
                    fail("naming_size_mismatch", "F" + std::to_string(f->id().value()) + " body " +
                                                     std::to_string(b.value()) + ": " +
                                                     std::to_string(nameIt->second.size(t)) + " name slots for " +
                                                     std::to_string(count) + " sub-shapes");
                    continue;
                }
                for (std::uint32_t i = 0; i < count; ++i) {
                    auto name = nameIt->second.nameOf(t, i);
                    if (!name) continue;
                    caps.push_back({{f->id(), t, *name}, b, sigOf(shape, t, i), p});
                }
            }
        }
    }
    out.counts.refsCaptured += caps.size();

    // 2. Pre-state self-check: the generator's prediction must agree with the geometry the kernel produced.
    for (const auto& c : caps) {
        const auto& fb = *before.at(c.pos);
        const FeatureGenerator* g = reg.find(fb.kind());
        if (!g) continue;
        auto pred = g->predict(fb, c.ref.name);
        if (!pred) continue;
        ++out.counts.predictedChecks;
        if (!anyMatch(*pred, c.sig))
            fail("pre_state_inconsistent", nameText(c.ref) + " lies at " + describe(c.sig) +
                                               " before the edit; prediction: " + candidatesText(*pred));
    }

    // 3. Post-edit resolution.
    for (const auto& c : caps) {
        auto q = positionOf(after, c.ref.feature);
        if (!q) {
            ++out.counts.skippedOwnerGone;
            continue;
        }
        if (after.isSuppressed(c.ref.feature)) {
            ++out.counts.skippedOwnerInactive;
            continue;
        }
        const bool affected = !prefixIdentical(before, after, *q);
        auto post = evaluatePrefix(ev, after, *q + 1);
        if (!post) continue;
        if (post->statuses[*q].state != FeatureStatus::State::Ok) {
            ++out.counts.skippedOwnerInactive;
            continue;
        }
        auto nameIt = post->namings.find(c.body);
        auto shapeIt = post->bodies.find(c.body);
        if (nameIt == post->namings.end() || shapeIt == post->bodies.end()) {
            ++out.counts.skippedOwnerGone;
            continue;
        }
        ++out.counts.refsChecked;

        auto res = resolve(c.ref, nameIt->second);
        if (!res) {
            const auto& e = res.error();
            if (e.code == ErrorCode::NotFound || e.code == ErrorCode::Ambiguous) {
                if (e.message.empty()) fail("missing_diagnostic", nameText(c.ref) + " was " + errorCodeName(e.code) + " without a message");
                else {
                    ++out.counts.diagnosed;
                    if (!affected) ++out.counts.stabilityLosses;
                }
            } else {
                fail("resolver_error", nameText(c.ref) + " returned " + errorCodeName(e.code) + ": " + e.message);
            }
            continue;
        }
        const naming::Resolution& r = *res;
        if (r.quality == naming::ResolveQuality::Exact) ++out.counts.resolvedExact;
        else ++out.counts.resolvedHistory;
        if (r.quality != naming::ResolveQuality::Exact && r.warnings.empty())
            fail("missing_warning", nameText(c.ref) + " resolved without a warning");

        const auto subCount = shapeIt->second.subShapeCount(naming::toSubShapeKind(c.ref.type));
        if (r.index >= subCount) {
            fail("index_out_of_range", nameText(c.ref) + " resolved to index " + std::to_string(r.index) + " of " +
                                           std::to_string(subCount));
            continue;
        }
        const GeomSig got = sigOf(shapeIt->second, c.ref.type, r.index);

        const FeatureGenerator* g = reg.find(after.at(*q)->kind());
        auto pred = g ? g->predict(*after.at(*q), c.ref.name) : std::nullopt;
        if (pred) {
            ++out.counts.predictedChecks;
            if (!anyMatch(*pred, got))
                fail("wrong_pick_predicted", nameText(c.ref) + " resolved (" + qualityName(r.quality) +
                                                 ") to index " + std::to_string(r.index) + " at " + describe(got) +
                                                 "; expected one of " + candidatesText(*pred));
        } else if (!affected) {
            ++out.counts.unaffectedChecks;
            if (!sameGeometry(got, c.sig))
                fail("wrong_pick_unaffected", nameText(c.ref) + " was " + describe(c.sig) +
                                                  " before an edit that did not touch its feature prefix; resolved (" +
                                                  qualityName(r.quality) + ") to index " +
                                                  std::to_string(r.index) + " at " + describe(got));
        } else {
            ++out.counts.unverifiedMoved;
        }
    }
}

} // namespace

Counters& Counters::operator+=(const Counters& o) {
    stepsChecked += o.stepsChecked;
    refsCaptured += o.refsCaptured;
    refsChecked += o.refsChecked;
    resolvedExact += o.resolvedExact;
    resolvedHistory += o.resolvedHistory;
    diagnosed += o.diagnosed;
    predictedChecks += o.predictedChecks;
    unaffectedChecks += o.unaffectedChecks;
    unverifiedMoved += o.unverifiedMoved;
    stabilityLosses += o.stabilityLosses;
    skippedOwnerGone += o.skippedOwnerGone;
    skippedOwnerInactive += o.skippedOwnerInactive;
    internalFeatureErrors += o.internalFeatureErrors;
    return *this;
}

ScenarioOutcome runScenario(const Scenario& scenario, const GeneratorRegistry& registry, const ResolveFn& resolve) {
    ScenarioOutcome out;
    Evaluator ev;
    Timeline tl;
    for (const auto& f : scenario.base) tl.append(f);

    // Baseline: an unedited timeline must resolve every name back to the same geometry.
    checkStep(ev, tl, tl, -1, registry, resolve, out);

    for (std::size_t k = 0; k < scenario.steps.size(); ++k) {
        Timeline after = tl;
        if (auto r = applyStep(after, scenario.steps[k]); !r) continue; // inapplicable step: a no-op
        ++out.counts.stepsChecked;
        checkStep(ev, tl, after, static_cast<int>(k), registry, resolve, out);
        tl = std::move(after);
    }
    return out;
}

FuzzSummary runFuzz(const FuzzConfig& config, const GeneratorRegistry& registry, const ResolveFn& resolve) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - start).count(); };

    FuzzSummary summary;
    std::set<std::string> internal;
    const std::size_t total = config.replaySingle ? 1 : config.scenarios;
    for (std::size_t i = 0; i < total; ++i) {
        if (config.timeBudgetSeconds > 0 && elapsed() >= config.timeBudgetSeconds) {
            summary.timedOut = true;
            break;
        }
        const std::uint64_t seed = config.replaySingle ? config.replaySeed : deriveSeed(config.seed, i);
        const Scenario sc = generateScenario(seed, registry, config.scenario);
        const ScenarioOutcome out = runScenario(sc, registry, resolve);
        ++summary.scenariosRun;
        summary.counts += out.counts;
        for (const auto& f : out.failures) ++summary.failureCategories[f.category];
        for (const auto& m : out.internalErrors)
            if (internal.size() < 20) internal.insert(m);

        if (out.failures.empty()) continue;
        ++summary.failingScenarios;
        const std::string category = out.failures.front().category;
        const Scenario shrunk = shrinkScenario(
            sc,
            [&](const Scenario& c) {
                const auto o = runScenario(c, registry, resolve);
                return std::any_of(o.failures.begin(), o.failures.end(),
                                   [&](const Failure& f) { return f.category == category; });
            },
            config.shrinkRuns);

        FailedScenario fs;
        fs.scenarioSeed = seed;
        fs.index = i;
        fs.category = category;
        fs.detail = out.failures.front().detail;
        fs.originalSteps = sc.steps.size();
        fs.shrunkSteps = shrunk.steps.size();
        fs.reproducer = describe(shrunk);
        summary.failures.push_back(std::move(fs));
    }
    summary.internalErrors.assign(internal.begin(), internal.end());
    summary.elapsedSeconds = elapsed();
    return summary;
}

std::string toJson(const FuzzConfig& config, const FuzzSummary& s) {
    using nlohmann::json;
    json j;
    j["master_seed"] = config.seed;
    j["replay"] = config.replaySingle;
    j["scenarios_requested"] = config.replaySingle ? 1 : config.scenarios;
    j["scenarios_run"] = s.scenariosRun;
    j["max_features"] = config.scenario.maxFeatures;
    j["max_steps"] = config.scenario.maxSteps;
    j["time_budget_s"] = config.timeBudgetSeconds;
    j["elapsed_s"] = s.elapsedSeconds;
    j["timed_out"] = s.timedOut;
    j["passed"] = s.failingScenarios == 0;
    j["failing_scenarios"] = s.failingScenarios;
    j["failure_categories"] = s.failureCategories;

    const Counters& c = s.counts;
    j["counters"] = {
        {"steps_checked", c.stepsChecked},
        {"refs_captured", c.refsCaptured},
        {"refs_checked", c.refsChecked},
        {"resolved_exact", c.resolvedExact},
        {"resolved_history_derived", c.resolvedHistory},
        {"diagnosed_not_found_or_ambiguous", c.diagnosed},
        {"predicted_checks", c.predictedChecks},
        {"unaffected_checks", c.unaffectedChecks},
        {"unverified_moved", c.unverifiedMoved},
        {"stability_losses_warning", c.stabilityLosses},
        {"skipped_owner_gone", c.skippedOwnerGone},
        {"skipped_owner_inactive", c.skippedOwnerInactive},
        {"internal_feature_errors", c.internalFeatureErrors},
    };

    json fails = json::array();
    for (const auto& f : s.failures) {
        fails.push_back({
            {"scenario_seed", f.scenarioSeed},
            {"index", f.index},
            {"category", f.category},
            {"detail", f.detail},
            {"steps_original", f.originalSteps},
            {"steps_minimal", f.shrunkSteps},
            {"reproducer", f.reproducer},
            {"replay_command", "ironwood-fuzz-naming --replay-seed " + std::to_string(f.scenarioSeed)},
        });
    }
    j["failures"] = std::move(fails);
    j["internal_errors"] = s.internalErrors;
    j["limitations"] = json::array({
        "Geometric signature = type + centroid + measure (kernel public API has no normal/axis accessor).",
        "Affected refs with no generator prediction are counted as unverified_moved, not checked.",
        "Prediction for extrude NewBody matches profile/extrude names by op+source tag; a swap between "
        "siblings of the same tag is only caught when the element is outside the candidate set.",
    });
    return j.dump(2);
}

} // namespace iw::fuzz
