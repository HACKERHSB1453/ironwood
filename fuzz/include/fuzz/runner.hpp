// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "core/result.hpp"
#include "fuzz/generator.hpp"
#include "fuzz/scenario.hpp"
#include "naming/naming_map.hpp"
#include "naming/resolve.hpp"

namespace iw::fuzz {

/// Oracle failures. Each one is a naming bug (or an oracle bug to triage); none is a diagnosed NotFound.
///  wrong_pick_predicted  : resolved to an element whose geometry differs from the generator's prediction
///  wrong_pick_unaffected : feature prefix unchanged by the edit, yet the resolved element's geometry changed
///  pre_state_inconsistent: a name on the pre-edit model contradicts the generator's prediction for it
///  missing_warning       : HistoryDerived resolution without a warning
///  missing_diagnostic    : NotFound/Ambiguous without a message
///  index_out_of_range    : resolution points past the shape's sub-shape count
///  resolver_error        : resolve() returned an unexpected error code for a well-formed reference
struct Failure {
    std::string category;
    int step = -1; ///< Index of the edit being checked; -1 = baseline check of the unedited base timeline.
    std::string detail;
};

/// Counters of one feature kind (owner of the resolved reference).
struct KindCounters {
    std::size_t exact = 0;
    std::size_t history = 0;
    std::size_t diagnosed = 0;
    std::size_t predicted = 0;   ///< Resolutions checked against a prediction (own, inherited or derived from sources).
    std::size_t unaffected = 0;  ///< No prediction; checked against the pre-edit geometry (prefix unchanged).
    std::size_t unverified = 0;  ///< Moved by the edit and no prediction: geometry not checked.
    std::size_t failures = 0;
    KindCounters& operator+=(const KindCounters& o);
};

/// Per-run counters. `refsChecked` counts ElementRef resolutions attempted after an edit.
struct Counters {
    std::size_t stepsChecked = 0;
    std::size_t refsCaptured = 0;
    std::size_t refsChecked = 0;
    std::size_t resolvedExact = 0;
    std::size_t resolvedHistory = 0;
    std::size_t ambiguous = 0;            ///< Diagnosed Ambiguous (subset of `diagnosed`).
    std::size_t notFound = 0;             ///< Diagnosed NotFound (subset of `diagnosed`).
    std::size_t diagnosed = 0;            ///< NotFound/Ambiguous with a message (the accepted outcome).
    std::size_t predictedChecks = 0;      ///< Resolutions checked against a prediction.
    std::size_t unaffectedChecks = 0;     ///< Resolutions with no prediction, checked against the pre-edit geometry.
    std::size_t unverifiedMoved = 0;      ///< Affected refs with no prediction: geometry not checked (see report).
    std::size_t preStateChecks = 0;       ///< Predictions compared with the kernel geometry before the edit.
    std::size_t volumeChecks = 0;         ///< Volume-invariant checks of join / cut / hole / move results.
    std::size_t roleChecks = 0;           ///< Role-stability failures across an edit (counted as failures too).
    std::size_t advisoryRoleChanges = 0;  ///< Advisory role changes of '.new' names (not failures).
    std::size_t detachedFeatures = 0;     ///< Features that were Ok before an edit and failed on an unresolvable reference after it.
    std::size_t holeRefFeatures = 0;      ///< Features holding a reference to a hole-created element that were Ok before an edit (F27 population).
    std::size_t holeRefDetached = 0;      ///< ... of which detached (F27: fillet/chamfer of a hole edge lost after an edit).
    std::size_t legitLoss = 0;            ///< Detached references classified legit_loss: the edit itself removed the element (see runner.cpp).
    std::size_t legitLossHole = 0;        ///< ... of which hole-created references (F27 population classified legit).
    /// legit_loss by cause: "overwritten_body" (a body written over without deriving from it) or "consumed_element" (a
    /// fillet / chamfer / cut that ate the referenced element earlier in the timeline).
    std::map<std::string, std::size_t> legitLossByCause;
    std::size_t stabilityLosses = 0;      ///< Unaffected ref that became NotFound/Ambiguous (accepted, reported).
    std::size_t skippedOwnerGone = 0;     ///< Owner feature removed by the edit.
    std::size_t skippedOwnerInactive = 0; ///< Owner suppressed or failed after the edit.
    std::size_t internalFeatureErrors = 0;///< Features that failed with ErrorCode::Internal (naming bugs surface here).
    std::map<std::string, KindCounters> perKind;      ///< By owner feature kind.
    std::map<std::string, std::size_t> unverifiedBy;  ///< "kind op type" -> unverified resolutions (diagnostics).

    Counters& operator+=(const Counters& o);
    std::size_t resolutions() const { return resolvedExact + resolvedHistory; }
    std::size_t verified() const { return predictedChecks + unaffectedChecks; }
};

struct ScenarioOutcome {
    Counters counts;
    std::vector<Failure> failures;
    std::vector<std::string> internalErrors; ///< Messages of Internal feature failures (not oracle failures).
    std::vector<std::string> notes;          ///< Observations that are not failures (F27 detached references, ...).
};

/// Runs every edit of `scenario` as a checked step: capture refs before, edit, resolve after, classify.
ScenarioOutcome runScenario(const Scenario& scenario, const GeneratorRegistry& registry,
                            const ResolveFn& resolve = naming::resolve);

/// Development aid: the timeline after the base and after each step, with every feature's names, their geometry and
/// whether the oracle has a prediction for them.
std::string explainScenario(const Scenario& scenario, const GeneratorRegistry& registry, const ResolveFn& resolve = naming::resolve);

/// Same as runScenario but in a forked worker process, so a crash or hang of the geometry kernel is reported as an oracle
/// failure ("kernel_crash" / "kernel_hang") instead of killing the campaign. POSIX only.
ScenarioOutcome runScenarioIsolated(const Scenario& scenario, const GeneratorRegistry& registry,
                                    const ResolveFn& resolve = naming::resolve);

struct FuzzConfig {
    std::uint64_t seed = 1;         ///< Master seed; scenario i uses deriveSeed(seed, i).
    std::size_t scenarios = 200;
    ScenarioConfig scenario;
    double timeBudgetSeconds = 0;   ///< 0 = unlimited.
    int shrinkRuns = 150;           ///< Predicate evaluations spent shrinking each failing scenario.
    bool isolate = false;           ///< Run each scenario in a forked worker (survives kernel crashes).
    bool trace = false;             ///< Print each scenario seed to stderr before running it (crash hunting).
    bool replaySingle = false;      ///< Run exactly one scenario whose scenario seed is `replaySeed`.
    std::uint64_t replaySeed = 0;
};

struct FailedScenario {
    std::uint64_t scenarioSeed = 0;
    std::size_t index = 0;
    std::string category;
    std::string detail;
    std::size_t originalSteps = 0;
    std::size_t shrunkSteps = 0;
    std::string reproducer; ///< Minimal scenario (after shrinking), as text.
};

struct FuzzSummary {
    std::size_t scenariosRun = 0;
    double elapsedSeconds = 0;
    bool timedOut = false;
    Counters counts;
    std::size_t failingScenarios = 0;
    std::map<std::string, std::size_t> failureCategories; ///< Every oracle failure, counted by category.
    std::vector<FailedScenario> failures; ///< One entry per failing scenario (first failure's category).
    std::vector<std::string> internalErrors; ///< Distinct Internal feature-failure messages (capped at 20).
    std::vector<std::string> notes;          ///< Observations, with scenario seeds (capped at 20).
};

FuzzSummary runFuzz(const FuzzConfig& config, const GeneratorRegistry& registry,
                    const ResolveFn& resolve = naming::resolve);

/// Machine-readable report (JSON object text).
std::string toJson(const FuzzConfig& config, const FuzzSummary& summary);

} // namespace iw::fuzz
