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

/// Resolver under test. The default is naming::resolve; tests inject deliberately broken resolvers to prove the
/// oracle catches silent wrong picks.
using ResolveFn = std::function<Result<naming::Resolution>(const naming::ElementRef&, const naming::NamingMap&)>;

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

/// Per-run counters. `refs_checked` counts ElementRef resolutions attempted after an edit.
struct Counters {
    std::size_t stepsChecked = 0;
    std::size_t refsCaptured = 0;
    std::size_t refsChecked = 0;
    std::size_t resolvedExact = 0;
    std::size_t resolvedHistory = 0;
    std::size_t diagnosed = 0;            ///< NotFound/Ambiguous with a message (the accepted outcome).
    std::size_t predictedChecks = 0;      ///< Resolutions checked against a generator prediction.
    std::size_t unaffectedChecks = 0;     ///< Resolutions checked against the pre-edit geometry (prefix unchanged).
    std::size_t unverifiedMoved = 0;      ///< Affected refs with no prediction: geometry not checked (see report).
    std::size_t stabilityLosses = 0;      ///< Unaffected ref that became NotFound/Ambiguous (accepted, reported).
    std::size_t skippedOwnerGone = 0;     ///< Owner feature removed by the edit.
    std::size_t skippedOwnerInactive = 0; ///< Owner suppressed or failed after the edit.
    std::size_t internalFeatureErrors = 0;///< Features that failed with ErrorCode::Internal (naming bugs surface here).

    Counters& operator+=(const Counters& o);
};

struct ScenarioOutcome {
    Counters counts;
    std::vector<Failure> failures;
    std::vector<std::string> internalErrors; ///< Messages of Internal feature failures (not oracle failures).
};

/// Runs every edit of `scenario` as a checked step: capture refs before, edit, resolve after, classify.
ScenarioOutcome runScenario(const Scenario& scenario, const GeneratorRegistry& registry,
                            const ResolveFn& resolve = naming::resolve);

struct FuzzConfig {
    std::uint64_t seed = 1;         ///< Master seed; scenario i uses deriveSeed(seed, i).
    std::size_t scenarios = 200;
    ScenarioConfig scenario;
    double timeBudgetSeconds = 0;   ///< 0 = unlimited.
    int shrinkRuns = 150;           ///< Predicate evaluations spent shrinking each failing scenario.
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
};

FuzzSummary runFuzz(const FuzzConfig& config, const GeneratorRegistry& registry,
                    const ResolveFn& resolve = naming::resolve);

/// Machine-readable report (JSON object text).
std::string toJson(const FuzzConfig& config, const FuzzSummary& summary);

} // namespace iw::fuzz
