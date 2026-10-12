// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "core/ids.hpp"
#include "core/result.hpp"
#include "fuzz/generator.hpp"
#include "naming/resolve.hpp"
#include "recompute/feature.hpp"
#include "recompute/timeline.hpp"

namespace iw::fuzz {

/// One timeline edit. Payload features are stored concretely, so a scenario replays bit-for-bit without the RNG.
enum class StepKind { Insert, EditDimension, Remove, Suppress, Unsuppress, Move };

struct Step {
    StepKind kind = StepKind::Insert;
    FeatureId target;                                   ///< Existing feature the step acts on (not used by Insert).
    std::size_t position = 0;                           ///< Insert / Move: destination index.
    std::shared_ptr<const recompute::Feature> feature;  ///< Insert payload / EditDimension replacement.
};

/// A complete fuzz case: a base timeline built in order, then a list of edits applied one at a time.
struct Scenario {
    std::uint64_t seed = 0;
    std::vector<std::shared_ptr<const recompute::Feature>> base;
    std::vector<Step> steps;
};

struct ScenarioConfig {
    int maxFeatures = 4; ///< Upper bound on timeline length at any point of a scenario.
    int maxSteps = 6;    ///< Upper bound on the number of edits per scenario.
};

/// Builds a scenario deterministically from `seed`. Generators are chosen uniformly among those registered.
Scenario generateScenario(std::uint64_t seed, const GeneratorRegistry& registry, const ScenarioConfig& config);

/// Applies one step to `timeline`. Errors: NotFound (target missing), InvalidArgument (bad payload).
Result<void> applyStep(recompute::Timeline& timeline, const Step& step);

/// Human-readable reproducer: seed, base features and steps with their JSON parameters.
std::string describe(const Scenario& scenario);

/// Pure function of the scenario: removes steps and base features while `keepsFailing` still holds.
/// Greedy and bounded by `maxRuns` evaluations of the predicate.
Scenario shrinkScenario(const Scenario& scenario, const std::function<bool(const Scenario&)>& keepsFailing,
                        int maxRuns);

} // namespace iw::fuzz
