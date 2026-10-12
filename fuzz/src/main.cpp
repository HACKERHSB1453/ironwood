// SPDX-License-Identifier: GPL-3.0-or-later
// ironwood-fuzz-naming: randomized check of persistent-name resolution across timeline edits (P1a gate, A6).
//
//   ironwood-fuzz-naming [--seed N] [--scenarios N] [--max-features N] [--max-steps N]
//                        [--time-budget SECONDS] [--nightly] [--replay-seed S] [--json PATH]
//
// Exit status: 0 = no oracle failures, 1 = oracle failures found, 2 = usage error.
// stdout receives the JSON summary (or nothing when --json PATH is given); stderr gets a human-readable digest
// with the minimal reproducer of every failing scenario.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

#include "fuzz/runner.hpp"

namespace {

void usage(std::FILE* f) {
    std::fprintf(f,
                 "usage: ironwood-fuzz-naming [--seed N] [--scenarios N] [--max-features N] [--max-steps N]\n"
                 "                            [--time-budget SECONDS] [--nightly] [--replay-seed S] [--json PATH]\n"
                 "  --seed N           master seed (default 1); scenario i uses deriveSeed(N, i)\n"
                 "  --scenarios N      number of scenarios (default 200; --nightly sets 5000)\n"
                 "  --max-features N   timeline length bound (default 4)\n"
                 "  --max-steps N      edits per scenario bound (default 6)\n"
                 "  --time-budget S    stop after S seconds of wall time (default: unlimited)\n"
                 "  --replay-seed S    run exactly the scenario with scenario seed S (from a failure report)\n"
                 "  --json PATH        write the JSON summary to PATH instead of stdout\n");
}

bool parseU64(const char* s, std::uint64_t& out) {
    if (!s || !*s) return false;
    char* end = nullptr;
    const unsigned long long v = std::strtoull(s, &end, 10);
    if (*end != '\0') return false;
    out = v;
    return true;
}

bool parseDouble(const char* s, double& out) {
    if (!s || !*s) return false;
    char* end = nullptr;
    const double v = std::strtod(s, &end);
    if (*end != '\0' || v < 0) return false;
    out = v;
    return true;
}

} // namespace

int main(int argc, char** argv) {
    iw::fuzz::FuzzConfig cfg;
    const char* jsonPath = nullptr;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
        std::uint64_t u = 0;
        double d = 0;
        if (std::strcmp(a, "--seed") == 0) {
            if (!parseU64(next(), cfg.seed)) return usage(stderr), 2;
        } else if (std::strcmp(a, "--scenarios") == 0) {
            if (!parseU64(next(), u) || u == 0) return usage(stderr), 2;
            cfg.scenarios = static_cast<std::size_t>(u);
        } else if (std::strcmp(a, "--max-features") == 0) {
            if (!parseU64(next(), u) || u == 0 || u > 64) return usage(stderr), 2;
            cfg.scenario.maxFeatures = static_cast<int>(u);
        } else if (std::strcmp(a, "--max-steps") == 0) {
            if (!parseU64(next(), u) || u == 0 || u > 1000) return usage(stderr), 2;
            cfg.scenario.maxSteps = static_cast<int>(u);
        } else if (std::strcmp(a, "--time-budget") == 0) {
            if (!parseDouble(next(), d)) return usage(stderr), 2;
            cfg.timeBudgetSeconds = d;
        } else if (std::strcmp(a, "--nightly") == 0) {
            cfg.scenarios = 5000;
        } else if (std::strcmp(a, "--replay-seed") == 0) {
            if (!parseU64(next(), cfg.replaySeed)) return usage(stderr), 2;
            cfg.replaySingle = true;
        } else if (std::strcmp(a, "--json") == 0) {
            jsonPath = next();
            if (!jsonPath) return usage(stderr), 2;
        } else if (std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0) {
            usage(stdout);
            return 0;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", a);
            usage(stderr);
            return 2;
        }
    }

    const auto registry = iw::fuzz::GeneratorRegistry::defaults();
    const auto summary = iw::fuzz::runFuzz(cfg, registry);
    const std::string json = iw::fuzz::toJson(cfg, summary);

    if (jsonPath) {
        std::ofstream out(jsonPath, std::ios::binary | std::ios::trunc);
        if (!out) {
            std::fprintf(stderr, "cannot write %s\n", jsonPath);
            return 2;
        }
        out << json << '\n';
    } else {
        std::printf("%s\n", json.c_str());
    }

    std::fprintf(stderr, "naming fuzz: %zu scenario(s) in %.1f s%s, %zu failing\n", summary.scenariosRun,
                 summary.elapsedSeconds, summary.timedOut ? " (time budget reached)" : "", summary.failingScenarios);
    for (const auto& f : summary.failures) {
        std::fprintf(stderr, "\nFAILURE [%s] scenario seed %llu (index %zu), minimal reproducer %zu of %zu edits:\n",
                     f.category.c_str(), static_cast<unsigned long long>(f.scenarioSeed), f.index, f.shrunkSteps,
                     f.originalSteps);
        std::fprintf(stderr, "  %s\n", f.detail.c_str());
        std::fprintf(stderr, "%s", f.reproducer.c_str());
        std::fprintf(stderr, "  replay: ironwood-fuzz-naming --replay-seed %llu\n",
                     static_cast<unsigned long long>(f.scenarioSeed));
    }
    return summary.failingScenarios == 0 ? 0 : 1;
}
