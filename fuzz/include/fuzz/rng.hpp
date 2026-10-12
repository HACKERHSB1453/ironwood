// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace iw::fuzz {

/// Deterministic, platform-independent PRNG (SplitMix64). std::*_distribution output is not portable across
/// standard libraries, so every draw the fuzzer makes goes through this type; a seed then reproduces a scenario
/// bit-for-bit on any machine.
class Rng {
public:
    explicit Rng(std::uint64_t seed) noexcept : state_(seed) {}

    std::uint64_t next() noexcept {
        std::uint64_t z = (state_ += 0x9e3779b97f4a7c15ULL);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }
    /// Uniform in [0, n); 0 when n == 0.
    std::uint64_t below(std::uint64_t n) noexcept { return n == 0 ? 0 : next() % n; }
    /// Uniform in [lo, hi] (inclusive) for integers.
    int intIn(int lo, int hi) noexcept {
        return lo + static_cast<int>(below(static_cast<std::uint64_t>(hi - lo + 1)));
    }
    /// Uniform in [0, 1).
    double unit() noexcept { return static_cast<double>(next() >> 11) * 0x1.0p-53; }
    /// Uniform in [lo, hi).
    double real(double lo, double hi) noexcept { return lo + (hi - lo) * unit(); }
    bool chance(double p) noexcept { return unit() < p; }

private:
    std::uint64_t state_;
};

/// Seed of scenario `index` of a campaign started with `master`. Stable: scenario i of seed s is always the same.
std::uint64_t deriveSeed(std::uint64_t master, std::uint64_t index) noexcept;

} // namespace iw::fuzz
