// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Internal helpers shared by the feature generators (not part of the fuzz library's public interface).

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "fuzz/generator.hpp"

namespace iw::fuzz::util {

struct Pick {
    BodyId body;
    naming::ElementName name;
    std::uint32_t index = 0;
};

/// Random named element of `type` over all bodies of the state. `accept` filters (index, name, signature).
template <class Filter>
std::optional<Pick> pickElement(Rng& rng, const GenContext& ctx, naming::ElementType type, Filter&& accept) {
    std::vector<Pick> all;
    for (const auto& [body, shape] : ctx.state.bodies)
        for (const auto& [name, idx] : ctx.state.namedElements(body, type)) {
            if (accept(body, shape, idx, name)) all.push_back({body, name, idx});
        }
    if (all.empty()) return std::nullopt;
    return all[rng.below(all.size())];
}

inline naming::ElementRef refOf(const GenContext& ctx, const Pick& p) {
    return naming::ElementRef{ctx.writerOf(p.body), p.name.type, p.name};
}

/// Name tagged by a feature op prefix ("hole.", "fillet.", ...).
inline bool opStartsWith(const naming::ElementName& n, std::string_view prefix) {
    return n.op.compare(0, prefix.size(), prefix) == 0;
}

/// Round to 0.1.
inline double round1(double v) { return std::round(v * 10.0) / 10.0; }

} // namespace iw::fuzz::util
