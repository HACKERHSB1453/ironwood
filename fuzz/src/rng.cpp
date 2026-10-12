// SPDX-License-Identifier: GPL-3.0-or-later
#include "fuzz/rng.hpp"

namespace iw::fuzz {

std::uint64_t deriveSeed(std::uint64_t master, std::uint64_t index) noexcept {
    Rng r(master ^ (index * 0x9e3779b97f4a7c15ULL + 0x632be59bd9b4e019ULL));
    return r.next();
}

} // namespace iw::fuzz
