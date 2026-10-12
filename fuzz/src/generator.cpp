// SPDX-License-Identifier: GPL-3.0-or-later
#include "fuzz/generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace iw::fuzz {

bool sameGeometry(const GeomSig& a, const GeomSig& b) {
    if (a.type != b.type) return false;
    constexpr double kCentroidTol = 1e-4; // mm
    for (std::size_t i = 0; i < 3; ++i)
        if (std::abs(a.key[i] - b.key[i]) > kCentroidTol) return false;
    const double m = std::abs(a.key[3] - b.key[3]);
    return m <= 1e-6 + 1e-6 * std::max(std::abs(a.key[3]), std::abs(b.key[3]));
}

std::string describe(const GeomSig& s) {
    static constexpr const char* kTypes[] = {"F", "E", "V"};
    char buf[160];
    std::snprintf(buf, sizeof buf, "%s(%.6g, %.6g, %.6g; m=%.6g)", kTypes[static_cast<int>(s.type)], s.key[0],
                  s.key[1], s.key[2], s.key[3]);
    return buf;
}

void GeneratorRegistry::add(std::shared_ptr<const FeatureGenerator> generator) {
    if (!generator) return;
    std::string kind(generator->kind());
    byKind_[std::move(kind)] = std::move(generator);
}

const FeatureGenerator* GeneratorRegistry::find(std::string_view kind) const {
    auto it = byKind_.find(kind);
    return it == byKind_.end() ? nullptr : it->second.get();
}

std::vector<const FeatureGenerator*> GeneratorRegistry::all() const {
    std::vector<const FeatureGenerator*> out;
    out.reserve(byKind_.size());
    for (const auto& [kind, gen] : byKind_) out.push_back(gen.get());
    return out;
}

GeneratorRegistry GeneratorRegistry::defaults() {
    GeneratorRegistry reg;
    reg.add(makeExtrudeGenerator());
    return reg;
}

} // namespace iw::fuzz
