// SPDX-License-Identifier: GPL-3.0-or-later
#include "fuzz/generator.hpp"

#include <algorithm>

namespace iw::fuzz {

FeatureId GenContext::writerOf(BodyId body) const {
    FeatureId out;
    for (std::size_t i = 0; i < timeline.size(); ++i) {
        const auto& f = timeline.at(i);
        for (BodyId b : f->writesBodies())
            if (b == body) out = f->id();
    }
    return out;
}

BodyId GenContext::freshBody() const {
    std::uint64_t top = 0;
    for (const auto& [id, shape] : state.bodies) top = std::max(top, id.value());
    for (BodyId b : bodies) top = std::max(top, b.value());
    return BodyId{std::max<std::uint64_t>(top, 9) + 1};
}

std::optional<std::uint32_t> PredictContext::indexOf(BodyId body, const naming::ElementName& name) const {
    auto n = in.namings.find(body);
    if (n == in.namings.end()) return std::nullopt;
    const auto exact = n->second.indicesOf(name);
    if (exact.size() == 1) return exact.front();
    if (!resolve) return std::nullopt;
    auto r = resolve(naming::ElementRef{FeatureId{}, name.type, name}, n->second);
    if (!r || r->index >= n->second.size(name.type)) return std::nullopt;
    return r->index;
}

std::optional<GeomSig> PredictContext::lookup(BodyId body, const naming::ElementName& name) const {
    auto b = in.bodies.find(body);
    if (b == in.bodies.end()) return std::nullopt;
    auto idx = indexOf(body, name);
    if (!idx) return std::nullopt;
    return readSig(b->second, name.type, *idx);
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
    reg.add(makeRevolveGenerator());
    reg.add(makeAxisGenerator());
    reg.add(makeFilletGenerator());
    reg.add(makeChamferGenerator());
    reg.add(makeHoleGenerator());
    reg.add(makeMoveGenerator());
    return reg;
}

} // namespace iw::fuzz
