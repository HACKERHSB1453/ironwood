// SPDX-License-Identifier: GPL-3.0-or-later
#include "naming/resolve.hpp"

#include <string_view>

namespace iw::naming {

namespace {
bool endsWith(std::string_view s, std::string_view suf) {
    return s.size() >= suf.size() && s.substr(s.size() - suf.size()) == suf;
}
} // namespace

Result<Resolution> resolve(const ElementRef& ref, const NamingMap& map) {
    if (ref.name.type != ref.type)
        return makeError(ErrorCode::InvalidArgument, "resolve: ref.type differs from ref.name.type");

    auto exact = map.indicesOf(ref.name);
    if (exact.size() == 1) return Resolution{exact.front(), ResolveQuality::Exact, {}};
    if (exact.size() > 1)
        return makeError(ErrorCode::Ambiguous,
                         "resolve: name " + ref.name.toString() + " carries several elements (corrupt map)");

    // History-derived: direct split / merge successors recorded in the names themselves.
    std::vector<std::uint32_t> successors;
    const auto n = map.size(ref.type);
    for (std::uint32_t i = 0; i < n; ++i) {
        auto nm = map.nameOf(ref.type, i);
        if (!nm) continue;
        if (!endsWith(nm->op, ".split") && !endsWith(nm->op, ".merge") && !endsWith(nm->op, ".part")) continue;
        for (const auto& s : nm->sources)
            if (s == ref.name) {
                successors.push_back(i);
                break;
            }
    }
    if (successors.size() == 1) {
        Resolution r;
        r.index = successors.front();
        r.quality = ResolveQuality::HistoryDerived;
        r.warnings.push_back("reference " + ref.name.toString() + " rebound through history to " +
                             map.nameOf(ref.type, r.index)->toString());
        return r;
    }
    if (successors.size() > 1)
        return makeError(ErrorCode::Ambiguous, "resolve: " + ref.name.toString() + " has " +
                                                   std::to_string(successors.size()) +
                                                   " history successors; disambiguation required");
    // A ".part" name (one piece of a tool element that was split) whose pieces have merged back into the whole: the
    // source element itself carries the unsplit name.
    if (endsWith(ref.name.op, ".part") && ref.name.sources.size() == 1) {
        auto whole = map.indicesOf(ref.name.sources.front());
        if (whole.size() == 1) {
            Resolution r;
            r.index = whole.front();
            r.quality = ResolveQuality::HistoryDerived;
            r.warnings.push_back("reference " + ref.name.toString() + " rebound through history to " +
                                 map.nameOf(ref.type, r.index)->toString());
            return r;
        }
    }
    // Geometric fallback is out of scope for v0.
    return makeError(ErrorCode::NotFound, "resolve: no element named " + ref.name.toString());
}

} // namespace iw::naming
