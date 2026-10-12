// SPDX-License-Identifier: GPL-3.0-or-later
#include "naming/naming_map.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <map>
#include <optional>
#include <set>

#include "naming/detail.hpp"

namespace iw::naming {

void NamingMap::resize(std::uint32_t faces, std::uint32_t edges, std::uint32_t vertices) {
    tables_[0].assign(faces, std::nullopt);
    tables_[1].assign(edges, std::nullopt);
    tables_[2].assign(vertices, std::nullopt);
}

Result<void> NamingMap::set(ElementType type, std::uint32_t index, ElementName name) {
    auto& t = tables_[static_cast<int>(type)];
    if (index >= t.size()) return makeError(ErrorCode::InvalidArgument, "element index out of range");
    if (name.type != type) return makeError(ErrorCode::InvalidArgument, "name type does not match table");
    t[index] = std::move(name);
    return {};
}

std::uint32_t NamingMap::size(ElementType type) const noexcept {
    return static_cast<std::uint32_t>(tables_[static_cast<int>(type)].size());
}

std::optional<ElementName> NamingMap::nameOf(ElementType type, std::uint32_t index) const {
    const auto& t = tables_[static_cast<int>(type)];
    if (index >= t.size()) return std::nullopt;
    return t[index];
}

std::vector<std::uint32_t> NamingMap::indicesOf(const ElementName& name) const {
    std::vector<std::uint32_t> out;
    const auto& t = tables_[static_cast<int>(name.type)];
    for (std::uint32_t i = 0; i < t.size(); ++i)
        if (t[i] && *t[i] == name) out.push_back(i);
    return out;
}

namespace detail {

namespace {

enum class Cls { Unchanged = 0, Modified = 1, Generated = 2 };

struct Origin {
    ElementName src;
    Cls cls;
    kernel::SubShapeKind inKind;
    bool split; // modified, and the input fans out to >1 outputs of this output kind
};

ElementType toElementType(kernel::SubShapeKind k) noexcept {
    switch (k) {
    case kernel::SubShapeKind::Face: return ElementType::Face;
    case kernel::SubShapeKind::Edge: return ElementType::Edge;
    case kernel::SubShapeKind::Vertex: return ElementType::Vertex;
    }
    return ElementType::Face;
}

const char* roleOf(kernel::SubShapeKind in, ElementType out) noexcept {
    using K = kernel::SubShapeKind;
    if (out == ElementType::Face) {
        if (in == K::Edge) return "side";
        if (in == K::Face) return "cap";
        return "face";
    }
    return out == ElementType::Edge ? "edge" : "vertex";
}

} // namespace

Result<NamingMap> deriveFromCounts(FeatureId feature, std::string_view opTag,
                                   const std::array<std::uint32_t, 3>& counts,
                                   const std::vector<const NamingMap*>& inputs,
                                   const kernel::ShapeHistory& history, const SiblingKeyFn& keyFn) {
    if (!feature.valid()) return makeError(ErrorCode::InvalidArgument, "deriveNamingMap: invalid feature id");
    if (opTag.empty()) return makeError(ErrorCode::InvalidArgument, "deriveNamingMap: empty op tag");

    std::vector<std::vector<Origin>> origins[3];
    for (int t = 0; t < 3; ++t) origins[t].resize(counts[static_cast<std::size_t>(t)]);

    Result<void> err{};
    auto inputName = [&](const kernel::InputSubShape& in) -> std::optional<ElementName> {
        if (in.operand >= inputs.size() || inputs[in.operand] == nullptr) {
            err = makeError(ErrorCode::InvalidArgument, "deriveNamingMap: history refers to a missing input map");
            return std::nullopt;
        }
        return inputs[in.operand]->nameOf(toElementType(in.kind), in.index); // unnamed -> nullopt, no error
    };
    auto outRange = [&](const kernel::OutputSubShape& o) {
        if (o.index >= counts[static_cast<std::size_t>(o.kind)]) {
            err = makeError(ErrorCode::InvalidArgument, "deriveNamingMap: history output index out of range");
            return false;
        }
        return true;
    };

    for (const auto& [in, out] : history.unchanged) {
        auto nm = inputName(in);
        if (!err) return std::unexpected(err.error());
        if (!nm || !outRange(out)) { if (!err) return std::unexpected(err.error()); continue; }
        origins[static_cast<int>(out.kind)][out.index].push_back({*nm, Cls::Unchanged, in.kind, false});
    }
    auto addMap = [&](const std::map<kernel::InputSubShape, std::vector<kernel::OutputSubShape>>& m, Cls cls)
        -> Result<void> {
        for (const auto& [in, outs] : m) {
            auto nm = inputName(in);
            if (!err) return err;
            if (!nm) continue;
            for (const auto& o : outs) {
                if (!outRange(o)) return err;
                std::size_t sameKind = 0;
                for (const auto& o2 : outs) sameKind += (o2.kind == o.kind);
                origins[static_cast<int>(o.kind)][o.index].push_back(
                    {*nm, cls, in.kind, cls == Cls::Modified && sameKind > 1});
            }
        }
        return {};
    };
    if (auto r = addMap(history.modified, Cls::Modified); !r) return std::unexpected(r.error());
    if (auto r = addMap(history.generated, Cls::Generated); !r) return std::unexpected(r.error());

    struct Pending {
        std::uint32_t index;
        ElementName name; // ordinal 0 until siblings are ranked
        bool keep;
    };
    std::vector<Pending> pending[3];
    const std::string op(opTag);

    for (int t = 0; t < 3; ++t) {
        const auto type = static_cast<ElementType>(t);
        for (std::uint32_t i = 0; i < counts[static_cast<std::size_t>(t)]; ++i) {
            auto& os = origins[t][i];
            ElementName n;
            n.feature = feature;
            n.type = type;
            if (os.empty()) {
                n.op = op + ".new";
                pending[t].push_back({i, std::move(n), false});
                continue;
            }
            std::vector<ElementName> srcs;
            for (const auto& o : os) srcs.push_back(o.src);
            std::sort(srcs.begin(), srcs.end());
            srcs.erase(std::unique(srcs.begin(), srcs.end()), srcs.end());

            bool anyKept = false, anySplit = false;
            for (const auto& o : os) {
                anyKept = anyKept || o.cls != Cls::Generated;
                anySplit = anySplit || o.split;
            }
            if (srcs.size() >= 2 && anyKept) { // N:1 merge
                n.op = op + ".merge";
                n.sources = std::move(srcs);
                pending[t].push_back({i, std::move(n), false});
            } else if (srcs.size() >= 2) { // generated from several inputs
                n.op = op + "." + roleOf(os.front().inKind, type);
                n.sources = std::move(srcs);
                pending[t].push_back({i, std::move(n), false});
            } else if (anyKept && !anySplit) { // unchanged or modified 1:1 -> keep name
                n = srcs.front();
                pending[t].push_back({i, std::move(n), true});
            } else if (anyKept) { // 1:N split
                n.op = op + ".split";
                n.sources = std::move(srcs);
                pending[t].push_back({i, std::move(n), false});
            } else { // generated
                n.op = op + "." + roleOf(os.front().inKind, type);
                n.sources = std::move(srcs);
                pending[t].push_back({i, std::move(n), false});
            }
        }
    }

    // Rank siblings sharing (feature, op, type, sources) by their geometric key; identical keys fall back to
    // kernel index (see the contract on deriveFromCounts in detail.hpp).
    NamingMap map;
    map.resize(counts[0], counts[1], counts[2]);
    for (int t = 0; t < 3; ++t) {
        std::map<ElementName, std::vector<std::uint32_t>> groups;
        std::vector<ElementName*> byIndex(counts[static_cast<std::size_t>(t)], nullptr);
        for (auto& p : pending[t]) {
            byIndex[p.index] = &p.name;
            if (!p.keep) groups[p.name].push_back(p.index);
        }
        for (auto& [proto, members] : groups) {
            if (members.size() > 1) {
                std::vector<std::pair<std::array<double, 4>, std::uint32_t>> keyed;
                bool allKeyed = static_cast<bool>(keyFn);
                for (auto idx : members) {
                    std::optional<std::array<double, 4>> k;
                    if (allKeyed) k = keyFn(static_cast<ElementType>(t), idx);
                    if (!k) allKeyed = false;
                    keyed.push_back({k ? *k : std::array<double, 4>{}, idx});
                }
                if (allKeyed)
                    std::sort(keyed.begin(), keyed.end());
                else
                    std::sort(keyed.begin(), keyed.end(),
                              [](const auto& a, const auto& b) { return a.second < b.second; });
                for (std::uint32_t ord = 0; ord < keyed.size(); ++ord) byIndex[keyed[ord].second]->ordinal = ord;
            }
        }
        std::set<ElementName> seen;
        for (auto& p : pending[t]) {
            if (!seen.insert(p.name).second)
                return makeError(ErrorCode::Internal,
                                 "deriveNamingMap: name collision for " + p.name.toString());
            if (auto r = map.set(static_cast<ElementType>(t), p.index, std::move(p.name)); !r)
                return std::unexpected(r.error());
        }
    }
    return map;
}

} // namespace detail

Result<NamingMap> deriveNamingMap(FeatureId feature, std::string_view opTag, const kernel::Shape& output,
                                  const std::vector<const NamingMap*>& inputs,
                                  const kernel::ShapeHistory& history) {
    if (output.isNull()) return makeError(ErrorCode::InvalidArgument, "deriveNamingMap: null output shape");
    const std::array<std::uint32_t, 3> counts{output.subShapeCount(kernel::SubShapeKind::Face),
                                              output.subShapeCount(kernel::SubShapeKind::Edge),
                                              output.subShapeCount(kernel::SubShapeKind::Vertex)};
    // Sibling ordinals come from the output's geometry (centroid x,y,z then measure), never from kernel index
    // alone. Geometry is read ONLY here, for sibling tie-breaks; everything else comes from the history.
    const detail::SiblingKeyFn keyFn = [&output](ElementType t, std::uint32_t i) {
        return std::optional<std::array<double, 4>>(output.subShapeKey(toSubShapeKind(t), i));
    };
    return detail::deriveFromCounts(feature, opTag, counts, inputs, history, keyFn);
}

} // namespace iw::naming
