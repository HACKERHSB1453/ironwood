// SPDX-License-Identifier: GPL-3.0-or-later
#include "core/params.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <numeric>
#include <optional>
#include <set>

namespace iw {

namespace {

constexpr std::size_t kNone = static_cast<std::size_t>(-1);

std::size_t indexOf(const std::vector<Parameter>& ps, std::string_view name) noexcept {
    for (std::size_t i = 0; i < ps.size(); ++i) {
        if (ps[i].name == name) return i;
    }
    return kNone;
}

Result<void> validateUnit(std::string_view unit) {
    if (unit.empty() || findUnit(unit)) return {};
    return makeError(ErrorCode::InvalidArgument, "unknown unit '" + std::string(unit) + "'");
}

Result<void> validateNewName(const std::vector<Parameter>& ps, std::string_view name) {
    if (!isValidParameterName(name)) {
        return makeError(ErrorCode::InvalidArgument, "invalid parameter name '" + std::string(name) + "'");
    }
    if (indexOf(ps, name) != kNone) {
        return makeError(ErrorCode::InvalidArgument, "parameter '" + std::string(name) + "' already exists");
    }
    return {};
}

// ---- dependency ordering -----------------------------------------------------------------------

struct TopoState {
    const std::vector<Parameter>& ps;
    std::vector<std::uint8_t> mark; // 0 = unvisited, 1 = on the current path, 2 = done
    std::vector<std::size_t> path;  // indices on the current DFS path
    std::vector<std::size_t> order;
};

std::optional<Error> visit(TopoState& st, std::size_t idx) {
    if (st.mark[idx] == 2) return std::nullopt;
    if (st.mark[idx] == 1) {
        std::string cycle;
        for (auto it = std::find(st.path.begin(), st.path.end(), idx); it != st.path.end(); ++it) {
            cycle += st.ps[*it].name + " -> ";
        }
        cycle += st.ps[idx].name;
        return Error{ErrorCode::InvalidArgument, "cyclic parameter dependency: " + cycle};
    }
    st.mark[idx] = 1;
    st.path.push_back(idx);
    for (const std::string& dep : st.ps[idx].expression.references()) {
        const std::size_t j = indexOf(st.ps, dep);
        if (j == kNone) continue; // unknown names are leaves; evaluating them reports NotFound
        if (auto err = visit(st, j)) return err;
    }
    st.path.pop_back();
    st.mark[idx] = 2;
    st.order.push_back(idx);
    return std::nullopt;
}

/// Dependency-first order of everything reachable from `roots`. Errors on a cycle, with the cycle path.
Result<std::vector<std::size_t>> topoOrder(const std::vector<Parameter>& ps, const std::vector<std::size_t>& roots) {
    TopoState st{ps, std::vector<std::uint8_t>(ps.size(), 0), {}, {}};
    for (std::size_t r : roots) {
        if (auto err = visit(st, r)) return std::unexpected(*err);
    }
    return st.order;
}

Result<void> checkAcyclic(const std::vector<Parameter>& ps) {
    std::vector<std::size_t> all(ps.size());
    std::iota(all.begin(), all.end(), std::size_t{0});
    auto order = topoOrder(ps, all);
    if (!order) return std::unexpected(order.error());
    return {};
}

/// Replaces `target` with `next` if `next` is acyclic; otherwise leaves `target` untouched.
Result<void> commit(std::vector<Parameter>& target, std::vector<Parameter> next) {
    auto ok = checkAcyclic(next);
    if (!ok) return ok;
    target = std::move(next);
    return {};
}

/// Evaluates `order` (dependency-first). Failures are stored per parameter and propagate to dependents,
/// so one bad parameter does not hide the others.
std::vector<Result<Quantity>> computeInOrder(const std::vector<Parameter>& ps, const std::vector<std::size_t>& order) {
    std::vector<Result<Quantity>> out(ps.size(), Quantity{});
    for (std::size_t idx : order) {
        const Parameter& p = ps[idx];
        const ParameterLookup lookup = [&ps, &out](std::string_view n) -> Result<Quantity> {
            const std::size_t j = indexOf(ps, n);
            if (j == kNone) return makeError(ErrorCode::NotFound, "unknown parameter '" + std::string(n) + "'");
            return out[j];
        };
        Result<Quantity> r = p.expression.evaluate(lookup);
        if (r && !p.unit.empty()) {
            const auto u = findUnit(p.unit);
            if (!u) {
                r = makeError(ErrorCode::Internal, "unknown unit '" + p.unit + "'");
            } else if (r->dim.unitless()) {
                r = Quantity{r->value * u->scale, u->dim}; // bare number read in the parameter's unit
            } else if (r->dim != u->dim) {
                r = makeError(ErrorCode::InvalidArgument,
                    "unit mismatch: parameter unit '" + p.unit + "' is " + std::string(dimensionName(u->dim)) +
                    " but the expression is " + std::string(dimensionName(r->dim)));
            }
        }
        if (!r) {
            const Error e = r.error();
            r = makeError(e.code, "parameter '" + p.name + "': " + e.message);
        }
        out[idx] = std::move(r);
    }
    return out;
}

/// FNV-1a, 64 bit. Stable across runs and platforms (unlike std::hash).
struct Fnv {
    std::uint64_t h = 14695981039346656037ULL;
    void byte(std::uint8_t b) noexcept {
        h ^= b;
        h *= 1099511628211ULL;
    }
    void u64(std::uint64_t v) noexcept {
        for (int i = 0; i < 8; ++i) byte(static_cast<std::uint8_t>(v >> (8 * i)));
    }
    void str(std::string_view s) noexcept {
        u64(s.size());
        for (char c : s) byte(static_cast<std::uint8_t>(c));
    }
};

} // namespace

// ---- mutators ----------------------------------------------------------------------------------

Result<void> ParameterTable::add(std::string name, std::string expression, std::string unit, std::string comment) {
    if (auto ok = validateNewName(params_, name); !ok) return ok;
    if (auto ok = validateUnit(unit); !ok) return ok;
    auto parsed = Expression::parse(expression);
    if (!parsed) return makeError(parsed.error().code, "parameter '" + name + "': " + parsed.error().message);
    std::vector<Parameter> next = params_;
    next.push_back(Parameter{std::move(name), std::move(*parsed), std::move(unit), std::move(comment), false});
    return commit(params_, std::move(next));
}

Result<void> ParameterTable::rename(std::string_view from, std::string_view to) {
    const std::size_t idx = indexOf(params_, from);
    if (idx == kNone) return makeError(ErrorCode::NotFound, "unknown parameter '" + std::string(from) + "'");
    if (from == to) return {};
    if (auto ok = validateNewName(params_, to); !ok) return ok;

    std::vector<Parameter> next = params_;
    next[idx].name = std::string(to);
    const std::string fromName(from);
    for (Parameter& p : next) {
        const auto refs = p.expression.references();
        if (!std::binary_search(refs.begin(), refs.end(), fromName)) continue;
        auto rewritten = renameIdentifier(p.expression.source(), from, to);
        if (!rewritten) return std::unexpected(rewritten.error());
        auto reparsed = Expression::parse(*rewritten);
        if (!reparsed) return std::unexpected(reparsed.error());
        p.expression = std::move(*reparsed);
    }
    return commit(params_, std::move(next));
}

Result<void> ParameterTable::remove(std::string_view name) {
    const std::size_t idx = indexOf(params_, name);
    if (idx == kNone) return makeError(ErrorCode::NotFound, "unknown parameter '" + std::string(name) + "'");
    const std::vector<std::string> users = dependents(name);
    if (!users.empty()) {
        std::string list;
        for (const std::string& u : users) {
            if (!list.empty()) list += ", ";
            list += u;
        }
        return makeError(ErrorCode::InvalidArgument,
            "parameter '" + std::string(name) + "' is used by: " + list);
    }
    std::vector<Parameter> next = params_;
    next.erase(next.begin() + static_cast<std::ptrdiff_t>(idx));
    return commit(params_, std::move(next));
}

Result<void> ParameterTable::setExpression(std::string_view name, std::string expression) {
    const std::size_t idx = indexOf(params_, name);
    if (idx == kNone) return makeError(ErrorCode::NotFound, "unknown parameter '" + std::string(name) + "'");
    auto parsed = Expression::parse(expression);
    if (!parsed) {
        return makeError(parsed.error().code, "parameter '" + std::string(name) + "': " + parsed.error().message);
    }
    std::vector<Parameter> next = params_;
    next[idx].expression = std::move(*parsed);
    return commit(params_, std::move(next));
}

Result<void> ParameterTable::setUnit(std::string_view name, std::string unit) {
    const std::size_t idx = indexOf(params_, name);
    if (idx == kNone) return makeError(ErrorCode::NotFound, "unknown parameter '" + std::string(name) + "'");
    if (auto ok = validateUnit(unit); !ok) return ok;
    params_[idx].unit = std::move(unit);
    return {};
}

Result<void> ParameterTable::setComment(std::string_view name, std::string comment) {
    const std::size_t idx = indexOf(params_, name);
    if (idx == kNone) return makeError(ErrorCode::NotFound, "unknown parameter '" + std::string(name) + "'");
    params_[idx].comment = std::move(comment);
    return {};
}

Result<void> ParameterTable::setFavourite(std::string_view name, bool favourite) {
    const std::size_t idx = indexOf(params_, name);
    if (idx == kNone) return makeError(ErrorCode::NotFound, "unknown parameter '" + std::string(name) + "'");
    params_[idx].favourite = favourite;
    return {};
}

// ---- queries -----------------------------------------------------------------------------------

const Parameter* ParameterTable::find(std::string_view name) const noexcept {
    const std::size_t idx = indexOf(params_, name);
    return idx == kNone ? nullptr : &params_[idx];
}

std::vector<std::string> ParameterTable::dependencies(std::string_view name) const {
    std::vector<std::string> out;
    const std::size_t idx = indexOf(params_, name);
    if (idx == kNone) return out;
    for (std::string& dep : params_[idx].expression.references()) {
        if (indexOf(params_, dep) != kNone) out.push_back(std::move(dep));
    }
    return out;
}

std::vector<std::string> ParameterTable::dependents(std::string_view name) const {
    std::vector<std::string> out;
    for (const Parameter& p : params_) {
        const auto refs = p.expression.references();
        if (std::binary_search(refs.begin(), refs.end(), std::string(name))) out.push_back(p.name);
    }
    std::sort(out.begin(), out.end());
    return out;
}

Result<std::vector<std::string>> ParameterTable::evaluationOrder() const {
    std::vector<std::size_t> all(params_.size());
    std::iota(all.begin(), all.end(), std::size_t{0});
    auto order = topoOrder(params_, all);
    if (!order) return std::unexpected(order.error());
    std::vector<std::string> names;
    names.reserve(order->size());
    for (std::size_t idx : *order) names.push_back(params_[idx].name);
    return names;
}

Result<Quantity> ParameterTable::evaluate(std::string_view name) const {
    const std::size_t idx = indexOf(params_, name);
    if (idx == kNone) return makeError(ErrorCode::NotFound, "unknown parameter '" + std::string(name) + "'");
    auto order = topoOrder(params_, {idx});
    if (!order) return std::unexpected(order.error());
    auto results = computeInOrder(params_, *order);
    return results[idx];
}

Result<ParameterTable::Values> ParameterTable::evaluateAll() const {
    std::vector<std::size_t> all(params_.size());
    std::iota(all.begin(), all.end(), std::size_t{0});
    auto order = topoOrder(params_, all);
    if (!order) return std::unexpected(order.error());
    auto results = computeInOrder(params_, *order);
    Values values;
    for (std::size_t i = 0; i < params_.size(); ++i) {
        if (!results[i]) return std::unexpected(results[i].error());
        values.emplace(params_[i].name, *results[i]);
    }
    return values;
}

std::uint64_t ParameterTable::hashOfUsed(std::span<const std::string> names) const {
    std::set<std::size_t> roots;
    std::set<std::string> missing;
    for (const std::string& n : names) {
        const std::size_t idx = indexOf(params_, n);
        if (idx == kNone) {
            missing.insert(n);
        } else {
            roots.insert(idx);
        }
    }

    Fnv f;
    const std::vector<std::size_t> rootVec(roots.begin(), roots.end());
    auto order = topoOrder(params_, rootVec);
    std::vector<std::size_t> closure;
    std::vector<Result<Quantity>> results;
    if (order) {
        closure = *order;
        results = computeInOrder(params_, *order);
    } else {
        // Unreachable for a table built only through the mutators; still hash something deterministic.
        f.str("cycle");
        f.str(order.error().message);
    }
    std::sort(closure.begin(), closure.end(),
        [this](std::size_t a, std::size_t b) { return params_[a].name < params_[b].name; });
    for (std::size_t idx : closure) {
        const Result<Quantity>& r = results[idx];
        f.str(params_[idx].name);
        if (r) {
            f.byte(static_cast<std::uint8_t>(r->dim.length));
            f.byte(static_cast<std::uint8_t>(r->dim.angle));
            const double v = r->value == 0.0 ? 0.0 : r->value; // -0.0 and +0.0 hash alike
            f.u64(std::bit_cast<std::uint64_t>(v));
        } else {
            f.byte(0xFF);
            f.str(r.error().message);
        }
    }
    for (const std::string& m : missing) {
        f.str("missing");
        f.str(m);
    }
    return f.h;
}

} // namespace iw
