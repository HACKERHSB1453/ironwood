// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "core/expr.hpp"
#include "core/result.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace iw {

/// A user parameter. `unit` is the display/input unit symbol ("mm", "in", "deg", ...). When it is empty the
/// parameter takes the dimension of its expression. When set, a bare number in the expression is read in
/// that unit (`in` with expression `2` is 2 in = 50.8 mm), and a dimensioned result must match it.
struct Parameter {
    std::string name;
    Expression expression;
    std::string unit;
    std::string comment;
    bool favourite = false;
};

/// The document parameter table. Value type: copy it to try a change, the mutators are atomic
/// (a rejected change leaves the table untouched).
///
/// Dependencies: a parameter depends on every parameter its expression references. Mutations that would
/// create a cycle are rejected with InvalidArgument whose message carries the cycle path
/// ("cyclic parameter dependency: a -> b -> a"). References to names that do not (yet) exist are allowed;
/// evaluating them fails with NotFound.
///
/// Thread-safety: const member functions may be called concurrently; mutators need external locking.
class ParameterTable {
public:
    using Values = std::map<std::string, Quantity, std::less<>>;

    /// Adds a parameter. Errors: InvalidArgument (bad or duplicate name, unknown unit, cycle),
    /// ParseError (bad expression).
    Result<void> add(std::string name, std::string expression, std::string unit = {}, std::string comment = {});
    /// Renames a parameter and rewrites every expression that references it (formatting preserved).
    /// Errors: NotFound (from), InvalidArgument (bad or taken new name, cycle).
    Result<void> rename(std::string_view from, std::string_view to);
    /// Removes a parameter. Refused while other parameters reference it (InvalidArgument listing them).
    Result<void> remove(std::string_view name);
    /// Replaces the expression of `name`. Errors: NotFound, ParseError, InvalidArgument (cycle).
    Result<void> setExpression(std::string_view name, std::string expression);
    /// Sets the display unit ("" = infer from expression). Errors: NotFound, InvalidArgument (unknown unit).
    Result<void> setUnit(std::string_view name, std::string unit);
    Result<void> setComment(std::string_view name, std::string comment);
    Result<void> setFavourite(std::string_view name, bool favourite);

    /// nullptr if absent. Pointers are invalidated by any mutator.
    const Parameter* find(std::string_view name) const noexcept;
    /// All parameters in insertion order.
    const std::vector<Parameter>& parameters() const noexcept { return params_; }
    std::size_t size() const noexcept { return params_.size(); }

    /// Direct dependencies of `name` (existing parameters it references), sorted.
    std::vector<std::string> dependencies(std::string_view name) const;
    /// Parameters that directly reference `name`, sorted.
    std::vector<std::string> dependents(std::string_view name) const;

    /// All parameters, dependencies before dependents. Errors: InvalidArgument (cycle; cannot occur after
    /// a successful mutation).
    Result<std::vector<std::string>> evaluationOrder() const;

    /// Value of one parameter, evaluating only what it transitively depends on.
    Result<Quantity> evaluate(std::string_view name) const;
    /// Value of every parameter. Returns the first error in evaluation order.
    Result<Values> evaluateAll() const;

    /// Stamp for the recompute contract: a hash of the values of `names` and of every parameter they
    /// transitively depend on (name, dimension and value bits; the error text for a failing parameter;
    /// "missing" for unknown names). Two tables give the same hash iff those inputs match, independent of
    /// the order of `names`. Features use it as part of paramHash() so that an edit invalidates exactly
    /// the features that depend on it.
    std::uint64_t hashOfUsed(std::span<const std::string> names) const;

private:
    std::vector<Parameter> params_;
};

} // namespace iw
