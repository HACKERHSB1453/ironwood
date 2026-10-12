// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "core/result.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace iw {

/// Physical dimension of a value as exponents of the internal base units: `length` is the exponent of
/// millimetres, `angle` the exponent of radians. Unitless is {0,0}. Examples: Length = {1,0},
/// Length^2 = {2,0}, Length/Angle = {1,-1}. Products and quotients add and subtract exponents; sums require
/// equal dimensions (see expr.hpp grammar notes for ^ and sqrt).
struct Dimension {
    std::int8_t length = 0;
    std::int8_t angle = 0;

    static const Dimension Unitless;
    static const Dimension Length;
    static const Dimension Angle;

    constexpr bool unitless() const noexcept { return length == 0 && angle == 0; }

    friend bool operator==(const Dimension&, const Dimension&) = default;
};
inline constexpr Dimension Dimension::Unitless{0, 0};
inline constexpr Dimension Dimension::Length{1, 0};
inline constexpr Dimension Dimension::Angle{0, 1};

/// Human-readable dimension: "unitless", "length", "angle", "length^2", "length angle^-1", ...
std::string dimensionName(Dimension d);

/// A dimensioned number in internal units.
struct Quantity {
    double value = 0.0;                    ///< mm (Length), radians (Angle), plain number (Unitless).
    Dimension dim = Dimension::Unitless;

    static constexpr Quantity mm(double v) noexcept { return {v, Dimension::Length}; }
    static constexpr Quantity radians(double v) noexcept { return {v, Dimension::Angle}; }
    static constexpr Quantity unitless(double v) noexcept { return {v, Dimension::Unitless}; }

    friend bool operator==(const Quantity&, const Quantity&) = default;
};

/// Human-readable form for UI/logs: "55.8 mm", "30 deg" (angles shown in degrees), "2", "9 mm^2".
std::string formatQuantity(Quantity q);

/// A unit symbol understood by expressions and parameters.
struct UnitInfo {
    Dimension dim = Dimension::Unitless;
    double scale = 1.0;                    ///< Internal units (mm or rad) per one of this unit.
};

/// Looks up a unit symbol: mm cm m in ft (length), deg rad (angle). Case-sensitive. nullopt if unknown.
std::optional<UnitInfo> findUnit(std::string_view symbol) noexcept;

/// True for names that cannot be used as parameter names: pi, function names, unit symbols.
bool isReservedExpressionName(std::string_view name) noexcept;

/// True if `name` matches [A-Za-z_][A-Za-z0-9_]* and is not reserved. Valid parameter name.
bool isValidParameterName(std::string_view name) noexcept;

/// Resolves a parameter reference to its value. Must return ErrorCode::NotFound for unknown names.
using ParameterLookup = std::function<Result<Quantity>(std::string_view name)>;

/// One node of the expression AST. Plain data; parentheses are not stored (print() re-adds them).
struct ExprNode {
    enum class Kind : std::uint8_t { Literal, Parameter, Negate, Binary, Call };
    enum class Op : std::uint8_t { Add, Sub, Mul, Div, Pow };

    Kind kind = Kind::Literal;
    Op op = Op::Add;          ///< Binary only.
    double number = 0.0;      ///< Literal only: the number as typed (before unit scaling).
    std::string text;         ///< Literal: unit symbol or "". Parameter: name. Call: function name.
    std::vector<ExprNode> children;
};

/// A parsed, unit-aware expression. Keeps the source text as typed and the AST.
/// Dimension rules: + and - need equal dimensions; * and / add and subtract exponents; ^ needs a unitless
/// exponent and, for a dimensioned base, an integer-valued exponent (|n| <= 12); sqrt halves exponents and
/// fails on odd ones; sin/cos/tan take an angle or a plain number (radians) and return a plain number;
/// asin/acos/atan take a plain number and return an angle; min/max need equal dimensions.
/// Grammar (lowest to highest precedence):
///   sum      := product (('+'|'-') product)*
///   product  := unary (('*'|'/') unary)*
///   unary    := ('-'|'+') unary | power
///   power    := primary ('^' unary)?          (right associative)
///   primary  := number unit? | name | name '(' args ')' | '(' sum ')'
/// Numbers may be followed by a unit (`2 in`, `30 deg`). Names are parameter references unless they are
/// `pi` or a function call. Functions: sin cos tan (argument: angle, or unitless taken as radians),
/// asin acos atan (unitless -> angle), sqrt (unitless), abs floor ceil round (keep dimension),
/// min max (same dimension, 1+ args).
class Expression {
public:
    /// Parses `source`. Errors: ParseError with the column of the offending token.
    static Result<Expression> parse(std::string_view source);

    /// Source text exactly as given to parse().
    const std::string& source() const noexcept { return source_; }

    /// Canonical, fully parenthesised text. parse(print()) yields an identical AST, so printing round-trips.
    std::string print() const;

    /// Evaluates against `lookup`. Errors: NotFound (from lookup), InvalidArgument (dimension mismatch,
    /// division by zero, domain errors, non-finite result).
    Result<Quantity> evaluate(const ParameterLookup& lookup) const;

    /// Sorted, unique parameter names referenced by this expression (direct references only).
    std::vector<std::string> references() const;

    const ExprNode& root() const noexcept { return root_; }

private:
    Expression(std::string source, ExprNode root) : source_(std::move(source)), root_(std::move(root)) {}

    std::string source_;
    ExprNode root_;
};

/// Rewrites every identifier token equal to `from` into `to`, keeping all other text (spacing, case of
/// other tokens) untouched. Used by parameter rename so user formatting survives. Errors: ParseError if
/// the source contains a character that cannot be tokenised.
Result<std::string> renameIdentifier(std::string_view source, std::string_view from, std::string_view to);

} // namespace iw
