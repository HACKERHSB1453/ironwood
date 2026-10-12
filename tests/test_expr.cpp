// SPDX-License-Identifier: GPL-3.0-or-later
// A1: expression parser/evaluator and ParameterTable.
#include "core/expr.hpp"
#include "core/params.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numbers>
#include <random>
#include <string>
#include <vector>

using namespace iw;

namespace {

// a = 2 mm, b = 0.5 rad, c = 1.5 (unitless); anything else is unknown.
ParameterLookup fixedLookup() {
    return [](std::string_view n) -> Result<Quantity> {
        if (n == "a") return Quantity::mm(2.0);
        if (n == "b") return Quantity::radians(0.5);
        if (n == "c") return Quantity::unitless(1.5);
        return makeError(ErrorCode::NotFound, "unknown parameter '" + std::string(n) + "'");
    };
}

Result<Quantity> ev(std::string_view src, const ParameterLookup& lk = fixedLookup()) {
    auto e = Expression::parse(src);
    if (!e) return std::unexpected(e.error());
    return e->evaluate(lk);
}

void expectQ(std::string_view src, double value, Dimension dim) {
    auto r = ev(src);
    ASSERT_TRUE(r.has_value()) << src << ": " << r.error().message;
    EXPECT_EQ(r->dim, dim) << src;
    EXPECT_NEAR(r->value, value, 1e-9 * std::max(1.0, std::fabs(value))) << src;
}

void expectErr(std::string_view src, ErrorCode code) {
    auto r = ev(src);
    ASSERT_FALSE(r.has_value()) << src << " unexpectedly evaluated";
    EXPECT_EQ(r.error().code, code) << src << ": " << r.error().message;
}

void expectParseErr(std::string_view src) {
    auto e = Expression::parse(src);
    ASSERT_FALSE(e.has_value()) << "'" << src << "' unexpectedly parsed";
    EXPECT_EQ(e.error().code, ErrorCode::ParseError) << "'" << src << "': " << e.error().message;
}

} // namespace

// ---- units and arithmetic ----------------------------------------------------------------------

TEST(ExprUnits, InchPlusMillimetres) {
    expectQ("2 in + 5 mm", 55.8, Dimension::Length); // acceptance: 2 in + 5 mm = 55.8 mm
}

TEST(ExprUnits, AllLengthUnits) {
    expectQ("1 m", 1000.0, Dimension::Length);
    expectQ("1 cm", 10.0, Dimension::Length);
    expectQ("1 ft", 304.8, Dimension::Length);
    expectQ("1 in", 25.4, Dimension::Length);
    expectQ("3 mm", 3.0, Dimension::Length);
}

TEST(ExprUnits, AngleUnits) {
    expectQ("30 deg", std::numbers::pi / 6.0, Dimension::Angle);
    expectQ("1 rad", 1.0, Dimension::Angle);
}

TEST(ExprUnits, DivisionOfSameDimensionIsUnitless) {
    expectQ("10 mm / 5 mm", 2.0, Dimension::Unitless);
    expectQ("90 deg / 30 deg", 3.0, Dimension::Unitless);
}

TEST(ExprArith, Precedence) {
    expectQ("1 + 2 * 3", 7.0, Dimension::Unitless);
    expectQ("(1 + 2) * 3", 9.0, Dimension::Unitless);
    expectQ("10 / 4", 2.5, Dimension::Unitless);
    expectQ("10 - 4 - 3", 3.0, Dimension::Unitless); // left associative
    expectQ("2 ^ 3 ^ 2", 512.0, Dimension::Unitless); // right associative
    expectQ("-2 ^ 2", -4.0, Dimension::Unitless);     // unary minus binds looser than ^
    expectQ("2 ^ -1", 0.5, Dimension::Unitless);
    expectQ("1 - -2", 3.0, Dimension::Unitless);
    expectQ("--3", 3.0, Dimension::Unitless);
    expectQ("+3", 3.0, Dimension::Unitless);
    expectQ("2*3", 6.0, Dimension::Unitless);
    expectQ(".5 + 1.", 1.5, Dimension::Unitless);
    expectQ("1e3 + 2E-1", 1000.2, Dimension::Unitless);
}

TEST(ExprArith, ParameterReference) {
    expectQ("a * 2", 4.0, Dimension::Length);
    expectQ("c * 2", 3.0, Dimension::Unitless);
    expectQ("b * 2", 1.0, Dimension::Angle);
}

TEST(ExprFunctions, Trigonometry) {
    expectQ("sin(30 deg)", 0.5, Dimension::Unitless);
    expectQ("cos(0)", 1.0, Dimension::Unitless);
    expectQ("tan(0 rad)", 0.0, Dimension::Unitless);
    expectQ("atan(1)", std::numbers::pi / 4.0, Dimension::Angle);
    expectQ("asin(1)", std::numbers::pi / 2.0, Dimension::Angle);
    expectQ("acos(1)", 0.0, Dimension::Angle);
}

TEST(ExprFunctions, Math) {
    expectQ("sqrt(16)", 4.0, Dimension::Unitless);
    expectQ("abs(-3 mm)", 3.0, Dimension::Length);
    expectQ("floor(2.7)", 2.0, Dimension::Unitless);
    expectQ("floor(-2.5)", -3.0, Dimension::Unitless);
    expectQ("ceil(2.1)", 3.0, Dimension::Unitless);
    expectQ("round(2.5)", 3.0, Dimension::Unitless);
    expectQ("min(3 mm, 2 mm, 5 mm)", 2.0, Dimension::Length);
    expectQ("max(1, 4, 2)", 4.0, Dimension::Unitless);
    expectQ("pi", std::numbers::pi, Dimension::Unitless);
    expectQ("2 * pi", 2.0 * std::numbers::pi, Dimension::Unitless);
}

TEST(ExprDimensions, MismatchedAdditionIsError) {
    expectErr("10 mm + 5 deg", ErrorCode::InvalidArgument); // acceptance
    expectErr("10 mm + 5", ErrorCode::InvalidArgument);
    expectErr("10 mm - 1 rad", ErrorCode::InvalidArgument);
}

TEST(ExprDimensions, InvalidProductsAndQuotients) {
    // Products and quotients of lengths are now valid (see ExprDimensions.ExponentAlgebra). What stays
    // invalid: a dimensioned exponent, a non-integer power of a dimensioned value, and out-of-range powers.
    expectErr("2 ^ (1 mm)", ErrorCode::InvalidArgument);
    expectErr("(2 mm)^0.5", ErrorCode::InvalidArgument);
    expectErr("(2 mm)^13", ErrorCode::InvalidArgument);
}

TEST(ExprDimensions, ExponentAlgebra) {
    const Dimension area{2, 0};
    const Dimension volume{3, 0};
    expectQ("(10 mm)*(2 mm)", 20.0, Dimension{2, 0});   // Length^2 is a valid intermediate
    expectQ("10 mm * 2 mm", 20.0, area);
    expectQ("(10 mm)^2", 100.0, area);
    expectQ("(2 mm)^3", 8.0, volume);
    expectQ("5 / (10 mm)", 0.5, Dimension{-1, 0});       // Length^-1
    expectQ("10 mm / 5 deg", 10.0 / (5.0 * std::numbers::pi / 180.0), Dimension{1, -1});
    expectQ("sqrt((3 mm)^2 + (4 mm)^2)", 5.0, Dimension::Length); // acceptance: diagonal = 5 mm
    expectQ("(2 mm)*(3 mm)/(6 mm)", 1.0, Dimension::Length);      // d1*d2/d3 with lengths
    expectQ("(2 mm)^3 / ((2 mm)*(2 mm))", 2.0, Dimension::Length); // acceptance: = 2 mm
    expectQ("(10 mm)^2 / (5 mm)", 20.0, Dimension::Length);
    expectQ("sqrt((4 mm)^2)", 4.0, Dimension::Length);
    expectQ("(2 mm)^0", 1.0, Dimension::Unitless);       // zero exponent cancels any dimension
    expectQ("(2 mm)^-1 * (4 mm)", 2.0, Dimension::Unitless);
    expectQ("(2 mm)^2 * (3 mm)^-2", 4.0 / 9.0, Dimension::Unitless);
}

TEST(ExprDimensions, ExponentRulesStillEnforced) {
    expectErr("(10 mm)^2 + 5 mm", ErrorCode::InvalidArgument); // Length^2 + Length
    expectErr("(10 mm)^2 + 5", ErrorCode::InvalidArgument);
    expectErr("sqrt(9 mm)", ErrorCode::InvalidArgument);       // odd exponent: no length square root
    expectErr("sqrt((2 mm)^3)", ErrorCode::InvalidArgument);
    expectErr("sin((3 mm)^2)", ErrorCode::InvalidArgument);    // trig needs angle or plain number
    expectErr("sqrt(-((3 mm)^2))", ErrorCode::InvalidArgument);
    expectErr("(2 deg)*(3 mm) + 1 mm", ErrorCode::InvalidArgument);
}

TEST(ExprDimensions, FunctionDomains) {
    expectErr("1 / 0", ErrorCode::InvalidArgument);
    expectErr("a / 0 mm", ErrorCode::InvalidArgument);
    expectErr("sqrt(-1)", ErrorCode::InvalidArgument);
    expectErr("sqrt(4 mm)", ErrorCode::InvalidArgument);
    expectErr("asin(2)", ErrorCode::InvalidArgument);
    expectErr("sin(3 mm)", ErrorCode::InvalidArgument);
    expectErr("asin(1 mm)", ErrorCode::InvalidArgument);
    expectErr("min(1 mm, 2)", ErrorCode::InvalidArgument);
    expectErr("0 ^ -1", ErrorCode::InvalidArgument); // infinity
}

TEST(ExprDimensions, UnknownParameterIsNotFound) {
    expectErr("unknown + 1", ErrorCode::NotFound);
}

TEST(ExprParse, ErrorsAreParseErrors) {
    expectParseErr("");
    expectParseErr("   ");
    expectParseErr("2 +");
    expectParseErr("(2");
    expectParseErr("2)");
    expectParseErr("2 foo");
    expectParseErr("2 pi");
    expectParseErr("foo(1)");
    expectParseErr("sin 3");
    expectParseErr("sin()");
    expectParseErr("sin(1, 2)");
    expectParseErr("min()");
    expectParseErr("1e400");
    expectParseErr("2 $");
    expectParseErr("mm");
    expectParseErr("1..2");
    expectParseErr(std::string(300, '(') + "1" + std::string(300, ')'));
}

TEST(ExprParse, SourceIsKept) {
    auto e = Expression::parse("2 in  +  5 mm");
    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(e->source(), "2 in  +  5 mm");
}

TEST(ExprParse, References) {
    auto e = Expression::parse("b * a + sin(a) + pi - b");
    ASSERT_TRUE(e.has_value());
    const std::vector<std::string> expected{"a", "b"};
    EXPECT_EQ(e->references(), expected);
}

TEST(ExprPrint, CanonicalFormReparses) {
    auto e = Expression::parse("2 in+5mm*(3-1)");
    ASSERT_TRUE(e.has_value());
    const std::string printed = e->print();
    auto again = Expression::parse(printed);
    ASSERT_TRUE(again.has_value()) << printed;
    EXPECT_EQ(again->print(), printed);
    auto r1 = e->evaluate(fixedLookup());
    auto r2 = again->evaluate(fixedLookup());
    ASSERT_TRUE(r1.has_value());
    ASSERT_TRUE(r2.has_value());
    EXPECT_EQ(r1->value, r2->value);
}

TEST(ExprPrint, FormatQuantity) {
    EXPECT_EQ(formatQuantity(Quantity::mm(55.8)), "55.8 mm");
    EXPECT_EQ(formatQuantity(Quantity::radians(std::numbers::pi / 6.0)), "30 deg");
    EXPECT_EQ(formatQuantity(Quantity::unitless(2.0)), "2");
}

// Random expressions: every generated string must parse, print must re-parse to the same print, and both
// trees must evaluate identically (same value, or the same error).
namespace {

std::string genNumber(std::mt19937& rng) {
    std::uniform_int_distribution<int> whole(0, 999);
    std::uniform_int_distribution<int> kind(0, 3);
    std::string s;
    if (kind(rng) == 0) {
        s = std::to_string(whole(rng)) + "." + std::to_string(whole(rng) % 100);
    } else {
        s = std::to_string(whole(rng));
    }
    static const char* kUnits[] = {"", "", "", " mm", " in", " cm", " m", " ft", " deg", " rad"};
    std::uniform_int_distribution<int> u(0, static_cast<int>(std::size(kUnits)) - 1);
    return s + kUnits[u(rng)];
}

std::string genExpr(std::mt19937& rng, int depth) {
    std::uniform_int_distribution<int> pick(0, 99);
    const int roll = pick(rng);
    if (depth <= 0 || roll < 25) {
        static const char* kNames[] = {"a", "b", "c", "pi", "zz"};
        if (roll % 3 == 0) {
            return kNames[static_cast<std::size_t>(roll) % std::size(kNames)];
        }
        return genNumber(rng);
    }
    static const char* kOps[] = {" + ", " - ", " * ", " / ", " ^ "};
    if (roll < 60) {
        const char* op = kOps[static_cast<std::size_t>(roll) % std::size(kOps)];
        return "(" + genExpr(rng, depth - 1) + op + genExpr(rng, depth - 1) + ")";
    }
    if (roll < 70) {
        return "(-" + genExpr(rng, depth - 1) + ")";
    }
    static const char* kUnary[] = {"sin", "cos", "sqrt", "abs", "floor", "round", "atan"};
    if (roll < 90) {
        const char* f = kUnary[static_cast<std::size_t>(roll) % std::size(kUnary)];
        return std::string(f) + "(" + genExpr(rng, depth - 1) + ")";
    }
    return (roll % 2 ? std::string("min(") : std::string("max(")) + genExpr(rng, depth - 1) + ", " +
           genExpr(rng, depth - 1) + ")";
}

} // namespace

TEST(ExprRoundTrip, ThousandRandomExpressions) {
    std::mt19937 rng(20260712u);
    std::size_t evaluated = 0;
    for (int i = 0; i < 1000; ++i) {
        const std::string src = genExpr(rng, 4);
        auto first = Expression::parse(src);
        ASSERT_TRUE(first.has_value()) << "generated '" << src << "': " << first.error().message;

        const std::string printed = first->print();
        auto second = Expression::parse(printed);
        ASSERT_TRUE(second.has_value()) << "printed '" << printed << "' from '" << src << "': "
                                        << second.error().message;
        ASSERT_EQ(second->print(), printed) << "from '" << src << "'";

        auto r1 = first->evaluate(fixedLookup());
        auto r2 = second->evaluate(fixedLookup());
        ASSERT_EQ(r1.has_value(), r2.has_value()) << src;
        if (r1) {
            ++evaluated;
            EXPECT_EQ(r1->dim, r2->dim) << src;
            EXPECT_EQ(r1->value, r2->value) << src;
        } else {
            EXPECT_EQ(r1.error(), r2.error()) << src;
        }
    }
    EXPECT_GT(evaluated, 50u); // the generator must produce a meaningful share of valid expressions
}

// ---- reserved names ----------------------------------------------------------------------------

TEST(ParamNames, ValidAndReserved) {
    EXPECT_TRUE(isValidParameterName("d1"));
    EXPECT_TRUE(isValidParameterName("_wall"));
    EXPECT_FALSE(isValidParameterName("1d"));
    EXPECT_FALSE(isValidParameterName("a-b"));
    EXPECT_FALSE(isValidParameterName(""));
    EXPECT_FALSE(isValidParameterName("pi"));
    EXPECT_FALSE(isValidParameterName("sin"));
    EXPECT_FALSE(isValidParameterName("mm"));
}

// ---- ParameterTable ----------------------------------------------------------------------------

TEST(ParamTable, DependentParameterEvaluates) {
    ParameterTable t;
    ASSERT_TRUE(t.add("d1", "10 mm"));
    ASSERT_TRUE(t.add("d2", "d1*2"));
    auto d2 = t.evaluate("d2");
    ASSERT_TRUE(d2.has_value()) << d2.error().message;
    EXPECT_EQ(d2->dim, Dimension::Length);
    EXPECT_NEAR(d2->value, 20.0, 1e-12); // acceptance: d1*2 with d1=10 mm -> 20 mm
}

TEST(ParamTable, DuplicateAndInvalidNamesRejected) {
    ParameterTable t;
    ASSERT_TRUE(t.add("d1", "1 mm"));
    auto dup = t.add("d1", "2 mm");
    ASSERT_FALSE(dup.has_value());
    EXPECT_EQ(dup.error().code, ErrorCode::InvalidArgument);
    auto bad = t.add("sin", "1");
    ASSERT_FALSE(bad.has_value());
    EXPECT_EQ(bad.error().code, ErrorCode::InvalidArgument);
    auto badExpr = t.add("x", "1 +");
    ASSERT_FALSE(badExpr.has_value());
    EXPECT_EQ(badExpr.error().code, ErrorCode::ParseError);
    EXPECT_EQ(t.size(), 1u);
    EXPECT_EQ(t.find("d1")->expression.source(), "1 mm"); // untouched by the failed add
}

TEST(ParamTable, CycleRejectedWithPath) {
    ParameterTable t;
    ASSERT_TRUE(t.add("a", "1 mm"));
    ASSERT_TRUE(t.add("b", "a + 1 mm"));
    auto r = t.setExpression("a", "b * 2");
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(r.error().message.find("a -> b -> a"), std::string::npos) << r.error().message;
    EXPECT_EQ(t.find("a")->expression.source(), "1 mm"); // rejected change leaves the table untouched
    ASSERT_TRUE(t.evaluate("b").has_value());
}

TEST(ParamTable, SelfReferenceIsCycle) {
    ParameterTable t;
    auto r = t.add("c", "c + 1");
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(r.error().message.find("c -> c"), std::string::npos) << r.error().message;
    EXPECT_EQ(t.size(), 0u);
}

TEST(ParamTable, LongCyclePath) {
    ParameterTable t;
    ASSERT_TRUE(t.add("p", "1 mm"));
    ASSERT_TRUE(t.add("q", "p"));
    ASSERT_TRUE(t.add("r", "q"));
    auto r = t.setExpression("p", "r");
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().message.find("p -> r -> q -> p"), std::string::npos) << r.error().message;
}

TEST(ParamTable, ForwardReferenceResolvesOnceDefined) {
    ParameterTable t;
    ASSERT_TRUE(t.add("b", "a * 2"));
    auto early = t.evaluate("b");
    ASSERT_FALSE(early.has_value());
    EXPECT_EQ(early.error().code, ErrorCode::NotFound);
    ASSERT_TRUE(t.add("a", "3 mm"));
    auto later = t.evaluate("b");
    ASSERT_TRUE(later.has_value());
    EXPECT_NEAR(later->value, 6.0, 1e-12);
}

TEST(ParamTable, RenameUpdatesReferences) {
    ParameterTable t;
    ASSERT_TRUE(t.add("a", "10 mm"));
    ASSERT_TRUE(t.add("b", "a * 2 + sin(a / 1 mm) * 0 mm"));
    ASSERT_TRUE(t.rename("a", "width"));
    EXPECT_EQ(t.find("a"), nullptr);
    ASSERT_NE(t.find("width"), nullptr);
    EXPECT_EQ(t.find("b")->expression.source(), "width * 2 + sin(width / 1 mm) * 0 mm"); // formatting kept
    auto b = t.evaluate("b");
    ASSERT_TRUE(b.has_value()) << b.error().message;
    EXPECT_NEAR(b->value, 20.0, 1e-12);
}

TEST(ParamTable, RenameRejectsTakenAndInvalidNames) {
    ParameterTable t;
    ASSERT_TRUE(t.add("a", "1 mm"));
    ASSERT_TRUE(t.add("b", "2 mm"));
    EXPECT_EQ(t.rename("a", "b").error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(t.rename("a", "sin").error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(t.rename("a", "9x").error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(t.rename("nope", "x").error().code, ErrorCode::NotFound);
    EXPECT_NE(t.find("a"), nullptr);
    EXPECT_TRUE(t.rename("a", "a"));
}

TEST(ParamTable, RenameThatWouldCreateCycleIsRejected) {
    ParameterTable t;
    ASSERT_TRUE(t.add("a", "1 mm"));
    ASSERT_TRUE(t.add("b", "c + 1 mm")); // c does not exist yet
    ASSERT_TRUE(t.add("z", "b"));
    auto r = t.rename("z", "c"); // now b -> c -> b
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(t.find("z"), nullptr);
    EXPECT_EQ(t.find("c"), nullptr);
}

TEST(ParamTable, RemoveRefusedWhileUsed) {
    ParameterTable t;
    ASSERT_TRUE(t.add("a", "1 mm"));
    ASSERT_TRUE(t.add("b", "a * 2"));
    ASSERT_TRUE(t.add("c", "b + a"));
    auto r = t.remove("a");
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(r.error().message.find("b, c"), std::string::npos) << r.error().message;
    ASSERT_TRUE(t.remove("c"));
    ASSERT_TRUE(t.remove("b"));
    ASSERT_TRUE(t.remove("a"));
    EXPECT_EQ(t.size(), 0u);
    EXPECT_EQ(t.remove("a").error().code, ErrorCode::NotFound);
}

TEST(ParamTable, DependencyQueries) {
    ParameterTable t;
    ASSERT_TRUE(t.add("a", "1 mm"));
    ASSERT_TRUE(t.add("b", "a * 2"));
    ASSERT_TRUE(t.add("c", "a + b"));
    EXPECT_EQ(t.dependencies("c"), (std::vector<std::string>{"a", "b"}));
    EXPECT_EQ(t.dependents("a"), (std::vector<std::string>{"b", "c"}));
    EXPECT_TRUE(t.dependents("c").empty());
}

TEST(ParamTable, EvaluationOrderIsDependencyFirst) {
    ParameterTable t;
    ASSERT_TRUE(t.add("c", "b + a")); // declared before its dependencies
    ASSERT_TRUE(t.add("b", "a * 2"));
    ASSERT_TRUE(t.add("a", "1 mm"));
    auto order = t.evaluationOrder();
    ASSERT_TRUE(order.has_value());
    const auto& o = *order;
    ASSERT_EQ(o.size(), 3u);
    auto pos = [&](const std::string& n) {
        return std::find(o.begin(), o.end(), n) - o.begin();
    };
    EXPECT_LT(pos("a"), pos("b"));
    EXPECT_LT(pos("b"), pos("c"));

    auto all = t.evaluateAll();
    ASSERT_TRUE(all.has_value());
    EXPECT_NEAR(all->at("c").value, 3.0, 1e-12);
}

TEST(ParamTable, UnitsCoerceBareNumbers) {
    ParameterTable t;
    ASSERT_TRUE(t.add("len", "2", "in"));
    ASSERT_TRUE(t.add("ang", "30", "deg"));
    ASSERT_TRUE(t.add("free", "2 in"));
    auto len = t.evaluate("len");
    ASSERT_TRUE(len.has_value());
    EXPECT_NEAR(len->value, 50.8, 1e-12);
    EXPECT_EQ(len->dim, Dimension::Length);
    auto ang = t.evaluate("ang");
    ASSERT_TRUE(ang.has_value());
    EXPECT_NEAR(ang->value, std::numbers::pi / 6.0, 1e-12);
    EXPECT_EQ(ang->dim, Dimension::Angle);
    EXPECT_NEAR(t.evaluate("free")->value, 50.8, 1e-12);
}

TEST(ParamTable, UnitMismatchAndUnknownUnit) {
    ParameterTable t;
    ASSERT_TRUE(t.add("bad_len", "5 deg", "mm")); // unit check is at evaluation: the dimension must match
    auto mismatch = t.evaluate("bad_len");
    ASSERT_FALSE(mismatch.has_value());
    EXPECT_EQ(mismatch.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(mismatch.error().message.find("unit mismatch"), std::string::npos);

    auto unknown = t.add("x", "1", "furlong");
    ASSERT_FALSE(unknown.has_value());
    EXPECT_EQ(unknown.error().code, ErrorCode::InvalidArgument);

    ASSERT_TRUE(t.setUnit("bad_len", ""));
    EXPECT_TRUE(t.evaluate("bad_len").has_value()); // inferred from expression once the unit is cleared
}

TEST(ParamTable, ErrorsCarryParameterContext) {
    ParameterTable t;
    ASSERT_TRUE(t.add("a", "1 mm"));
    ASSERT_TRUE(t.add("b", "a / 0"));  // division by zero
    ASSERT_TRUE(t.add("c", "b + 1"));  // inherits the failure
    auto r = t.evaluate("c");
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(r.error().message.find("parameter 'b'"), std::string::npos) << r.error().message;
    EXPECT_TRUE(t.evaluate("a").has_value()); // independent parameters still evaluate
    EXPECT_FALSE(t.evaluateAll().has_value());
}

TEST(ParamTable, MetadataSetters) {
    ParameterTable t;
    ASSERT_TRUE(t.add("a", "1 mm", "", "wall thickness"));
    EXPECT_EQ(t.find("a")->comment, "wall thickness");
    EXPECT_FALSE(t.find("a")->favourite);
    ASSERT_TRUE(t.setFavourite("a", true));
    EXPECT_TRUE(t.find("a")->favourite);
    ASSERT_TRUE(t.setComment("a", "outer wall"));
    EXPECT_EQ(t.find("a")->comment, "outer wall");
    EXPECT_EQ(t.setFavourite("zz", true).error().code, ErrorCode::NotFound);
    ASSERT_TRUE(t.setExpression("a", "2 mm"));
    EXPECT_NEAR(t.evaluate("a")->value, 2.0, 1e-12);
    EXPECT_EQ(t.setExpression("a", "2 +").error().code, ErrorCode::ParseError);
    EXPECT_EQ(t.find("a")->expression.source(), "2 mm");
}

TEST(ParamTable, HashOfUsedTracksTransitiveDependencies) {
    ParameterTable t;
    ASSERT_TRUE(t.add("a", "10 mm"));
    ASSERT_TRUE(t.add("b", "a * 2"));
    ASSERT_TRUE(t.add("d", "5 mm"));

    const std::vector<std::string> useB{"b"};
    const std::vector<std::string> useBA{"a", "b"};
    const std::vector<std::string> useAB{"b", "a"};
    const std::uint64_t h1 = t.hashOfUsed(useB);
    EXPECT_EQ(h1, t.hashOfUsed(useB));          // deterministic
    EXPECT_EQ(t.hashOfUsed(useBA), t.hashOfUsed(useAB)); // independent of input order

    ParameterTable edited = t;
    ASSERT_TRUE(edited.setExpression("a", "11 mm"));
    EXPECT_NE(edited.hashOfUsed(useB), h1); // a transitive dependency changed

    ParameterTable unrelated = t;
    ASSERT_TRUE(unrelated.setExpression("d", "6 mm"));
    EXPECT_EQ(unrelated.hashOfUsed(useB), h1); // unrelated parameter edit keeps the stamp

    ParameterTable sameValue = t;
    ASSERT_TRUE(sameValue.setExpression("a", "5 mm * 2")); // different text, same value
    EXPECT_EQ(sameValue.hashOfUsed(useB), h1);

    const std::vector<std::string> missing{"nope"};
    EXPECT_NE(t.hashOfUsed(missing), t.hashOfUsed(useB));
}

TEST(ParamTable, HashOfUsedDistinguishesFailures) {
    ParameterTable t;
    ASSERT_TRUE(t.add("a", "1 mm"));
    ASSERT_TRUE(t.add("b", "a * 2"));
    const std::vector<std::string> useB{"b"};
    const std::uint64_t ok = t.hashOfUsed(useB);
    ParameterTable broken = t;
    ASSERT_TRUE(broken.setExpression("a", "1 mm + 1 deg"));
    EXPECT_NE(broken.hashOfUsed(useB), ok);
}

TEST(ParamTable, ExponentDimensionsAndUnitAssignment) {
    ParameterTable t;
    ASSERT_TRUE(t.add("d1", "2 mm"));
    ASSERT_TRUE(t.add("d2", "3 mm"));
    ASSERT_TRUE(t.add("d3", "6 mm"));
    ASSERT_TRUE(t.add("ratio", "d1*d2/d3", "mm"));  // acceptance: lengths product/quotient = length
    auto ratio = t.evaluate("ratio");
    ASSERT_TRUE(ratio.has_value()) << ratio.error().message;
    EXPECT_NEAR(ratio->value, 1.0, 1e-12);
    EXPECT_EQ(ratio->dim, Dimension::Length);

    ASSERT_TRUE(t.add("area", "d1*d2")); // no unit: the parameter takes Length^2
    EXPECT_EQ(t.evaluate("area")->dim, (Dimension{2, 0}));

    ASSERT_TRUE(t.add("len2", "d1*d2", "mm")); // acceptance: a length field rejects Length^2
    auto rejected = t.evaluate("len2");
    ASSERT_FALSE(rejected.has_value());
    EXPECT_EQ(rejected.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(rejected.error().message.find("unit mismatch"), std::string::npos) << rejected.error().message;

    ASSERT_TRUE(t.add("sq", "sqrt(d1*d1 + d2*d2)", "mm"));
    EXPECT_NEAR(t.evaluate("sq")->value, std::sqrt(13.0), 1e-12);
}

