// SPDX-License-Identifier: GPL-3.0-or-later
#include "core/expr.hpp"

#include "core/units.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <set>
#include <system_error>

namespace iw {

namespace {

constexpr std::size_t kMaxDepth = 200;

struct FuncInfo {
    std::string_view name;
    int minArgs;
    int maxArgs; // -1 = unbounded
};

constexpr FuncInfo kFunctions[] = {
    {"sin", 1, 1},   {"cos", 1, 1},   {"tan", 1, 1},  {"asin", 1, 1}, {"acos", 1, 1},
    {"atan", 1, 1},  {"sqrt", 1, 1},  {"abs", 1, 1},  {"floor", 1, 1}, {"ceil", 1, 1},
    {"round", 1, 1}, {"min", 1, -1},  {"max", 1, -1},
};

const FuncInfo* findFunction(std::string_view name) noexcept {
    for (const FuncInfo& f : kFunctions) {
        if (f.name == name) return &f;
    }
    return nullptr;
}

bool isIdentStart(char c) noexcept {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}
bool isIdentChar(char c) noexcept { return isIdentStart(c) || (c >= '0' && c <= '9'); }
bool isDigit(char c) noexcept { return c >= '0' && c <= '9'; }

enum class Tok : std::uint8_t { Number, Ident, Plus, Minus, Star, Slash, Caret, LParen, RParen, Comma, End };

struct Token {
    Tok kind = Tok::End;
    std::size_t begin = 0;
    std::size_t end = 0;
    double number = 0.0;
    std::string_view text;
};

Error parseError(std::size_t offset, std::string_view msg) {
    return Error{ErrorCode::ParseError, "column " + std::to_string(offset + 1) + ": " + std::string(msg)};
}

Result<std::vector<Token>> tokenize(std::string_view s) {
    std::vector<Token> out;
    std::size_t i = 0;
    while (i < s.size()) {
        const char c = s[i];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            ++i;
            continue;
        }
        if (isDigit(c) || (c == '.' && i + 1 < s.size() && isDigit(s[i + 1]))) {
            const std::size_t b = i;
            while (i < s.size() && isDigit(s[i])) ++i;
            if (i < s.size() && s[i] == '.') {
                ++i;
                while (i < s.size() && isDigit(s[i])) ++i;
            }
            if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
                std::size_t j = i + 1;
                if (j < s.size() && (s[j] == '+' || s[j] == '-')) ++j;
                if (j < s.size() && isDigit(s[j])) {
                    i = j;
                    while (i < s.size() && isDigit(s[i])) ++i;
                }
            }
            double v = 0.0;
            const char* first = s.data() + b;
            const char* last = s.data() + i;
            const auto [ptr, ec] = std::from_chars(first, last, v);
            if (ec != std::errc() || ptr != last || !std::isfinite(v)) {
                return std::unexpected(parseError(b, "invalid number"));
            }
            out.push_back(Token{Tok::Number, b, i, v, s.substr(b, i - b)});
            continue;
        }
        if (isIdentStart(c)) {
            const std::size_t b = i;
            while (i < s.size() && isIdentChar(s[i])) ++i;
            out.push_back(Token{Tok::Ident, b, i, 0.0, s.substr(b, i - b)});
            continue;
        }
        Tok kind = Tok::End;
        switch (c) {
        case '+': kind = Tok::Plus; break;
        case '-': kind = Tok::Minus; break;
        case '*': kind = Tok::Star; break;
        case '/': kind = Tok::Slash; break;
        case '^': kind = Tok::Caret; break;
        case '(': kind = Tok::LParen; break;
        case ')': kind = Tok::RParen; break;
        case ',': kind = Tok::Comma; break;
        default:
            return std::unexpected(parseError(i, std::string("unexpected character '") + c + "'"));
        }
        out.push_back(Token{kind, i, i + 1, 0.0, s.substr(i, 1)});
        ++i;
    }
    out.push_back(Token{Tok::End, s.size(), s.size(), 0.0, {}});
    return out;
}

ExprNode makeBinary(ExprNode::Op op, ExprNode lhs, ExprNode rhs) {
    ExprNode n;
    n.kind = ExprNode::Kind::Binary;
    n.op = op;
    n.children.push_back(std::move(lhs));
    n.children.push_back(std::move(rhs));
    return n;
}

class Parser {
public:
    explicit Parser(std::vector<Token> tokens) : toks_(std::move(tokens)) {}

    Result<ExprNode> parseAll() {
        auto e = parseSum();
        if (!e) return e;
        if (peek().kind != Tok::End) {
            return std::unexpected(parseError(peek().begin, "unexpected '" + std::string(peek().text) + "'"));
        }
        return e;
    }

private:
    const Token& peek() const noexcept { return toks_[pos_]; }
    void advance() noexcept {
        if (toks_[pos_].kind != Tok::End) ++pos_;
    }

    struct DepthGuard {
        std::size_t& depth;
        explicit DepthGuard(std::size_t& d) : depth(d) { ++depth; }
        ~DepthGuard() { --depth; }
    };

    Result<ExprNode> parseSum() {
        DepthGuard guard(depth_);
        if (depth_ > kMaxDepth) return std::unexpected(parseError(peek().begin, "expression nested too deeply"));
        auto lhs = parseProduct();
        if (!lhs) return lhs;
        ExprNode cur = std::move(*lhs);
        while (peek().kind == Tok::Plus || peek().kind == Tok::Minus) {
            const ExprNode::Op op = peek().kind == Tok::Plus ? ExprNode::Op::Add : ExprNode::Op::Sub;
            advance();
            auto rhs = parseProduct();
            if (!rhs) return rhs;
            cur = makeBinary(op, std::move(cur), std::move(*rhs));
        }
        return cur;
    }

    Result<ExprNode> parseProduct() {
        auto lhs = parseUnary();
        if (!lhs) return lhs;
        ExprNode cur = std::move(*lhs);
        while (peek().kind == Tok::Star || peek().kind == Tok::Slash) {
            const ExprNode::Op op = peek().kind == Tok::Star ? ExprNode::Op::Mul : ExprNode::Op::Div;
            advance();
            auto rhs = parseUnary();
            if (!rhs) return rhs;
            cur = makeBinary(op, std::move(cur), std::move(*rhs));
        }
        return cur;
    }

    Result<ExprNode> parseUnary() {
        if (peek().kind == Tok::Minus) {
            advance();
            auto operand = parseUnary();
            if (!operand) return operand;
            ExprNode n;
            n.kind = ExprNode::Kind::Negate;
            n.children.push_back(std::move(*operand));
            return n;
        }
        if (peek().kind == Tok::Plus) {
            advance();
            return parseUnary();
        }
        return parsePower();
    }

    Result<ExprNode> parsePower() {
        auto base = parsePrimary();
        if (!base) return base;
        if (peek().kind != Tok::Caret) return base;
        advance();
        auto exponent = parseUnary();
        if (!exponent) return exponent;
        return makeBinary(ExprNode::Op::Pow, std::move(*base), std::move(*exponent));
    }

    Result<ExprNode> parsePrimary() {
        const Token tok = peek();
        switch (tok.kind) {
        case Tok::Number: {
            advance();
            ExprNode n;
            n.kind = ExprNode::Kind::Literal;
            n.number = tok.number;
            if (peek().kind == Tok::Ident && findUnit(peek().text)) {
                n.text = std::string(peek().text);
                advance();
            }
            return n;
        }
        case Tok::Ident: {
            advance();
            const std::string name(tok.text);
            if (peek().kind == Tok::LParen) {
                return parseCall(tok, name);
            }
            if (name == "pi") {
                ExprNode n;
                n.kind = ExprNode::Kind::Literal;
                n.number = std::numbers::pi;
                return n;
            }
            if (findUnit(name)) {
                return std::unexpected(parseError(tok.begin, "unit '" + name + "' must follow a number"));
            }
            if (findFunction(name)) {
                return std::unexpected(parseError(tok.begin, "function '" + name + "' needs arguments in '('"));
            }
            ExprNode n;
            n.kind = ExprNode::Kind::Parameter;
            n.text = name;
            return n;
        }
        case Tok::LParen: {
            advance();
            auto inner = parseSum();
            if (!inner) return inner;
            if (peek().kind != Tok::RParen) {
                return std::unexpected(parseError(peek().begin, "expected ')'"));
            }
            advance();
            return inner;
        }
        case Tok::End:
            return std::unexpected(parseError(tok.begin, "unexpected end of expression"));
        default:
            return std::unexpected(parseError(tok.begin, "expected a number, a name or '('"));
        }
    }

    Result<ExprNode> parseCall(const Token& nameTok, const std::string& name) {
        const FuncInfo* info = findFunction(name);
        if (!info) return std::unexpected(parseError(nameTok.begin, "unknown function '" + name + "'"));
        advance(); // '('
        ExprNode n;
        n.kind = ExprNode::Kind::Call;
        n.text = name;
        if (peek().kind != Tok::RParen) {
            while (true) {
                auto arg = parseSum();
                if (!arg) return arg;
                n.children.push_back(std::move(*arg));
                if (peek().kind == Tok::Comma) {
                    advance();
                    continue;
                }
                break;
            }
        }
        if (peek().kind != Tok::RParen) {
            return std::unexpected(parseError(peek().begin, "expected ')' after arguments of '" + name + "'"));
        }
        advance();
        const auto argc = static_cast<int>(n.children.size());
        if (argc < info->minArgs || (info->maxArgs >= 0 && argc > info->maxArgs)) {
            return std::unexpected(parseError(nameTok.begin, "wrong number of arguments for '" + name + "'"));
        }
        return n;
    }

    std::vector<Token> toks_;
    std::size_t pos_ = 0;
    std::size_t depth_ = 0;
};

// ---- printing ----------------------------------------------------------------------------------

std::string fmtNumber(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.17g", v);
    return buf;
}

void printNode(const ExprNode& n, std::string& out) {
    switch (n.kind) {
    case ExprNode::Kind::Literal:
        out += fmtNumber(n.number);
        if (!n.text.empty()) {
            out += ' ';
            out += n.text;
        }
        return;
    case ExprNode::Kind::Parameter:
        out += n.text;
        return;
    case ExprNode::Kind::Negate:
        out += "(-";
        printNode(n.children[0], out);
        out += ')';
        return;
    case ExprNode::Kind::Binary: {
        const char* op = "+";
        switch (n.op) {
        case ExprNode::Op::Add: op = " + "; break;
        case ExprNode::Op::Sub: op = " - "; break;
        case ExprNode::Op::Mul: op = " * "; break;
        case ExprNode::Op::Div: op = " / "; break;
        case ExprNode::Op::Pow: op = " ^ "; break;
        }
        out += '(';
        printNode(n.children[0], out);
        out += op;
        printNode(n.children[1], out);
        out += ')';
        return;
    }
    case ExprNode::Kind::Call:
        out += n.text;
        out += '(';
        for (std::size_t i = 0; i < n.children.size(); ++i) {
            if (i) out += ", ";
            printNode(n.children[i], out);
        }
        out += ')';
        return;
    }
}

void collectReferences(const ExprNode& n, std::set<std::string>& out) {
    if (n.kind == ExprNode::Kind::Parameter) out.insert(n.text);
    for (const ExprNode& c : n.children) collectReferences(c, out);
}

// ---- evaluation --------------------------------------------------------------------------------

Error evalError(ErrorCode code, std::string msg) { return Error{code, std::move(msg)}; }

Result<Quantity> finite(Quantity q) {
    if (!std::isfinite(q.value)) return std::unexpected(evalError(ErrorCode::InvalidArgument, "result is not a finite number"));
    return q;
}

constexpr int kMaxDimExponent = 12;

Result<Dimension> makeDimension(int length, int angle) {
    if (length > kMaxDimExponent || length < -kMaxDimExponent || angle > kMaxDimExponent || angle < -kMaxDimExponent) {
        return std::unexpected(evalError(ErrorCode::InvalidArgument, "dimension exponent out of range"));
    }
    return Dimension{static_cast<std::int8_t>(length), static_cast<std::int8_t>(angle)};
}

Result<Quantity> evalNode(const ExprNode& n, const ParameterLookup& lookup) {
    switch (n.kind) {
    case ExprNode::Kind::Literal: {
        if (n.text.empty()) return Quantity::unitless(n.number);
        const auto u = findUnit(n.text);
        if (!u) return std::unexpected(evalError(ErrorCode::Internal, "unit lost after parse: " + n.text));
        return finite(Quantity{n.number * u->scale, u->dim});
    }
    case ExprNode::Kind::Parameter: {
        if (!lookup) return std::unexpected(evalError(ErrorCode::NotFound, "unknown parameter '" + n.text + "'"));
        return lookup(n.text);
    }
    case ExprNode::Kind::Negate: {
        auto v = evalNode(n.children[0], lookup);
        if (!v) return v;
        return Quantity{-v->value, v->dim};
    }
    case ExprNode::Kind::Binary: {
        auto lr = evalNode(n.children[0], lookup);
        if (!lr) return lr;
        auto rr = evalNode(n.children[1], lookup);
        if (!rr) return rr;
        const Quantity l = *lr;
        const Quantity r = *rr;
        switch (n.op) {
        case ExprNode::Op::Add:
        case ExprNode::Op::Sub: {
            if (l.dim != r.dim) {
                return std::unexpected(evalError(ErrorCode::InvalidArgument,
                    "cannot add or subtract " + dimensionName(l.dim) + " and " + dimensionName(r.dim)));
            }
            const double v = n.op == ExprNode::Op::Add ? l.value + r.value : l.value - r.value;
            return finite(Quantity{v, l.dim});
        }
        case ExprNode::Op::Mul: {
            auto d = makeDimension(l.dim.length + r.dim.length, l.dim.angle + r.dim.angle);
            if (!d) return std::unexpected(d.error());
            return finite(Quantity{l.value * r.value, *d});
        }
        case ExprNode::Op::Div: {
            if (r.value == 0.0) return std::unexpected(evalError(ErrorCode::InvalidArgument, "division by zero"));
            auto d = makeDimension(l.dim.length - r.dim.length, l.dim.angle - r.dim.angle);
            if (!d) return std::unexpected(d.error());
            return finite(Quantity{l.value / r.value, *d});
        }
        case ExprNode::Op::Pow: {
            if (!r.dim.unitless()) {
                return std::unexpected(evalError(ErrorCode::InvalidArgument, "'^' needs a plain number as exponent"));
            }
            if (l.dim.unitless()) return finite(Quantity::unitless(std::pow(l.value, r.value)));
            if (r.value != std::trunc(r.value) || std::fabs(r.value) > kMaxDimExponent) {
                return std::unexpected(evalError(ErrorCode::InvalidArgument,
                    "a dimensioned value can only be raised to a small integer power"));
            }
            const int k = static_cast<int>(r.value);
            auto d = makeDimension(l.dim.length * k, l.dim.angle * k);
            if (!d) return std::unexpected(d.error());
            return finite(Quantity{std::pow(l.value, r.value), *d});
        }
        }
        return std::unexpected(evalError(ErrorCode::Internal, "unknown binary operator"));
    }
    case ExprNode::Kind::Call: {
        std::vector<Quantity> args;
        args.reserve(n.children.size());
        for (const ExprNode& c : n.children) {
            auto a = evalNode(c, lookup);
            if (!a) return a;
            args.push_back(*a);
        }
        const std::string& f = n.text;
        const Quantity a0 = args[0];
        if (f == "sin" || f == "cos" || f == "tan") {
            if (!(a0.dim == Dimension::Angle || a0.dim.unitless())) {
                return std::unexpected(evalError(ErrorCode::InvalidArgument, f + " expects an angle or a plain number"));
            }
            const double x = a0.value; // angles are already radians; plain numbers are taken as radians
            const double v = f == "sin" ? std::sin(x) : f == "cos" ? std::cos(x) : std::tan(x);
            return finite(Quantity::unitless(v));
        }
        if (f == "asin" || f == "acos" || f == "atan") {
            if (!a0.dim.unitless()) {
                return std::unexpected(evalError(ErrorCode::InvalidArgument, f + " expects a plain number"));
            }
            if ((f == "asin" || f == "acos") && (a0.value < -1.0 || a0.value > 1.0)) {
                return std::unexpected(evalError(ErrorCode::InvalidArgument, f + " argument outside [-1, 1]"));
            }
            const double v = f == "asin" ? std::asin(a0.value) : f == "acos" ? std::acos(a0.value) : std::atan(a0.value);
            return finite(Quantity::radians(v));
        }
        if (f == "sqrt") {
            if (a0.dim.length % 2 != 0 || a0.dim.angle % 2 != 0) {
                return std::unexpected(evalError(ErrorCode::InvalidArgument,
                    "sqrt needs even dimension exponents, got " + dimensionName(a0.dim)));
            }
            if (a0.value < 0.0) return std::unexpected(evalError(ErrorCode::InvalidArgument, "sqrt of a negative number"));
            const Dimension half{static_cast<std::int8_t>(a0.dim.length / 2), static_cast<std::int8_t>(a0.dim.angle / 2)};
            return finite(Quantity{std::sqrt(a0.value), half});
        }
        if (f == "abs") return Quantity{std::fabs(a0.value), a0.dim};
        if (f == "floor") return Quantity{std::floor(a0.value), a0.dim};
        if (f == "ceil") return Quantity{std::ceil(a0.value), a0.dim};
        if (f == "round") return Quantity{std::round(a0.value), a0.dim};
        if (f == "min" || f == "max") {
            Quantity best = args[0];
            for (const Quantity& q : args) {
                if (q.dim != best.dim) {
                    return std::unexpected(evalError(ErrorCode::InvalidArgument,
                        f + " arguments must have the same dimension"));
                }
                if (f == "min" ? q.value < best.value : q.value > best.value) best.value = q.value;
            }
            return best;
        }
        return std::unexpected(evalError(ErrorCode::Internal, "unknown function '" + f + "'"));
    }
    }
    return std::unexpected(evalError(ErrorCode::Internal, "unknown expression node"));
}

} // namespace

// ---- public free functions ---------------------------------------------------------------------

std::string dimensionName(Dimension d) {
    if (d.unitless()) return "unitless";
    std::string out;
    auto add = [&out](const char* base, int exp) {
        if (exp == 0) return;
        if (!out.empty()) out += ' ';
        out += base;
        if (exp != 1) out += "^" + std::to_string(exp);
    };
    add("length", d.length);
    add("angle", d.angle);
    return out;
}

std::string formatQuantity(Quantity q) {
    char buf[96];
    if (q.dim == Dimension::Length) {
        std::snprintf(buf, sizeof buf, "%.10g mm", q.value);
    } else if (q.dim == Dimension::Angle) {
        std::snprintf(buf, sizeof buf, "%.10g deg", radiansToDegrees(q.value));
    } else if (q.dim.unitless()) {
        std::snprintf(buf, sizeof buf, "%.10g", q.value);
    } else {
        std::snprintf(buf, sizeof buf, "%.10g %s", q.value, dimensionName(q.dim).c_str());
    }
    return buf;
}

std::optional<UnitInfo> findUnit(std::string_view symbol) noexcept {
    // Only the real symbols: the length parser also accepts quote characters, which are not tokens here.
    if (symbol != "\"" && symbol != "'") {
        if (const auto lu = parseLengthUnit(symbol)) return UnitInfo{Dimension::Length, mmPerUnit(*lu)};
    }
    if (symbol == "deg") return UnitInfo{Dimension::Angle, degreesToRadians(1.0)};
    if (symbol == "rad") return UnitInfo{Dimension::Angle, 1.0};
    return std::nullopt;
}

bool isReservedExpressionName(std::string_view name) noexcept {
    return name == "pi" || findFunction(name) != nullptr || findUnit(name).has_value();
}

bool isValidParameterName(std::string_view name) noexcept {
    if (name.empty() || !isIdentStart(name.front())) return false;
    for (char c : name) {
        if (!isIdentChar(c)) return false;
    }
    return !isReservedExpressionName(name);
}

Result<Expression> Expression::parse(std::string_view source) {
    auto tokens = tokenize(source);
    if (!tokens) return std::unexpected(tokens.error());
    if (tokens->size() == 1) return std::unexpected(Error{ErrorCode::ParseError, "empty expression"});
    Parser parser(std::move(*tokens));
    auto root = parser.parseAll();
    if (!root) return std::unexpected(root.error());
    return Expression(std::string(source), std::move(*root));
}

std::string Expression::print() const {
    std::string out;
    printNode(root_, out);
    return out;
}

Result<Quantity> Expression::evaluate(const ParameterLookup& lookup) const {
    return evalNode(root_, lookup);
}

std::vector<std::string> Expression::references() const {
    std::set<std::string> names;
    collectReferences(root_, names);
    return {names.begin(), names.end()};
}

Result<std::string> renameIdentifier(std::string_view source, std::string_view from, std::string_view to) {
    auto tokens = tokenize(source);
    if (!tokens) return std::unexpected(tokens.error());
    std::string out;
    std::size_t copied = 0;
    for (const Token& t : *tokens) {
        if (t.kind != Tok::Ident || t.text != from) continue;
        out.append(source.substr(copied, t.begin - copied));
        out.append(to);
        copied = t.end;
    }
    out.append(source.substr(copied));
    return out;
}

} // namespace iw
