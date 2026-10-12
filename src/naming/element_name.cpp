// SPDX-License-Identifier: GPL-3.0-or-later
#include "naming/element_name.hpp"

#include <charconv>
#include <limits>
#include <string>
#include <system_error>
#include <algorithm>

namespace iw::naming {

std::strong_ordering operator<=>(const ElementName& a, const ElementName& b) {
    if (auto c = a.feature <=> b.feature; c != 0) return c;
    if (auto c = a.op <=> b.op; c != 0) return c;
    if (auto c = a.type <=> b.type; c != 0) return c;
    if (auto c = a.ordinal <=> b.ordinal; c != 0) return c;
    return std::lexicographical_compare_three_way(a.sources.begin(), a.sources.end(), b.sources.begin(),
                                                  b.sources.end(),
                                                  [](const ElementName& x, const ElementName& y) { return x <=> y; });
}

std::size_t ElementName::hash() const noexcept {
    std::size_t h = std::hash<FeatureId>{}(feature);
    auto mix = [&h](std::size_t v) { h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2); };
    mix(std::hash<std::string>{}(op));
    mix(static_cast<std::size_t>(type));
    mix(ordinal);
    for (const auto& s : sources) mix(s.hash());
    return h;
}

namespace {

char typeChar(ElementType t) noexcept {
    switch (t) {
    case ElementType::Face: return 'F';
    case ElementType::Edge: return 'E';
    case ElementType::Vertex: return 'V';
    }
    return 'F';
}

void append(std::string& out, const ElementName& n) {
    out += 'F';
    out += std::to_string(n.feature.value());
    out += ':';
    out += n.op;
    out += ':';
    out += typeChar(n.type);
    out += '#';
    out += std::to_string(n.ordinal);
    out += '[';
    bool first = true;
    for (const auto& s : n.sources) {
        if (!first) out += ',';
        first = false;
        append(out, s);
    }
    out += ']';
}

constexpr int kMaxDepth = 64;

class Parser {
public:
    explicit Parser(std::string_view t) : t_(t) {}

    Result<ElementName> parseAll() {
        auto n = name(0);
        if (!n) return n;
        if (pos_ != t_.size()) return fail("trailing characters");
        return n;
    }

private:
    std::unexpected<Error> fail(const char* what) const {
        return makeError(ErrorCode::ParseError,
                         std::string("ElementName parse: ") + what + " at offset " + std::to_string(pos_));
    }
    bool eat(char c) {
        if (pos_ < t_.size() && t_[pos_] == c) {
            ++pos_;
            return true;
        }
        return false;
    }
    bool number(std::uint64_t& v) {
        const char* b = t_.data() + pos_;
        const char* e = t_.data() + t_.size();
        if (b == e || *b < '0' || *b > '9') return false;
        auto [p, ec] = std::from_chars(b, e, v);
        if (ec != std::errc{}) return false;
        pos_ += static_cast<std::size_t>(p - b);
        return true;
    }
    static bool opChar(char c) {
        return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '.';
    }

    Result<ElementName> name(int depth) {
        if (depth > kMaxDepth) return fail("nesting too deep");
        ElementName n;
        std::uint64_t v = 0;
        if (!eat('F')) return fail("expected 'F'");
        if (!number(v)) return fail("expected feature id");
        n.feature = FeatureId{v};
        if (!eat(':')) return fail("expected ':'");
        std::size_t start = pos_;
        while (pos_ < t_.size() && opChar(t_[pos_])) ++pos_;
        if (pos_ == start) return fail("empty op");
        n.op = std::string(t_.substr(start, pos_ - start));
        if (!eat(':')) return fail("expected ':' after op");
        if (eat('F')) n.type = ElementType::Face;
        else if (eat('E')) n.type = ElementType::Edge;
        else if (eat('V')) n.type = ElementType::Vertex;
        else return fail("expected element type F/E/V");
        if (!eat('#')) return fail("expected '#'");
        if (!number(v) || v > std::numeric_limits<std::uint32_t>::max()) return fail("bad ordinal");
        n.ordinal = static_cast<std::uint32_t>(v);
        if (!eat('[')) return fail("expected '['");
        if (!eat(']')) {
            do {
                auto s = name(depth + 1);
                if (!s) return s;
                n.sources.push_back(std::move(*s));
            } while (eat(','));
            if (!eat(']')) return fail("expected ']'");
        }
        return n;
    }

    std::string_view t_;
    std::size_t pos_ = 0;
};

} // namespace

std::string ElementName::toString() const {
    std::string out;
    append(out, *this);
    return out;
}

Result<ElementName> ElementName::parse(std::string_view text) { return Parser(text).parseAll(); }

} // namespace iw::naming
