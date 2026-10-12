// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <compare>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "core/ids.hpp"
#include "core/result.hpp"
#include "kernel/shape.hpp"

/// Persistent topological naming (ARCHITECTURE section 5). Everything stored in a document refers to
/// B-rep sub-shapes by NAME, never by index; indices are only valid within one computed shape.
namespace iw::naming {

enum class ElementType : std::uint8_t { Face, Edge, Vertex };

/// Maps to the kernel's index space.
constexpr kernel::SubShapeKind toSubShapeKind(ElementType t) noexcept {
    switch (t) {
    case ElementType::Face: return kernel::SubShapeKind::Face;
    case ElementType::Edge: return kernel::SubShapeKind::Edge;
    case ElementType::Vertex: return kernel::SubShapeKind::Vertex;
    }
    return kernel::SubShapeKind::Face;
}

/// A persistent, history-derived element identity: "the `ordinal`-th `type` element that operation `op` of
/// feature `feature` produced from `sources`". Pure value type: totally ordered, hashable, serialisable.
///
/// Examples: extrude side face from sketch edge E -> {feature=F3, op="extrude.side", sources=[E-name],
/// ordinal=0}; primitive box face -> {F1, "box.face", sources=[], ordinal=2}.
/// `ordinal` disambiguates siblings with identical (feature, op, sources) - e.g. the pieces of a face split
/// by a boolean. It is assigned by ascending geometric sibling key (centroid x, y, z, then measure), with kernel
/// index only as the tie-break for identical keys; it does NOT depend on the kernel's output ordering.
struct ElementName {
    FeatureId feature;                 ///< Feature whose operation created this element.
    std::string op;                    ///< Operation tag, dotted lower-case ("extrude.side", "fuse.split").
    ElementType type = ElementType::Face;
    std::uint32_t ordinal = 0;         ///< Sibling disambiguator, 0-based.
    std::vector<ElementName> sources;  ///< Names of the input elements this one derives from (order kept).

    friend bool operator==(const ElementName&, const ElementName&) = default;
    friend std::strong_ordering operator<=>(const ElementName& a, const ElementName& b);

    /// Canonical text form, stable across versions: "F<id>:<op>:<F|E|V>#<ordinal>[<src>,<src>...]".
    std::string toString() const;
    /// Inverse of toString(). Errors: ParseError.
    static Result<ElementName> parse(std::string_view text);
    /// Structural hash (consistent with ==).
    std::size_t hash() const noexcept;
};

/// A reference from a feature parameter to an element of a feature's result.
struct ElementRef {
    FeatureId feature;     ///< Feature whose result (at its timeline position) the name is looked up in.
    ElementType type = ElementType::Face;
    ElementName name;
    friend bool operator==(const ElementRef&, const ElementRef&) = default;
};

} // namespace iw::naming

template <> struct std::hash<iw::naming::ElementName> {
    std::size_t operator()(const iw::naming::ElementName& n) const noexcept { return n.hash(); }
};
