// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "core/result.hpp"
#include "core/math.hpp"
#include "kernel/history.hpp"
#include "naming/element_name.hpp"

namespace iw::naming {

/// Bidirectional table between sub-shape indices (kernel ordering contract) and ElementNames for ONE
/// computed shape (one body of one feature result). Built by the feature that produced the shape, using
/// the previous NamingMap(s) of its inputs plus the kernel's ShapeHistory. Immutable once handed to the
/// evaluator; reading concurrently is safe, mutation is single-threaded.
class NamingMap {
public:
    /// Pre-sizes the per-type tables to the shape's sub-shape counts; all entries start unnamed.
    void resize(std::uint32_t faces, std::uint32_t edges, std::uint32_t vertices);
    /// Assigns `name` to element `index` of `type`. InvalidArgument if out of range or `name.type` differs.
    Result<void> set(ElementType type, std::uint32_t index, ElementName name);

    std::uint32_t size(ElementType type) const noexcept;
    /// Name of element `index`; nullopt if out of range or unnamed.
    std::optional<ElementName> nameOf(ElementType type, std::uint32_t index) const;
    /// All indices currently carrying exactly `name` (usually 0 or 1; >1 means the name is ambiguous).
    std::vector<std::uint32_t> indicesOf(const ElementName& name) const;

private:
    std::vector<std::optional<ElementName>> tables_[3];
};

/// Derives the NamingMap of an operation's result.
///  `inputs[i]` = NamingMap of operand i; `history` = kernel history; `feature`/`opTag` identify the operation.
/// Policy (ARCHITECTURE section 5): unchanged -> keep source name; modified 1:1 -> same name; modified 1:N
/// (split) -> sources=[old], op "<opTag>.split"; N:1 (merge) -> sources = all inputs sorted; generated ->
/// op "<opTag>.<role>" with source; unexplained output elements -> fresh names with op "<opTag>.new".
/// Ordinals of siblings (split pieces, and any elements sharing one name prototype) are assigned by
/// ascending geometric key (centroid x, y, z, then measure; kernel::Shape::subShapeKey). Kernel index is
/// only the final tie-break for siblings with identical keys. Deriving reads geometry ONLY for these
/// sibling tie-breaks; which names an element gets is decided by the history alone.
///
/// Sibling frame (N1 / F27): sibling keys are expressed in this orthonormal frame (origin + axes) instead of world
/// coordinates, so that moving or rotating the owning feature's geometry as a whole does not change the ranking.
/// Without a frame the keys are world coordinates (still translation-invariant, since lexicographic order commutes with a
/// translation, but not rotation-invariant). Degenerate edges (zero length) are keyed by their vertex position, never by the
/// origin (kernel::stableSubShapeKey).
/// `partOperandFrom`: pieces of a 1:N split whose every origin is an operand >= this value (a tool body whose elements are
/// named by role, e.g. the hole tool) are named "<source op>.part" instead of "<opTag>.split" (default: never).
struct SiblingFrame {
    Vec3 origin{0, 0, 0};
    Vec3 xAxis{1, 0, 0};
    Vec3 yAxis{0, 1, 0};
    Vec3 zAxis{0, 0, 1};
};

Result<NamingMap> deriveNamingMap(FeatureId feature, std::string_view opTag, const kernel::Shape& output,
                                  const std::vector<const NamingMap*>& inputs,
                                  const kernel::ShapeHistory& history, const SiblingFrame* frame = nullptr,
                                  std::uint32_t partOperandFrom = UINT32_MAX);

} // namespace iw::naming
