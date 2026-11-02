// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>
#include <vector>

#include "core/result.hpp"
#include "kernel/history.hpp"
#include "naming/element_name.hpp"
#include "naming/naming_map.hpp"

/// Implementation detail of the naming module. NOT part of the public naming API: only naming_map.cpp and
/// the naming unit tests include this header.
namespace iw::naming::detail {

/// Geometric sibling key of element `index` of `type`: {centroid x, y, z, measure}, as returned by
/// kernel::Shape::subShapeKey. nullopt = unavailable (then the siblings fall back to kernel index order).
using SiblingKeyFn = std::function<std::optional<std::array<double, 4>>(ElementType, std::uint32_t)>;

/// Topological neighbours of element `index` of `type` in the OUTPUT shape (F42), used to rank siblings by identity instead
/// of position: an Edge -> its adjacent faces, a Vertex -> its incident edges, a Face -> nothing (empty). Indices of the
/// next-higher-dimension type, i.e. elements that are named before `type` is ranked.
using NeighbourFn = std::function<std::vector<std::uint32_t>(ElementType, std::uint32_t)>;

/// Core of deriveNamingMap, driven by sub-shape counts of the output and a sibling key function.
///
/// Sibling ordinals: elements that share (feature, op, type, sources) are ranked by ascending key
/// (lexicographic over {x, y, z, measure}). If two siblings have IDENTICAL keys the tie is broken by
/// kernel index (ascending), so the result stays deterministic for a given output.
/// `partOperandFrom`: when every origin of a 1:N split comes from an operand >= this value (a tool body, e.g. the hole tool),
/// the pieces are named "<source op>.part" (sources=[source]) instead of "<opTag>.split", so they read as roles of the tool. If `keyFn` is empty
/// or returns nullopt for any sibling of a group, that whole group is ranked by kernel index instead.
/// `neighbours` (F42, optional): when set, siblings are ranked FIRST by the sorted names of their neighbours (faces are
/// named before edges, edges before vertices), and only siblings with identical neighbour names fall back to the key /
/// kernel-index order above. Names of neighbours do not depend on placement, so these ordinals are rotation-invariant.
Result<NamingMap> deriveFromCounts(FeatureId feature, std::string_view opTag,
                                   const std::array<std::uint32_t, 3>& counts,
                                   const std::vector<const NamingMap*>& inputs,
                                   const kernel::ShapeHistory& history, const SiblingKeyFn& keyFn,
                                   std::uint32_t partOperandFrom = UINT32_MAX, const NeighbourFn& neighbours = {});

} // namespace iw::naming::detail
