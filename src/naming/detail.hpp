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

/// Core of deriveNamingMap, driven by sub-shape counts of the output and a sibling key function.
///
/// Sibling ordinals: elements that share (feature, op, type, sources) are ranked by ascending key
/// (lexicographic over {x, y, z, measure}). If two siblings have IDENTICAL keys the tie is broken by
/// kernel index (ascending), so the result stays deterministic for a given output.
/// `partOperandFrom`: when every origin of a 1:N split comes from an operand >= this value (a tool body, e.g. the hole tool),
/// the pieces are named "<source op>.part" (sources=[source]) instead of "<opTag>.split", so they read as roles of the tool. If `keyFn` is empty
/// or returns nullopt for any sibling of a group, that whole group is ranked by kernel index instead.
Result<NamingMap> deriveFromCounts(FeatureId feature, std::string_view opTag,
                                   const std::array<std::uint32_t, 3>& counts,
                                   const std::vector<const NamingMap*>& inputs,
                                   const kernel::ShapeHistory& history, const SiblingKeyFn& keyFn,
                                   std::uint32_t partOperandFrom = UINT32_MAX);

} // namespace iw::naming::detail
