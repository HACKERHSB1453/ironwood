// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Qt-free descriptions of timeline features: the title shown on a timeline item, the glyph class used to draw its
// icon, and which body (if any) a feature produces.

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/ids.hpp"
#include "recompute/feature.hpp"
#include "recompute/timeline.hpp"

namespace iw::ui::timeline {

/// Icon family of a feature kind (the timeline paints one glyph per family).
enum class IconFamily { Sketch, Solid, Edit, Construction };

/// Title of a feature kind, e.g. "extrude" -> "Extrude"; unknown kinds are returned as given.
std::string kindTitle(std::string_view kind);
IconFamily iconFamilyOf(std::string_view kind);

/// Timeline labels: each feature's own name (Feature::name(), set by the document on creation and by rename). A feature
/// without a name (never produced by features::Document) falls back to "Extrude 1", "Extrude 2", ... numbered per kind.
std::vector<std::string> displayNames(const recompute::Timeline& tl);

/// Name of the timeline icon of a feature kind ("timeline_extrude", ...; "missing" for unknown kinds).
std::string timelineIconName(std::string_view kind);

/// The body a feature writes (produces or modifies), if any.
std::optional<BodyId> producedBody(const recompute::Feature& f);

/// Index of the first feature with `id`, if present.
std::optional<std::size_t> indexOfFeature(const recompute::Timeline& tl, FeatureId id);

} // namespace iw::ui::timeline
