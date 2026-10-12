// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/result.hpp"
#include "naming/element_name.hpp"
#include "naming/naming_map.hpp"

namespace iw::naming {

/// How a reference was bound. Anything other than Exact must be surfaced to the user (ARCHITECTURE section 5:
/// Fusion-style warning icon on the timeline) and logged.
enum class ResolveQuality {
    Exact,            ///< The name exists in the map exactly once.
    HistoryDerived,   ///< Name absent but a unique successor exists via recorded history (split/merge). Warning.
    GeometricFallback ///< Bound by surface type/normal/centroid/adjacency heuristics. Warning, conservative.
};

struct Resolution {
    std::uint32_t index = 0;  ///< Sub-shape index in the map's shape (kernel ordering contract).
    ResolveQuality quality = ResolveQuality::Exact;
    std::vector<std::string> warnings; ///< Non-empty iff quality != Exact.
};

/// Resolves `ref` against the NamingMap of the shape it targets (the caller selects the map by ref.feature).
///
/// Semantics (conservative rebinding):
///  * exactly one index carries ref.name              -> Resolution{Exact}.
///  * several indices carry the name (should not happen; map corruption) -> Error Ambiguous. Never guess.
///  * name absent, one unique history successor        -> HistoryDerived, with a warning.
///  * name absent, history gives several successors    -> Error Ambiguous (caller shows a disambiguation UI).
///  * name absent, no history; geometric fallback is allowed ONLY when it yields a single candidate -> warning.
///  * nothing found                                    -> Error NotFound (feature is then marked broken/red).
/// The function is pure and thread-safe.
Result<Resolution> resolve(const ElementRef& ref, const NamingMap& map);

} // namespace iw::naming
