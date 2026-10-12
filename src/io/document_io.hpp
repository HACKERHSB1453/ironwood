// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <filesystem>
#include <string>

#include "core/result.hpp"
#include "core/units.hpp"
#include "recompute/feature.hpp"
#include "recompute/timeline.hpp"

namespace iw::io {

/// The persisted document: metadata + the feature timeline. Derived data (shapes, meshes) is NOT stored
/// in v0; it is recomputed on load. See FORMAT.md (v0, UNSTABLE - may change without migration).
struct Document {
    std::string name;
    LengthUnit displayUnit = LengthUnit::Millimeter;
    std::uint64_t nextId = 1;   ///< Next unallocated Feature/Body/Component id (never reuse).
    recompute::Timeline timeline;
};

/// Writes `doc` to the directory `path` (created; must not be a file). Atomic-ish: writes to a temp
/// sibling then renames. Errors: IoError, InvalidArgument.
Result<void> saveDocument(const std::filesystem::path& path, const Document& doc);

/// Reads a document directory. Features are constructed via `registry`; unknown kinds -> NotFound.
/// Errors: IoError, ParseError (bad manifest / unsupported format version), NotFound.
Result<Document> loadDocument(const std::filesystem::path& path, const recompute::FeatureRegistry& registry);

} // namespace iw::io
