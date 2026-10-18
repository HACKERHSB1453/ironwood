// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>

#include "core/params.hpp"
#include "core/result.hpp"
#include "core/units.hpp"
#include "recompute/feature.hpp"
#include "recompute/timeline.hpp"

namespace iw::io {

/// The persisted document: metadata, the parameter table (format v1+) and the feature timeline. Derived data
/// (shapes, meshes) is NOT stored; it is recomputed on load. See FORMAT.md (UNSTABLE: no migration promise yet).
struct Document {
    std::string name;
    LengthUnit displayUnit = LengthUnit::Millimeter;
    std::uint64_t nextId = 1;   ///< Next unallocated Feature/Body/Component id (never reuse).
    ParameterTable parameters;  ///< User parameters, persisted with name, expression, unit, comment, favourite.
    recompute::Timeline timeline;
    /// Per-kind counter of the default feature names ("Extrude" -> 3 means the last default was "Extrude 3").
    /// Persisted so deleted features' numbers are not handed out again (format v2; absent in v0/v1 = empty).
    std::map<std::string, std::uint64_t> nameCounters;
};

/// Manifest format version written by saveDocument. Version 0 (no "parameters") and version 1 (no feature names or
/// name counters) still load.
inline constexpr std::uint64_t kDocumentFormatVersion = 2;

/// Writes `doc` to the directory `path` (created; must not be a file). Atomic-ish: writes to a temp
/// sibling then renames. Errors: IoError, InvalidArgument.
Result<void> saveDocument(const std::filesystem::path& path, const Document& doc);

/// Reads a document directory. Features are constructed via `registry`; unknown kinds -> NotFound.
/// Errors: IoError, ParseError (bad manifest / unsupported format version), NotFound.
Result<Document> loadDocument(const std::filesystem::path& path, const recompute::FeatureRegistry& registry);

} // namespace iw::io
