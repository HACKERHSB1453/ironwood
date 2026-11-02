// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Import feature (F31): a base feature holding the solids of a STEP file.
//
// GEOMETRY STORAGE (decision): the native document has no B-rep blob storage yet (P0_FOLLOWUPS F3), so the feature
// stores the SOURCE PATH, not the geometry, and re-reads the file whenever it is computed (first evaluation after a
// load or an edit; afterwards the evaluator's stamp cache serves the result, so an unchanged timeline does not re-read
// the file). A missing or unreadable file fails the feature with a clear IoError naming the path. A content hash taken
// at import time is kept in the parameters; when the file on disk no longer matches it, the feature still imports the
// new content but reports a Warning, because element names (ranked by geometry) may then change.
// Follow-up (F3): embed the geometry as a BRep blob in the document (Fusion-style) and re-read only on an explicit
// "re-link / update from file" command.
//
// Each solid of the file (assembly occurrences expanded, world placement applied, depth-first node order, then the
// solid order inside a node) becomes one body: solid k is written to `bodies[k]`. Body ids are allocated when the
// feature is created (writesBodies() must be static), so the solid count is fixed at that time: if the file later has
// fewer solids the feature fails; extra solids are ignored with a Warning.
//
// Naming: an import has no history, so every element gets a fresh name {feature, "import.solid<k>.new", ordinal} with
// ordinals ranked by geometric key (naming::deriveNamingMap with no inputs) — stable for as long as the file content is
// unchanged, independent of OCCT's sub-shape order.

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "core/cancel.hpp"
#include "core/ids.hpp"
#include "core/result.hpp"
#include "kernel/shape.hpp"
#include "recompute/feature.hpp"

namespace iw::features {

struct ImportParams {
    std::string path;           ///< STEP file path as given (relative paths resolve against the process working dir).
    std::vector<BodyId> bodies; ///< One body per imported solid, in import order. Non-empty, unique, valid ids.
    std::string contentHash;    ///< 16 hex digits (FNV-1a 64 of the file bytes) at import time; empty = unchecked.
};

/// One world-placed solid read from a STEP file.
struct ImportedSolid {
    std::string name; ///< Name of the assembly node that holds the solid (may be empty).
    kernel::Shape shape;
};

/// Reads `path` (kernel::importStep) and returns its solids in import order (see header comment). Calls into the OCCT
/// STEP reader are serialised by a process-wide mutex. Errors: IoError (missing / unreadable file), ParseError,
/// KernelFailure (placement), Cancelled.
Result<std::vector<ImportedSolid>> readStepSolids(const std::filesystem::path& path, CancelToken cancel = {});

/// FNV-1a 64 hash of the file bytes as 16 lower-case hex digits. Errors: IoError.
Result<std::string> fileContentHash(const std::filesystem::path& path);

class ImportFeature final : public recompute::Feature {
public:
    ImportFeature(FeatureId id, ImportParams params) : id_(id), params_(std::move(params)) {}
    static Result<std::shared_ptr<recompute::Feature>> fromJson(FeatureId id, std::string_view json);

    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "import"; }
    std::vector<BodyId> writesBodies() const override { return params_.bodies; }
    std::uint64_t paramHash() const override;
    std::string paramsJson() const override;
    /// Errors: InvalidArgument (no bodies, bad ids, a body already exists), IoError (file missing / unreadable),
    /// ParseError, NotFound (the file has fewer solids than bodies), Cancelled.
    Result<recompute::FeatureResult> compute(const recompute::EvalContext& ctx) const override;
    const ImportParams& params() const noexcept { return params_; }

private:
    FeatureId id_;
    ImportParams params_;
};

/// Convenience for commands (Python, UI): reads `path` once, allocates one body id per solid with `allocateBody`, and
/// returns the parameters (with the content hash). Errors: as readStepSolids, InvalidArgument (no solids in the file).
template <class BodyAllocator>
Result<ImportParams> makeImportParams(const std::filesystem::path& path, BodyAllocator&& allocateBody, CancelToken cancel = {}) {
    auto solids = readStepSolids(path, cancel);
    if (!solids) return std::unexpected(solids.error());
    if (solids->empty()) return makeError(ErrorCode::InvalidArgument, "import: '" + path.string() + "' contains no solids");
    auto hash = fileContentHash(path);
    if (!hash) return std::unexpected(hash.error());
    ImportParams p;
    p.path = path.string();
    p.contentHash = std::move(*hash);
    for (std::size_t i = 0; i < solids->size(); ++i) p.bodies.push_back(allocateBody());
    return p;
}

void registerImportFeature(recompute::FeatureRegistry& registry);

} // namespace iw::features
