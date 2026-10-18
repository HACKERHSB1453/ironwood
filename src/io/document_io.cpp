// SPDX-License-Identifier: GPL-3.0-or-later
#include "io/document_io.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <fstream>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>

// Vendored nlohmann/json v3.12.0 (MIT, third_party/nlohmann). Private to this translation unit.
#include <nlohmann/json.hpp>

namespace iw::io {
namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

constexpr const char* kFormatTag = "ironwood-document";
constexpr std::uint64_t kFormatVersion = kDocumentFormatVersion; // v1: "parameters" array; v2: feature "name", "nameCounters"
constexpr std::uint64_t kMaxSafeId = (std::uint64_t{1} << 53) - 1; // FORMAT.md: ids < 2^53 in v0
constexpr const char* kManifestName = "manifest.json";

/// Flushes a file or directory to stable storage (POSIX fsync). False if it cannot be opened or synced.
bool fsyncPath(const fs::path& path) {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) return false;
    const bool ok = ::fsync(fd) == 0;
    ::close(fd);
    return ok;
}

/// Reads a persisted id (FeatureId/BodyId-sized integer): unsigned JSON number in [1, 2^53).
bool readSafeId(const json& v, std::uint64_t& out) {
    if (!v.is_number_unsigned()) return false;
    out = v.get<std::uint64_t>();
    return out >= 1 && out <= kMaxSafeId;
}

/// Member lookup on a JSON object; nullptr if `obj` is not an object or the key is absent.
const json* member(const json& obj, const char* key) {
    if (!obj.is_object()) return nullptr;
    auto it = obj.find(key);
    return it == obj.end() ? nullptr : &*it;
}

Result<json> buildManifest(const Document& doc) {
    json features = json::array();
    json suppressed = json::array();
    std::set<std::uint64_t> seen;
    std::uint64_t maxId = 0;
    for (std::size_t i = 0; i < doc.timeline.size(); ++i) {
        const recompute::Feature& f = *doc.timeline.at(i);
        const std::uint64_t id = f.id().value();
        if (id == 0 || id > kMaxSafeId) return makeError(ErrorCode::InvalidArgument, "feature id out of range");
        if (!seen.insert(id).second) return makeError(ErrorCode::InvalidArgument, "duplicate feature id");
        if (f.kind().empty()) return makeError(ErrorCode::InvalidArgument, "feature kind is empty");
        json params = json::parse(f.paramsJson(), nullptr, false);
        if (params.is_discarded() || !params.is_object())
            return makeError(ErrorCode::InvalidArgument, "feature params are not a JSON object");
        json entry = json::object();
        entry["id"] = id;
        entry["kind"] = std::string(f.kind());
        entry["params"] = std::move(params);
        // Written only when named, so an unnamed feature keeps the v1 entry shape.
        if (!f.name().empty()) entry["name"] = f.name();
        features.push_back(std::move(entry));
        if (doc.timeline.isSuppressed(f.id())) suppressed.push_back(id);
        maxId = std::max(maxId, id);
    }
    if (doc.nextId < 1 || doc.nextId > kMaxSafeId || doc.nextId <= maxId)
        return makeError(ErrorCode::InvalidArgument, "nextId must be in [1, 2^53) and greater than every feature id");

    // null = no rollback marker (equivalent to "at end"; Timeline reports end when unset).
    json rollback = nullptr;
    const std::size_t size = doc.timeline.size();
    if (doc.timeline.rollbackMarker() != size) rollback = doc.timeline.rollbackMarker();

    json params = json::array();
    for (const Parameter& p : doc.parameters.parameters()) {
        json e = json::object();
        e["name"] = p.name;
        e["expression"] = p.expression.source();
        e["unit"] = p.unit;
        e["comment"] = p.comment;
        e["favourite"] = p.favourite;
        params.push_back(std::move(e));
    }

    json timeline = json::object();
    timeline["rollbackMarker"] = rollback;
    timeline["suppressed"] = std::move(suppressed);
    timeline["features"] = std::move(features);

    json manifest = json::object();
    manifest["format"] = kFormatTag;
    manifest["version"] = kFormatVersion;
    manifest["name"] = doc.name;
    manifest["displayUnit"] = std::string(unitSymbol(doc.displayUnit));
    manifest["nextId"] = doc.nextId;
    manifest["timeline"] = std::move(timeline);
    manifest["parameters"] = std::move(params);
    json counters = json::object();
    for (const auto& [kind, n] : doc.nameCounters) counters[kind] = n;
    manifest["nameCounters"] = std::move(counters);
    return manifest;
}

/// Writes `text` as `<target>/manifest.json` via a staging directory and a rename. An existing target
/// directory is moved aside first and restored if the final rename fails.
Result<void> writeAtomically(const fs::path& target, const std::string& text) {
    fs::path parent = target.parent_path();
    if (parent.empty()) parent = ".";
    std::error_code ec;
    fs::create_directories(parent, ec);
    if (ec) return makeError(ErrorCode::IoError, "cannot create parent directory: " + ec.message());

    std::random_device rd;
    const std::string suffix = std::to_string(rd());
    const std::string base = target.filename().string();
    const fs::path staging = parent / (base + ".saving-" + suffix);
    const fs::path backup = parent / (base + ".old-" + suffix);
    auto dropStaging = [&staging] {
        std::error_code e;
        fs::remove_all(staging, e);
    };

    fs::create_directory(staging, ec);
    if (ec) return makeError(ErrorCode::IoError, "cannot create staging directory: " + ec.message());
    {
        std::ofstream out(staging / kManifestName, std::ios::binary | std::ios::trunc);
        if (!out) {
            dropStaging();
            return makeError(ErrorCode::IoError, "cannot write manifest.json");
        }
        out << text;
        out.flush();
        if (!out) {
            dropStaging();
            return makeError(ErrorCode::IoError, "failed writing manifest.json");
        }
    }
    // F3: the manifest and the staging directory entry must be on stable storage before the rename makes
    // them visible as the document; otherwise a power cut can publish an empty or truncated manifest.
    if (!fsyncPath(staging / kManifestName) || !fsyncPath(staging)) {
        dropStaging();
        return makeError(ErrorCode::IoError, "cannot flush document to disk");
    }

    const bool hadTarget = fs::exists(target, ec);
    if (ec) {
        dropStaging();
        return makeError(ErrorCode::IoError, "cannot inspect target: " + ec.message());
    }
    if (hadTarget) {
        fs::rename(target, backup, ec);
        if (ec) {
            dropStaging();
            return makeError(ErrorCode::IoError, "cannot move existing document aside: " + ec.message());
        }
    }
    fs::rename(staging, target, ec);
    if (ec) {
        std::error_code restore;
        if (hadTarget) fs::rename(backup, target, restore);
        dropStaging();
        return makeError(ErrorCode::IoError, "cannot publish document: " + ec.message());
    }
    // Make the renames durable: the parent directory holds the new entry for `target`.
    if (!fsyncPath(parent)) return makeError(ErrorCode::IoError, "cannot flush document directory to disk");
    if (hadTarget) {
        std::error_code e;
        fs::remove_all(backup, e);
    }
    return {};
}

Result<Document> buildDocument(const json& m, const recompute::FeatureRegistry& registry) {
    const json* fmt = member(m, "format");
    if (!fmt || !fmt->is_string() || fmt->get<std::string>() != kFormatTag)
        return makeError(ErrorCode::ParseError, "manifest is not an ironwood document");
    const json* version = member(m, "version");
    if (!version || !version->is_number_unsigned())
        return makeError(ErrorCode::ParseError, "manifest version is missing or invalid");
    if (version->get<std::uint64_t>() > kFormatVersion)
        return makeError(ErrorCode::ParseError, "unsupported document format version " +
                                                    std::to_string(version->get<std::uint64_t>()));

    const json* name = member(m, "name");
    const json* unit = member(m, "displayUnit");
    const json* nextId = member(m, "nextId");
    const json* timeline = member(m, "timeline");
    if (!name || !name->is_string()) return makeError(ErrorCode::ParseError, "manifest name is invalid");
    if (!unit || !unit->is_string()) return makeError(ErrorCode::ParseError, "manifest displayUnit is invalid");
    if (!nextId) return makeError(ErrorCode::ParseError, "manifest nextId is missing");
    if (!timeline || !timeline->is_object()) return makeError(ErrorCode::ParseError, "manifest timeline is invalid");

    Document doc;
    doc.name = name->get<std::string>();
    auto parsedUnit = parseLengthUnit(unit->get<std::string>());
    if (!parsedUnit) return makeError(ErrorCode::ParseError, "unknown displayUnit: " + unit->get<std::string>());
    doc.displayUnit = *parsedUnit;
    if (!readSafeId(*nextId, doc.nextId)) return makeError(ErrorCode::ParseError, "nextId is invalid");

    // v1: parameters. A v0 manifest has none, which means an empty table.
    if (const json* params = member(m, "parameters")) {
        if (!params->is_array()) return makeError(ErrorCode::ParseError, "manifest parameters is invalid");
        for (const json& e : *params) {
            const json* name = member(e, "name");
            const json* expr = member(e, "expression");
            const json* unit = member(e, "unit");
            const json* comment = member(e, "comment");
            const json* fav = member(e, "favourite");
            if (!name || !name->is_string() || !expr || !expr->is_string() || !unit || !unit->is_string() || !comment ||
                !comment->is_string() || !fav || !fav->is_boolean())
                return makeError(ErrorCode::ParseError, "manifest parameter entry is invalid");
            // Saved in insertion order, so every reference to a parameter defined earlier resolves as before.
            if (auto r = doc.parameters.add(name->get<std::string>(), expr->get<std::string>(), unit->get<std::string>(),
                                            comment->get<std::string>());
                !r)
                return makeError(ErrorCode::ParseError, "manifest parameter '" + name->get<std::string>() + "': " + r.error().message);
            if (fav->get<bool>()) {
                if (auto r = doc.parameters.setFavourite(name->get<std::string>(), true); !r)
                    return makeError(ErrorCode::ParseError, r.error().message);
            }
        }
    } else if (version->get<std::uint64_t>() >= 1) {
        return makeError(ErrorCode::ParseError, "manifest version 1 requires 'parameters'");
    }

    // v2: per-kind default-name counters (optional; absent in v0/v1).
    if (const json* counters = member(m, "nameCounters")) {
        if (!counters->is_object()) return makeError(ErrorCode::ParseError, "manifest nameCounters is invalid");
        for (const auto& [kind, n] : counters->items()) {
            if (!n.is_number_unsigned()) return makeError(ErrorCode::ParseError, "manifest nameCounters is invalid");
            doc.nameCounters[kind] = n.get<std::uint64_t>();
        }
    }

    const json* features = member(*timeline, "features");
    if (!features || !features->is_array()) return makeError(ErrorCode::ParseError, "timeline.features is invalid");

    std::set<std::uint64_t> ids;
    std::uint64_t maxId = 0;
    for (const json& entry : *features) {
        const json* idValue = member(entry, "id");
        const json* kind = member(entry, "kind");
        const json* params = member(entry, "params");
        std::uint64_t id = 0;
        if (!idValue || !readSafeId(*idValue, id)) return makeError(ErrorCode::ParseError, "feature id is invalid");
        if (!kind || !kind->is_string() || kind->get<std::string>().empty())
            return makeError(ErrorCode::ParseError, "feature kind is invalid");
        if (!params || !params->is_object()) return makeError(ErrorCode::ParseError, "feature params are invalid");
        if (!ids.insert(id).second) return makeError(ErrorCode::ParseError, "duplicate feature id");
        maxId = std::max(maxId, id);

        Result<std::shared_ptr<recompute::Feature>> created =
            registry.create(kind->get<std::string>(), FeatureId(id), params->dump());
        if (!created) return std::unexpected(created.error());
        if ((*created)->id() != FeatureId(id))
            return makeError(ErrorCode::ParseError, "feature factory returned a different id");
        // v2: optional display name (absent in v0/v1 files; the Document assigns defaults after loading).
        if (const json* fname = member(entry, "name")) {
            if (!fname->is_string()) return makeError(ErrorCode::ParseError, "feature name is invalid");
            (*created)->setName(fname->get<std::string>());
        }
        doc.timeline.append(std::move(*created));
    }
    if (doc.nextId <= maxId) return makeError(ErrorCode::ParseError, "nextId must be greater than every feature id");

    const json* suppressed = member(*timeline, "suppressed");
    if (!suppressed || !suppressed->is_array()) return makeError(ErrorCode::ParseError, "timeline.suppressed is invalid");
    for (const json& v : *suppressed) {
        std::uint64_t id = 0;
        if (!readSafeId(v, id) || !ids.contains(id))
            return makeError(ErrorCode::ParseError, "timeline.suppressed refers to an unknown feature");
        doc.timeline.setSuppressed(FeatureId(id), true);
    }

    const json* marker = member(*timeline, "rollbackMarker");
    if (!marker) return makeError(ErrorCode::ParseError, "timeline.rollbackMarker is missing");
    if (!marker->is_null()) {
        if (!marker->is_number_unsigned() || marker->get<std::uint64_t>() > doc.timeline.size())
            return makeError(ErrorCode::ParseError, "timeline.rollbackMarker is out of range");
        doc.timeline.setRollbackMarker(static_cast<std::size_t>(marker->get<std::uint64_t>()));
    }
    return doc;
}

} // namespace

Result<void> saveDocument(const fs::path& path, const Document& doc) {
    if (path.empty()) return makeError(ErrorCode::InvalidArgument, "empty document path");
    fs::path target = path.lexically_normal();
    if (!target.has_filename()) target = target.parent_path(); // "name/" -> "name"
    if (target.empty()) return makeError(ErrorCode::InvalidArgument, "empty document path");

    std::error_code ec;
    if (fs::exists(target, ec) && !fs::is_directory(target, ec))
        return makeError(ErrorCode::InvalidArgument, "document path exists and is not a directory");

    // Serialise completely before touching the filesystem: a bad document never disturbs the target.
    Result<json> manifest = buildManifest(doc);
    if (!manifest) return std::unexpected(manifest.error());
    std::string text;
    try {
        text = manifest->dump(2) + "\n";
    } catch (const json::exception& e) {
        return makeError(ErrorCode::InvalidArgument, std::string("cannot serialise document: ") + e.what());
    }
    return writeAtomically(target, text);
}

Result<Document> loadDocument(const fs::path& path, const recompute::FeatureRegistry& registry) {
    if (path.empty()) return makeError(ErrorCode::InvalidArgument, "empty document path");
    std::error_code ec;
    if (!fs::exists(path, ec)) return makeError(ErrorCode::NotFound, "document not found: " + path.string());
    if (!fs::is_directory(path, ec)) return makeError(ErrorCode::IoError, "not a document directory: " + path.string());

    const fs::path manifestPath = path / kManifestName;
    if (!fs::exists(manifestPath, ec)) return makeError(ErrorCode::ParseError, "document has no manifest.json");
    std::ifstream in(manifestPath, std::ios::binary);
    if (!in) return makeError(ErrorCode::IoError, "cannot open manifest.json");
    std::ostringstream buffer;
    buffer << in.rdbuf();
    if (in.bad()) return makeError(ErrorCode::IoError, "cannot read manifest.json");

    try {
        json manifest = json::parse(buffer.str(), nullptr, false);
        if (manifest.is_discarded()) return makeError(ErrorCode::ParseError, "manifest.json is not valid JSON");
        return buildDocument(manifest, registry);
    } catch (const json::exception& e) {
        return makeError(ErrorCode::ParseError, std::string("manifest.json is malformed: ") + e.what());
    }
}

} // namespace iw::io
