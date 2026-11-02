// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/import_feature.hpp"

#include <array>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <set>
#include <system_error>
#include <utility>

#include <nlohmann/json.hpp>

#include "features/feature_common.hpp"
#include "kernel/exchange.hpp"
#include "kernel/ops.hpp"
#include "naming/naming_map.hpp"

namespace iw::features {
namespace {

using json = nlohmann::json;
using recompute::Diagnostic;
using recompute::EvalContext;
using recompute::FeatureResult;

/// kernel::importStep uses OCCT global state (kernel/exchange.hpp: one thread at a time).
std::mutex& stepReaderMutex() {
    static std::mutex m;
    return m;
}

/// Row-major 4x4 product a * b.
Transform multiply(const Transform& a, const Transform& b) {
    Transform r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += a.m[static_cast<std::size_t>(i * 4 + k)] * b.m[static_cast<std::size_t>(k * 4 + j)];
            r.m[static_cast<std::size_t>(i * 4 + j)] = s;
        }
    return r;
}

std::string hex16(std::uint64_t v) {
    std::array<char, 17> buf{};
    std::snprintf(buf.data(), buf.size(), "%016llx", static_cast<unsigned long long>(v));
    return std::string(buf.data(), 16);
}

std::string missingMessage(const std::string& path) {
    return "import: source file '" + path +
           "' is missing; imported geometry is not embedded in the document yet, so the file must stay available "
           "(restore it at that path or re-import)";
}

} // namespace

Result<std::vector<ImportedSolid>> readStepSolids(const std::filesystem::path& path, CancelToken cancel) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) return makeError(ErrorCode::IoError, missingMessage(path.string()));
    Result<kernel::ImportedAssembly> assembly = [&] {
        std::lock_guard lock(stepReaderMutex());
        return kernel::importStep(path, cancel);
    }();
    if (!assembly) return std::unexpected(assembly.error());

    // World placement of every node: parent world * local (nodes are stored parent before child).
    const auto& nodes = assembly->nodes;
    std::vector<Transform> world(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const std::int32_t parent = nodes[i].parent;
        world[i] = parent >= 0 && static_cast<std::size_t>(parent) < i ? multiply(world[static_cast<std::size_t>(parent)], nodes[i].local)
                                                                         : nodes[i].local;
    }
    std::vector<ImportedSolid> out;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i].shape.isNull()) continue;
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "import cancelled");
        kernel::Shape placed = nodes[i].shape;
        if (!(world[i] == Transform{})) {
            auto moved = kernel::transform(placed, world[i], cancel);
            if (!moved) return std::unexpected(moved.error());
            placed = std::move(moved->shape);
        }
        for (auto& solid : placed.solids()) out.push_back(ImportedSolid{nodes[i].name, std::move(solid)});
    }
    return out;
}

Result<std::string> fileContentHash(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return makeError(ErrorCode::IoError, missingMessage(path.string()));
    common::Fnv f;
    std::array<char, 1 << 16> buf{};
    while (in) {
        in.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const auto n = static_cast<std::size_t>(in.gcount());
        if (n > 0) f.bytes(std::string_view(buf.data(), n));
    }
    if (in.bad()) return makeError(ErrorCode::IoError, "import: cannot read '" + path.string() + "'");
    return hex16(f.h);
}

std::uint64_t ImportFeature::paramHash() const {
    common::Fnv f;
    f.bytes("import/v1:");
    f.bytes(paramsJson());
    return f.h;
}

std::string ImportFeature::paramsJson() const {
    json bodies = json::array();
    for (BodyId b : params_.bodies) bodies.push_back(b.value());
    json j{{"path", params_.path}, {"bodies", bodies}, {"content_hash", params_.contentHash}};
    return j.dump();
}

Result<std::shared_ptr<recompute::Feature>> ImportFeature::fromJson(FeatureId id, std::string_view text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::ParseError, "import: params are not a JSON object");
    ImportParams p;
    auto path = j.find("path");
    if (path == j.end() || !path->is_string() || path->get<std::string>().empty())
        return makeError(ErrorCode::ParseError, "import: 'path' must be a non-empty string");
    p.path = path->get<std::string>();
    auto bodies = j.find("bodies");
    if (bodies == j.end() || !bodies->is_array() || bodies->empty())
        return makeError(ErrorCode::ParseError, "import: 'bodies' must be a non-empty array of body ids");
    std::set<std::uint64_t> seen;
    for (const auto& b : *bodies) {
        if (!b.is_number_unsigned() || b.get<std::uint64_t>() == 0)
            return makeError(ErrorCode::ParseError, "import: 'bodies' entries must be non-zero unsigned ids");
        if (!seen.insert(b.get<std::uint64_t>()).second) return makeError(ErrorCode::ParseError, "import: duplicate body id");
        p.bodies.push_back(BodyId{b.get<std::uint64_t>()});
    }
    if (auto h = j.find("content_hash"); h != j.end()) {
        if (!h->is_string()) return makeError(ErrorCode::ParseError, "import: 'content_hash' must be a string");
        p.contentHash = h->get<std::string>();
    }
    return std::shared_ptr<recompute::Feature>(std::make_shared<ImportFeature>(id, std::move(p)));
}

Result<FeatureResult> ImportFeature::compute(const EvalContext& ctx) const {
    const ImportParams& p = params_;
    if (p.path.empty()) return makeError(ErrorCode::InvalidArgument, "import: no source file");
    if (p.bodies.empty()) return makeError(ErrorCode::InvalidArgument, "import: no target bodies");
    std::set<BodyId> unique;
    for (BodyId b : p.bodies) {
        if (!b.valid()) return makeError(ErrorCode::InvalidArgument, "import: invalid body id");
        if (!unique.insert(b).second) return makeError(ErrorCode::InvalidArgument, "import: duplicate body id");
        if (ctx.body(b)) return makeError(ErrorCode::InvalidArgument, "import: body " + std::to_string(b.value()) + " already exists");
    }

    auto solids = readStepSolids(p.path, ctx.cancel);
    if (!solids) return std::unexpected(solids.error());
    std::vector<Diagnostic> diags;
    if (!p.contentHash.empty()) {
        auto hash = fileContentHash(p.path);
        if (!hash) return std::unexpected(hash.error());
        if (*hash != p.contentHash)
            diags.push_back({Diagnostic::Severity::Warning,
                             "import: '" + p.path + "' changed since it was imported; face and edge names may have changed",
                             std::nullopt});
    }
    if (solids->size() < p.bodies.size())
        return makeError(ErrorCode::NotFound, "import: '" + p.path + "' now contains " + std::to_string(solids->size()) +
                                                  " solid(s); the import expects " + std::to_string(p.bodies.size()));
    if (solids->size() > p.bodies.size())
        diags.push_back({Diagnostic::Severity::Warning,
                         "import: '" + p.path + "' now contains " + std::to_string(solids->size()) + " solids; only the first " +
                             std::to_string(p.bodies.size()) + " are imported",
                         std::nullopt});

    FeatureResult out;
    for (std::size_t k = 0; k < p.bodies.size(); ++k) {
        if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "import cancelled");
        const kernel::Shape& shape = (*solids)[k].shape;
        auto names = naming::deriveNamingMap(id_, "import.solid" + std::to_string(k), shape, {}, kernel::ShapeHistory{});
        if (!names) return std::unexpected(names.error());
        out.bodies.emplace(p.bodies[k], shape);
        out.namings.emplace(p.bodies[k], std::move(*names));
    }
    out.diagnostics = std::move(diags);
    out.attributes.emplace("source", p.path);
    out.attributes.emplace("solids", std::to_string(p.bodies.size()));
    return out;
}

void registerImportFeature(recompute::FeatureRegistry& registry) {
    registry.add("import", [](FeatureId id, std::string_view p) { return ImportFeature::fromJson(id, p); });
}

} // namespace iw::features
