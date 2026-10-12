// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/import_job.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <optional>
#include <string>
#include <utility>

namespace iw::ui {
namespace {

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

double msSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

std::string lowerExtension(const fs::path& p) {
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
    return ext;
}

/// Depth-first walk: records every node with geometry together with its world placement and the colour it
/// inherits (own colour first, then the nearest ancestor's, then the default grey).
void collectLeaves(const kernel::ImportedAssembly& a, std::uint32_t idx, const Transform& parentWorld,
                   std::optional<kernel::Color> inherited, std::vector<ImportedLeaf>& out,
                   std::vector<Transform>& worldOf) {
    const kernel::AssemblyNode& n = a.nodes[idx];
    const Transform world = multiply(parentWorld, n.local);
    worldOf[idx] = world;
    const std::optional<kernel::Color> color = n.color ? n.color : inherited;
    if (!n.shape.isNull()) {
        ImportedLeaf leaf;
        leaf.node = idx;
        leaf.world = world;
        if (color) leaf.color = {color->r, color->g, color->b};
        out.push_back(std::move(leaf));
    }
    for (std::uint32_t c : n.children) collectLeaves(a, c, world, color, out, worldOf);
}

} // namespace

bool isSupportedCadFile(const fs::path& path) {
    const std::string ext = lowerExtension(path);
    return ext == ".step" || ext == ".stp" || ext == ".stl";
}

Transform multiply(const Transform& a, const Transform& b) {
    Transform r;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col) {
            double s = 0;
            for (int k = 0; k < 4; ++k)
                s += a.m[static_cast<std::size_t>(row * 4 + k)] * b.m[static_cast<std::size_t>(k * 4 + col)];
            r.m[static_cast<std::size_t>(row * 4 + col)] = s;
        }
    return r;
}

Result<LoadedModel> loadModel(const fs::path& path, const CancelToken& cancel, ImportProgress& progress) {
    if (!isSupportedCadFile(path)) return makeError(ErrorCode::InvalidArgument, "unsupported file type: " + path.string());
    LoadedModel model;
    model.sourceName = path.filename().string();
    progress.stage = 0;
    progress.done = 0;
    progress.total = 0;

    const auto t0 = Clock::now();
    if (lowerExtension(path) == ".stl") {
        auto shape = kernel::importStl(path, cancel);
        if (!shape) return std::unexpected(shape.error());
        kernel::AssemblyNode node;
        node.name = path.stem().string();
        node.shape = std::move(*shape);
        model.assembly.nodes.push_back(std::move(node));
        model.assembly.roots.push_back(0);
        model.assembly.lengthUnitInFile = "mm";
    } else {
        auto assembly = kernel::importStep(path, cancel);
        if (!assembly) return std::unexpected(assembly.error());
        model.assembly = std::move(*assembly);
    }
    model.importMs = msSince(t0);
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "import cancelled");

    std::vector<ImportedLeaf> leaves;
    std::vector<Transform> worldOf(model.assembly.nodes.size());
    for (std::uint32_t r : model.assembly.roots)
        collectLeaves(model.assembly, r, Transform{}, std::nullopt, leaves, worldOf);
    progress.total = static_cast<std::uint32_t>(leaves.size());
    progress.stage = 1;

    // Occurrences of one product share a Shape handle: tessellate each distinct shape once.
    const auto t1 = Clock::now();
    std::vector<std::pair<kernel::Shape, std::shared_ptr<const tess::MeshData>>> cache;
    const tess::TessParams params{};
    for (ImportedLeaf& leaf : leaves) {
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "import cancelled");
        const kernel::Shape& shape = model.assembly.nodes[leaf.node].shape;
        auto hit = std::find_if(cache.begin(), cache.end(), [&](const auto& e) { return e.first.isSame(shape); });
        if (hit != cache.end()) {
            leaf.mesh = hit->second;
        } else {
            auto mesh = tess::tessellate(shape, params, cancel);
            if (!mesh) return std::unexpected(mesh.error());
            leaf.mesh = std::make_shared<const tess::MeshData>(std::move(*mesh));
            cache.emplace_back(shape, leaf.mesh);
        }
        progress.done = progress.done + 1;
    }
    model.tessellateMs = msSince(t1);
    model.leaves = std::move(leaves);
    return model;
}

} // namespace iw::ui
