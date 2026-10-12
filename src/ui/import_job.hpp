// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "core/cancel.hpp"
#include "core/math.hpp"
#include "core/result.hpp"
#include "kernel/exchange.hpp"
#include "tess/tessellate.hpp"

namespace iw::ui {

/// One geometry-carrying node of an imported assembly, tessellated once and placed in world space.
struct ImportedLeaf {
    std::uint32_t node = 0;                                ///< Index into LoadedModel::assembly.nodes.
    std::shared_ptr<const tess::MeshData> mesh;            ///< Shared between occurrences of the same Shape.
    Transform world;                                       ///< Product of the placements from the root down.
    std::array<float, 3> color{0.68f, 0.72f, 0.78f};       ///< STEP colour (own or inherited), default grey.
};

/// Result of a successful import: the assembly tree (names, hierarchy) plus renderable leaves.
struct LoadedModel {
    kernel::ImportedAssembly assembly;
    std::vector<ImportedLeaf> leaves;   ///< Viewport body order (depth-first over the tree).
    std::string sourceName;             ///< File name without directory.
    double importMs = 0;                ///< Wall time of kernel::importStep / importStl.
    double tessellateMs = 0;            ///< Wall time of all tess::tessellate calls.
};

/// Progress shared between an import job (writer) and the GUI poller (reader). Lock-free.
struct ImportProgress {
    std::atomic<std::uint32_t> stage{0};  ///< 0 = reading the file, 1 = tessellating leaves.
    std::atomic<std::uint32_t> done{0};   ///< Leaves tessellated so far.
    std::atomic<std::uint32_t> total{0};  ///< Leaves to tessellate (known after reading).
};

/// True for the file types the GUI opens (.step, .stp, .stl; case-insensitive).
bool isSupportedCadFile(const std::filesystem::path& path);

/// Reads `path` and tessellates every leaf shape. Pure C++ (no Qt): meant to run on a JobSystem worker.
/// Errors come from the kernel (IoError, ParseError, ...) or Cancelled when `cancel` is observed.
Result<LoadedModel> loadModel(const std::filesystem::path& path, const CancelToken& cancel, ImportProgress& progress);

/// Row-major 4x4 product a * b (apply b first, then a).
Transform multiply(const Transform& a, const Transform& b);

} // namespace iw::ui
