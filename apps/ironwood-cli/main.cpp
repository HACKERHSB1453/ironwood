// SPDX-License-Identifier: GPL-3.0-or-later
#include <sys/resource.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

#include "core/version.hpp"
#include "kernel/exchange.hpp"
#include "kernel/shape.hpp"
#include "tess/tessellate.hpp"

namespace {

using Clock = std::chrono::steady_clock;

void usage() {
    std::puts("usage: ironwood-cli --version\n"
              "       ironwood-cli import <file.step|file.stp|file.stl> [--stats]");
}

enum class Format { Step, Stl, Unknown };

Format formatOf(const std::filesystem::path& path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext == ".step" || ext == ".stp") return Format::Step;
    if (ext == ".stl") return Format::Stl;
    return Format::Unknown;
}

// Totals over the geometry of an import. For an assembly every occurrence (node) is counted, so instanced
// products count once per placement. Solid count = shapes whose top-level type is Solid; compounds are not
// enumerated (the kernel exposes no solid iterator yet), so they contribute to faces/edges/triangles only.
struct ShapeStats {
    std::size_t solids = 0, faces = 0, edges = 0, vertices = 0, triangles = 0;
    bool meshFailed = false;
};

void accumulate(ShapeStats& s, const iw::kernel::Shape& shape) {
    if (shape.isNull()) return;
    if (shape.type() == iw::kernel::ShapeType::Solid) ++s.solids;
    s.faces += shape.subShapeCount(iw::kernel::SubShapeKind::Face);
    s.edges += shape.subShapeCount(iw::kernel::SubShapeKind::Edge);
    s.vertices += shape.subShapeCount(iw::kernel::SubShapeKind::Vertex);
    auto mesh = iw::tess::tessellate(shape, {});
    if (!mesh) {
        std::fprintf(stderr, "warning: tessellation failed [%s]: %s\n", iw::errorCodeName(mesh.error().code),
                     mesh.error().message.c_str());
        s.meshFailed = true;
        return;
    }
    s.triangles += mesh->indices.size() / 3;
}

double peakRssMiB() {
    rusage ru{};
    if (getrusage(RUSAGE_SELF, &ru) != 0) return -1.0;
    return static_cast<double>(ru.ru_maxrss) / 1024.0; // ru_maxrss is KiB on Linux
}

void printStats(const ShapeStats& s, double importMs) {
    std::printf("solids: %zu\nfaces: %zu\nedges: %zu\nvertices: %zu\n", s.solids, s.faces, s.edges, s.vertices);
    std::printf("triangles: %zu (linear deflection 0.1 mm)\n", s.triangles);
    std::printf("import time (cold, incl. XDE read): %.1f ms\n", importMs);
    std::printf("peak RSS: %.1f MiB\n", peakRssMiB());
}

int cmdImport(int argc, char** argv) {
    if (argc < 3) {
        usage();
        return 64;
    }
    bool stats = false;
    for (int i = 3; i < argc; ++i)
        if (std::strcmp(argv[i], "--stats") == 0) stats = true;

    const std::filesystem::path file = argv[2];
    const Format format = formatOf(file);
    if (format == Format::Unknown) {
        std::fprintf(stderr, "error: unsupported file extension '%s' (expected .step, .stp or .stl)\n",
                     file.extension().string().c_str());
        return 64;
    }

    if (format == Format::Stl) {
        const auto t0 = Clock::now();
        auto shape = iw::kernel::importStl(file);
        const double importMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        if (!shape) {
            std::fprintf(stderr, "error [%s]: %s\n", iw::errorCodeName(shape.error().code),
                         shape.error().message.c_str());
            return 2;
        }
        if (stats) {
            std::printf("nodes: 1\nroots: 1\n");
            ShapeStats s;
            accumulate(s, *shape);
            printStats(s, importMs);
            if (s.meshFailed) return 2;
        }
        return 0;
    }

    const auto t0 = Clock::now();
    auto asmbly = iw::kernel::importStep(file);
    const double importMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    if (!asmbly) {
        std::fprintf(stderr, "error [%s]: %s\n", iw::errorCodeName(asmbly.error().code),
                     asmbly.error().message.c_str());
        return 2;
    }
    if (stats) {
        std::printf("nodes: %zu\nroots: %zu\nlength unit in file: %s\n", asmbly->nodes.size(),
                    asmbly->roots.size(), asmbly->lengthUnitInFile.c_str());
        ShapeStats s;
        for (const auto& node : asmbly->nodes) accumulate(s, node.shape);
        printStats(s, importMs);
        if (s.meshFailed) return 2;
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc >= 2 && (std::strcmp(argv[1], "--version") == 0 || std::strcmp(argv[1], "-V") == 0)) {
        std::printf("ironwood-cli %s (OCCT %s)\n", iw::kVersion, iw::kernel::occtVersion().c_str());
        return 0;
    }
    if (argc >= 2 && std::strcmp(argv[1], "import") == 0) return cmdImport(argc, argv);
    usage();
    return 64;
}
