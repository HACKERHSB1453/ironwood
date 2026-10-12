// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "core/cancel.hpp"
#include "core/math.hpp"
#include "core/result.hpp"
#include "kernel/shape.hpp"

/// File exchange (STEP via OCCT XDE, STL). Same contract as ops.hpp: no exceptions, thread-safety UNVERIFIED
/// (OCCT STEP readers use global state; call from one thread at a time until THREAD_SAFETY.md says otherwise).
namespace iw::kernel {

struct Color {
    float r = 0, g = 0, b = 0, a = 1;
    friend bool operator==(const Color&, const Color&) = default;
};

/// One node of an imported assembly tree (flat storage; parent/children are indices into `nodes`).
struct AssemblyNode {
    std::string name;
    std::optional<Color> color;     ///< Resolved surface colour if the file defines one.
    Transform local;                ///< Placement relative to the parent node.
    Shape shape;                    ///< Geometry owned by THIS node (null for pure grouping nodes).
    std::int32_t parent = -1;       ///< -1 for roots.
    std::vector<std::uint32_t> children;
};

/// Tree produced by importStep. `nodes[roots[i]]` are the top-level nodes. Instances (occurrences) of the
/// same product are expanded: each occurrence is its own node, sharing the same Shape handle (O(1) copies).
struct ImportedAssembly {
    std::vector<AssemblyNode> nodes;
    std::vector<std::uint32_t> roots;
    std::string lengthUnitInFile; ///< e.g. "mm", "inch"; geometry is already converted to mm.
};

/// Reads a STEP file (AP203/214/242 geometry) with names, colours and assembly structure via XDE.
/// Errors: IoError (cannot open), ParseError (not STEP / corrupt), Cancelled.
Result<ImportedAssembly> importStep(const std::filesystem::path& path, CancelToken cancel = {});

/// Reads a binary or ASCII STL into a shape made of one planar triangle face per facet (sewn when
/// possible). Units are taken as mm.
Result<Shape> importStl(const std::filesystem::path& path, CancelToken cancel = {});

/// Writes a STEP file (overwrites). Names/colours/transforms from the assembly are preserved.
Result<void> exportStep(const std::filesystem::path& path, const ImportedAssembly& assembly);
/// Convenience: writes a single shape as a one-node assembly.
Result<void> exportStep(const std::filesystem::path& path, const Shape& shape);

struct StlOptions {
    double linearDeflection = 0.1;   ///< mm
    double angularDeflection = 0.5;  ///< radians
    bool binary = true;
};
/// Tessellates (parallel inside the kernel) and writes an STL.
Result<void> exportStl(const std::filesystem::path& path, const Shape& shape, const StlOptions& options = {},
                       CancelToken cancel = {});

/// One solid to export as a 3MF object. `color` is the sRGB display colour (alpha written as opaque); absent
/// means the exporter's neutral grey.
struct MeshBody {
    std::string name;
    Shape shape;
    std::optional<Color> color;
};

/// Writes a 3MF package (OPC zip: [Content_Types].xml, _rels/.rels, 3D/3dmodel.model) with one <object> per body,
/// `unit="millimeter"`, and colours as one <basematerials> group. Each body is tessellated with `options`
/// (same BRepMesh parameters as exportStl; per-face vertices, no welding, so counts equal tess::tessellate with
/// the same deflections). The zip is written with the STORED method (no compression) and CRC-32 checked entries.
/// Errors: InvalidArgument (no bodies, null shape, empty path, bad deflection), KernelFailure (a face
/// failed to mesh or OCCT threw), IoError, Cancelled.
Result<void> export3mf(const std::filesystem::path& path, const std::vector<MeshBody>& bodies,
                       const StlOptions& options = {}, CancelToken cancel = {});

} // namespace iw::kernel
