// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <vector>

#include "core/cancel.hpp"
#include "core/math.hpp"
#include "core/result.hpp"
#include "kernel/shape.hpp"

namespace iw::tess {

/// Renderable triangle mesh of one Shape plus the topology mapping needed for picking/highlighting.
/// Plain value type (movable, thread-safe to read). Single index space across all faces (one draw call).
struct MeshData {
    /// A face's triangles occupy `indices[firstIndex, firstIndex+indexCount)`. Entry i = B-rep face index i
    /// (kernel sub-shape ordering). Faces that failed to mesh have indexCount == 0.
    struct FaceRange {
        std::uint32_t firstIndex = 0;
        std::uint32_t indexCount = 0;
    };
    /// Discretised B-rep edge as xyz triples (mm); entry i = B-rep edge index i. Empty if none.
    struct EdgePolyline {
        std::vector<float> points;
    };

    std::vector<float> positions;  ///< xyz per vertex (mm).
    std::vector<float> normals;    ///< xyz per vertex, unit length, same count as positions.
    std::vector<std::uint32_t> indices;  ///< Triangle list, CCW seen from outside.
    std::vector<FaceRange> faceRanges;   ///< size == shape.subShapeCount(Face).
    std::vector<EdgePolyline> edgePolylines; ///< size == shape.subShapeCount(Edge).
    Box3 bounds;
};

struct TessParams {
    double linearDeflection = 0.1;   ///< Max chordal deviation; absolute mm, or fraction of size if `relative`.
    double angularDeflection = 0.5;  ///< Max angle between adjacent facet normals, radians.
    bool relative = false;
};

/// Tessellates `shape`. Internally parallel (per-face, via kernel/TBB); the call itself blocks the caller,
/// so invoke it from a job, not the UI thread. Triangulation is crack-free across shared edges.
/// Does not mutate `shape` (any OCCT triangulation cache is kept private to the call).
/// Deterministic: same shape + params -> bit-identical MeshData. Errors: InvalidArgument (null shape),
/// KernelFailure, Cancelled (token observed; partial result discarded).
Result<MeshData> tessellate(const kernel::Shape& shape, const TessParams& params, CancelToken cancel = {});

} // namespace iw::tess
