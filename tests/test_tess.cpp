// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#include "core/cancel.hpp"
#include "kernel/ops.hpp"
#include "kernel/shape.hpp"
#include "tess/tessellate.hpp"

using namespace iw;
using iw::kernel::Shape;
using iw::kernel::SubShapeKind;
using iw::tess::MeshData;
using iw::tess::TessParams;
using iw::tess::tessellate;

namespace {

Shape boxShape(double dx = 10, double dy = 20, double dz = 30) {
    auto r = kernel::makeBox({0, 0, 0}, dx, dy, dz);
    EXPECT_TRUE(r.has_value());
    return r->shape;
}

Shape cylinderShape(double radius = 5, double height = 40) {
    auto r = kernel::makeCylinder({0, 0, 0}, {0, 0, 1}, radius, height);
    EXPECT_TRUE(r.has_value());
    return r->shape;
}

Shape sphereShape(double radius = 5) {
    auto r = kernel::makeSphere({1, 2, 3}, radius);
    EXPECT_TRUE(r.has_value());
    return r->shape;
}

MeshData mustTessellate(const Shape& s, TessParams p = {}) {
    auto r = tessellate(s, p);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message);
    return r ? std::move(*r) : MeshData{};
}

double triangleArea(const MeshData& m, std::uint32_t tri) {
    const auto v = [&](std::uint32_t i) {
        return std::array<double, 3>{m.positions[3 * m.indices[i]], m.positions[3 * m.indices[i] + 1],
                                     m.positions[3 * m.indices[i] + 2]};
    };
    const auto a = v(3 * tri), b = v(3 * tri + 1), c = v(3 * tri + 2);
    const double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
    const double wx = c[0] - a[0], wy = c[1] - a[1], wz = c[2] - a[2];
    const double cx = uy * wz - uz * wy, cy = uz * wx - ux * wz, cz = ux * wy - uy * wx;
    return 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
}

double signedVolume(const MeshData& m) {
    double vol = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const auto p = [&](std::uint32_t i, int k) { return static_cast<double>(m.positions[3 * i + k]); };
        const std::uint32_t i0 = m.indices[t], i1 = m.indices[t + 1], i2 = m.indices[t + 2];
        vol += (p(i0, 0) * (p(i1, 1) * p(i2, 2) - p(i1, 2) * p(i2, 1)) -
                p(i0, 1) * (p(i1, 0) * p(i2, 2) - p(i1, 2) * p(i2, 0)) +
                p(i0, 2) * (p(i1, 0) * p(i2, 1) - p(i1, 1) * p(i2, 0))) /
               6.0;
    }
    return vol;
}

// Welds vertices by quantised position and counts undirected triangle edges. Crack-free and closed <=> every
// edge is shared by exactly two triangles.
bool isWatertight(const MeshData& m) {
    using Key = std::tuple<long long, long long, long long>;
    std::map<Key, std::uint32_t> weld;
    std::vector<std::uint32_t> id(m.positions.size() / 3);
    for (std::size_t v = 0; v < id.size(); ++v) {
        Key k{std::llround(m.positions[3 * v] * 1e4), std::llround(m.positions[3 * v + 1] * 1e4),
              std::llround(m.positions[3 * v + 2] * 1e4)};
        id[v] = weld.emplace(k, static_cast<std::uint32_t>(weld.size())).first->second;
    }
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        for (int e = 0; e < 3; ++e) {
            std::uint32_t a = id[m.indices[t + e]], b = id[m.indices[t + (e + 1) % 3]];
            if (a == b) continue;
            if (a > b) std::swap(a, b);
            ++edges[{a, b}];
        }
    }
    for (const auto& [edge, count] : edges)
        if (count != 2) return false;
    return !edges.empty();
}

} // namespace

TEST(Tess, NullShapeIsInvalidArgument) {
    EXPECT_EQ(tessellate(Shape{}, {}).error().code, ErrorCode::InvalidArgument);
}

TEST(Tess, NonPositiveDeflectionIsInvalidArgument) {
    TessParams p;
    p.linearDeflection = 0;
    EXPECT_EQ(tessellate(boxShape(), p).error().code, ErrorCode::InvalidArgument);
    p = {};
    p.angularDeflection = -1;
    EXPECT_EQ(tessellate(boxShape(), p).error().code, ErrorCode::InvalidArgument);
}

TEST(Tess, BoxHasTwelveTrianglesSixFacesTwelveEdges) {
    const Shape box = boxShape();
    const MeshData m = mustTessellate(box);
    EXPECT_EQ(m.indices.size() / 3, 12u);
    ASSERT_EQ(m.faceRanges.size(), 6u);
    ASSERT_EQ(m.faceRanges.size(), box.subShapeCount(SubShapeKind::Face));
    for (const auto& fr : m.faceRanges) EXPECT_GT(fr.indexCount, 0u);
    std::uint32_t covered = 0;
    for (const auto& fr : m.faceRanges) covered += fr.indexCount;
    EXPECT_EQ(covered, m.indices.size());

    ASSERT_EQ(m.edgePolylines.size(), 12u);
    ASSERT_EQ(m.edgePolylines.size(), box.subShapeCount(SubShapeKind::Edge));
    for (const auto& e : m.edgePolylines) EXPECT_GE(e.points.size(), 6u);
    EXPECT_EQ(m.normals.size(), m.positions.size());
}

TEST(Tess, FaceRangesMatchKernelFaceOrdering) {
    // Each face range's triangle area must equal the kernel's measure for the same face index (ordering contract).
    const Shape box = boxShape(10, 20, 30);
    const MeshData m = mustTessellate(box);
    for (std::size_t f = 0; f < m.faceRanges.size(); ++f) {
        const auto& fr = m.faceRanges[f];
        double area = 0;
        for (std::uint32_t t = fr.firstIndex / 3; t < (fr.firstIndex + fr.indexCount) / 3; ++t)
            area += triangleArea(m, t);
        const double expected = box.subShapeKey(SubShapeKind::Face, f)[3];
        EXPECT_NEAR(area, expected, expected * 1e-6) << "face " << f;
    }
}

TEST(Tess, EdgePolylineLengthMatchesKernelEdgeOrdering) {
    const Shape cyl = cylinderShape();
    const MeshData m = mustTessellate(cyl);
    ASSERT_EQ(m.edgePolylines.size(), cyl.subShapeCount(SubShapeKind::Edge));
    for (std::size_t e = 0; e < m.edgePolylines.size(); ++e) {
        const auto& p = m.edgePolylines[e].points;
        double len = 0;
        for (std::size_t i = 3; i + 2 < p.size(); i += 3) {
            const double dx = p[i] - p[i - 3], dy = p[i + 1] - p[i - 2], dz = p[i + 2] - p[i - 1];
            len += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        const double expected = cyl.subShapeKey(SubShapeKind::Edge, e)[3];
        if (expected < 1e-9) continue; // degenerate seam/pole edges
        // Chordal discretisation at linearDeflection 0.1 on R5 is ~0.7% short of the true arc length.
        EXPECT_NEAR(len, expected, expected * 2e-2 + 1e-6) << "edge " << e;
    }
}

TEST(Tess, CylinderNormalsAreUnitLength) {
    const MeshData m = mustTessellate(cylinderShape());
    ASSERT_FALSE(m.normals.empty());
    for (std::size_t i = 0; i < m.normals.size(); i += 3) {
        const double len = std::sqrt(static_cast<double>(m.normals[i]) * m.normals[i] +
                                     static_cast<double>(m.normals[i + 1]) * m.normals[i + 1] +
                                     static_cast<double>(m.normals[i + 2]) * m.normals[i + 2]);
        ASSERT_NEAR(len, 1.0, 1e-5) << "vertex " << i / 3;
    }
}

TEST(Tess, SphereNormalsUnitLengthIncludingPoles) {
    // Pole vertices have no usable surface normal (the parametrisation collapses there); their normals come from
    // the accumulated triangle fallback and must still be unit length and point outward.
    const double R = 5;
    const MeshData m = mustTessellate(sphereShape(R));
    ASSERT_FALSE(m.normals.empty());
    ASSERT_EQ(m.normals.size(), m.positions.size());
    std::size_t nearPole = 0;
    for (std::size_t v = 0; v < m.normals.size() / 3; ++v) {
        const double nx = m.normals[3 * v], ny = m.normals[3 * v + 1], nz = m.normals[3 * v + 2];
        ASSERT_NEAR(std::sqrt(nx * nx + ny * ny + nz * nz), 1.0, 1e-5) << "vertex " << v;
        const double dx = m.positions[3 * v] - 1.0, dy = m.positions[3 * v + 1] - 2.0, dz = m.positions[3 * v + 2] - 3.0;
        const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
        ASSERT_NEAR(len, R, 1e-4) << "vertex " << v << " is not on the sphere";
        EXPECT_GT((dx * nx + dy * ny + dz * nz) / len, 0.99) << "vertex " << v << " normal not outward";
        if (std::abs(dz) > 0.99 * R) ++nearPole;
    }
    EXPECT_GT(nearPole, 0u) << "mesh has no vertices near the poles to check";
}

TEST(Tess, OutwardOrientationGivesBoxVolume) {
    // CCW-from-outside winding: signed volume is positive and equals the box volume.
    const MeshData m = mustTessellate(boxShape(10, 20, 30));
    EXPECT_NEAR(signedVolume(m), 10.0 * 20.0 * 30.0, 1e-2);
    // Normals point outward: each vertex normal agrees with its offset from the box centre.
    for (std::size_t v = 0; v < m.positions.size() / 3; ++v) {
        const double dx = m.positions[3 * v] - 5.0, dy = m.positions[3 * v + 1] - 10.0,
                     dz = m.positions[3 * v + 2] - 15.0;
        const double dot = dx * m.normals[3 * v] + dy * m.normals[3 * v + 1] + dz * m.normals[3 * v + 2];
        EXPECT_GE(dot, -1e-3) << "vertex " << v;
    }
}

TEST(Tess, TighterDeflectionGivesMoreTriangles) {
    const Shape cyl = cylinderShape(5, 40);
    TessParams coarse, fine;
    coarse.linearDeflection = 1.0;
    fine.linearDeflection = 0.05;
    const auto nCoarse = mustTessellate(cyl, coarse).indices.size();
    const auto nFine = mustTessellate(cyl, fine).indices.size();
    EXPECT_GT(nFine, nCoarse);
}

TEST(Tess, BoxIsWatertight) {
    EXPECT_TRUE(isWatertight(mustTessellate(boxShape())));
}

TEST(Tess, CylinderIsCrackFree) {
    EXPECT_TRUE(isWatertight(mustTessellate(cylinderShape())));
}

TEST(Tess, BitIdenticalOverFiftyRuns) {
    const Shape cyl = cylinderShape();
    const MeshData ref = mustTessellate(cyl);
    for (int run = 1; run < 50; ++run) {
        const MeshData m = mustTessellate(cyl);
        ASSERT_EQ(m.positions, ref.positions) << "run " << run;
        ASSERT_EQ(m.normals, ref.normals) << "run " << run;
        ASSERT_EQ(m.indices, ref.indices) << "run " << run;
        ASSERT_EQ(m.faceRanges.size(), ref.faceRanges.size());
        for (std::size_t f = 0; f < m.faceRanges.size(); ++f) {
            ASSERT_EQ(m.faceRanges[f].firstIndex, ref.faceRanges[f].firstIndex);
            ASSERT_EQ(m.faceRanges[f].indexCount, ref.faceRanges[f].indexCount);
        }
        ASSERT_EQ(m.edgePolylines.size(), ref.edgePolylines.size());
        for (std::size_t e = 0; e < m.edgePolylines.size(); ++e)
            ASSERT_EQ(m.edgePolylines[e].points, ref.edgePolylines[e].points);
    }
}

TEST(Tess, CancelledTokenReturnsCancelled) {
    CancelSource cs;
    cs.cancel();
    EXPECT_EQ(tessellate(boxShape(), {}, cs.token()).error().code, ErrorCode::Cancelled);
    EXPECT_EQ(tessellate(cylinderShape(), {}, cs.token()).error().code, ErrorCode::Cancelled);
}

TEST(Tess, InputShapeIsUnchangedAfterTessellation) {
    const Shape box = boxShape();
    const auto boundsBefore = box.boundingBox();
    const auto facesBefore = box.subShapeCount(SubShapeKind::Face);
    const auto stampBefore = box.stamp();
    const MeshData first = mustTessellate(box);
    TessParams fine;
    fine.linearDeflection = 0.01;
    (void)mustTessellate(box, fine);
    const MeshData again = mustTessellate(box);
    EXPECT_EQ(box.boundingBox().min, boundsBefore.min);
    EXPECT_EQ(box.boundingBox().max, boundsBefore.max);
    EXPECT_EQ(box.subShapeCount(SubShapeKind::Face), facesBefore);
    EXPECT_EQ(box.stamp(), stampBefore);
    EXPECT_EQ(again.positions, first.positions);
    EXPECT_EQ(again.indices, first.indices);
}

TEST(Tess, BoundsCoverAllVertices) {
    const MeshData m = mustTessellate(boxShape(10, 20, 30));
    EXPECT_NEAR(m.bounds.min.x, 0, 1e-6);
    EXPECT_NEAR(m.bounds.min.y, 0, 1e-6);
    EXPECT_NEAR(m.bounds.min.z, 0, 1e-6);
    EXPECT_NEAR(m.bounds.max.x, 10, 1e-6);
    EXPECT_NEAR(m.bounds.max.y, 20, 1e-6);
    EXPECT_NEAR(m.bounds.max.z, 30, 1e-6);
}
