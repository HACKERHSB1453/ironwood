// SPDX-License-Identifier: GPL-3.0-or-later
// B6: kernel measurement, mass properties and section model (Qt-free). Analytic values for box/cylinder/sphere.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include "kernel/measure.hpp"
#include "kernel/ops.hpp"

namespace iw::kernel::measure {
namespace {

constexpr double kTol = 1e-6;
constexpr double kPi = std::numbers::pi;

Shape boxAt(Vec3 o, double dx, double dy, double dz) {
    auto r = makeBox(o, dx, dy, dz);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message);
    return r ? r->shape : Shape{};
}

Entity body(const Shape& s) { return Entity{EntityKind::Body, s, 0}; }

std::optional<std::uint32_t> findVertex(const Shape& s, Vec3 p) {
    for (std::uint32_t i = 0; i < s.subShapeCount(SubShapeKind::Vertex); ++i) {
        auto v = vertexPoint(Entity{EntityKind::Vertex, s, i});
        if (v && length(*v - p) < 1e-7) return i;
    }
    return std::nullopt;
}

/// Planar face whose outward normal is `n` (unit axis) and whose plane passes through `p`.
std::optional<std::uint32_t> findFace(const Shape& s, Vec3 n, Vec3 p) {
    for (std::uint32_t i = 0; i < s.subShapeCount(SubShapeKind::Face); ++i) {
        const FaceGeometry g = s.faceGeometry(i);
        if (g.kind != FaceGeometry::Kind::Plane) continue;
        if (length(g.normal - n) < 1e-7 && std::abs(dot(g.normal, p - g.point)) < 1e-7) return i;
    }
    return std::nullopt;
}

std::optional<std::uint32_t> findEdgeOfLength(const Shape& s, double len) {
    for (std::uint32_t i = 0; i < s.subShapeCount(SubShapeKind::Edge); ++i) {
        auto l = edgeLength(Entity{EntityKind::Edge, s, i});
        if (l && std::abs(*l - len) < 1e-7) return i;
    }
    return std::nullopt;
}

/// Edge whose centroid (Shape::subShapeKey, {x,y,z,length}) equals `centroid`.
std::optional<std::uint32_t> findEdgeAt(const Shape& s, Vec3 centroid) {
    for (std::uint32_t i = 0; i < s.subShapeCount(SubShapeKind::Edge); ++i) {
        const auto k = s.subShapeKey(SubShapeKind::Edge, i);
        if (length(Vec3{k[0], k[1], k[2]} - centroid) < 1e-7) return i;
    }
    return std::nullopt;
}

// ----------------------------------------------------------------------------------------- mass properties

TEST(MeasureMass, BoxVolumeAreaCentreAndBounds) {
    const Shape b = boxAt({0, 0, 0}, 10, 20, 30);
    auto p = bodyProperties(b, 0.0);
    ASSERT_TRUE(p.has_value()) << p.error().message;
    EXPECT_NEAR(p->volume, 6000.0, kTol);
    EXPECT_NEAR(p->surfaceArea, 2200.0, kTol);
    EXPECT_NEAR(p->centreOfMass.x, 5.0, kTol);
    EXPECT_NEAR(p->centreOfMass.y, 10.0, kTol);
    EXPECT_NEAR(p->centreOfMass.z, 15.0, kTol);
    EXPECT_NEAR(p->boundingBox.max.z, 30.0, 1e-6);
    EXPECT_EQ(p->mass, 0.0);
}

TEST(MeasureMass, BoxDensityMassAndPrincipalMoments) {
    const Shape b = boxAt({0, 0, 0}, 10, 20, 30);
    auto p = bodyProperties(b, 1000.0); // water-like: 1000 kg/m^3 = 1e-6 kg/mm^3
    ASSERT_TRUE(p.has_value()) << p.error().message;
    EXPECT_NEAR(p->mass, 6.0e-3, 1e-9);
    // Box inertia about the centre, unit density: I_x = V/12 (b^2 + c^2) etc., here scaled to kg*mm^2.
    // V = 6000; Ix = 500*(400+900)=650000; Iy = 500*(100+900)=500000; Iz = 500*(100+400)=250000 (mm^5).
    std::array<double, 3> m{p->principalMoments.x, p->principalMoments.y, p->principalMoments.z};
    std::sort(m.begin(), m.end());
    EXPECT_NEAR(m[0], 0.25, 1e-6);
    EXPECT_NEAR(m[1], 0.50, 1e-6);
    EXPECT_NEAR(m[2], 0.65, 1e-6);
}

TEST(MeasureMass, CylinderAnalytic) {
    auto c = makeCylinder({0, 0, 0}, {0, 0, 1}, 5.0, 20.0);
    ASSERT_TRUE(c.has_value());
    auto p = bodyProperties(c->shape, 0.0);
    ASSERT_TRUE(p.has_value());
    EXPECT_NEAR(p->volume, 500.0 * kPi, 1e-6);
    EXPECT_NEAR(p->surfaceArea, 250.0 * kPi, 1e-6);
    EXPECT_NEAR(p->centreOfMass.z, 10.0, 1e-6);
    EXPECT_NEAR(p->centreOfMass.x, 0.0, 1e-6);
}

TEST(MeasureMass, SphereAnalytic) {
    auto s = makeSphere({1, 2, 3}, 3.0);
    ASSERT_TRUE(s.has_value());
    auto p = bodyProperties(s->shape, 0.0);
    ASSERT_TRUE(p.has_value());
    EXPECT_NEAR(p->volume, 36.0 * kPi, 1e-5);
    EXPECT_NEAR(p->surfaceArea, 36.0 * kPi, 1e-5);
    EXPECT_NEAR(p->centreOfMass.x, 1.0, 1e-6);
    EXPECT_NEAR(p->centreOfMass.y, 2.0, 1e-6);
    EXPECT_NEAR(p->centreOfMass.z, 3.0, 1e-6);
}

TEST(MeasureMass, RejectsBadInput) {
    EXPECT_EQ(bodyProperties(Shape{}, 0.0).error().code, ErrorCode::InvalidArgument);
    const Shape b = boxAt({0, 0, 0}, 1, 1, 1);
    EXPECT_EQ(bodyProperties(b, -1.0).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(bodyProperties(b, std::numeric_limits<double>::quiet_NaN()).error().code, ErrorCode::InvalidArgument);
}

// ------------------------------------------------------------------------------------------ single entities

TEST(MeasureSingle, EdgeLengthFaceAreaAndPoint) {
    const Shape b = boxAt({0, 0, 0}, 10, 20, 30);
    auto e = findEdgeOfLength(b, 10.0);
    ASSERT_TRUE(e.has_value());
    auto len = edgeLength(Entity{EntityKind::Edge, b, *e});
    ASSERT_TRUE(len.has_value());
    EXPECT_NEAR(*len, 10.0, kTol);

    auto top = findFace(b, {0, 0, 1}, {0, 0, 30});
    ASSERT_TRUE(top.has_value());
    auto area = faceArea(Entity{EntityKind::Face, b, *top});
    ASSERT_TRUE(area.has_value());
    EXPECT_NEAR(*area, 200.0, kTol); // 10 x 20

    auto v = findVertex(b, {10, 20, 30});
    ASSERT_TRUE(v.has_value());
    auto pt = vertexPoint(Entity{EntityKind::Vertex, b, *v});
    ASSERT_TRUE(pt.has_value());
    EXPECT_NEAR(pt->x, 10.0, kTol);
    EXPECT_NEAR(pt->y, 20.0, kTol);
    EXPECT_NEAR(pt->z, 30.0, kTol);
}

TEST(MeasureSingle, CircleRadiusDiameterCentre) {
    auto c = makeCylinder({0, 0, 0}, {0, 0, 1}, 5.0, 20.0);
    ASSERT_TRUE(c.has_value());
    int circles = 0;
    int atTop = 0;
    for (std::uint32_t i = 0; i < c->shape.subShapeCount(SubShapeKind::Edge); ++i) {
        auto info = circleOf(Entity{EntityKind::Edge, c->shape, i});
        if (!info) continue;
        ++circles;
        EXPECT_NEAR(info->radius, 5.0, kTol);
        EXPECT_NEAR(info->diameter, 10.0, kTol);
        EXPECT_NEAR(info->centre.x, 0.0, kTol);
        EXPECT_NEAR(info->centre.y, 0.0, kTol);
        if (std::abs(info->centre.z - 20.0) < kTol) ++atTop;
    }
    EXPECT_EQ(circles, 2);  // top and bottom rims
    EXPECT_EQ(atTop, 1);
}

TEST(MeasureSingle, CircleRejectsLine) {
    const Shape b = boxAt({0, 0, 0}, 1, 1, 1);
    auto e = findEdgeOfLength(b, 1.0);
    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(circleOf(Entity{EntityKind::Edge, b, *e}).error().code, ErrorCode::InvalidArgument);
}

TEST(MeasureSingle, OutOfRangeIndexIsNotFound) {
    const Shape b = boxAt({0, 0, 0}, 1, 1, 1);
    EXPECT_EQ(faceArea(Entity{EntityKind::Face, b, 999}).error().code, ErrorCode::NotFound);
    EXPECT_EQ(vertexPoint(Entity{EntityKind::Edge, b, 0}).error().code, ErrorCode::InvalidArgument);
}

// ------------------------------------------------------------------------------------------ distances

TEST(MeasureDistance, BoxToBoxGap) {
    const Shape a = boxAt({0, 0, 0}, 10, 10, 10);
    const Shape b = boxAt({20, 0, 0}, 10, 10, 10);
    auto c = minimumDistance(body(a), body(b));
    ASSERT_TRUE(c.has_value()) << c.error().message;
    EXPECT_NEAR(c->distance, 10.0, 1e-6);
    EXPECT_NEAR(c->onA.x, 10.0, 1e-6);
    EXPECT_NEAR(c->onB.x, 20.0, 1e-6);
}

TEST(MeasureDistance, VertexToFace) {
    const Shape b = boxAt({0, 0, 0}, 10, 10, 10);
    auto v = findVertex(b, {0, 0, 0});
    auto f = findFace(b, {0, 0, 1}, {0, 0, 10});
    ASSERT_TRUE(v && f);
    // Vertex (0,0,0) to the top face z=10 is 10 mm away, closest point (0,0,10) on the face.
    auto c = minimumDistance(Entity{EntityKind::Vertex, b, *v}, Entity{EntityKind::Face, b, *f});
    ASSERT_TRUE(c.has_value());
    EXPECT_NEAR(c->distance, 10.0, 1e-6);
    EXPECT_NEAR(c->onB.z, 10.0, 1e-6);
}

TEST(MeasureDistance, PointToPointSameAsEuclidean) {
    const Shape a = boxAt({0, 0, 0}, 1, 1, 1);
    const Shape b = boxAt({3, 4, 0}, 1, 1, 1);
    auto va = findVertex(a, {0, 0, 0});
    auto vb = findVertex(b, {4, 5, 1});
    ASSERT_TRUE(va && vb);
    auto c = minimumDistance(Entity{EntityKind::Vertex, a, *va}, Entity{EntityKind::Vertex, b, *vb});
    ASSERT_TRUE(c.has_value());
    EXPECT_NEAR(c->distance, std::sqrt(16.0 + 25.0 + 1.0), 1e-9);
}

TEST(MeasureDistance, OverlappingBodiesAreZero) {
    const Shape a = boxAt({0, 0, 0}, 10, 10, 10);
    const Shape b = boxAt({5, 5, 5}, 10, 10, 10);
    auto c = minimumDistance(body(a), body(b));
    ASSERT_TRUE(c.has_value());
    EXPECT_NEAR(c->distance, 0.0, 1e-9);
}

// ------------------------------------------------------------------------------------------ angles

TEST(MeasureAngle, FacesPerpendicularParallelAndOpposite) {
    const Shape b = boxAt({0, 0, 0}, 10, 10, 10);
    auto top = findFace(b, {0, 0, 1}, {0, 0, 10});
    auto side = findFace(b, {1, 0, 0}, {10, 0, 0});
    auto bottom = findFace(b, {0, 0, -1}, {0, 0, 0});
    ASSERT_TRUE(top && side && bottom);
    auto perp = angleBetween(Entity{EntityKind::Face, b, *top}, Entity{EntityKind::Face, b, *side});
    ASSERT_TRUE(perp.has_value());
    EXPECT_NEAR(*perp, 90.0, 1e-6);
    auto opp = angleBetween(Entity{EntityKind::Face, b, *top}, Entity{EntityKind::Face, b, *bottom});
    ASSERT_TRUE(opp.has_value());
    EXPECT_NEAR(*opp, 180.0, 1e-6);
    auto same = angleBetween(Entity{EntityKind::Face, b, *top}, Entity{EntityKind::Face, b, *top});
    ASSERT_TRUE(same.has_value());
    EXPECT_NEAR(*same, 0.0, 1e-6);
}

TEST(MeasureAngle, CurvedFaceRejected) {
    const Shape b = boxAt({0, 0, 0}, 10, 10, 10);
    auto c = makeCylinder({20, 0, 0}, {0, 0, 1}, 2.0, 5.0);
    ASSERT_TRUE(c.has_value());
    auto top = findFace(b, {0, 0, 1}, {0, 0, 10});
    ASSERT_TRUE(top.has_value());
    // Find the lateral (non-planar) face of the cylinder.
    std::optional<std::uint32_t> lateral;
    for (std::uint32_t i = 0; i < c->shape.subShapeCount(SubShapeKind::Face); ++i)
        if (c->shape.faceGeometry(i).kind == FaceGeometry::Kind::Cylinder) lateral = i;
    ASSERT_TRUE(lateral.has_value());
    // A non-planar face on either side is rejected (InvalidArgument), never given a bogus angle.
    auto r = angleBetween(Entity{EntityKind::Face, b, *top}, Entity{EntityKind::Face, c->shape, *lateral});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
}

TEST(MeasureAngle, EdgesAndEdgeFace) {
    const Shape b = boxAt({0, 0, 0}, 10, 10, 10);
    // Box edges along x, y, z at the origin corner have centroids (5,0,0), (0,5,0), (0,0,5).
    auto ex = findEdgeAt(b, {5, 0, 0});
    auto ey = findEdgeAt(b, {0, 5, 0});
    auto ez = findEdgeAt(b, {0, 0, 5});
    ASSERT_TRUE(ex && ey && ez);
    auto xy = angleBetween(Entity{EntityKind::Edge, b, *ex}, Entity{EntityKind::Edge, b, *ey});
    ASSERT_TRUE(xy.has_value());
    EXPECT_NEAR(*xy, 90.0, 1e-6);
    auto top = findFace(b, {0, 0, 1}, {0, 0, 10});
    ASSERT_TRUE(top.has_value());
    auto ef = angleBetween(Entity{EntityKind::Edge, b, *ex}, Entity{EntityKind::Face, b, *top});
    ASSERT_TRUE(ef.has_value());
    EXPECT_NEAR(*ef, 0.0, 1e-6); // an edge of the top face lies in its plane
    auto ezf = angleBetween(Entity{EntityKind::Face, b, *top}, Entity{EntityKind::Edge, b, *ez});
    ASSERT_TRUE(ezf.has_value());
    EXPECT_NEAR(*ezf, 90.0, 1e-6); // vertical edge is normal to the top face
}

// ------------------------------------------------------------------------------------------ rows

TEST(MeasureRows, DescribeAndBetween) {
    const Shape b = boxAt({0, 0, 0}, 10, 10, 10);
    auto top = findFace(b, {0, 0, 1}, {0, 0, 10});
    auto side = findFace(b, {1, 0, 0}, {10, 0, 0});
    ASSERT_TRUE(top && side);
    auto rows = describe(Entity{EntityKind::Face, b, *top});
    ASSERT_TRUE(rows.has_value());
    ASSERT_EQ(rows->size(), 1u);
    EXPECT_EQ((*rows)[0].unit, "mm^2");
    EXPECT_NEAR((*rows)[0].value, 100.0, kTol);

    auto rel = between(Entity{EntityKind::Face, b, *top}, Entity{EntityKind::Face, b, *side});
    ASSERT_TRUE(rel.has_value());
    bool sawAngle = false;
    for (const auto& r : *rel)
        if (r.label == "Angle") {
            sawAngle = true;
            EXPECT_NEAR(r.value, 90.0, 1e-6);
            EXPECT_EQ(r.unit, "deg");
        }
    EXPECT_TRUE(sawAngle);
}

TEST(MeasureRows, CylinderDescribeReportsCircle) {
    auto c = makeCylinder({0, 0, 0}, {0, 0, 1}, 5.0, 20.0);
    ASSERT_TRUE(c.has_value());
    bool found = false;
    for (std::uint32_t i = 0; i < c->shape.subShapeCount(SubShapeKind::Edge) && !found; ++i) {
        auto rows = describe(Entity{EntityKind::Edge, c->shape, i});
        ASSERT_TRUE(rows.has_value());
        for (const auto& r : *rows)
            if (r.label == "Diameter") {
                EXPECT_NEAR(r.value, 10.0, kTol);
                found = true;
            }
    }
    EXPECT_TRUE(found);
}

// ------------------------------------------------------------------------------------------ section model

TEST(SectionModel, DefaultsAndRequest) {
    section::Model m;
    ASSERT_TRUE(m.setPlane({0, 0, 5}, {0, 0, 1}).has_value());
    m.setOffset(2.0);
    m.setEnabled(true);
    EXPECT_NEAR(m.effectivePoint().z, 7.0, kTol);
    const section::RenderRequest r = m.request();
    EXPECT_TRUE(r.enabled);
    EXPECT_NEAR(r.clipPlane[0], 0.0, kTol);
    EXPECT_NEAR(r.clipPlane[2], 1.0, kTol);
    EXPECT_NEAR(r.clipPlane[3], -7.0, kTol);
    // Kept side is the normal side: z=8 is kept, z=6 is clipped.
    EXPECT_GT(m.signedDistance({0, 0, 8}), 0.0);
    EXPECT_LT(m.signedDistance({0, 0, 6}), 0.0);
}

TEST(SectionModel, FlipReversesKeptSide) {
    section::Model m;
    ASSERT_TRUE(m.setPlane({0, 0, 0}, {0, 0, 2}).has_value()); // normal is normalised
    EXPECT_NEAR(length(m.normal()), 1.0, kTol);
    m.flip();
    EXPECT_TRUE(m.flipped());
    EXPECT_NEAR(m.effectiveNormal().z, -1.0, kTol);
    EXPECT_GT(m.signedDistance({0, 0, -1}), 0.0);
    EXPECT_LT(m.signedDistance({0, 0, 1}), 0.0);
    const section::RenderRequest r = m.request();
    EXPECT_NEAR(r.clipPlane[2], -1.0, kTol);
    EXPECT_NEAR(r.clipPlane[3], 0.0, kTol);
    m.flip();
    EXPECT_FALSE(m.flipped());
}

TEST(SectionModel, OffsetMovesAlongUnflippedNormal) {
    section::Model m;
    ASSERT_TRUE(m.setPlane({1, 2, 3}, {1, 0, 0}).has_value());
    m.flip();
    m.setOffset(4.0);
    EXPECT_NEAR(m.effectivePoint().x, 5.0, kTol);
    EXPECT_NEAR(m.effectivePoint().y, 2.0, kTol);
}

TEST(SectionModel, RejectsZeroNormalAndKeepsPreviousPlane) {
    section::Model m;
    ASSERT_TRUE(m.setPlane({0, 0, 1}, {0, 1, 0}).has_value());
    auto bad = m.setPlane({0, 0, 0}, {0, 0, 0});
    EXPECT_EQ(bad.error().code, ErrorCode::InvalidArgument);
    EXPECT_NEAR(m.normal().y, 1.0, kTol);
}

TEST(SectionModel, DisabledByDefault) {
    section::Model m;
    EXPECT_FALSE(m.request().enabled);
}

} // namespace
} // namespace iw::kernel::measure
