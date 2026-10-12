// SPDX-License-Identifier: GPL-3.0-or-later
// F16: massProperties uses the property kind matching the shape's dimension. Solids: volume properties. Faces:
// surface properties (before the fix, the centre of mass and inertia of a face came from volume properties, which
// are meaningless for a surface). Wires/edges have no public constructor in the kernel facade yet; their linear
// path is not exercised here (see report).
#include <gtest/gtest.h>

#include <vector>

#include "kernel/internal/profile_support.hpp"
#include "kernel/ops.hpp"

using namespace iw;
using namespace iw::kernel;

namespace {

Shape square(double side, double z) {
    const std::vector<Vec3> pts = {{0, 0, z}, {side, 0, z}, {side, side, z}, {0, side, z}};
    auto f = makePolygonFace(pts);
    EXPECT_TRUE(f.has_value());
    return f ? *f : Shape{};
}

} // namespace

TEST(KernelMass, PlanarFaceUsesSurfaceProperties) {
    const Shape face = square(10, 3);
    auto mp = massProperties(face);
    ASSERT_TRUE(mp.has_value());
    EXPECT_NEAR(mp->surfaceArea, 100.0, 1e-9);
    EXPECT_DOUBLE_EQ(mp->volume, 0.0);
    EXPECT_NEAR(mp->centerOfMass.x, 5.0, 1e-9);
    EXPECT_NEAR(mp->centerOfMass.y, 5.0, 1e-9);
    EXPECT_NEAR(mp->centerOfMass.z, 3.0, 1e-9);
    // Unit-density plate of side a about its centre: Ixx = Iyy = a^4/12, Izz = a^4/6 (surface integrals, mm^4).
    const double a4over12 = 10.0 * 10.0 * 10.0 * 10.0 / 12.0;
    EXPECT_NEAR(mp->inertia[0], a4over12, 1e-6 * a4over12); // Ixx
    EXPECT_NEAR(mp->inertia[4], a4over12, 1e-6 * a4over12); // Iyy
    EXPECT_NEAR(mp->inertia[8], 2.0 * a4over12, 1e-6 * a4over12); // Izz
}

TEST(KernelMass, FaceCentreFollowsPlaneOffset) {
    // Same square moved in z: only the centre of mass z changes, area and in-plane inertia do not.
    auto low = massProperties(square(4, 0));
    auto high = massProperties(square(4, 7));
    ASSERT_TRUE(low.has_value());
    ASSERT_TRUE(high.has_value());
    EXPECT_NEAR(high->centerOfMass.z - low->centerOfMass.z, 7.0, 1e-9);
    EXPECT_NEAR(high->surfaceArea, low->surfaceArea, 1e-9);
    EXPECT_NEAR(high->inertia[0], low->inertia[0], 1e-9);
}

TEST(KernelMass, SolidBoxStillUsesVolumeProperties) {
    auto b = makeBox({0, 0, 0}, 2, 4, 6);
    ASSERT_TRUE(b.has_value());
    auto mp = massProperties(b->shape);
    ASSERT_TRUE(mp.has_value());
    EXPECT_NEAR(mp->volume, 48.0, 1e-9);
    EXPECT_NEAR(mp->surfaceArea, 2 * (8 + 12 + 24), 1e-9);
    EXPECT_NEAR(mp->centerOfMass.x, 1.0, 1e-9);
    EXPECT_NEAR(mp->centerOfMass.y, 2.0, 1e-9);
    EXPECT_NEAR(mp->centerOfMass.z, 3.0, 1e-9);
}
