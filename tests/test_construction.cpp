// SPDX-License-Identifier: GPL-3.0-or-later
// F26: construction geometry follow-ups. Tangent plane (cylinder / cone, angle convention, expression-driven),
// axis from two planes (faces and construction planes, parallel error), point at edge/plane intersection (line, arc,
// circle with two hits, no hit, edge in plane), validated circle-centre point (full circle, arc, non-circular edge),
// the kernel helpers behind them, and registry / io round trips of the new kinds.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <functional>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "core/params.hpp"
#include "features/construction_features.hpp"
#include "features/edge_features.hpp"
#include "features/extrude_feature.hpp"
#include "features/revolve_feature.hpp"
#include "io/document_io.hpp"
#include "kernel/ops.hpp"
#include "recompute/evaluator.hpp"

using namespace iw;
using namespace iw::features;
using namespace iw::recompute;
using naming::ElementRef;
using naming::ElementType;

namespace {

constexpr BodyId kB1{1}, kB2{2};
constexpr double kPi = std::numbers::pi;

bool near(double a, double b, double tol = 1e-6) { return std::abs(a - b) <= tol; }

std::string mm(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.12f mm", v);
    return buf;
}

EvalResult evaluate(const Timeline& tl, const ParameterTable* params = nullptr) {
    Evaluator ev;
    auto r = ev.evaluate(tl, params);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message);
    return r ? *r : EvalResult{};
}

/// Evaluates and requires every feature to be Ok.
EvalResult evalAllOk(const Timeline& tl, const ParameterTable* params = nullptr) {
    EvalResult r = evaluate(tl, params);
    for (const auto& s : r.statuses) EXPECT_EQ(s.state, FeatureStatus::State::Ok) << s.error.message;
    return r;
}

/// 10 x 10 x 10 box from the origin onto `body`.
std::shared_ptr<Feature> box10(std::uint64_t id = 1, BodyId body = kB1) {
    ExtrudeParams p;
    PolygonSource src;
    src.points = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    src.vector = {0, 0, 10};
    p.source = src;
    p.operation = ExtrudeParams::Operation::NewBody;
    p.body = body;
    return std::make_shared<ExtrudeFeature>(FeatureId{id}, std::move(p));
}

std::shared_ptr<Feature> axisTwoPoints(std::uint64_t id, Vec3 a, Vec3 b) {
    ConstructionAxisParams p;
    p.kind = ConstructionAxisParams::Kind::TwoPoints;
    p.a = {mm(a.x), mm(a.y), mm(a.z)};
    p.b = {mm(b.x), mm(b.y), mm(b.z)};
    return std::make_shared<ConstructionAxisFeature>(FeatureId{id}, p);
}

/// Full revolve of an XY-plane polygon about the construction axis `axisFeature` (the Y axis in these tests).
std::shared_ptr<Feature> revolvePolygon(std::uint64_t id, std::vector<std::array<double, 2>> pts, std::uint64_t axisFeature,
                                        BodyId body) {
    RevolveParams p;
    PolygonSource src;
    src.planeNormal = {0, 0, 1};
    src.planeXDir = {1, 0, 0};
    src.points = std::move(pts);
    p.source = src;
    p.axis.kind = RevolveAxis::Kind::Construction;
    p.axis.construction = FeatureId{axisFeature};
    p.extent = RevolveParams::Extent::Full;
    p.angle = "360 deg";
    p.body = body;
    p.operation = common::BodyOp::NewBody;
    return std::make_shared<RevolveFeature>(FeatureId{id}, p);
}

std::optional<ElementRef> refWhere(const EvalResult& r, BodyId body, ElementType type,
                                   const std::function<bool(std::uint32_t, const std::array<double, 4>&)>& pred) {
    const kernel::Shape& s = r.bodies.at(body);
    const naming::NamingMap& m = r.namings.at(body);
    const auto kind = naming::toSubShapeKind(type);
    for (std::uint32_t i = 0; i < s.subShapeCount(kind); ++i) {
        if (!pred(i, s.subShapeKey(kind, i))) continue;
        auto nm = m.nameOf(type, i);
        if (!nm) continue;
        return ElementRef{nm->feature, type, *nm};
    }
    return std::nullopt;
}

std::optional<std::uint32_t> indexWhere(const kernel::Shape& s, kernel::SubShapeKind kind,
                                        const std::function<bool(std::uint32_t, const std::array<double, 4>&)>& pred) {
    for (std::uint32_t i = 0; i < s.subShapeCount(kind); ++i)
        if (pred(i, s.subShapeKey(kind, i))) return i;
    return std::nullopt;
}

/// Face of `body` whose surface geometry satisfies `pred`.
std::optional<ElementRef> faceWhere(const EvalResult& r, BodyId body, const std::function<bool(const kernel::FaceGeometry&)>& pred) {
    const kernel::Shape& s = r.bodies.at(body);
    return refWhere(r, body, ElementType::Face, [&](std::uint32_t i, const auto&) { return pred(s.faceGeometry(i)); });
}

/// Planar box face with outward normal `n` through the point `p`.
std::optional<ElementRef> boxFace(const EvalResult& r, BodyId body, Vec3 n, Vec3 p) {
    return faceWhere(r, body, [&](const kernel::FaceGeometry& g) {
        return g.kind == kernel::FaceGeometry::Kind::Plane && near(g.normal.x, n.x) && near(g.normal.y, n.y) &&
               near(g.normal.z, n.z) && near(dot(g.point - p, n), 0.0);
    });
}

PlaneRef faceRef(BodyId body, ElementRef face) {
    PlaneRef r;
    r.isFace = true;
    r.body = body;
    r.face = std::move(face);
    return r;
}

PlaneRef featureRef(std::uint64_t id) {
    PlaneRef r;
    r.isFace = false;
    r.construction = FeatureId{id};
    return r;
}

std::shared_ptr<Feature> offsetPlane(std::uint64_t id, PlaneRef base, std::string distance) {
    ConstructionPlaneParams p;
    p.kind = ConstructionPlaneParams::Kind::Offset;
    p.base = std::move(base);
    p.distance = std::move(distance);
    return std::make_shared<ConstructionPlaneFeature>(FeatureId{id}, p);
}

std::shared_ptr<Feature> threePointPlane(std::uint64_t id, Vec3 a, Vec3 b, Vec3 c) {
    ConstructionPlaneParams p;
    p.kind = ConstructionPlaneParams::Kind::ThreePoints;
    p.points = {{{mm(a.x), mm(a.y), mm(a.z)}, {mm(b.x), mm(b.y), mm(b.z)}, {mm(c.x), mm(c.y), mm(c.z)}}};
    return std::make_shared<ConstructionPlaneFeature>(FeatureId{id}, p);
}

std::shared_ptr<Feature> tangentPlane(std::uint64_t id, BodyId body, ElementRef face, std::string angle) {
    ConstructionPlaneParams p;
    p.kind = ConstructionPlaneParams::Kind::Tangent;
    p.base = faceRef(body, std::move(face));
    p.angle = std::move(angle);
    return std::make_shared<ConstructionPlaneFeature>(FeatureId{id}, p);
}

std::shared_ptr<Feature> twoPlaneAxis(std::uint64_t id, PlaneRef a, PlaneRef b) {
    ConstructionAxisParams p;
    p.kind = ConstructionAxisParams::Kind::TwoPlanes;
    p.planeA = std::move(a);
    p.planeB = std::move(b);
    return std::make_shared<ConstructionAxisFeature>(FeatureId{id}, p);
}

std::shared_ptr<Feature> edgePlanePoint(std::uint64_t id, BodyId body, ElementRef edge, PlaneRef plane) {
    ConstructionPointParams p;
    p.kind = ConstructionPointParams::Kind::EdgePlane;
    p.body = body;
    p.ref = std::move(edge);
    p.plane = std::move(plane);
    return std::make_shared<ConstructionPointFeature>(FeatureId{id}, p);
}

std::shared_ptr<Feature> circleCentre(std::uint64_t id, BodyId body, ElementRef edge) {
    ConstructionPointParams p;
    p.kind = ConstructionPointParams::Kind::CircleCentre;
    p.body = body;
    p.ref = std::move(edge);
    return std::make_shared<ConstructionPointFeature>(FeatureId{id}, p);
}

/// Vertical box edge at (10,10): centroid (10,10,5), length 10.
bool isCornerEdge(std::uint32_t, const std::array<double, 4>& k) { return near(k[0], 10) && near(k[1], 10) && near(k[3], 10); }

/// Timeline prefix: [1] Y axis, [2] tube r in [2,5], y in [0,10] about it (body kB2).
void appendTube(Timeline& tl) {
    tl.append(axisTwoPoints(1, {0, 0, 0}, {0, 1, 0}));
    tl.append(revolvePolygon(2, {{2, 0}, {5, 0}, {5, 10}, {2, 10}}, 1, kB2));
}

/// Full circle of radius `r` at y = `y` on the tube (centroid on the axis, length 2 pi r).
std::function<bool(std::uint32_t, const std::array<double, 4>&)> tubeCircle(double r, double y) {
    return [r, y](std::uint32_t, const std::array<double, 4>& k) {
        return near(k[0], 0) && near(k[1], y) && near(k[2], 0) && near(k[3], 2 * kPi * r, 1e-5);
    };
}

bool isOuterCylinder(const kernel::FaceGeometry& g) {
    return g.kind == kernel::FaceGeometry::Kind::Cylinder && near(g.radius, 5);
}

const Diagnostic* findWarning(const FeatureStatus& s) {
    for (const auto& d : s.diagnostics)
        if (d.severity == Diagnostic::Severity::Warning) return &d;
    return nullptr;
}

} // namespace

// ---- tangent plane -------------------------------------------------------------------------------------------------

TEST(ConstructionTangentPlane, CylinderAngleConventionAndExpressionDrivenAngle) {
    ParameterTable params;
    ASSERT_TRUE(params.add("ang", "0 deg"));
    Timeline tl;
    appendTube(tl);
    EvalResult base = evalAllOk(tl);
    auto outer = faceWhere(base, kB2, isOuterCylinder);
    ASSERT_TRUE(outer.has_value());
    tl.append(tangentPlane(3, kB2, *outer, "ang"));

    // Tube axis +Y -> reference direction +X; 0 deg touches the +X side.
    EvalResult r0 = evalAllOk(tl, &params);
    ASSERT_TRUE(r0.statuses[2].construction.has_value());
    const ConstructionGeometry g0 = *r0.statuses[2].construction;
    EXPECT_EQ(g0.kind, ConstructionGeometry::Kind::Plane);
    EXPECT_NEAR(g0.origin.x, 5.0, 1e-9);
    EXPECT_NEAR(g0.origin.z, 0.0, 1e-9);
    EXPECT_NEAR(g0.normal.x, 1.0, 1e-9);
    EXPECT_NEAR(std::abs(g0.xDir.y), 1.0, 1e-9); // the ruling runs along the axis

    // 90 deg: counter-clockwise about +Y from +X is (+Y) x (+X) = -Z.
    ASSERT_TRUE(params.setExpression("ang", "90 deg"));
    EvalResult r90 = evalAllOk(tl, &params);
    const ConstructionGeometry g90 = *r90.statuses[2].construction;
    EXPECT_NEAR(g90.origin.x, 0.0, 1e-9);
    EXPECT_NEAR(g90.origin.z, -5.0, 1e-9);
    EXPECT_NEAR(g90.normal.z, -1.0, 1e-9);

    // Arbitrary angle: the plane is tangent (distance from the axis to the plane = radius, normal radial).
    ASSERT_TRUE(params.setExpression("ang", "37 deg"));
    EvalResult r37 = evalAllOk(tl, &params);
    const ConstructionGeometry g = *r37.statuses[2].construction;
    EXPECT_NEAR(std::abs(dot(Vec3{0, 4, 0} - g.origin, g.normal)), 5.0, 1e-9);
    EXPECT_NEAR(g.normal.y, 0.0, 1e-9);
    EXPECT_NEAR(dot(g.normal, g.xDir), 0.0, 1e-9);
    const double a = 37.0 * kPi / 180.0;
    EXPECT_NEAR(g.normal.x, std::cos(a), 1e-9);
    EXPECT_NEAR(g.normal.z, -std::sin(a), 1e-9);
}

TEST(ConstructionTangentPlane, ConeTangentPlaneContainsTheGeneratrix) {
    Timeline tl;
    tl.append(axisTwoPoints(1, {0, 0, 0}, {0, 1, 0}));
    // Trapezoid revolved about Y: outer conical face from radius 5 at y = 0 to radius 3 at y = 10.
    tl.append(revolvePolygon(2, {{2, 0}, {5, 0}, {3, 10}, {2, 10}}, 1, kB2));
    EvalResult base = evalAllOk(tl);
    auto cone = faceWhere(base, kB2, [](const kernel::FaceGeometry& g) { return g.kind == kernel::FaceGeometry::Kind::Cone; });
    ASSERT_TRUE(cone.has_value());
    tl.append(tangentPlane(3, kB2, *cone, "0 deg"));
    EvalResult r = evalAllOk(tl);
    const ConstructionGeometry g = *r.statuses[2].construction;
    // At 0 deg (+X side) the generatrix runs from (5,0,0) to (3,10,0): both lie on the plane.
    EXPECT_NEAR(dot(Vec3{5, 0, 0} - g.origin, g.normal), 0.0, 1e-7);
    EXPECT_NEAR(dot(Vec3{3, 10, 0} - g.origin, g.normal), 0.0, 1e-7);
    const Vec3 gen = Vec3{3, 10, 0} - Vec3{5, 0, 0};
    EXPECT_NEAR(std::abs(dot(g.xDir, gen * (1.0 / length(gen)))), 1.0, 1e-9);
    // Normal points away from the axis: an axis point is behind the plane.
    EXPECT_GT(g.normal.x, 0.0);
    EXPECT_LT(dot(Vec3{0, 5, 0} - g.origin, g.normal), 0.0);
    EXPECT_NEAR(length(g.normal), 1.0, 1e-12);
    EXPECT_NEAR(dot(g.normal, g.xDir), 0.0, 1e-9);
}

TEST(ConstructionTangentPlane, PlanarFaceIsRejected) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    tl.append(tangentPlane(2, kB1, *boxFace(base, kB1, {0, 0, 1}, {0, 0, 10}), "0 deg"));
    EvalResult r = evaluate(tl);
    ASSERT_EQ(r.statuses.size(), 2u);
    EXPECT_EQ(r.statuses[1].state, FeatureStatus::State::Failed);
    EXPECT_NE(r.statuses[1].error.message.find("not cylindrical or conical"), std::string::npos) << r.statuses[1].error.message;
}

// ---- axis from two planes ------------------------------------------------------------------------------------------

TEST(ConstructionTwoPlanesAxis, FacesAndConstructionPlanes) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    const ElementRef top = *boxFace(base, kB1, {0, 0, 1}, {0, 0, 10});
    const ElementRef right = *boxFace(base, kB1, {1, 0, 0}, {10, 0, 0});
    // [2] faces top x right -> line along Y through (10, *, 10).
    tl.append(twoPlaneAxis(2, faceRef(kB1, top), faceRef(kB1, right)));
    // [3] offset plane x = 13, [4] face top x construction plane -> (13, *, 10).
    tl.append(offsetPlane(3, faceRef(kB1, right), "3 mm"));
    tl.append(twoPlaneAxis(4, faceRef(kB1, top), featureRef(3)));
    // [5] three-point plane z = x (through the origin), [6] its intersection with plane x = 13 -> (13, *, 13).
    tl.append(threePointPlane(5, {0, 0, 0}, {0, 1, 0}, {1, 0, 1}));
    tl.append(twoPlaneAxis(6, featureRef(5), featureRef(3)));
    EvalResult r = evalAllOk(tl);
    ASSERT_EQ(r.statuses.size(), 6u);

    const ConstructionGeometry a = *r.statuses[1].construction;
    EXPECT_EQ(a.kind, ConstructionGeometry::Kind::Axis);
    EXPECT_NEAR(a.direction.y, 1.0, 1e-12); // (0,0,1) x (1,0,0) = (0,1,0)
    EXPECT_NEAR(a.origin.x, 10.0, 1e-9);
    EXPECT_NEAR(a.origin.y, 0.0, 1e-9); // closest point to the world origin
    EXPECT_NEAR(a.origin.z, 10.0, 1e-9);

    const ConstructionGeometry b = *r.statuses[3].construction;
    EXPECT_NEAR(std::abs(b.direction.y), 1.0, 1e-12);
    EXPECT_NEAR(b.origin.x, 13.0, 1e-9);
    EXPECT_NEAR(b.origin.z, 10.0, 1e-9);

    const ConstructionGeometry c = *r.statuses[5].construction;
    EXPECT_NEAR(std::abs(c.direction.y), 1.0, 1e-12);
    EXPECT_NEAR(c.origin.x, 13.0, 1e-9);
    EXPECT_NEAR(c.origin.y, 0.0, 1e-9);
    EXPECT_NEAR(c.origin.z, 13.0, 1e-9);
}

TEST(ConstructionTwoPlanesAxis, ParallelPlanesGiveAClearError) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    const ElementRef top = *boxFace(base, kB1, {0, 0, 1}, {0, 0, 10});
    const ElementRef bottom = *boxFace(base, kB1, {0, 0, -1}, {0, 0, 0});
    tl.append(twoPlaneAxis(2, faceRef(kB1, top), faceRef(kB1, bottom)));
    tl.append(offsetPlane(3, faceRef(kB1, top), "4 mm"));
    tl.append(twoPlaneAxis(4, faceRef(kB1, top), featureRef(3))); // coincident orientation, parallel
    EvalResult r = evaluate(tl);
    ASSERT_EQ(r.statuses.size(), 4u);
    for (std::size_t i : {std::size_t{1}, std::size_t{3}}) {
        EXPECT_EQ(r.statuses[i].state, FeatureStatus::State::Failed);
        EXPECT_EQ(r.statuses[i].error.code, ErrorCode::InvalidArgument);
        EXPECT_NE(r.statuses[i].error.message.find("parallel"), std::string::npos) << r.statuses[i].error.message;
    }
    EXPECT_EQ(r.statuses[2].state, FeatureStatus::State::Ok);
}

// ---- point at edge / plane -----------------------------------------------------------------------------------------

TEST(ConstructionEdgePlanePoint, LinearEdgeWithExpressionDrivenOffsetPlane) {
    ParameterTable params;
    ASSERT_TRUE(params.add("drop", "3 mm"));
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    const ElementRef top = *boxFace(base, kB1, {0, 0, 1}, {0, 0, 10});
    tl.append(offsetPlane(2, faceRef(kB1, top), "-drop"));
    const ElementRef corner = *refWhere(base, kB1, ElementType::Edge, isCornerEdge);
    tl.append(edgePlanePoint(3, kB1, corner, featureRef(2)));
    EvalResult r = evalAllOk(tl, &params);
    ConstructionGeometry g = *r.statuses[2].construction;
    EXPECT_EQ(g.kind, ConstructionGeometry::Kind::Point);
    EXPECT_NEAR(g.origin.x, 10.0, 1e-9);
    EXPECT_NEAR(g.origin.y, 10.0, 1e-9);
    EXPECT_NEAR(g.origin.z, 7.0, 1e-9);
    EXPECT_EQ(findWarning(r.statuses[2]), nullptr);

    ASSERT_TRUE(params.setExpression("drop", "8 mm"));
    r = evalAllOk(tl, &params);
    EXPECT_NEAR(r.statuses[2].construction->origin.z, 2.0, 1e-9);

    // The plane above the box misses the (bounded) edge.
    ASSERT_TRUE(params.setExpression("drop", "-5 mm"));
    r = evaluate(tl, &params);
    EXPECT_EQ(r.statuses[2].state, FeatureStatus::State::Failed);
    EXPECT_NE(r.statuses[2].error.message.find("does not intersect"), std::string::npos) << r.statuses[2].error.message;
}

TEST(ConstructionEdgePlanePoint, EdgeLyingInThePlaneIsRejected) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    const ElementRef top = *boxFace(base, kB1, {0, 0, 1}, {0, 0, 10});
    // Top edge along X at y = 0, z = 10.
    auto topEdge = refWhere(base, kB1, ElementType::Edge, [](std::uint32_t, const std::array<double, 4>& k) {
        return near(k[0], 5) && near(k[1], 0) && near(k[2], 10);
    });
    ASSERT_TRUE(topEdge.has_value());
    tl.append(edgePlanePoint(2, kB1, *topEdge, faceRef(kB1, top)));
    EvalResult r = evaluate(tl);
    EXPECT_EQ(r.statuses[1].state, FeatureStatus::State::Failed);
    EXPECT_NE(r.statuses[1].error.message.find("lies in the plane"), std::string::npos) << r.statuses[1].error.message;
}

TEST(ConstructionEdgePlanePoint, ArcEdgeSingleHit) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    FilletParams fp;
    fp.body = kB1;
    fp.edges = {*refWhere(base, kB1, ElementType::Edge, isCornerEdge)};
    fp.radius = "2 mm";
    tl.append(std::make_shared<FilletFeature>(FeatureId{2}, fp));
    EvalResult filleted = evalAllOk(tl);
    // Top quarter arc: centre (8,8,10), radius 2, length pi.
    auto arc = refWhere(filleted, kB1, ElementType::Edge, [](std::uint32_t, const std::array<double, 4>& k) {
        return near(k[2], 10) && near(k[3], kPi, 1e-5);
    });
    ASSERT_TRUE(arc.has_value());
    tl.append(threePointPlane(3, {9, 0, 0}, {9, 1, 0}, {9, 0, 1})); // x = 9
    tl.append(edgePlanePoint(4, kB1, *arc, featureRef(3)));
    EvalResult r = evalAllOk(tl);
    const ConstructionGeometry g = *r.statuses[3].construction;
    EXPECT_NEAR(g.origin.x, 9.0, 1e-7);
    EXPECT_NEAR(g.origin.y, 8.0 + std::sqrt(3.0), 1e-7);
    EXPECT_NEAR(g.origin.z, 10.0, 1e-7);
    EXPECT_EQ(findWarning(r.statuses[3]), nullptr);
}

TEST(ConstructionEdgePlanePoint, FullCircleTwoHitsPicksOneDeterministicallyWithAWarning) {
    Timeline tl;
    appendTube(tl);
    EvalResult base = evalAllOk(tl);
    auto circle = refWhere(base, kB2, ElementType::Edge, tubeCircle(5, 10));
    ASSERT_TRUE(circle.has_value());
    tl.append(threePointPlane(3, {0, 0, 0}, {0, 1, 0}, {0, 0, 1})); // x = 0: meets the circle at z = +-5
    tl.append(edgePlanePoint(4, kB2, *circle, featureRef(3)));
    EvalResult r1 = evalAllOk(tl);
    EvalResult r2 = evalAllOk(tl);
    const ConstructionGeometry g = *r1.statuses[3].construction;
    EXPECT_NEAR(g.origin.x, 0.0, 1e-7);
    EXPECT_NEAR(g.origin.y, 10.0, 1e-7);
    EXPECT_NEAR(std::abs(g.origin.z), 5.0, 1e-7);
    EXPECT_EQ(r2.statuses[3].construction->origin.z, g.origin.z);
    const Diagnostic* w = findWarning(r1.statuses[3]);
    ASSERT_NE(w, nullptr);
    EXPECT_NE(w->message.find("2 times"), std::string::npos) << w->message;
}

// ---- circle centre -------------------------------------------------------------------------------------------------

TEST(ConstructionCircleCentre, FullCircleArcAndNonCircularEdge) {
    Timeline tl;
    tl.append(box10());
    EvalResult base = evalAllOk(tl);
    FilletParams fp;
    fp.body = kB1;
    fp.edges = {*refWhere(base, kB1, ElementType::Edge, isCornerEdge)};
    fp.radius = "2 mm";
    tl.append(std::make_shared<FilletFeature>(FeatureId{2}, fp));
    tl.append(axisTwoPoints(3, {0, 0, 0}, {0, 1, 0}));
    tl.append(revolvePolygon(4, {{2, 0}, {5, 0}, {5, 10}, {2, 10}}, 3, kB2));
    EvalResult mid = evalAllOk(tl);
    auto arc = refWhere(mid, kB1, ElementType::Edge, [](std::uint32_t, const std::array<double, 4>& k) {
        return near(k[2], 10) && near(k[3], kPi, 1e-5);
    });
    ASSERT_TRUE(arc.has_value());
    auto full = refWhere(mid, kB2, ElementType::Edge, tubeCircle(2, 10));
    ASSERT_TRUE(full.has_value());
    auto straight = refWhere(mid, kB1, ElementType::Edge, [](std::uint32_t, const std::array<double, 4>& k) {
        return near(k[0], 0) && near(k[1], 0) && near(k[3], 10); // vertical edge at the origin
    });
    ASSERT_TRUE(straight.has_value());
    tl.append(circleCentre(5, kB1, *arc));
    tl.append(circleCentre(6, kB2, *full));
    tl.append(circleCentre(7, kB1, *straight));
    EvalResult r = evaluate(tl);
    ASSERT_EQ(r.statuses.size(), 7u);

    // Arc: the circle's centre, not the arc's centroid.
    ASSERT_EQ(r.statuses[4].state, FeatureStatus::State::Ok) << r.statuses[4].error.message;
    EXPECT_NEAR(r.statuses[4].construction->origin.x, 8.0, 1e-7);
    EXPECT_NEAR(r.statuses[4].construction->origin.y, 8.0, 1e-7);
    EXPECT_NEAR(r.statuses[4].construction->origin.z, 10.0, 1e-7);

    ASSERT_EQ(r.statuses[5].state, FeatureStatus::State::Ok) << r.statuses[5].error.message;
    EXPECT_NEAR(r.statuses[5].construction->origin.x, 0.0, 1e-7);
    EXPECT_NEAR(r.statuses[5].construction->origin.y, 10.0, 1e-7);
    EXPECT_NEAR(r.statuses[5].construction->origin.z, 0.0, 1e-7);

    EXPECT_EQ(r.statuses[6].state, FeatureStatus::State::Failed);
    EXPECT_EQ(r.statuses[6].error.code, ErrorCode::InvalidArgument);
    EXPECT_NE(r.statuses[6].error.message.find("not a circle"), std::string::npos) << r.statuses[6].error.message;
}

// ---- kernel helpers ------------------------------------------------------------------------------------------------

TEST(ConstructionKernel, CircleFromEdgeFullCircleArcAndLine) {
    Timeline tl;
    appendTube(tl);
    tl.append(box10(3, kB1));
    EvalResult base = evalAllOk(tl);
    const kernel::Shape& tube = base.bodies.at(kB2);
    auto fullIdx = indexWhere(tube, kernel::SubShapeKind::Edge, tubeCircle(5, 0));
    ASSERT_TRUE(fullIdx.has_value());
    auto c = kernel::circleFromEdge(tube, *fullIdx);
    ASSERT_TRUE(c.has_value()) << c.error().message;
    EXPECT_TRUE(c->fullCircle);
    EXPECT_NEAR(c->radius, 5.0, 1e-9);
    EXPECT_NEAR(std::abs(c->normal.y), 1.0, 1e-9);
    EXPECT_NEAR(length(c->centre), 0.0, 1e-9);

    auto arcShape = kernel::filletEdges(base.bodies.at(kB1),
                                        std::vector<std::uint32_t>{*indexWhere(base.bodies.at(kB1), kernel::SubShapeKind::Edge, isCornerEdge)},
                                        2.0);
    ASSERT_TRUE(arcShape.has_value());
    auto arcIdx = indexWhere(arcShape->shape, kernel::SubShapeKind::Edge, [](std::uint32_t, const std::array<double, 4>& k) {
        return near(k[2], 0) && near(k[3], kPi, 1e-5);
    });
    ASSERT_TRUE(arcIdx.has_value());
    auto arc = kernel::circleFromEdge(arcShape->shape, *arcIdx);
    ASSERT_TRUE(arc.has_value()) << arc.error().message;
    EXPECT_FALSE(arc->fullCircle);
    EXPECT_NEAR(arc->radius, 2.0, 1e-9);
    EXPECT_NEAR(arc->centre.x, 8.0, 1e-9);
    EXPECT_NEAR(arc->centre.y, 8.0, 1e-9);
    EXPECT_NEAR(arc->centre.z, 0.0, 1e-9);

    auto line = kernel::circleFromEdge(base.bodies.at(kB1), *indexWhere(base.bodies.at(kB1), kernel::SubShapeKind::Edge, isCornerEdge));
    ASSERT_FALSE(line.has_value());
    EXPECT_EQ(line.error().code, ErrorCode::InvalidArgument);
    auto out = kernel::circleFromEdge(tube, 10000);
    ASSERT_FALSE(out.has_value());
    EXPECT_EQ(out.error().code, ErrorCode::InvalidArgument);
    EXPECT_FALSE(kernel::circleFromEdge(kernel::Shape{}, 0).has_value());
}

TEST(ConstructionKernel, PlanePlaneIntersection) {
    kernel::Plane a{{0, 0, 5}, {0, 0, 1}, {1, 0, 0}};
    kernel::Plane b{{3, 7, 0}, {1, 0, 0}, {0, 1, 0}};
    auto ax = kernel::planePlaneIntersection(a, b);
    ASSERT_TRUE(ax.has_value());
    EXPECT_NEAR(ax->direction.y, 1.0, 1e-12);
    EXPECT_NEAR(ax->origin.x, 3.0, 1e-12);
    EXPECT_NEAR(ax->origin.y, 0.0, 1e-12);
    EXPECT_NEAR(ax->origin.z, 5.0, 1e-12);
    // Oblique planes: the point lies on both planes and is the closest to the origin (perpendicular to the line).
    kernel::Plane c{{1, 2, 3}, {1, 1, 0}, {0, 0, 1}};
    kernel::Plane d{{-4, 0, 2}, {0, 1, 2}, {1, 0, 0}};
    auto cd = kernel::planePlaneIntersection(c, d);
    ASSERT_TRUE(cd.has_value());
    EXPECT_NEAR(dot(cd->origin - c.origin, c.normal), 0.0, 1e-9);
    EXPECT_NEAR(dot(cd->origin - d.origin, d.normal), 0.0, 1e-9);
    EXPECT_NEAR(dot(cd->origin, cd->direction), 0.0, 1e-9);
    EXPECT_NEAR(length(cd->direction), 1.0, 1e-12);

    kernel::Plane e{{0, 0, 9}, {0, 0, -2}, {1, 0, 0}};
    auto par = kernel::planePlaneIntersection(a, e);
    ASSERT_FALSE(par.has_value());
    EXPECT_EQ(par.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(par.error().message.find("parallel"), std::string::npos);
    kernel::Plane z{{0, 0, 0}, {0, 0, 0}, {1, 0, 0}};
    EXPECT_FALSE(kernel::planePlaneIntersection(a, z).has_value());
}

TEST(ConstructionKernel, EdgePlaneIntersectionNearestToEdgeStart) {
    Timeline tl;
    appendTube(tl);
    EvalResult base = evalAllOk(tl);
    const kernel::Shape& tube = base.bodies.at(kB2);
    auto idx = indexWhere(tube, kernel::SubShapeKind::Edge, tubeCircle(5, 10));
    ASSERT_TRUE(idx.has_value());
    // Plane x = 0 cuts the circle twice; a plane tilted slightly changes nothing about the count.
    auto hit = kernel::edgePlaneIntersection(tube, *idx, kernel::Plane{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}});
    ASSERT_TRUE(hit.has_value()) << hit.error().message;
    EXPECT_EQ(hit->count, 2u);
    // Nearest-to-start rule: the seam vertex of the revolved circle is (5,10,0). Planes z = +3 and z = -3 each cut the
    // circle at x = +-4. Whichever way the circle runs from the seam, the hit reached first lies at x = +4 for one of
    // the planes and at x = -4 for the other (a rule like "smallest x" or "first solver root" would not mirror).
    auto up = kernel::edgePlaneIntersection(tube, *idx, kernel::Plane{{0, 0, 3}, {0, 0, 1}, {1, 0, 0}});
    auto down = kernel::edgePlaneIntersection(tube, *idx, kernel::Plane{{0, 0, -3}, {0, 0, 1}, {1, 0, 0}});
    ASSERT_TRUE(up.has_value() && down.has_value());
    EXPECT_EQ(up->count, 2u);
    EXPECT_NEAR(std::abs(up->point.x), 4.0, 1e-6);
    EXPECT_NEAR(up->point.x, -down->point.x, 1e-6);
    EXPECT_NEAR(up->point.z, 3.0, 1e-6);
    EXPECT_NEAR(down->point.z, -3.0, 1e-6);
    // Plane tangent to the circle at x = 5: one (double) contact point.
    auto touch = kernel::edgePlaneIntersection(tube, *idx, kernel::Plane{{5, 0, 0}, {1, 0, 0}, {0, 1, 0}});
    ASSERT_TRUE(touch.has_value()) << touch.error().message;
    EXPECT_EQ(touch->count, 1u);
    EXPECT_NEAR(touch->point.x, 5.0, 1e-6);
    EXPECT_NEAR(touch->point.z, 0.0, 1e-3);
    // Misses and coplanar edges are errors.
    auto miss = kernel::edgePlaneIntersection(tube, *idx, kernel::Plane{{6, 0, 0}, {1, 0, 0}, {0, 1, 0}});
    ASSERT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error().code, ErrorCode::InvalidArgument);
    auto in = kernel::edgePlaneIntersection(tube, *idx, kernel::Plane{{0, 10, 0}, {0, 1, 0}, {1, 0, 0}});
    ASSERT_FALSE(in.has_value());
    EXPECT_NE(in.error().message.find("lies"), std::string::npos);
    EXPECT_FALSE(kernel::edgePlaneIntersection(tube, *idx, kernel::Plane{{0, 0, 0}, {0, 0, 0}, {1, 0, 0}}).has_value());
}

TEST(ConstructionKernel, TangentPlaneOnFaceRejectsOtherSurfaces) {
    auto sphere = kernel::makeSphere({0, 0, 0}, 3);
    ASSERT_TRUE(sphere.has_value());
    auto r = kernel::tangentPlaneOnFace(sphere->shape, 0, 0.0);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
    auto cyl = kernel::makeCylinder({0, 0, 0}, {0, 0, -1}, 2, 5); // axis -Z: canonical axis is +Z
    ASSERT_TRUE(cyl.has_value());
    auto side = indexWhere(cyl->shape, kernel::SubShapeKind::Face, [&](std::uint32_t i, const auto&) {
        return cyl->shape.faceGeometry(i).kind == kernel::FaceGeometry::Kind::Cylinder;
    });
    ASSERT_TRUE(side.has_value());
    auto p = kernel::tangentPlaneOnFace(cyl->shape, *side, kPi / 2);
    ASSERT_TRUE(p.has_value()) << p.error().message;
    // Canonical +Z, reference +X, 90 deg -> +Y.
    EXPECT_NEAR(p->normal.y, 1.0, 1e-9);
    EXPECT_NEAR(p->origin.y, 2.0, 1e-9);
    EXPECT_NEAR(p->xDir.z, 1.0, 1e-9);
    EXPECT_FALSE(kernel::tangentPlaneOnFace(cyl->shape, *side, std::nan("")).has_value());
}

// ---- json / registry / io ------------------------------------------------------------------------------------------

TEST(ConstructionJson, NewKindsRoundTripThroughRegistryAndIo) {
    Timeline tl;
    appendTube(tl);
    tl.append(box10(3, kB1));
    EvalResult base = evalAllOk(tl);
    const ElementRef top = *boxFace(base, kB1, {0, 0, 1}, {0, 0, 10});
    const ElementRef right = *boxFace(base, kB1, {1, 0, 0}, {10, 0, 0});
    tl.append(tangentPlane(4, kB2, *faceWhere(base, kB2, isOuterCylinder), "15 deg + 15 deg"));
    tl.append(offsetPlane(5, faceRef(kB1, top), "-2 mm"));
    tl.append(twoPlaneAxis(6, faceRef(kB1, right), featureRef(5)));
    tl.append(edgePlanePoint(7, kB1, *refWhere(base, kB1, ElementType::Edge, isCornerEdge), featureRef(5)));
    tl.append(circleCentre(8, kB2, *refWhere(base, kB2, ElementType::Edge, tubeCircle(5, 0))));
    tl.append(twoPlaneAxis(9, featureRef(5), featureRef(4)));

    FeatureRegistry reg;
    registerFeatures(reg);
    for (std::size_t i = 0; i < tl.size(); ++i) {
        auto again = reg.create(tl.at(i)->kind(), tl.at(i)->id(), tl.at(i)->paramsJson());
        ASSERT_TRUE(again.has_value()) << tl.at(i)->kind() << ": " << again.error().message;
        EXPECT_EQ((*again)->paramsJson(), tl.at(i)->paramsJson()) << tl.at(i)->kind();
        EXPECT_EQ((*again)->paramHash(), tl.at(i)->paramHash()) << tl.at(i)->kind();
        EXPECT_EQ((*again)->explicitRefs().size(), tl.at(i)->explicitRefs().size()) << tl.at(i)->kind();
        EXPECT_EQ((*again)->readsBodies(), tl.at(i)->readsBodies()) << tl.at(i)->kind();
    }
    // Reads/refs of the new kinds.
    EXPECT_EQ(tl.at(5)->readsBodies(), std::vector<BodyId>{kB1});
    EXPECT_EQ(tl.at(5)->explicitRefs().size(), 1u);
    EXPECT_EQ(tl.at(6)->explicitRefs().size(), 1u);
    EXPECT_TRUE(tl.at(8)->readsBodies().empty());
    EXPECT_EQ(tl.at(3)->parameterReferences().size(), 0u);

    io::Document doc;
    doc.name = "f26-roundtrip";
    doc.nextId = 20;
    for (std::size_t i = 0; i < tl.size(); ++i) doc.timeline.append(tl.at(i));
    const auto dir = std::filesystem::temp_directory_path() / ("iw_f26_rt_" + std::to_string(std::random_device{}()) + ".iwd");
    std::filesystem::remove_all(dir);
    ASSERT_TRUE(io::saveDocument(dir, doc).has_value());
    auto loaded = io::loadDocument(dir, reg);
    std::filesystem::remove_all(dir);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    ASSERT_EQ(loaded->timeline.size(), tl.size());
    EvalResult a = evalAllOk(tl);
    EvalResult b = evalAllOk(loaded->timeline);
    ASSERT_EQ(a.statuses.size(), b.statuses.size());
    for (std::size_t i = 0; i < tl.size(); ++i) {
        EXPECT_EQ(loaded->timeline.at(i)->paramsJson(), tl.at(i)->paramsJson());
        ASSERT_EQ(a.statuses[i].construction.has_value(), b.statuses[i].construction.has_value());
        if (a.statuses[i].construction) {
            EXPECT_EQ(a.statuses[i].construction->origin.x, b.statuses[i].construction->origin.x);
            EXPECT_EQ(a.statuses[i].construction->origin.z, b.statuses[i].construction->origin.z);
        }
    }
    // Spot-check values of the evaluated chain: tangent at 30 deg, axis (10, *, 8), point (10, 10, 8).
    EXPECT_NEAR(a.statuses[3].construction->normal.x, std::cos(kPi / 6), 1e-9);
    EXPECT_NEAR(a.statuses[5].construction->origin.x, 10.0, 1e-9);
    EXPECT_NEAR(a.statuses[5].construction->origin.z, 8.0, 1e-9);
    EXPECT_NEAR(a.statuses[6].construction->origin.z, 8.0, 1e-9);
}

TEST(ConstructionJson, MalformedNewKindsAreRejected) {
    FeatureRegistry reg;
    registerFeatures(reg);
    // Tangent plane needs a face, not a construction feature.
    EXPECT_FALSE(reg.create("plane", FeatureId{1}, R"({"kind":"tangent","base":{"feature":3},"angle":"0 deg"})").has_value());
    EXPECT_FALSE(reg.create("plane", FeatureId{1}, R"({"kind":"tangent","angle":"0 deg"})").has_value());
    EXPECT_FALSE(reg.create("axis", FeatureId{1}, R"({"kind":"two_planes","plane_a":{"feature":3}})").has_value());
    EXPECT_FALSE(reg.create("point", FeatureId{1}, R"({"kind":"bogus","body":1})").has_value());
    // Construction-plane-only two_planes axis parses (no bodies).
    auto ok = reg.create("axis", FeatureId{1}, R"({"kind":"two_planes","plane_a":{"feature":3},"plane_b":{"feature":4}})");
    ASSERT_TRUE(ok.has_value()) << ok.error().message;
    EXPECT_TRUE((*ok)->readsBodies().empty());
    EXPECT_TRUE((*ok)->explicitRefs().empty());
}
