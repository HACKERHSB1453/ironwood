// SPDX-License-Identifier: GPL-3.0-or-later
// F42: edge direction + topological adjacency in the kernel Shape API, and fillet / chamfer names that stay the same when
// the base body is rotated (not only translated) before the edge operation.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "features/edge_features.hpp"
#include "features/extrude_feature.hpp"
#include "features/move_feature.hpp"
#include "kernel/ops.hpp"
#include "kernel/shape.hpp"
#include "recompute/evaluator.hpp"

using namespace iw;
using namespace iw::features;
using namespace iw::recompute;
using kernel::Shape;
using kernel::SubShapeKind;
using naming::ElementRef;
using naming::ElementType;

namespace {

constexpr BodyId kB1{1};

bool near(double a, double b, double tol = 1e-6) { return std::abs(a - b) <= tol; }
bool nearV(const Vec3& a, const Vec3& b, double tol = 1e-6) { return length(a - b) <= tol; }

Shape box10() {
    auto b = kernel::makeBox({0, 0, 0}, 10, 10, 10);
    EXPECT_TRUE(b.has_value());
    return b ? b->shape : Shape{};
}

Vec3 vertexPoint(const Shape& s, std::uint32_t v) {
    const auto k = s.subShapeKey(SubShapeKind::Vertex, v);
    return {k[0], k[1], k[2]};
}

// ---- kernel adjacency ---------------------------------------------------------------------------------------------

TEST(KernelAdjacency, BoxCountsAndPerElementQueriesAgreeWithTables) {
    const Shape s = box10();
    ASSERT_EQ(s.subShapeCount(SubShapeKind::Face), 6u);
    ASSERT_EQ(s.subShapeCount(SubShapeKind::Edge), 12u);
    ASSERT_EQ(s.subShapeCount(SubShapeKind::Vertex), 8u);

    const auto e2f = s.adjacency(SubShapeKind::Edge, SubShapeKind::Face);
    const auto f2e = s.adjacency(SubShapeKind::Face, SubShapeKind::Edge);
    const auto e2v = s.adjacency(SubShapeKind::Edge, SubShapeKind::Vertex);
    const auto v2e = s.adjacency(SubShapeKind::Vertex, SubShapeKind::Edge);
    const auto f2v = s.adjacency(SubShapeKind::Face, SubShapeKind::Vertex);
    const auto v2f = s.adjacency(SubShapeKind::Vertex, SubShapeKind::Face);
    ASSERT_EQ(e2f.size(), 12u);
    ASSERT_EQ(f2e.size(), 6u);
    ASSERT_EQ(v2e.size(), 8u);
    for (std::uint32_t e = 0; e < 12; ++e) {
        EXPECT_EQ(e2f[e].size(), 2u);
        EXPECT_EQ(s.edgeFaces(e), e2f[e]);
        EXPECT_EQ(e2v[e].size(), 2u);
        auto ordered = s.edgeVertices(e);
        ASSERT_EQ(ordered.size(), 2u);
        std::sort(ordered.begin(), ordered.end());
        EXPECT_EQ(ordered, e2v[e]);
        // Each adjacent face lists the edge back.
        for (auto f : e2f[e]) EXPECT_TRUE(std::count(f2e[f].begin(), f2e[f].end(), e)) << "face " << f << " edge " << e;
        // The two faces of a box edge are perpendicular planes.
        const auto g0 = s.faceGeometry(e2f[e][0]);
        const auto g1 = s.faceGeometry(e2f[e][1]);
        EXPECT_NEAR(dot(g0.normal, g1.normal), 0.0, 1e-12);
    }
    for (std::uint32_t f = 0; f < 6; ++f) {
        EXPECT_EQ(f2e[f].size(), 4u);
        EXPECT_EQ(s.faceEdges(f), f2e[f]);
        EXPECT_EQ(f2v[f].size(), 4u);
    }
    for (std::uint32_t v = 0; v < 8; ++v) {
        EXPECT_EQ(v2e[v].size(), 3u);
        EXPECT_EQ(s.vertexEdges(v), v2e[v]);
        EXPECT_EQ(v2f[v].size(), 3u);
    }
    // Identity table and out-of-range / null behaviour.
    const auto ff = s.adjacency(SubShapeKind::Face, SubShapeKind::Face);
    ASSERT_EQ(ff.size(), 6u);
    EXPECT_EQ(ff[4], std::vector<std::uint32_t>{4});
    EXPECT_TRUE(s.edgeFaces(12).empty());
    EXPECT_TRUE(s.faceEdges(6).empty());
    EXPECT_TRUE(s.edgeVertices(99).empty());
    EXPECT_TRUE(s.vertexEdges(8).empty());
    EXPECT_TRUE(Shape{}.adjacency(SubShapeKind::Edge, SubShapeKind::Face).empty());
    EXPECT_EQ(Shape{}.edgeGeometry(0).kind, kernel::EdgeGeometry::Kind::Other);
}

TEST(KernelAdjacency, BoxEdgeGeometryMatchesItsVerticesAndFaces) {
    const Shape s = box10();
    for (std::uint32_t e = 0; e < s.subShapeCount(SubShapeKind::Edge); ++e) {
        const kernel::EdgeGeometry g = s.edgeGeometry(e);
        EXPECT_EQ(g.kind, kernel::EdgeGeometry::Kind::Line);
        EXPECT_FALSE(g.degenerate);
        EXPECT_FALSE(g.closed);
        EXPECT_NEAR(g.length, 10.0, 1e-9);
        EXPECT_NEAR(length(g.startTangent), 1.0, 1e-12);
        EXPECT_TRUE(nearV(g.startTangent, g.endTangent));
        EXPECT_TRUE(nearV(g.direction, g.startTangent));
        // start/end follow edgeVertices order; the tangent points from start to end.
        const auto vs = s.edgeVertices(e);
        ASSERT_EQ(vs.size(), 2u);
        EXPECT_TRUE(nearV(g.start, vertexPoint(s, vs[0])));
        EXPECT_TRUE(nearV(g.end, vertexPoint(s, vs[1])));
        EXPECT_TRUE(nearV((g.end - g.start) * 0.1, g.startTangent));
        // The edge lies in both adjacent planes: the tangent is perpendicular to both normals.
        for (auto f : s.edgeFaces(e)) EXPECT_NEAR(dot(s.faceGeometry(f).normal, g.startTangent), 0.0, 1e-12);
    }
}

TEST(KernelAdjacency, CylinderCircleSeamAndRotationCovariance) {
    auto cyl = kernel::makeCylinder({0, 0, 0}, {0, 0, 1}, 2.0, 5.0);
    ASSERT_TRUE(cyl.has_value());
    const Shape& s = cyl->shape;
    int circles = 0, seams = 0;
    for (std::uint32_t e = 0; e < s.subShapeCount(SubShapeKind::Edge); ++e) {
        const auto g = s.edgeGeometry(e);
        if (g.kind == kernel::EdgeGeometry::Kind::Circle) {
            ++circles;
            EXPECT_TRUE(g.closed);
            EXPECT_NEAR(g.radius, 2.0, 1e-9);
            EXPECT_NEAR(std::abs(g.direction.z), 1.0, 1e-12);
            EXPECT_NEAR(g.length, 4.0 * std::numbers::pi, 1e-6);
            EXPECT_EQ(s.edgeVertices(e).size(), 1u);
            EXPECT_EQ(s.edgeFaces(e).size(), 2u); // lateral face + cap
            EXPECT_NEAR(dot(g.startTangent, g.direction), 0.0, 1e-12);
        } else if (g.kind == kernel::EdgeGeometry::Kind::Line) {
            ++seams;
            EXPECT_EQ(s.edgeFaces(e).size(), 1u); // the seam bounds only the lateral face
            EXPECT_NEAR(g.length, 5.0, 1e-9);
        }
    }
    EXPECT_EQ(circles, 2);
    EXPECT_EQ(seams, 1);

    // Rotating the shape rotates the edge directions with it (they are intrinsic to the edge).
    const double c = std::cos(0.7), sn = std::sin(0.7);
    Transform rotX;
    rotX.m = {1, 0, 0, 3, 0, c, -sn, 0, 0, sn, c, 0, 0, 0, 0, 1};
    const Shape box = box10();
    auto moved = kernel::transform(box, rotX);
    ASSERT_TRUE(moved.has_value());
    ASSERT_EQ(moved->shape.subShapeCount(SubShapeKind::Edge), 12u);
    for (std::uint32_t e = 0; e < 12; ++e) {
        const auto a = box.edgeGeometry(e);
        const auto b = moved->shape.edgeGeometry(e);
        EXPECT_TRUE(nearV(rotX.apply(a.start), b.start));
        EXPECT_TRUE(nearV(rotX.apply(a.startTangent) - rotX.apply({0, 0, 0}), b.startTangent));
        EXPECT_EQ(box.edgeFaces(e), moved->shape.edgeFaces(e));
    }
}

// ---- fillet / chamfer names under rotation ----------------------------------------------------------------------

ExtrudeParams boxParams() {
    ExtrudeParams p;
    PolygonSource src;
    src.points = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    src.vector = {0, 0, 10};
    p.source = src;
    p.operation = ExtrudeParams::Operation::NewBody;
    p.body = kB1;
    return p;
}

std::string mm(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.12f mm", v);
    return buf;
}

struct Placement {
    double angleDeg = 0;
    Vec3 axisOrigin{0, 0, 0};
    Vec3 axisDir{0, 0, 1};
    Vec3 shift{0, 0, 0};
    /// Point map of the Move feature (Rodrigues rotation about the axis, then translation).
    Vec3 apply(const Vec3& p) const {
        const double a = angleDeg * std::numbers::pi / 180.0;
        const Vec3 k = axisDir * (1.0 / length(axisDir));
        const Vec3 v = p - axisOrigin;
        const Vec3 r = v * std::cos(a) + cross(k, v) * std::sin(a) + k * (dot(k, v) * (1 - std::cos(a)));
        return r + axisOrigin + shift;
    }
};

std::shared_ptr<Feature> moveFeature(const Placement& pl) {
    MoveParams m;
    m.body = kB1;
    char deg[64];
    std::snprintf(deg, sizeof deg, "%.12f deg", pl.angleDeg);
    m.angle = deg;
    m.axisOrigin = pl.axisOrigin;
    m.axisDirection = pl.axisDir;
    m.translate = {mm(pl.shift.x), mm(pl.shift.y), mm(pl.shift.z)};
    return std::make_shared<MoveFeature>(FeatureId{2}, m);
}

EvalResult evalAllOk(const Timeline& tl) {
    Evaluator ev;
    auto r = ev.evaluate(tl, nullptr);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message);
    if (!r) return {};
    for (const auto& s : r->statuses) EXPECT_EQ(s.state, FeatureStatus::State::Ok) << s.error.message;
    return *r;
}

/// Edges of the unmoved box selected by centroid (x, y, z), as names (box names survive the Move).
std::vector<ElementRef> boxEdges(const std::vector<Vec3>& centroids) {
    Timeline tl;
    tl.append(std::make_shared<ExtrudeFeature>(FeatureId{1}, boxParams()));
    const EvalResult r = evalAllOk(tl);
    const Shape& s = r.bodies.at(kB1);
    std::vector<ElementRef> out;
    for (const Vec3& c : centroids)
        for (std::uint32_t i = 0; i < s.subShapeCount(SubShapeKind::Edge); ++i) {
            const auto k = s.subShapeKey(SubShapeKind::Edge, i);
            if (!nearV({k[0], k[1], k[2]}, c)) continue;
            auto nm = r.namings.at(kB1).nameOf(ElementType::Edge, i);
            EXPECT_TRUE(nm.has_value());
            if (nm) out.push_back(ElementRef{nm->feature, ElementType::Edge, *nm});
        }
    EXPECT_EQ(out.size(), centroids.size());
    return out;
}

using EdgeOp = std::function<std::shared_ptr<Feature>(const std::vector<ElementRef>&)>;

EvalResult buildPlaced(const Placement& pl, const std::vector<ElementRef>& edges, const EdgeOp& op) {
    Timeline tl;
    tl.append(std::make_shared<ExtrudeFeature>(FeatureId{1}, boxParams()));
    tl.append(moveFeature(pl));
    tl.append(op(edges));
    return evalAllOk(tl);
}

/// Every element of the reference result must carry the same name as its image under `pl` in the placed result.
/// Returns the number of elements whose op is "<tag>.new" in a sibling group of size > 1 (to prove the test bites).
int expectSameNames(const EvalResult& ref, const EvalResult& placed, const Placement& pl, const std::string& tag) {
    const Shape& a = ref.bodies.at(kB1);
    const Shape& b = placed.bodies.at(kB1);
    const auto& na = ref.namings.at(kB1);
    const auto& nb = placed.namings.at(kB1);
    int multiNew = 0;
    for (ElementType t : {ElementType::Face, ElementType::Edge, ElementType::Vertex}) {
        const auto kind = naming::toSubShapeKind(t);
        EXPECT_EQ(a.subShapeCount(kind), b.subShapeCount(kind));
        for (std::uint32_t i = 0; i < a.subShapeCount(kind); ++i) {
            const auto ka = a.subShapeKey(kind, i);
            const Vec3 target = pl.apply({ka[0], ka[1], ka[2]});
            std::optional<std::uint32_t> match;
            for (std::uint32_t j = 0; j < b.subShapeCount(kind); ++j) {
                const auto kb = b.subShapeKey(kind, j);
                if (nearV({kb[0], kb[1], kb[2]}, target, 1e-5) && near(ka[3], kb[3], 1e-5)) { match = j; break; }
            }
            EXPECT_TRUE(match.has_value()) << "no image for element " << i;
            if (!match) continue;
            const auto nameA = na.nameOf(t, i);
            const auto nameB = nb.nameOf(t, *match);
            EXPECT_TRUE(nameA && nameB);
            if (!nameA || !nameB) continue;
            EXPECT_EQ(nameA->toString(), nameB->toString()) << "element " << i << " renamed by the placement";
            if (nameA->op == tag + ".new" && nameA->ordinal > 0) ++multiNew;
        }
    }
    return multiNew;
}

const std::vector<Vec3> kCornerEdges{{10, 10, 5}, {5, 10, 10}, {10, 5, 10}}; // three edges meeting at (10,10,10)

std::shared_ptr<Feature> filletOf(const std::vector<ElementRef>& edges) {
    FilletParams p;
    p.body = kB1;
    p.edges = edges;
    p.radius = "2 mm";
    p.tangentChain = false;
    return std::make_shared<FilletFeature>(FeatureId{3}, p);
}

std::shared_ptr<Feature> chamferOf(const std::vector<ElementRef>& edges) {
    ChamferParams p;
    p.body = kB1;
    p.edges = edges;
    p.mode = ChamferParams::Mode::TwoDistances;
    p.distance = "1.5 mm";
    p.distance2 = "2.5 mm";
    p.tangentChain = false;
    return std::make_shared<ChamferFeature>(FeatureId{3}, p);
}

const std::vector<Placement> kPlacements{
    {90, {0, 0, 0}, {0, 0, 1}, {0, 0, 0}},          // quarter turn: swaps x and y ordering
    {180, {5, 5, 5}, {1, 0, 0}, {0, 0, 0}},         // half turn about X: flips y and z ordering
    {37, {1, 2, 3}, {1, 2, 3}, {20, -7, 4}},        // generic rotation + translation
    {-123, {0, 0, 0}, {0, 1, 1}, {0, 0, 0}},
};

TEST(NamingRotation, FilletFaceNamesAreStableWhenTheBaseBodyIsRotated) {
    const auto edges = boxEdges(kCornerEdges);
    const EvalResult ref = buildPlaced({}, edges, filletOf);
    int multiNew = 0;
    for (const Placement& pl : kPlacements) {
        SCOPED_TRACE(pl.angleDeg);
        multiNew = std::max(multiNew, expectSameNames(ref, buildPlaced(pl, edges, filletOf), pl, "fillet"));
    }
    EXPECT_GT(multiNew, 0) << "the corner fillet must create several fillet.new siblings for this test to be meaningful";
}

TEST(NamingRotation, SingleEdgeFilletAndChamferNamesAreStableWhenRotated) {
    for (const auto& centroids : {std::vector<Vec3>{{10, 10, 5}}, std::vector<Vec3>{{5, 0, 0}, {0, 5, 0}}}) {
        const auto edges = boxEdges(centroids);
        for (const EdgeOp& op : {EdgeOp(filletOf), EdgeOp(chamferOf)}) {
            const EvalResult ref = buildPlaced({}, edges, op);
            for (const Placement& pl : kPlacements) {
                SCOPED_TRACE(pl.angleDeg);
                expectSameNames(ref, buildPlaced(pl, edges, op), pl, "x");
            }
        }
    }
}

TEST(NamingRotation, CornerChamferNamesAreStableWhenRotated) {
    const auto edges = boxEdges(kCornerEdges);
    const EvalResult ref = buildPlaced({}, edges, chamferOf);
    for (const Placement& pl : kPlacements) {
        SCOPED_TRACE(pl.angleDeg);
        expectSameNames(ref, buildPlaced(pl, edges, chamferOf), pl, "chamfer");
    }
}

} // namespace
