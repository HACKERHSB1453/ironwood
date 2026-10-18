// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <span>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "kernel/exchange.hpp"
#include "kernel/internal/profile_support.hpp"
#include "kernel/internal/validate.hpp"
#include "kernel/ops.hpp"

using namespace iw;
using namespace iw::kernel;
namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2 * kPi;

fs::path tmpDir() {
    static const fs::path d = [] {
        auto p = fs::temp_directory_path() / "ironwood_test_kernel";
        fs::create_directories(p);
        return p;
    }();
    return d;
}

Shape box(double dx, double dy, double dz, Vec3 o = {}) {
    auto r = makeBox(o, dx, dy, dz);
    EXPECT_TRUE(r.has_value());
    return r->shape;
}

double volume(const Shape& s) {
    auto m = massProperties(s);
    EXPECT_TRUE(m.has_value());
    return m ? m->volume : -1;
}

struct Counts {
    std::uint32_t f, e, v;
    friend bool operator==(const Counts&, const Counts&) = default;
};
Counts counts(const Shape& s) {
    return {s.subShapeCount(SubShapeKind::Face), s.subShapeCount(SubShapeKind::Edge),
            s.subShapeCount(SubShapeKind::Vertex)};
}

/// Full ordering fingerprint: counts + every sub-shape key in index order.
std::vector<std::array<double, 4>> fingerprint(const Shape& s) {
    std::vector<std::array<double, 4>> fp;
    for (auto k : {SubShapeKind::Face, SubShapeKind::Edge, SubShapeKind::Vertex})
        for (std::uint32_t i = 0; i < s.subShapeCount(k); ++i) fp.push_back(s.subShapeKey(k, i));
    return fp;
}

} // namespace

TEST(Kernel, StampCombineIsDeterministicAndOrderSensitive) {
    ShapeStamp a{1};
    EXPECT_EQ(ShapeStamp::combine(a, 5), ShapeStamp::combine(a, 5));
    EXPECT_NE(ShapeStamp::combine(ShapeStamp::combine(a, 1), 2), ShapeStamp::combine(ShapeStamp::combine(a, 2), 1));
}

TEST(Kernel, NullShapeAndOcctLinked) {
    Shape s;
    EXPECT_TRUE(s.isNull());
    EXPECT_EQ(s.type(), ShapeType::Null);
    EXPECT_EQ(s.subShapeCount(SubShapeKind::Face), 0u);
    EXPECT_TRUE(s.boundingBox().isEmpty());
    EXPECT_FALSE(s.isSame(s));
    EXPECT_FALSE(occtVersion().empty());
}

TEST(Kernel, BoxVolumeBoundsCountsAndStamp) {
    Shape b = box(2, 3, 4, {1, 1, 1});
    EXPECT_EQ(b.type(), ShapeType::Solid);
    EXPECT_NEAR(volume(b), 24.0, 1e-9);
    EXPECT_EQ((counts(b)), (Counts{6, 12, 8}));
    Box3 bb = b.boundingBox();
    EXPECT_NEAR(bb.min.x, 1, 1e-9);
    EXPECT_NEAR(bb.max.z, 5, 1e-9);
    Shape s = b.withStamp(ShapeStamp{42});
    EXPECT_EQ(s.stamp(), ShapeStamp{42});
    EXPECT_TRUE(s.isSame(b));
    EXPECT_FALSE(box(2, 3, 4).isSame(box(2, 3, 4))); // distinct geometry instances
}

TEST(Kernel, PrimitiveArgumentValidation) {
    EXPECT_EQ(makeBox({}, 0, 1, 1).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(makeBox({}, 1, -1, 1).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(makeCylinder({}, {0, 0, 0}, 1, 1).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(makeCylinder({}, {0, 0, 1}, 0, 1).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(massProperties(Shape{}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(fuse(Shape{}, box(1, 1, 1)).error().code, ErrorCode::InvalidArgument);
}

TEST(Kernel, CylinderVolumeExactAndAxis) {
    auto c = makeCylinder({0, 0, 0}, {0, 0, 2}, 3, 5);
    ASSERT_TRUE(c);
    auto mp = massProperties(c->shape);
    ASSERT_TRUE(mp);
    EXPECT_NEAR(mp->volume, kPi * 9 * 5, 1e-9 * kPi * 45);
    EXPECT_NEAR(mp->surfaceArea, 2 * kPi * 3 * 5 + 2 * kPi * 9, 1e-8);
    EXPECT_NEAR(mp->centerOfMass.z, 2.5, 1e-9);
    Box3 bb = c->shape.boundingBox();
    EXPECT_NEAR(bb.max.z, 5, 1e-7);
    EXPECT_NEAR(bb.max.x, 3, 1e-7);
    // Axis along +X
    auto cx = makeCylinder({0, 0, 0}, {1, 0, 0}, 1, 10);
    ASSERT_TRUE(cx);
    EXPECT_NEAR(cx->shape.boundingBox().max.x, 10, 1e-7);
}

TEST(Kernel, SphereVolumeAreaCentreAndBounds) {
    const double r = 4;
    auto s = makeSphere({1, 2, 3}, r);
    ASSERT_TRUE(s);
    EXPECT_EQ(s->shape.type(), ShapeType::Solid);
    auto mp = massProperties(s->shape);
    ASSERT_TRUE(mp);
    const double vExact = 4.0 / 3.0 * kPi * r * r * r;
    const double aExact = 4.0 * kPi * r * r;
    EXPECT_NEAR(mp->volume, vExact, 1e-6 * vExact);
    EXPECT_NEAR(mp->surfaceArea, aExact, 1e-6 * aExact);
    EXPECT_NEAR(mp->centerOfMass.x, 1, 1e-6);
    EXPECT_NEAR(mp->centerOfMass.y, 2, 1e-6);
    EXPECT_NEAR(mp->centerOfMass.z, 3, 1e-6);
    Box3 bb = s->shape.boundingBox();
    EXPECT_NEAR(bb.min.x, 1 - r, 1e-6);
    EXPECT_NEAR(bb.max.x, 1 + r, 1e-6);
    EXPECT_NEAR(bb.min.z, 3 - r, 1e-6);
    EXPECT_NEAR(bb.max.z, 3 + r, 1e-6);
}

TEST(Kernel, SphereArgumentValidation) {
    EXPECT_EQ(makeSphere({}, 0).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(makeSphere({}, -2).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(makeSphere({std::nan(""), 0, 0}, 1).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(makeSphere({0, 0, 0}, std::numeric_limits<double>::infinity()).error().code, ErrorCode::InvalidArgument);
}

TEST(Kernel, MassPropertiesInertiaOfBox) {
    auto mp = massProperties(box(2, 4, 6));
    ASSERT_TRUE(mp);
    EXPECT_NEAR(mp->centerOfMass.x, 1, 1e-9);
    EXPECT_NEAR(mp->inertia[0], 48.0 * (16 + 36) / 12.0, 1e-6); // m/12 (b^2 + c^2), m = 48
    EXPECT_NEAR(mp->inertia[1], 0, 1e-6);
}

TEST(Kernel, SubShapeKeyBoxFaces) {
    Shape b = box(2, 4, 6);
    std::set<std::array<double, 4>> got, want;
    for (std::uint32_t i = 0; i < 6; ++i) got.insert(b.subShapeKey(SubShapeKind::Face, i));
    want = {{0, 2, 3, 24}, {2, 2, 3, 24}, {1, 0, 3, 12}, {1, 4, 3, 12}, {1, 2, 0, 8}, {1, 2, 6, 8}};
    EXPECT_EQ(got, want);
    // Edges: 12, lengths 2/4/6 x4 each
    int l2 = 0, l4 = 0, l6 = 0;
    for (std::uint32_t i = 0; i < 12; ++i) {
        auto k = b.subShapeKey(SubShapeKind::Edge, i);
        (k[3] == 2 ? l2 : k[3] == 4 ? l4 : l6)++;
    }
    EXPECT_EQ(l2, 4); EXPECT_EQ(l4, 4); EXPECT_EQ(l6, 4);
    // Vertices: measure 0, coordinates are corners
    for (std::uint32_t i = 0; i < 8; ++i) {
        auto k = b.subShapeKey(SubShapeKind::Vertex, i);
        EXPECT_EQ(k[3], 0.0);
        EXPECT_TRUE(k[0] == 0 || k[0] == 2);
        EXPECT_TRUE(k[1] == 0 || k[1] == 4);
        EXPECT_TRUE(k[2] == 0 || k[2] == 6);
    }
    EXPECT_EQ(b.subShapeKey(SubShapeKind::Face, 99), (std::array<double, 4>{}));
    EXPECT_EQ(Shape{}.subShapeKey(SubShapeKind::Face, 0), (std::array<double, 4>{}));
}

TEST(Kernel, ExtrudeRectangleCountsAndHistory) {
    const std::vector<Vec3> pts{{0, 0, 0}, {4, 0, 0}, {4, 3, 0}, {0, 3, 0}};
    for (bool asFace : {false, true}) {
        auto prof = asFace ? makePolygonFace(pts) : makePolygonWire(pts);
        ASSERT_TRUE(prof);
        auto r = extrudeProfile(*prof, {0, 0, 5});
        ASSERT_TRUE(r) << r.error().message;
        EXPECT_EQ(r->shape.type(), ShapeType::Solid);
        EXPECT_EQ((counts(r->shape)), (Counts{6, 12, 8}));
        EXPECT_NEAR(volume(r->shape), 60.0, 1e-9);

        const ShapeHistory& h = r->history;
        ASSERT_EQ(prof->subShapeCount(SubShapeKind::Edge), 4u);
        std::set<std::uint32_t> sideFaces;
        for (std::uint32_t e = 0; e < 4; ++e) {
            auto it = h.generated.find({0, SubShapeKind::Edge, e});
            ASSERT_NE(it, h.generated.end()) << "edge " << e;
            ASSERT_EQ(it->second.size(), 1u);
            EXPECT_EQ(it->second[0].kind, SubShapeKind::Face);
            sideFaces.insert(it->second[0].index);
            // the side face's area is edge length * 5
            auto edgeKey = prof->subShapeKey(SubShapeKind::Edge, e);
            auto faceKey = r->shape.subShapeKey(SubShapeKind::Face, it->second[0].index);
            EXPECT_NEAR(faceKey[3], edgeKey[3] * 5, 1e-9);
        }
        EXPECT_EQ(sideFaces.size(), 4u);
        if (asFace) {
            auto m = h.modified.find({0, SubShapeKind::Face, 0});
            auto g = h.generated.find({0, SubShapeKind::Face, 0});
            ASSERT_NE(m, h.modified.end());
            ASSERT_NE(g, h.generated.end());
            ASSERT_EQ(m->second.size(), 1u);
            ASSERT_EQ(g->second.size(), 1u);
            EXPECT_NE(m->second[0], g->second[0]);
            auto start = r->shape.subShapeKey(SubShapeKind::Face, m->second[0].index);
            auto end = r->shape.subShapeKey(SubShapeKind::Face, g->second[0].index);
            EXPECT_NEAR(start[2], 0, 1e-9);
            EXPECT_NEAR(end[2], 5, 1e-9);
            EXPECT_NEAR(start[3], 12, 1e-9);
        }
        for (std::uint32_t v = 0; v < 4; ++v) EXPECT_TRUE(h.generated.contains({0, SubShapeKind::Vertex, v}));
    }
}

TEST(Kernel, ExtrudeErrors) {
    const std::vector<Vec3> pts{{0, 0, 0}, {4, 0, 0}, {4, 3, 0}};
    auto prof = makePolygonFace(pts);
    ASSERT_TRUE(prof);
    EXPECT_EQ(extrudeProfile(*prof, {0, 0, 0}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(extrudeProfile(*prof, {1, 0, 0}).error().code, ErrorCode::InvalidArgument); // in-plane
    EXPECT_EQ(extrudeProfile(Shape{}, {0, 0, 1}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(extrudeProfile(box(1, 1, 1), {0, 0, 1}).error().code, ErrorCode::InvalidArgument);
    CancelSource cs;
    cs.cancel();
    EXPECT_EQ(extrudeProfile(*prof, {0, 0, 1}, cs.token()).error().code, ErrorCode::Cancelled);
    const std::vector<Vec3> nonplanar{{0, 0, 0}, {1, 0, 0}, {1, 1, 1}, {0, 1, 0}};
    EXPECT_EQ(makePolygonFace(nonplanar).error().code, ErrorCode::InvalidArgument);
}

TEST(Kernel, BooleanVolumes) {
    Shape a = box(10, 10, 10);
    Shape b = box(10, 10, 10, {5, 0, 0});
    auto f = fuse(a, b);
    ASSERT_TRUE(f) << f.error().message;
    EXPECT_NEAR(volume(f->shape), 1500, 1e-6);
    auto c = cut(a, b);
    ASSERT_TRUE(c);
    EXPECT_NEAR(volume(c->shape), 500, 1e-6);
    auto i = common(a, b);
    ASSERT_TRUE(i);
    EXPECT_NEAR(volume(i->shape), 500, 1e-6);
    // Hole through a box
    auto cyl = makeCylinder({5, 5, -1}, {0, 0, 1}, 2, 12);
    ASSERT_TRUE(cyl);
    auto holed = cut(a, cyl->shape);
    ASSERT_TRUE(holed);
    EXPECT_NEAR(volume(holed->shape), 1000 - kPi * 4 * 10, 1e-6);
    // Disjoint common -> KernelFailure (empty)
    EXPECT_EQ(common(a, box(1, 1, 1, {50, 50, 50})).error().code, ErrorCode::KernelFailure);
    CancelSource cs;
    cs.cancel();
    EXPECT_EQ(fuse(a, b, cs.token()).error().code, ErrorCode::Cancelled);
}

TEST(Kernel, BooleanDoesNotMutateInputs) {
    Shape a = box(10, 10, 10);
    Shape b = box(10, 10, 10, {5, 0, 0});
    auto before = fingerprint(a);
    ASSERT_TRUE(fuse(a, b));
    EXPECT_EQ(before, fingerprint(a));
}

TEST(Kernel, CutHistoryClassifiesFaces) {
    Shape a = box(10, 10, 10);
    auto cyl = makeCylinder({5, 5, -1}, {0, 0, 1}, 2, 12);
    ASSERT_TRUE(cyl);
    auto r = cut(a, cyl->shape);
    ASSERT_TRUE(r);
    const ShapeHistory& h = r->history;
    // 4 side faces untouched; top and bottom faces are modified (they gain a hole).
    int unchangedFaces = 0, modifiedFaces = 0;
    for (std::uint32_t f = 0; f < 6; ++f) {
        InputSubShape k{0, SubShapeKind::Face, f};
        if (h.unchanged.contains(k)) ++unchangedFaces;
        if (h.modified.contains(k)) ++modifiedFaces;
        // no face is in two classes or in none
        EXPECT_EQ(h.unchanged.contains(k) + h.modified.contains(k) + h.deleted.contains(k), 1) << f;
    }
    EXPECT_EQ(unchangedFaces, 4);
    EXPECT_EQ(modifiedFaces, 2);
    // Tool's lateral face survives as the hole wall; its two caps are deleted.
    int toolFacesKept = 0, toolFacesDeleted = 0;
    for (std::uint32_t f = 0; f < 3; ++f) {
        InputSubShape k{1, SubShapeKind::Face, f};
        if (h.modified.contains(k)) ++toolFacesKept;
        if (h.deleted.contains(k)) ++toolFacesDeleted;
    }
    EXPECT_EQ(toolFacesKept, 1);
    EXPECT_EQ(toolFacesDeleted, 2);
    // Every output index referenced is in range.
    for (const auto& [k, outs] : h.modified)
        for (const auto& o : outs) EXPECT_LT(o.index, r->shape.subShapeCount(o.kind));
}

TEST(Kernel, OrderingIdenticalAcross100Builds) {
    const std::vector<Vec3> pts{{0, 0, 0}, {4, 0, 0}, {4, 3, 0}, {0, 3, 0}};
    auto build = [&] {
        auto prof = makePolygonFace(pts);
        auto ex = extrudeProfile(*prof, {0, 0, 5});
        auto cyl = makeCylinder({2, 1.5, -1}, {0, 0, 1}, 1, 8);
        auto cutr = cut(ex->shape, cyl->shape);
        auto fz = fuse(cutr->shape, box(2, 2, 2, {3, 2, 4}).withStamp({}));
        return std::pair{fingerprint(fz->shape), fz->history};
    };
    const auto ref = build();
    ASSERT_GT(ref.first.size(), 10u);
    for (int i = 0; i < 100; ++i) {
        const auto cur = build();
        ASSERT_EQ(cur.first, ref.first) << "iteration " << i;
        ASSERT_EQ(cur.second.generated, ref.second.generated) << i;
        ASSERT_EQ(cur.second.modified, ref.second.modified) << i;
        ASSERT_EQ(cur.second.deleted, ref.second.deleted) << i;
        ASSERT_EQ(cur.second.unchanged, ref.second.unchanged) << i;
    }
}

// ------------------------------------------------------------------------------------------ exchange

TEST(KernelIo, StepErrorPaths) {
    EXPECT_EQ(importStep(tmpDir() / "does_not_exist.step").error().code, ErrorCode::IoError);
    EXPECT_EQ(importStl(tmpDir() / "does_not_exist.stl").error().code, ErrorCode::IoError);
    const auto garbage = tmpDir() / "garbage.step";
    { std::ofstream(garbage) << "this is definitely not a STEP file\nat all 12345\n"; }
    EXPECT_EQ(importStep(garbage).error().code, ErrorCode::ParseError);
    const auto garbageStl = tmpDir() / "garbage.stl";
    { std::ofstream(garbageStl) << "solid nope\nthis is not facets\nendsolid\n"; }
    EXPECT_EQ(importStl(garbageStl).error().code, ErrorCode::ParseError);
    EXPECT_EQ(exportStep(tmpDir() / "no_such_dir" / "x.step", box(1, 1, 1)).error().code, ErrorCode::IoError);
    EXPECT_EQ(exportStep(tmpDir() / "x.step", Shape{}).error().code, ErrorCode::InvalidArgument);
    CancelSource cs;
    cs.cancel();
    EXPECT_EQ(importStep(garbage, cs.token()).error().code, ErrorCode::Cancelled);
}

TEST(KernelIo, StepSingleShapeRoundTrip) {
    Shape b = box(10, 20, 30);
    const auto p = tmpDir() / "single.step";
    ASSERT_TRUE(exportStep(p, b));
    auto in = importStep(p);
    ASSERT_TRUE(in) << in.error().message;
    ASSERT_EQ(in->roots.size(), 1u);
    ASSERT_EQ(in->nodes.size(), 1u);
    EXPECT_EQ(in->nodes[0].name, "Part");
    EXPECT_EQ(in->lengthUnitInFile, "mm");
    EXPECT_NEAR(volume(in->nodes[0].shape), 6000, 1e-6);
    EXPECT_EQ((counts(in->nodes[0].shape)), (Counts{6, 12, 8}));
}

TEST(KernelIo, StepAssemblyRoundTripKeepsNamesColoursTransformsVolume) {
    Shape bx = box(10, 10, 10);
    Shape cy = makeCylinder({0, 0, 0}, {0, 0, 1}, 2, 7)->shape;

    ImportedAssembly a;
    auto node = [&](std::string name, Shape s, std::optional<Color> c, Transform t, std::int32_t parent) {
        AssemblyNode n;
        n.name = std::move(name);
        n.shape = std::move(s);
        n.color = c;
        n.local = t;
        n.parent = parent;
        a.nodes.push_back(std::move(n));
        return static_cast<std::uint32_t>(a.nodes.size() - 1);
    };
    Transform t1; t1.m[3] = 100;                    // translate x by 100
    Transform t2; t2.m[7] = -30; t2.m[11] = 5;
    Transform rot; rot.m = {0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; // 90 deg about Z
    const Color red{1, 0, 0, 1}, green{0, 0.5f, 0, 1};
    const auto root = node("Main", Shape{}, std::nullopt, Transform{}, -1);
    const auto c1 = node("Cube A", bx, red, t1, 0);
    const auto c2 = node("Cube B", bx, green, t2, 0);     // second instance of same shape
    const auto c3 = node("Pin", cy, std::nullopt, rot, 0);
    a.roots = {root};
    a.nodes[root].children = {c1, c2, c3};

    const auto p = tmpDir() / "asm.step";
    ASSERT_TRUE(exportStep(p, a));
    auto in = importStep(p);
    ASSERT_TRUE(in) << in.error().message;
    ASSERT_EQ(in->roots.size(), 1u);
    ASSERT_EQ(in->nodes.size(), 4u);
    const AssemblyNode& r = in->nodes[in->roots[0]];
    EXPECT_EQ(r.name, "Main");
    EXPECT_TRUE(r.shape.isNull());
    ASSERT_EQ(r.children.size(), 3u);

    double totalVol = 0;
    std::map<std::string, const AssemblyNode*> byName;
    for (auto ci : r.children) {
        const AssemblyNode& n = in->nodes[ci];
        EXPECT_EQ(n.parent, static_cast<std::int32_t>(in->roots[0]));
        byName[n.name] = &n;
        totalVol += volume(n.shape);
    }
    ASSERT_EQ(byName.size(), 3u);
    EXPECT_NEAR(totalVol, 1000 + 1000 + kPi * 4 * 7, 1e-6);
    // Instances share geometry handles.
    EXPECT_TRUE(byName["Cube A"]->shape.impl() == byName["Cube B"]->shape.impl());
    // Colours.
    ASSERT_TRUE(byName["Cube A"]->color);
    ASSERT_TRUE(byName["Cube B"]->color);
    EXPECT_NEAR(byName["Cube A"]->color->r, 1.0, 1e-3);
    EXPECT_NEAR(byName["Cube A"]->color->g, 0.0, 1e-3);
    EXPECT_NEAR(byName["Cube B"]->color->g, 0.5, 1e-3);
    EXPECT_FALSE(byName["Pin"]->color.has_value());
    // Transforms.
    EXPECT_NEAR(byName["Cube A"]->local.m[3], 100, 1e-9);
    EXPECT_NEAR(byName["Cube B"]->local.m[7], -30, 1e-9);
    EXPECT_NEAR(byName["Cube B"]->local.m[11], 5, 1e-9);
    EXPECT_NEAR(byName["Pin"]->local.m[1], -1, 1e-9);
    EXPECT_NEAR(byName["Pin"]->local.m[4], 1, 1e-9);
}

TEST(KernelIo, StepRootColourAndCancel) {
    ImportedAssembly a;
    AssemblyNode n;
    n.name = "Painted ünïcode";
    n.shape = box(1, 2, 3);
    n.color = Color{0.25f, 0.5f, 0.75f, 1};
    a.nodes.push_back(n);
    a.roots = {0};
    const auto p = tmpDir() / "paint.step";
    ASSERT_TRUE(exportStep(p, a));
    auto in = importStep(p);
    ASSERT_TRUE(in);
    ASSERT_EQ(in->nodes.size(), 1u);
    EXPECT_EQ(in->nodes[0].name, "Painted ünïcode");
    ASSERT_TRUE(in->nodes[0].color);
    EXPECT_NEAR(in->nodes[0].color->r, 0.25, 2e-3);
    EXPECT_NEAR(in->nodes[0].color->b, 0.75, 2e-3);
    EXPECT_EQ(exportStep(tmpDir() / "bad.step", ImportedAssembly{}).error().code, ErrorCode::InvalidArgument);
}

TEST(KernelIo, StepImportConvertsInchesToMm) {
    // Export a 10 mm cube, then rewrite the unit declaration to inch (file says 10 inch cube -> 254 mm).
    const auto p = tmpDir() / "unit_src.step";
    ASSERT_TRUE(exportStep(p, box(10, 10, 10)));
    std::string text;
    { std::ifstream f(p); text.assign(std::istreambuf_iterator<char>(f), {}); }
    const std::string mm = "#346 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );";
    const auto pos = text.find(mm);
    ASSERT_NE(pos, std::string::npos) << "exporter unit layout changed";
    text.replace(pos, mm.size(),
                 "#346 = ( CONVERSION_BASED_UNIT('INCH',#9990001) LENGTH_UNIT() NAMED_UNIT(#9990003) );\n"
                 "#9990001 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#9990002);\n"
                 "#9990002 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n"
                 "#9990003 = DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);");
    const auto p2 = tmpDir() / "unit_inch.step";
    { std::ofstream(p2) << text; }
    auto in = importStep(p2);
    ASSERT_TRUE(in) << in.error().message;
    EXPECT_EQ(in->lengthUnitInFile, "inch");
    EXPECT_NEAR(volume(in->nodes[0].shape), 254.0 * 254.0 * 254.0, 1e-3);
}

TEST(KernelIo, StlRoundTripAreaWithin1Percent) {
    auto cyl = makeCylinder({0, 0, 0}, {0, 0, 1}, 5, 12);
    ASSERT_TRUE(cyl);
    Shape part = fuse(cyl->shape, box(6, 6, 6, {2, 2, 0}))->shape;
    const double area0 = massProperties(part)->surfaceArea;
    for (bool binary : {true, false}) {
        const auto p = tmpDir() / (binary ? "part_bin.stl" : "part_ascii.stl");
        StlOptions o;
        o.binary = binary;
        o.linearDeflection = 0.01;
        o.angularDeflection = 0.1;
        ASSERT_TRUE(exportStl(p, part, o));
        ASSERT_GT(fs::file_size(p), 100u);
        auto in = importStl(p);
        ASSERT_TRUE(in) << in.error().message;
        const double area1 = massProperties(*in)->surfaceArea;
        EXPECT_NEAR(area1, area0, 0.01 * area0) << (binary ? "binary" : "ascii");
        EXPECT_GT(in->subShapeCount(SubShapeKind::Face), 50u);
    }
    // Exporting must not have mutated the input shape.
    EXPECT_NEAR(massProperties(part)->surfaceArea, area0, 1e-9);
    CancelSource cs;
    cs.cancel();
    EXPECT_EQ(exportStl(tmpDir() / "c.stl", part, {}, cs.token()).error().code, ErrorCode::Cancelled);
    EXPECT_EQ(exportStl(tmpDir() / "x.stl", Shape{}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(exportStl(tmpDir() / "no_dir" / "x.stl", part).error().code, ErrorCode::IoError);
}

// ------------------------------------------------------------------------------------------ P1a A4 ops

namespace {

using Arr4 = std::array<double, 4>;

/// Index of the sub-shape whose geometric key (centroid x,y,z, measure) matches `want` within 1e-6.
std::optional<std::uint32_t> findSub(const Shape& s, SubShapeKind k, const Arr4& want) {
    for (std::uint32_t i = 0; i < s.subShapeCount(k); ++i) {
        const Arr4 key = s.subShapeKey(k, i);
        bool ok = true;
        for (int c = 0; c < 4; ++c) ok = ok && std::abs(key[c] - want[c]) <= 1e-6;
        if (ok) return i;
    }
    return std::nullopt;
}

/// Output faces referenced by any history entry (generated, modified or unchanged).
std::set<std::uint32_t> coveredFaces(const ShapeHistory& h) {
    std::set<std::uint32_t> out;
    auto take = [&](const std::vector<OutputSubShape>& v) {
        for (const auto& o : v)
            if (o.kind == SubShapeKind::Face) out.insert(o.index);
    };
    for (const auto& [k, v] : h.generated) take(v);
    for (const auto& [k, v] : h.modified) take(v);
    for (const auto& [k, o] : h.unchanged) take({o});
    return out;
}

void expectEveryOutputFaceMapped(const ShapeResult& r) {
    const auto cov = coveredFaces(r.history);
    for (std::uint32_t f = 0; f < r.shape.subShapeCount(SubShapeKind::Face); ++f)
        EXPECT_TRUE(cov.contains(f)) << "output face " << f << " has no history entry";
}

std::vector<std::uint32_t> ids(std::initializer_list<std::uint32_t> l) { return std::vector<std::uint32_t>(l); }

} // namespace

TEST(KernelOps, PlaneAndAxisHelpers) {
    auto p = planeFromThreePoints({0, 0, 2}, {1, 0, 2}, {0, 1, 2});
    ASSERT_TRUE(p);
    EXPECT_NEAR(p->normal.z, 1, 1e-12);
    EXPECT_NEAR(p->origin.z, 2, 1e-12);
    EXPECT_EQ(planeFromThreePoints({0, 0, 0}, {1, 1, 1}, {2, 2, 2}).error().code, ErrorCode::InvalidArgument);

    auto pn = planeFromPointNormal({1, 2, 3}, {0, 3, 4});
    ASSERT_TRUE(pn);
    EXPECT_NEAR(length(pn->normal), 1, 1e-12);
    EXPECT_NEAR(length(pn->xDir), 1, 1e-12);
    EXPECT_NEAR(dot(pn->normal, pn->xDir), 0, 1e-12);
    // Deterministic: same normal -> same frame
    EXPECT_EQ(planeFromPointNormal({0, 0, 0}, {0, 3, 4})->xDir, planeFromPointNormal({9, 9, 9}, {0, 6, 8})->xDir);
    EXPECT_EQ(planeFromPointNormal({}, {0, 0, 0}).error().code, ErrorCode::InvalidArgument);

    // Box faces: outward normals
    Shape b = box(10, 10, 10);
    auto top = findSub(b, SubShapeKind::Face, {5, 5, 10, 100});
    auto bottom = findSub(b, SubShapeKind::Face, {5, 5, 0, 100});
    ASSERT_TRUE(top && bottom);
    auto pt = planeFromFace(b, *top);
    ASSERT_TRUE(pt) << pt.error().message;
    EXPECT_NEAR(pt->normal.z, 1, 1e-9);
    EXPECT_NEAR(pt->origin.z, 10, 1e-9);
    auto pb = planeFromFace(b, *bottom);
    ASSERT_TRUE(pb);
    EXPECT_NEAR(pb->normal.z, -1, 1e-9);
    EXPECT_EQ(planeFromFace(b, 99).error().code, ErrorCode::InvalidArgument);
    // Cylinder: exactly one non-planar face (the lateral surface) is rejected.
    Shape cyl = makeCylinder({0, 0, 0}, {0, 0, 1}, 1, 2)->shape;
    int rejected = 0;
    for (std::uint32_t f = 0; f < cyl.subShapeCount(SubShapeKind::Face); ++f) {
        auto r = planeFromFace(cyl, f);
        if (!r) {
            EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
            ++rejected;
        }
    }
    EXPECT_EQ(rejected, 1);

    auto edge = findSub(b, SubShapeKind::Edge, {5, 0, 10, 10});
    ASSERT_TRUE(edge);
    auto ax = axisFromEdge(b, *edge);
    ASSERT_TRUE(ax) << ax.error().message;
    EXPECT_NEAR(std::abs(ax->direction.x), 1, 1e-9);
    EXPECT_NEAR(length(ax->direction), 1, 1e-12);
    EXPECT_EQ(axisFromEdge(cyl, 0).error().code, ErrorCode::InvalidArgument); // circle edge, not a line
    EXPECT_EQ(axisFromPoints({1, 1, 1}, {1, 1, 1}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(axisFromEdge(b, 999).error().code, ErrorCode::InvalidArgument);
}

TEST(KernelOps, RevolveRingVolumeHistoryAndErrors) {
    const std::vector<Vec3> pts{{1, 0, 0}, {3, 0, 0}, {3, 0, 2}, {1, 0, 2}};
    auto face = makePolygonFace(pts);
    ASSERT_TRUE(face);
    const Axis zAxis{{0, 0, 0}, {0, 0, 1}};
    auto full = revolve(*face, zAxis, kTwoPi);
    ASSERT_TRUE(full) << full.error().message;
    EXPECT_EQ(full->shape.type(), ShapeType::Solid);
    EXPECT_NEAR(volume(full->shape), kPi * (9 - 1) * 2, 1e-6);
    EXPECT_EQ(full->shape.subShapeCount(SubShapeKind::Face), 4u); // inner + outer cylinder, two annular caps
    expectEveryOutputFaceMapped(*full);
    for (std::uint32_t e = 0; e < 4; ++e) EXPECT_TRUE(full->history.generated.contains({0, SubShapeKind::Edge, e})) << e;

    auto quarter = revolve(*face, zAxis, kPi / 2);
    ASSERT_TRUE(quarter) << quarter.error().message;
    EXPECT_NEAR(volume(quarter->shape), kPi * 8 * 2 / 4, 1e-6);
    // Partial revolve: start cap modified, end cap generated (face-profile convention).
    EXPECT_TRUE(quarter->history.modified.contains({0, SubShapeKind::Face, 0}));
    EXPECT_TRUE(quarter->history.generated.contains({0, SubShapeKind::Face, 0}));
    expectEveryOutputFaceMapped(*quarter);

    // Wire profile gives the same solid.
    auto wire = makePolygonWire(pts);
    ASSERT_TRUE(wire);
    auto fromWire = revolve(*wire, zAxis, kPi / 2);
    ASSERT_TRUE(fromWire) << fromWire.error().message;
    EXPECT_NEAR(volume(fromWire->shape), volume(quarter->shape), 1e-9);

    EXPECT_EQ(revolve(*face, zAxis, 0).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(revolve(*face, zAxis, 7.0).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(revolve(*face, {{0, 0, 0}, {0, 0, 0}}, 1).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(revolve(Shape{}, zAxis, 1).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(revolve(box(1, 1, 1), zAxis, 1).error().code, ErrorCode::InvalidArgument);
    CancelSource cs;
    cs.cancel();
    EXPECT_EQ(revolve(*face, zAxis, 1, cs.token()).error().code, ErrorCode::Cancelled);
}

TEST(KernelOps, FilletBoxEdgeVolumeHistoryAndDeterminism) {
    Shape b = box(10, 10, 10);
    auto e = findSub(b, SubShapeKind::Edge, {5, 0, 10, 10});
    ASSERT_TRUE(e);
    const double r = 1.0;
    auto f = filletEdges(b, ids({*e}), r);
    ASSERT_TRUE(f) << f.error().message;
    // Removed material per unit length: (1 - pi/4) r^2
    EXPECT_NEAR(volume(f->shape), 1000 - 10 * (1 - kPi / 4) * r * r, 1e-6);
    EXPECT_EQ(f->shape.subShapeCount(SubShapeKind::Face), 7u);
    // Exactly one new face, generated from the filleted edge: a quarter cylinder of area pi/2 * r * L.
    auto gen = f->history.generated.find({0, SubShapeKind::Edge, *e});
    ASSERT_NE(gen, f->history.generated.end());
    ASSERT_EQ(gen->second.size(), 1u);
    EXPECT_EQ(gen->second[0].kind, SubShapeKind::Face);
    EXPECT_NEAR(f->shape.subShapeKey(SubShapeKind::Face, gen->second[0].index)[3], kPi / 2 * r * 10, 1e-6);
    // The two faces adjacent to the edge are trimmed, and so are the two end faces at its vertices; the remaining
    // two faces (bottom and back) are untouched.
    int modifiedFaces = 0, unchangedFaces = 0;
    for (std::uint32_t fi = 0; fi < 6; ++fi) {
        modifiedFaces += f->history.modified.contains({0, SubShapeKind::Face, fi});
        unchangedFaces += f->history.unchanged.contains({0, SubShapeKind::Face, fi});
    }
    EXPECT_EQ(modifiedFaces, 4);
    EXPECT_EQ(unchangedFaces, 2);
    expectEveryOutputFaceMapped(*f);
    EXPECT_NEAR(volume(b), 1000, 1e-9); // input untouched

    const auto refFp = fingerprint(f->shape);
    for (int i = 0; i < 20; ++i) {
        auto g = filletEdges(b, ids({*e}), r);
        ASSERT_TRUE(g);
        ASSERT_EQ(fingerprint(g->shape), refFp) << i;
        ASSERT_EQ(g->history.generated, f->history.generated) << i;
        ASSERT_EQ(g->history.modified, f->history.modified) << i;
        ASSERT_EQ(g->history.deleted, f->history.deleted) << i;
    }

    // Input validation
    EXPECT_EQ(filletEdges(b, {}, r).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(filletEdges(b, ids({999}), r).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(filletEdges(b, ids({*e}), 0).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(filletEdges(Shape{}, ids({0}), r).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(filletEdges(*makePolygonFace(std::vector<Vec3>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}), ids({0}), r).error().code,
              ErrorCode::InvalidArgument);
    CancelSource cs;
    cs.cancel();
    EXPECT_EQ(filletEdges(b, ids({*e}), r, cs.token()).error().code, ErrorCode::Cancelled);
}

TEST(KernelOps, FilletRadiusTooLargeNamesTheEdge) {
    Shape small = box(1, 1, 1);
    auto e = findSub(small, SubShapeKind::Edge, {0.5, 0, 1, 1});
    ASSERT_TRUE(e);
    auto f = filletEdges(small, ids({*e}), 1.5);
    ASSERT_FALSE(f);
    EXPECT_EQ(f.error().code, ErrorCode::KernelFailure);
    EXPECT_NE(f.error().message.find("fillet radius too large for edge " + std::to_string(*e)), std::string::npos)
        << f.error().message;
}

TEST(KernelOps, ChamferBoxEdgeVolumeAreaAndDeterminism) {
    Shape b = box(10, 10, 10);
    auto e = findSub(b, SubShapeKind::Edge, {5, 0, 10, 10});
    ASSERT_TRUE(e);
    // Asymmetric: triangle legs 2 and 3 -> removed 0.5 * 2 * 3 * 10 = 30.
    auto c = chamferEdges(b, ids({*e}), 2, 3);
    ASSERT_TRUE(c) << c.error().message;
    EXPECT_NEAR(volume(c->shape), 970, 1e-6);
    EXPECT_EQ(c->shape.subShapeCount(SubShapeKind::Face), 7u);
    auto gen = c->history.generated.find({0, SubShapeKind::Edge, *e});
    ASSERT_NE(gen, c->history.generated.end());
    ASSERT_EQ(gen->second.size(), 1u);
    EXPECT_NEAR(c->shape.subShapeKey(SubShapeKind::Face, gen->second[0].index)[3], 10 * std::sqrt(13.0), 1e-6);
    expectEveryOutputFaceMapped(*c);

    // Equal legs via distance + angle (45 deg): removed 0.5 * 2 * 2 * 10 = 20.
    auto a = chamferEdgesByAngle(b, ids({*e}), 2, kPi / 4);
    ASSERT_TRUE(a) << a.error().message;
    EXPECT_NEAR(volume(a->shape), 980, 1e-6);

    const auto refFp = fingerprint(c->shape);
    for (int i = 0; i < 20; ++i) {
        auto g = chamferEdges(b, ids({*e}), 2, 3);
        ASSERT_TRUE(g);
        ASSERT_EQ(fingerprint(g->shape), refFp) << i;
        ASSERT_EQ(g->history.generated, c->history.generated) << i;
    }
    EXPECT_EQ(chamferEdges(b, ids({*e}), 0, 3).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(chamferEdgesByAngle(b, ids({*e}), 2, 0).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(chamferEdges(b, ids({12345}), 1, 1).error().code, ErrorCode::InvalidArgument);
}

TEST(KernelOps, ChamferTooLargeNamesTheEdge) {
    Shape small = box(1, 1, 1);
    auto e = findSub(small, SubShapeKind::Edge, {0.5, 0, 1, 1});
    ASSERT_TRUE(e);
    auto c = chamferEdges(small, ids({*e}), 2, 2);
    ASSERT_FALSE(c);
    EXPECT_EQ(c.error().code, ErrorCode::KernelFailure);
    EXPECT_NE(c.error().message.find("chamfer distance too large for edge " + std::to_string(*e)), std::string::npos)
        << c.error().message;
}

TEST(KernelOps, HoleThroughRemovesExactVolume) {
    Shape b = box(10, 10, 10);
    HoleSpec spec;
    spec.diameter = 2;
    spec.through = true;
    // Top-face entry, drilling down (-Z).
    auto h = hole(b, Axis{{5, 5, 10}, {0, 0, -1}}, spec);
    ASSERT_TRUE(h) << h.error().message;
    EXPECT_NEAR(volume(h->shape), 1000 - kPi * 1 * 10, 1e-6);
    EXPECT_EQ(h->shape.subShapeCount(SubShapeKind::Face), 7u); // six box faces (top + bottom gain a hole) + wall
    expectEveryOutputFaceMapped(*h);

    // Side-face entry, through along +X: same removed volume.
    auto side = hole(b, Axis{{0, 5, 5}, {1, 0, 0}}, spec);
    ASSERT_TRUE(side) << side.error().message;
    EXPECT_NEAR(volume(side->shape), 1000 - kPi * 10, 1e-6);

    const auto refFp = fingerprint(h->shape);
    for (int i = 0; i < 20; ++i) {
        auto g = hole(b, Axis{{5, 5, 10}, {0, 0, -1}}, spec);
        ASSERT_TRUE(g);
        ASSERT_EQ(fingerprint(g->shape), refFp) << i;
        ASSERT_EQ(g->history.modified, h->history.modified) << i;
    }
}

TEST(KernelOps, HoleBlindWithDrillPointAndFlatBottom) {
    Shape b = box(10, 10, 10);
    HoleSpec spec;
    spec.diameter = 2;
    spec.depth = 5;
    spec.drillPointAngle = kPi; // flat bottom
    auto flat = hole(b, Axis{{5, 5, 10}, {0, 0, -1}}, spec);
    ASSERT_TRUE(flat) << flat.error().message;
    EXPECT_NEAR(volume(flat->shape), 1000 - kPi * 1 * 5, 1e-6);

    // 118 deg drill point: cone of height r / tan(59 deg) below the flat-bottom depth.
    spec.drillPointAngle = 2.0943951023931953;
    auto drilled = hole(b, Axis{{5, 5, 10}, {0, 0, -1}}, spec);
    ASSERT_TRUE(drilled) << drilled.error().message;
    const double coneH = 1.0 / std::tan(2.0943951023931953 / 2);
    EXPECT_NEAR(volume(drilled->shape), 1000 - kPi * 5 - kPi * coneH / 3, 1e-6);
    expectEveryOutputFaceMapped(*drilled);
}

TEST(KernelOps, HoleCounterboreAndCountersinkVolumes) {
    Shape b = box(10, 10, 10);
    // Counterbore dia 4 depth 3 on a through hole dia 2: 12 pi (cbore) + 7 pi (hole below it).
    HoleSpec cb;
    cb.kind = HoleKind::Counterbore;
    cb.diameter = 2;
    cb.through = true;
    cb.counterboreDiameter = 4;
    cb.counterboreDepth = 3;
    auto c = hole(b, Axis{{5, 5, 10}, {0, 0, -1}}, cb);
    ASSERT_TRUE(c) << c.error().message;
    EXPECT_NEAR(volume(c->shape), 1000 - 19 * kPi, 1e-6);
    expectEveryOutputFaceMapped(*c);

    // Countersink dia 4, 90 deg, on a through hole dia 2: cone cavity 7 pi / 3 + hole 9 pi.
    HoleSpec cs;
    cs.kind = HoleKind::Countersink;
    cs.diameter = 2;
    cs.through = true;
    cs.countersinkDiameter = 4;
    cs.countersinkAngle = kPi / 2;
    auto s = hole(b, Axis{{5, 5, 10}, {0, 0, -1}}, cs);
    ASSERT_TRUE(s) << s.error().message;
    EXPECT_NEAR(volume(s->shape), 1000 - 34 * kPi / 3, 1e-6);
    expectEveryOutputFaceMapped(*s);
}

TEST(KernelOps, HoleValidationAndErrors) {
    Shape b = box(10, 10, 10);
    const Axis down{{5, 5, 10}, {0, 0, -1}};
    HoleSpec ok;
    ok.diameter = 2;
    ok.depth = 5;
    EXPECT_TRUE(hole(b, down, ok));

    HoleSpec noDia = ok;
    noDia.diameter = 0;
    EXPECT_EQ(hole(b, down, noDia).error().code, ErrorCode::InvalidArgument);
    HoleSpec noDepth = ok;
    noDepth.depth = 0;
    EXPECT_EQ(hole(b, down, noDepth).error().code, ErrorCode::InvalidArgument);
    HoleSpec badDrill = ok;
    badDrill.drillPointAngle = 4.0;
    EXPECT_EQ(hole(b, down, badDrill).error().code, ErrorCode::InvalidArgument);
    HoleSpec badCb = ok;
    badCb.kind = HoleKind::Counterbore;
    badCb.counterboreDiameter = 1.5; // not larger than the hole
    badCb.counterboreDepth = 1;
    EXPECT_EQ(hole(b, down, badCb).error().code, ErrorCode::InvalidArgument);
    HoleSpec badCs = ok;
    badCs.kind = HoleKind::Countersink;
    badCs.countersinkDiameter = 4;
    badCs.countersinkAngle = kPi / 2;
    badCs.depth = 0.5; // countersink (depth 1) deeper than the hole
    EXPECT_EQ(hole(b, down, badCs).error().code, ErrorCode::InvalidArgument);
    // Through hole whose entry lies beyond the part along the drill direction
    HoleSpec thr = ok;
    thr.through = true;
    EXPECT_EQ(hole(b, Axis{{5, 5, 20}, {0, 0, 1}}, thr).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(hole(b, Axis{{5, 5, 10}, {0, 0, 0}}, ok).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(hole(Shape{}, down, ok).error().code, ErrorCode::InvalidArgument);
    CancelSource cs;
    cs.cancel();
    EXPECT_EQ(hole(b, down, ok, cs.token()).error().code, ErrorCode::Cancelled);
}

TEST(KernelOps, TransformIsRigidAndMapsSubShapesOneToOne) {
    Shape b = box(2, 3, 4);
    // 90 deg about +Z, then translate by (5, 0, 0): x' = -y + 5, y' = x.
    Transform t;
    t.m = {0, -1, 0, 5, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    auto r = transform(b, t);
    ASSERT_TRUE(r) << r.error().message;
    const Box3 bb = r->shape.boundingBox();
    EXPECT_NEAR(bb.min.x, 2, 1e-7);
    EXPECT_NEAR(bb.max.x, 5, 1e-7);
    EXPECT_NEAR(bb.min.y, 0, 1e-7);
    EXPECT_NEAR(bb.max.y, 2, 1e-7);
    EXPECT_NEAR(volume(r->shape), 24, 1e-9);
    for (std::uint32_t f = 0; f < 6; ++f) {
        auto it = r->history.modified.find({0, SubShapeKind::Face, f});
        ASSERT_NE(it, r->history.modified.end());
        ASSERT_EQ(it->second, (std::vector<OutputSubShape>{{SubShapeKind::Face, f}}));
    }
    expectEveryOutputFaceMapped(*r);

    Transform scaled = t;
    scaled.m[0] = 2;
    EXPECT_EQ(transform(b, scaled).error().code, ErrorCode::InvalidArgument);
    Transform mirror;
    mirror.m = {-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    EXPECT_EQ(transform(b, mirror).error().code, ErrorCode::InvalidArgument);
    Transform projective = t;
    projective.m[12] = 1;
    EXPECT_EQ(transform(b, projective).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(transform(Shape{}, t).error().code, ErrorCode::InvalidArgument);
}

TEST(KernelOps, IntersectWithMatchesCommon) {
    Shape a = box(10, 10, 10);
    Shape c = box(10, 10, 10, {5, 0, 0});
    auto i = intersectWith(a, c);
    ASSERT_TRUE(i);
    EXPECT_NEAR(volume(i->shape), 500, 1e-6);
}

// ---- B1b (F15 / F19 / G1 chains) -----------------------------------------------------------------------------------

namespace {
double b1bVolume(const Shape& s) {
    auto mp = massProperties(s);
    EXPECT_TRUE(mp.has_value());
    return mp ? mp->volume : 0.0;
}
} // namespace

TEST(KernelB1b, MirrorReflectsGeometryKeepsVolumeAndHistory) {
    auto box = makeBox({1, 2, 3}, 4, 5, 6); // x in [1,5]
    ASSERT_TRUE(box.has_value());
    auto m = mirror(box->shape, Plane{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}});
    ASSERT_TRUE(m.has_value()) << m.error().message;
    EXPECT_NEAR(b1bVolume(m->shape), 120.0, 1e-6);
    const Box3 bb = m->shape.boundingBox();
    EXPECT_NEAR(bb.min.x, -5.0, 1e-6);
    EXPECT_NEAR(bb.max.x, -1.0, 1e-6);
    ASSERT_EQ(m->shape.subShapeCount(SubShapeKind::Face), box->shape.subShapeCount(SubShapeKind::Face));
    for (std::uint32_t i = 0; i < box->shape.subShapeCount(SubShapeKind::Face); ++i) {
        auto it = m->history.modified.find({0, SubShapeKind::Face, i});
        ASSERT_NE(it, m->history.modified.end());
        ASSERT_EQ(it->second.size(), 1u);
        EXPECT_EQ(it->second[0].index, i);
    }
    // Mirror of a mirror is the identity placement (volume and bounds).
    auto back = mirror(m->shape, Plane{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}});
    ASSERT_TRUE(back.has_value());
    EXPECT_NEAR(back->shape.boundingBox().min.x, 1.0, 1e-6);
}

TEST(KernelB1b, MirrorRejectsZeroNormalAndNullShape) {
    auto box = makeBox({0, 0, 0}, 1, 1, 1);
    ASSERT_TRUE(box.has_value());
    EXPECT_EQ(mirror(box->shape, Plane{{0, 0, 0}, {0, 0, 0}, {1, 0, 0}}).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(mirror(Shape{}, Plane{}).error().code, ErrorCode::InvalidArgument);
}

TEST(KernelB1b, UniformScaleAboutAPoint) {
    auto box = makeBox({1, 0, 0}, 2, 2, 2);
    ASSERT_TRUE(box.has_value());
    auto s = scale(box->shape, {0, 0, 0}, 2.0);
    ASSERT_TRUE(s.has_value()) << s.error().message;
    EXPECT_NEAR(b1bVolume(s->shape), 8.0 * 8.0, 1e-6);
    EXPECT_NEAR(s->shape.boundingBox().min.x, 2.0, 1e-6);
    EXPECT_NEAR(s->shape.boundingBox().max.x, 6.0, 1e-6);
    EXPECT_EQ(scale(box->shape, {0, 0, 0}, 0.0).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(scale(box->shape, {0, 0, 0}, -1.0).error().code, ErrorCode::InvalidArgument);
}

TEST(KernelB1b, FaceGeometryGivesNormalsAndAxes) {
    auto box = makeBox({0, 0, 0}, 2, 3, 4);
    ASSERT_TRUE(box.has_value());
    int planes = 0;
    for (std::uint32_t i = 0; i < box->shape.subShapeCount(SubShapeKind::Face); ++i) {
        const FaceGeometry g = box->shape.faceGeometry(i);
        ASSERT_EQ(g.kind, FaceGeometry::Kind::Plane);
        ++planes;
        // The outward normal points away from the box centre (1, 1.5, 2).
        const Vec3 c{1, 1.5, 2};
        EXPECT_GT(dot(g.normal, g.point - c), 0.0);
        EXPECT_NEAR(length(g.normal), 1.0, 1e-12);
    }
    EXPECT_EQ(planes, 6);
    auto cyl = makeCylinder({0, 0, 0}, {0, 0, 1}, 2.5, 5);
    ASSERT_TRUE(cyl.has_value());
    int cylinders = 0;
    for (std::uint32_t i = 0; i < cyl->shape.subShapeCount(SubShapeKind::Face); ++i) {
        const FaceGeometry g = cyl->shape.faceGeometry(i);
        if (g.kind != FaceGeometry::Kind::Cylinder) continue;
        ++cylinders;
        EXPECT_NEAR(std::abs(g.direction.z), 1.0, 1e-12);
        EXPECT_NEAR(g.radius, 2.5, 1e-9);
    }
    EXPECT_EQ(cylinders, 1);
    EXPECT_EQ(box->shape.faceGeometry(999).kind, FaceGeometry::Kind::Other);
}

TEST(KernelB1b, TangentChainOfABoxEdgeIsItself) {
    auto box = makeBox({0, 0, 0}, 1, 1, 1);
    ASSERT_TRUE(box.has_value());
    const std::uint32_t e[] = {0};
    auto chain = tangentChain(box->shape, e);
    ASSERT_TRUE(chain.has_value()) << chain.error().message;
    EXPECT_EQ(*chain, std::vector<std::uint32_t>{0});
    const std::uint32_t bad[] = {500};
    EXPECT_EQ(tangentChain(box->shape, bad).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(tangentChain(box->shape, std::span<const std::uint32_t>{}).error().code, ErrorCode::InvalidArgument);
}

TEST(KernelB1b, TangentChainFollowsTheTangentEdgesOfAFilletedCorner) {
    auto box = makeBox({0, 0, 0}, 10, 10, 10);
    ASSERT_TRUE(box.has_value());
    // Vertical edge at (10, 10): fillet it, then chain from the top fillet arc (length pi/2 for r = 1).
    std::uint32_t vertical = 0;
    for (std::uint32_t i = 0; i < box->shape.subShapeCount(SubShapeKind::Edge); ++i) {
        const auto k = box->shape.subShapeKey(SubShapeKind::Edge, i);
        if (std::abs(k[0] - 10) < 1e-6 && std::abs(k[1] - 10) < 1e-6 && std::abs(k[3] - 10) < 1e-6) vertical = i;
    }
    const std::uint32_t vEdge[] = {vertical};
    auto f = filletEdges(box->shape, vEdge, 1.0);
    ASSERT_TRUE(f.has_value()) << f.error().message;
    std::uint32_t arc = UINT32_MAX;
    for (std::uint32_t i = 0; i < f->shape.subShapeCount(SubShapeKind::Edge); ++i) {
        const auto k = f->shape.subShapeKey(SubShapeKind::Edge, i);
        if (std::abs(k[3] - std::numbers::pi / 2) < 1e-6 && std::abs(k[2] - 10) < 1e-6) { arc = i; break; }
    }
    ASSERT_NE(arc, UINT32_MAX);
    const std::uint32_t arcEdge[] = {arc};
    auto chain = tangentChain(f->shape, arcEdge);
    ASSERT_TRUE(chain.has_value()) << chain.error().message;
    // The arc meets two top straight edges tangentially; those meet the next corner at 90 degrees, so the chain is 3.
    EXPECT_EQ(chain->size(), 3u);
}

// ---------------------------------------------------------------------------------------------------------------
// K1 robustness: result validation (F38), edge-op pre-validation (F37/F39). Seed-level regressions (the exact
// replayed scenarios of F37/F38/F39) run through ctest from tests/kernel_robustness.cmake.

namespace {
Box3 makeBoxRange(Vec3 lo, Vec3 hi) {
    Box3 b;
    b.min = lo;
    b.max = hi;
    return b;
}
} // namespace

TEST(KernelCheck, BooleanVolumeInvariants) {
    using check::BooleanKind;
    // Fuse: max(|A|,|B|) <= |R| <= |A|+|B|
    EXPECT_TRUE(check::volumesPlausible(BooleanKind::Fuse, 10, 10, 10));  // identical operands
    EXPECT_TRUE(check::volumesPlausible(BooleanKind::Fuse, 10, 10, 15));
    EXPECT_TRUE(check::volumesPlausible(BooleanKind::Fuse, 10, 10, 20));
    EXPECT_FALSE(check::volumesPlausible(BooleanKind::Fuse, 10, 4, 9.9));   // volume lost (F38 class)
    EXPECT_FALSE(check::volumesPlausible(BooleanKind::Fuse, 10, 4, 20.5));  // volume created
    // Cut: |R| <= |A|
    EXPECT_TRUE(check::volumesPlausible(BooleanKind::Cut, 10, 4, 6));
    EXPECT_TRUE(check::volumesPlausible(BooleanKind::Cut, 10, 4, 10));
    EXPECT_FALSE(check::volumesPlausible(BooleanKind::Cut, 10, 4, 10.5));
    // Common: |R| <= min(|A|,|B|)
    EXPECT_TRUE(check::volumesPlausible(BooleanKind::Common, 10, 4, 4));
    EXPECT_FALSE(check::volumesPlausible(BooleanKind::Common, 10, 4, 5));
    // Relative tolerance of 1e-5 of (|A|+|B|): a 1e-6 relative slip passes, a 1e-3 one fails.
    EXPECT_TRUE(check::volumesPlausible(BooleanKind::Fuse, 1000, 0, 1000 * (1 - 1e-6)));
    EXPECT_FALSE(check::volumesPlausible(BooleanKind::Fuse, 1000, 0, 1000 * (1 - 1e-3)));
    // Non-finite or negative volumes never pass.
    EXPECT_FALSE(check::volumesPlausible(BooleanKind::Fuse, 10, 10, std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(check::volumesPlausible(BooleanKind::Cut, 10, 4, -1));
}

TEST(KernelCheck, BoundsGrowthAllowance) {
    const Box3 in = makeBoxRange({0, 0, 0}, {10, 10, 10});
    EXPECT_TRUE(check::boundsWithin(in, makeBoxRange({-0.5, 0, 0}, {10.5, 10, 10.5}), 1.0));
    EXPECT_FALSE(check::boundsWithin(in, makeBoxRange({-0.5, 0, 0}, {10.5, 10, 10.5}), 0.1));
    // F39 class: a sliver face hundreds of mm away from a 10 mm body
    EXPECT_FALSE(check::boundsWithin(in, makeBoxRange({0, -77, -21}, {10, 45, 11}), 1.6));
    // Empty boxes never match
    EXPECT_FALSE(check::boundsWithin(Box3{}, in, 1.0));
    EXPECT_FALSE(check::boundsWithin(in, Box3{}, 1.0));
}

TEST(KernelCheck, SolidValidityRejectsNonSolids) {
    EXPECT_TRUE(check::isSolidValid(box(10, 10, 10)));
    EXPECT_FALSE(check::isSolidValid(Shape{}));
    auto face = makePolygonFace(std::vector<Vec3>{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}});
    ASSERT_TRUE(face.has_value());
    EXPECT_FALSE(check::isSolidValid(*face)); // a face is not a solid
}

TEST(KernelCheck, SolidValidityVerdictIsMemoisedConsistently) {
    // K2: the structural verdict is memoised on the shape. Repeated calls, and calls on a copy (which carries the memo),
    // must agree with the first, uncached verdict; a non-solid stays rejected on every call.
    const Shape b = box(10, 10, 10);
    EXPECT_TRUE(check::isSolidValid(b));
    EXPECT_TRUE(check::isSolidValid(b));
    const Shape copy = b;
    EXPECT_TRUE(check::isSolidValid(copy));
    auto face = makePolygonFace(std::vector<Vec3>{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}});
    ASSERT_TRUE(face.has_value());
    EXPECT_FALSE(check::isSolidValid(*face));
    EXPECT_FALSE(check::isSolidValid(*face));
}

TEST(KernelCheck, CoincidentAndTouchingOperandsKeepExactVolumes) {
    const Shape a = box(10, 10, 10);
    auto same = fuse(a, a);
    ASSERT_TRUE(same.has_value()) << same.error().message;
    EXPECT_NEAR(volume(same->shape), 1000, 1e-6);
    auto touching = fuse(a, box(10, 10, 10, {10, 0, 0}));
    ASSERT_TRUE(touching.has_value()) << touching.error().message;
    EXPECT_NEAR(volume(touching->shape), 2000, 1e-6);
    // Cutting a solid from itself leaves nothing: a clean failure, never a zero-volume solid.
    EXPECT_EQ(cut(a, a).error().code, ErrorCode::KernelFailure);
    // A 1e-9 mm overlap is a sliver-level case; the result must still satisfy the volume invariant.
    auto sliver = fuse(a, box(10, 10, 10, {10 - 1e-9, 0, 0}));
    ASSERT_TRUE(sliver.has_value()) << sliver.error().message;
    EXPECT_NEAR(volume(sliver->shape), 2000, 1e-3);
}

TEST(KernelOps, ChamferSetbackMustStayShorterThanAdjacentEdges) {
    // Thin plate 1 x 10 x 10: the 10 mm edge along y at x = 0, z = 0 has 1 mm neighbours along x.
    Shape plate = box(1, 10, 10);
    auto e = findSub(plate, SubShapeKind::Edge, {0, 5, 0, 10});
    ASSERT_TRUE(e.has_value());
    auto c = chamferEdges(plate, ids({*e}), 2, 2);
    ASSERT_FALSE(c.has_value());
    EXPECT_EQ(c.error().code, ErrorCode::KernelFailure);
    EXPECT_NE(c.error().message.find("reaches an adjacent edge"), std::string::npos) << c.error().message;
    // A setback inside the neighbours' length is fine.
    auto ok = chamferEdges(plate, ids({*e}), 0.5, 0.5);
    EXPECT_TRUE(ok.has_value()) << (ok ? "" : ok.error().message);
}

TEST(KernelOps, DegenerateEdgesAreRejectedBeforeOcct) {
    // A sphere's poles are degenerate (zero-length) edges of its parametrisation.
    auto sphere = makeSphere({0, 0, 0}, 5);
    ASSERT_TRUE(sphere.has_value());
    std::optional<std::uint32_t> degenerate;
    for (std::uint32_t i = 0; i < sphere->shape.subShapeCount(SubShapeKind::Edge); ++i)
        if (sphere->shape.subShapeKey(SubShapeKind::Edge, i)[3] < 1e-9) { degenerate = i; break; }
    ASSERT_TRUE(degenerate.has_value()) << "sphere has no zero-length edge to test with";
    auto f = filletEdges(sphere->shape, ids({*degenerate}), 0.5);
    ASSERT_FALSE(f.has_value());
    EXPECT_EQ(f.error().code, ErrorCode::KernelFailure);
    auto c = chamferEdges(sphere->shape, ids({*degenerate}), 0.5, 0.5);
    ASSERT_FALSE(c.has_value());
    EXPECT_EQ(c.error().code, ErrorCode::KernelFailure);
}
