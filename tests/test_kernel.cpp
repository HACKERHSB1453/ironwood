// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <vector>

#include "kernel/exchange.hpp"
#include "kernel/internal/profile_support.hpp"
#include "kernel/ops.hpp"

using namespace iw;
using namespace iw::kernel;
namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;

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
