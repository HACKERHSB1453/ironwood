// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <unordered_set>

#include "kernel/internal/profile_support.hpp"
#include "kernel/ops.hpp"
#include "naming/detail.hpp"
#include "naming/resolve.hpp"

using namespace iw;
using namespace iw::naming;

namespace {
ElementName name(std::uint64_t f, const char* op, std::uint32_t ord, std::vector<ElementName> src = {}) {
    ElementName n;
    n.feature = FeatureId{f};
    n.op = op;
    n.ordinal = ord;
    n.sources = std::move(src);
    return n;
}
} // namespace

TEST(Naming, NamesAreComparableAndHashable) {
    auto a = name(1, "extrude.side", 0, {name(0, "sketch.edge", 3)});
    auto b = a;
    EXPECT_EQ(a, b);
    EXPECT_EQ(a.hash(), b.hash());
    b.ordinal = 1;
    EXPECT_NE(a, b);
    EXPECT_TRUE((a <=> b) < 0);
    std::unordered_set<ElementName> set{a, b, a};
    EXPECT_EQ(set.size(), 2u);
}

TEST(Naming, NamingMapLookup) {
    NamingMap m;
    m.resize(2, 0, 0);
    ASSERT_TRUE(m.set(ElementType::Face, 1, name(1, "box.face", 1)));
    EXPECT_FALSE(m.set(ElementType::Face, 5, name(1, "box.face", 1)));
    EXPECT_EQ(m.indicesOf(name(1, "box.face", 1)), std::vector<std::uint32_t>{1});
    EXPECT_FALSE(m.nameOf(ElementType::Face, 0));
}

// ---- ElementName text form --------------------------------------------------------------------------

namespace {
ElementName randomName(std::mt19937_64& rng, int depth) {
    static const char* ops[] = {"box.face", "extrude.side", "fuse.split", "cut.merge", "sketch.edge", "a", "x_1.y2"};
    ElementName n;
    n.feature = FeatureId{rng() % 100000 + 1};
    n.op = ops[rng() % 7];
    n.type = static_cast<ElementType>(rng() % 3);
    n.ordinal = static_cast<std::uint32_t>(rng() % 50);
    if (depth < 4) {
        auto cnt = rng() % 4;
        for (std::uint64_t i = 0; i < cnt; ++i) n.sources.push_back(randomName(rng, depth + 1));
    }
    return n;
}
} // namespace

TEST(Naming, ToStringGrammar) {
    auto a = name(3, "extrude.side", 2, {name(0, "sketch.edge", 7)});
    EXPECT_EQ(a.toString(), "F3:extrude.side:F#2[F0:sketch.edge:F#7[]]");
    EXPECT_EQ(name(1, "box.face", 0).toString(), "F1:box.face:F#0[]");
}

TEST(Naming, ParseRoundTripRandom) {
    std::mt19937_64 rng(12345);
    for (int i = 0; i < 1000; ++i) {
        auto n = randomName(rng, 0);
        auto parsed = ElementName::parse(n.toString());
        ASSERT_TRUE(parsed) << n.toString() << ": " << parsed.error().message;
        EXPECT_EQ(*parsed, n);
        EXPECT_EQ(parsed->toString(), n.toString());
    }
}

TEST(Naming, ParseRejectsGarbage) {
    for (const char* bad : {"", "x", "F:op:F#0[]", "F1::F#0[]", "F1:op:Q#0[]", "F1:op:F0[]", "F1:op:F#[]",
                            "F1:op:F#0", "F1:op:F#0[", "F1:op:F#0[]x", "F1:op:F#0[F2:a:F#0[],]",
                            "F1:op:F#99999999999[]", "F1:Op:F#0[]", "F99999999999999999999:a:F#0[]"}) {
        auto r = ElementName::parse(bad);
        ASSERT_FALSE(r) << bad;
        EXPECT_EQ(r.error().code, ErrorCode::ParseError) << bad;
    }
    std::string deep;
    for (int i = 0; i < 1000; ++i) deep += "F1:a:F#0[";
    auto r = ElementName::parse(deep);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::ParseError);
}

// ---- deriveNamingMap (hand-built histories) ----------------------------------------------------------

namespace {
using kernel::InputSubShape;
using kernel::OutputSubShape;
using kernel::ShapeHistory;
using kernel::SubShapeKind;
constexpr auto kF = SubShapeKind::Face;
constexpr auto kE = SubShapeKind::Edge;

NamingMap primitiveMap(std::uint64_t feat, std::uint32_t faces, std::uint32_t edges = 0) {
    NamingMap m;
    m.resize(faces, edges, 0);
    for (std::uint32_t i = 0; i < faces; ++i) EXPECT_TRUE(m.set(ElementType::Face, i, name(feat, "box.face", i)));
    for (std::uint32_t i = 0; i < edges; ++i) {
        auto n = name(feat, "box.edge", i);
        n.type = ElementType::Edge;
        EXPECT_TRUE(m.set(ElementType::Edge, i, n));
    }
    return m;
}

Result<NamingMap> derive(FeatureId f, const char* op, std::array<std::uint32_t, 3> counts,
                         const std::vector<const NamingMap*>& in, const ShapeHistory& h,
                         const naming::detail::SiblingKeyFn& key = {}) {
    return naming::detail::deriveFromCounts(f, op, counts, in, h, key);
}

void expectAllNamedDistinct(const NamingMap& m, ElementType t) {
    std::set<ElementName> seen;
    for (std::uint32_t i = 0; i < m.size(t); ++i) {
        auto n = m.nameOf(t, i);
        ASSERT_TRUE(n) << "unnamed index " << i;
        EXPECT_TRUE(seen.insert(*n).second) << "duplicate " << n->toString();
    }
}
} // namespace

TEST(NamingDerive, ExtrudeRectangleGivesSixDistinctNames) {
    // Profile: 1 face (F0 of sketch feature 1) + 4 edges. Output: side faces 0..3, start cap 4, end cap 5.
    NamingMap prof;
    prof.resize(1, 4, 0);
    ASSERT_TRUE(prof.set(ElementType::Face, 0, name(1, "sketch.profile", 0)));
    for (std::uint32_t i = 0; i < 4; ++i) {
        auto n = name(1, "sketch.edge", i);
        n.type = ElementType::Edge;
        ASSERT_TRUE(prof.set(ElementType::Edge, i, n));
    }
    ShapeHistory h;
    for (std::uint32_t i = 0; i < 4; ++i) h.generated[{0, kE, i}] = {{kF, i}};
    h.modified[{0, kF, 0}] = {{kF, 4}};
    h.generated[{0, kF, 0}] = {{kF, 5}};
    auto r = derive(FeatureId{2}, "extrude", {6, 12, 8}, {&prof}, h);
    ASSERT_TRUE(r) << r.error().message;
    expectAllNamedDistinct(*r, ElementType::Face);
    expectAllNamedDistinct(*r, ElementType::Edge);
    expectAllNamedDistinct(*r, ElementType::Vertex);
    auto side2 = r->nameOf(ElementType::Face, 2);
    EXPECT_EQ(side2->op, "extrude.side");
    EXPECT_EQ(side2->feature, FeatureId{2});
    ASSERT_EQ(side2->sources.size(), 1u);
    EXPECT_EQ(side2->sources[0], *prof.nameOf(ElementType::Edge, 2));
    EXPECT_EQ(*r->nameOf(ElementType::Face, 4), *prof.nameOf(ElementType::Face, 0)); // start cap keeps name
    EXPECT_EQ(r->nameOf(ElementType::Face, 5)->op, "extrude.cap");
    EXPECT_EQ(r->nameOf(ElementType::Edge, 0)->op, "extrude.new");
}

TEST(NamingDerive, PrimitiveWithoutHistoryGetsNewNames) {
    auto r = derive(FeatureId{1}, "box", {6, 12, 8}, {}, {});
    ASSERT_TRUE(r);
    expectAllNamedDistinct(*r, ElementType::Face);
    EXPECT_EQ(r->nameOf(ElementType::Face, 3)->op, "box.new");
    EXPECT_EQ(r->nameOf(ElementType::Face, 3)->ordinal, 3u);
}

TEST(NamingDerive, FuseKeepsSurvivorsAndSplitsGetDistinctOrdinals) {
    auto a = primitiveMap(1, 6);
    auto b = primitiveMap(2, 6);
    ShapeHistory h;
    // A faces 0..4 untouched -> 0..4; A face 5 split into 5 and 6; B face 0 trimmed 1:1 -> 7; B face 1 deleted.
    for (std::uint32_t i = 0; i < 5; ++i) h.unchanged[{0, kF, i}] = {kF, i};
    h.modified[{0, kF, 5}] = {{kF, 5}, {kF, 6}};
    h.modified[{1, kF, 0}] = {{kF, 7}};
    h.deleted.insert({1, kF, 1});
    auto r = derive(FeatureId{3}, "fuse", {8, 0, 0}, {&a, &b}, h);
    ASSERT_TRUE(r) << r.error().message;
    expectAllNamedDistinct(*r, ElementType::Face);
    for (std::uint32_t i = 0; i < 5; ++i) EXPECT_EQ(*r->nameOf(ElementType::Face, i), *a.nameOf(ElementType::Face, i));
    EXPECT_EQ(*r->nameOf(ElementType::Face, 7), *b.nameOf(ElementType::Face, 0));
    auto s0 = *r->nameOf(ElementType::Face, 5);
    auto s1 = *r->nameOf(ElementType::Face, 6);
    EXPECT_EQ(s0.op, "fuse.split");
    EXPECT_EQ(s1.op, "fuse.split");
    EXPECT_EQ(s0.feature, FeatureId{3});
    EXPECT_NE(s0.ordinal, s1.ordinal);
    EXPECT_EQ(s0.sources, std::vector<ElementName>{*a.nameOf(ElementType::Face, 5)});
    EXPECT_EQ(s1.sources, s0.sources);
    // Output index 1 is unnamed-by-history? No: face index 1.. all named; index 2 of B (not in history) absent.
}

TEST(NamingDerive, SplitOrdinalsFollowGeometricKeyNotKernelOrder) {
    auto a = primitiveMap(1, 1);
    ShapeHistory h;
    h.modified[{0, kF, 0}] = {{kF, 0}, {kF, 1}, {kF, 2}};
    // Geometric key (e.g. centroid x): kernel order 0,1,2 corresponds to x = 30,10,20.
    const double x[3] = {30, 10, 20};
    auto key = [&](ElementType, std::uint32_t i) -> std::optional<std::array<double, 4>> {
        return std::array<double, 4>{x[i], 0, 0, 0};
    };
    auto r = derive(FeatureId{2}, "cut", {3, 0, 0}, {&a}, h, key);
    ASSERT_TRUE(r);
    EXPECT_EQ(r->nameOf(ElementType::Face, 1)->ordinal, 0u);
    EXPECT_EQ(r->nameOf(ElementType::Face, 2)->ordinal, 1u);
    EXPECT_EQ(r->nameOf(ElementType::Face, 0)->ordinal, 2u);
    // The same geometry delivered in a permuted kernel order yields the same name for the same piece.
    ShapeHistory h2;
    h2.modified[{0, kF, 0}] = {{kF, 2}, {kF, 0}, {kF, 1}};
    const double x2[3] = {10, 20, 30}; // piece with x=10 is now index 0, etc.
    auto key2 = [&](ElementType, std::uint32_t i) -> std::optional<std::array<double, 4>> {
        return std::array<double, 4>{x2[i], 0, 0, 0};
    };
    auto r2 = derive(FeatureId{2}, "cut", {3, 0, 0}, {&a}, h2, key2);
    ASSERT_TRUE(r2);
    EXPECT_EQ(*r2->nameOf(ElementType::Face, 0), *r->nameOf(ElementType::Face, 1)); // x=10
    EXPECT_EQ(*r2->nameOf(ElementType::Face, 1), *r->nameOf(ElementType::Face, 2)); // x=20
    EXPECT_EQ(*r2->nameOf(ElementType::Face, 2), *r->nameOf(ElementType::Face, 0)); // x=30
}

TEST(NamingDerive, MergeUsesSortedSources) {
    auto a = primitiveMap(1, 2);
    ShapeHistory h;
    h.modified[{0, kF, 1}] = {{kF, 0}};
    h.modified[{0, kF, 0}] = {{kF, 0}};
    auto r = derive(FeatureId{2}, "fuse", {1, 0, 0}, {&a}, h);
    ASSERT_TRUE(r);
    auto n = *r->nameOf(ElementType::Face, 0);
    EXPECT_EQ(n.op, "fuse.merge");
    ASSERT_EQ(n.sources.size(), 2u);
    EXPECT_TRUE(n.sources[0] < n.sources[1]);
}

TEST(NamingDerive, DeterministicAndInvalidInputs) {
    auto a = primitiveMap(1, 3);
    ShapeHistory h;
    h.modified[{0, kF, 0}] = {{kF, 0}, {kF, 1}};
    auto r1 = derive(FeatureId{2}, "cut", {3, 0, 0}, {&a}, h);
    auto r2 = derive(FeatureId{2}, "cut", {3, 0, 0}, {&a}, h);
    ASSERT_TRUE(r1 && r2);
    for (std::uint32_t i = 0; i < 3; ++i) EXPECT_EQ(r1->nameOf(ElementType::Face, i), r2->nameOf(ElementType::Face, i));

    ShapeHistory bad;
    bad.unchanged[{0, kF, 0}] = {kF, 99};
    EXPECT_EQ(derive(FeatureId{2}, "cut", {3, 0, 0}, {&a}, bad).error().code, ErrorCode::InvalidArgument);
    ShapeHistory bad2;
    bad2.unchanged[{5, kF, 0}] = {kF, 0};
    EXPECT_EQ(derive(FeatureId{2}, "cut", {3, 0, 0}, {&a}, bad2).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(derive(FeatureId{}, "cut", {3, 0, 0}, {&a}, {}).error().code, ErrorCode::InvalidArgument);
    // Duplicate input names kept into two outputs must not silently collide.
    NamingMap dup;
    dup.resize(2, 0, 0);
    ASSERT_TRUE(dup.set(ElementType::Face, 0, name(1, "box.face", 0)));
    ASSERT_TRUE(dup.set(ElementType::Face, 1, name(1, "box.face", 0)));
    ShapeHistory hd;
    hd.unchanged[{0, kF, 0}] = {kF, 0};
    hd.unchanged[{0, kF, 1}] = {kF, 1};
    EXPECT_EQ(derive(FeatureId{2}, "cut", {2, 0, 0}, {&dup}, hd).error().code, ErrorCode::Internal);
}

TEST(NamingDerive, PublicEntryRejectsNullShape) {
    ShapeHistory h;
    EXPECT_EQ(deriveNamingMap(FeatureId{1}, "box", kernel::Shape{}, {}, h).error().code, ErrorCode::InvalidArgument);
}

// ---- resolve ----------------------------------------------------------------------------------------

namespace {
ElementRef refTo(const ElementName& n) {
    ElementRef r;
    r.feature = FeatureId{1};
    r.type = n.type;
    r.name = n;
    return r;
}
} // namespace

TEST(NamingResolve, ExactAndNotFound) {
    auto m = primitiveMap(1, 3);
    auto r = resolve(refTo(name(1, "box.face", 2)), m);
    ASSERT_TRUE(r);
    EXPECT_EQ(r->index, 2u);
    EXPECT_EQ(r->quality, ResolveQuality::Exact);
    EXPECT_TRUE(r->warnings.empty());
    auto nf = resolve(refTo(name(1, "box.face", 9)), m);
    ASSERT_FALSE(nf);
    EXPECT_EQ(nf.error().code, ErrorCode::NotFound);
}

TEST(NamingResolve, CorruptDuplicateIsAmbiguous) {
    NamingMap m;
    m.resize(2, 0, 0);
    ASSERT_TRUE(m.set(ElementType::Face, 0, name(1, "box.face", 0)));
    ASSERT_TRUE(m.set(ElementType::Face, 1, name(1, "box.face", 0)));
    EXPECT_EQ(resolve(refTo(name(1, "box.face", 0)), m).error().code, ErrorCode::Ambiguous);
}

TEST(NamingResolve, TypeMismatchRejected) {
    auto m = primitiveMap(1, 1);
    auto ref = refTo(name(1, "box.face", 0));
    ref.type = ElementType::Edge;
    EXPECT_EQ(resolve(ref, m).error().code, ErrorCode::InvalidArgument);
}

TEST(NamingResolve, DeletedSourceIsNotFoundAndSplitIsAmbiguous) {
    // Pipeline: box face 5 is split by a cut; face 4 is deleted.
    auto a = primitiveMap(1, 6);
    ShapeHistory h;
    for (std::uint32_t i = 0; i < 4; ++i) h.unchanged[{0, kF, i}] = {kF, i};
    h.modified[{0, kF, 5}] = {{kF, 4}, {kF, 5}};
    h.deleted.insert({0, kF, 4});
    auto out = derive(FeatureId{2}, "cut", {6, 0, 0}, {&a}, h);
    ASSERT_TRUE(out);
    EXPECT_EQ(resolve(refTo(name(1, "box.face", 3)), *out)->quality, ResolveQuality::Exact);
    EXPECT_EQ(resolve(refTo(name(1, "box.face", 4)), *out).error().code, ErrorCode::NotFound);
    auto amb = resolve(refTo(name(1, "box.face", 5)), *out);
    ASSERT_FALSE(amb); // two successors: never pick
    EXPECT_EQ(amb.error().code, ErrorCode::Ambiguous);
}

TEST(NamingResolve, UniqueSuccessorIsHistoryDerivedWithWarning) {
    // Source face 1 was merged with face 0 into one output face; reference to face 1 rebinds with a warning.
    auto a = primitiveMap(1, 2);
    ShapeHistory h;
    h.modified[{0, kF, 0}] = {{kF, 0}};
    h.modified[{0, kF, 1}] = {{kF, 0}};
    auto out = derive(FeatureId{2}, "fuse", {1, 0, 0}, {&a}, h);
    ASSERT_TRUE(out);
    auto r = resolve(refTo(name(1, "box.face", 1)), *out);
    ASSERT_TRUE(r);
    EXPECT_EQ(r->index, 0u);
    EXPECT_EQ(r->quality, ResolveQuality::HistoryDerived);
    EXPECT_FALSE(r->warnings.empty());
}

TEST(NamingResolve, SplitSuccessorUniqueWhenOnlyOnePieceSurvives) {
    NamingMap m;
    m.resize(1, 0, 0);
    auto src = name(1, "box.face", 5);
    ASSERT_TRUE(m.set(ElementType::Face, 0, name(2, "cut.split", 1, {src})));
    auto r = resolve(refTo(src), m);
    ASSERT_TRUE(r);
    EXPECT_EQ(r->quality, ResolveQuality::HistoryDerived);
    // A generated element that merely lists the name as a source is NOT a successor.
    NamingMap g;
    g.resize(1, 0, 0);
    ASSERT_TRUE(g.set(ElementType::Face, 0, name(2, "extrude.side", 0, {src})));
    EXPECT_EQ(resolve(refTo(src), g).error().code, ErrorCode::NotFound);
}

// ---- integration with real kernel shapes (kernel ops produce the ShapeHistory) -----------------------

namespace {
using kernel::Shape;
using kernel::SubShapeKind;

Shape boxShape(Vec3 origin, double dx, double dy, double dz) {
    auto r = kernel::makeBox(origin, dx, dy, dz);
    EXPECT_TRUE(r) << r.error().message;
    return r ? r->shape : Shape{};
}

NamingMap boxNames(std::uint64_t feature, const Shape& s) {
    auto r = deriveNamingMap(FeatureId{feature}, "box", s, {}, {});
    EXPECT_TRUE(r) << r.error().message;
    return r ? std::move(*r) : NamingMap{};
}

/// Index of the face whose geometric key has centroid z == `z` and area == `area` (mm, mm^2).
std::uint32_t faceAt(const Shape& s, double z, double area) {
    for (std::uint32_t i = 0; i < s.subShapeCount(SubShapeKind::Face); ++i) {
        auto k = s.subShapeKey(SubShapeKind::Face, i);
        if (std::abs(k[2] - z) < 1e-6 && std::abs(k[3] - area) < 1e-6) return i;
    }
    ADD_FAILURE() << "no face with z=" << z << " area=" << area;
    return 0;
}

/// Name -> geometric key of every face of `op` in `m`; keys are read from the shape `m` describes.
std::map<ElementName, std::array<double, 4>> faceKeysOfOp(const NamingMap& m, const Shape& s,
                                                          const std::string& op) {
    std::map<ElementName, std::array<double, 4>> out;
    for (std::uint32_t i = 0; i < m.size(ElementType::Face); ++i) {
        auto n = m.nameOf(ElementType::Face, i);
        if (n && n->op == op) out[*n] = s.subShapeKey(SubShapeKind::Face, i);
    }
    return out;
}

void expectAllTypesDistinct(const NamingMap& m) {
    expectAllNamedDistinct(m, ElementType::Face);
    expectAllNamedDistinct(m, ElementType::Edge);
    expectAllNamedDistinct(m, ElementType::Vertex);
}
} // namespace

TEST(NamingIntegration, ExtrudeRectangleGivesSixDistinctNames) {
    const std::array<Vec3, 4> pts{Vec3{0, 0, 0}, Vec3{10, 0, 0}, Vec3{10, 4, 0}, Vec3{0, 4, 0}};
    auto profile = kernel::makePolygonFace(pts);
    ASSERT_TRUE(profile) << profile.error().message;
    auto profNames = deriveNamingMap(FeatureId{1}, "sketch", *profile, {}, {});
    ASSERT_TRUE(profNames) << profNames.error().message;
    auto ex = kernel::extrudeProfile(*profile, Vec3{0, 0, 5});
    ASSERT_TRUE(ex) << ex.error().message;
    ASSERT_EQ(ex->shape.subShapeCount(SubShapeKind::Face), 6u);
    auto m = deriveNamingMap(FeatureId{2}, "extrude", ex->shape, {&*profNames}, ex->history);
    ASSERT_TRUE(m) << m.error().message;
    ASSERT_EQ(m->size(ElementType::Face), 6u);
    expectAllTypesDistinct(*m);
    // Four side faces come from the four profile edges, each keeping its own source.
    std::set<ElementName> sides;
    for (std::uint32_t i = 0; i < 6; ++i) {
        auto n = m->nameOf(ElementType::Face, i);
        ASSERT_TRUE(n);
        if (n->op == "extrude.side") {
            EXPECT_EQ(n->sources.size(), 1u);
            sides.insert(n->sources.front());
        }
    }
    EXPECT_EQ(sides.size(), 4u);
}

TEST(NamingIntegration, FuseSplitOrdinalsFollowGeometryAndNotOperandOrder) {
    // A: 10 mm cube. B: a slab crossing A's top face along x, so A's top face is split into two pieces
    // (y in [0,4] and y in [6,10]) and the strip y in [4,6] is consumed.
    const Shape a = boxShape({0, 0, 0}, 10, 10, 10);
    const Shape b = boxShape({-5, 4, 5}, 20, 2, 10);
    const NamingMap na = boxNames(1, a);
    const NamingMap nb = boxNames(2, b);
    auto r1 = kernel::fuse(a, b);
    auto r2 = kernel::fuse(b, a); // same solid, operands swapped: different history indices and kernel order
    ASSERT_TRUE(r1) << r1.error().message;
    ASSERT_TRUE(r2) << r2.error().message;
    auto m1 = deriveNamingMap(FeatureId{3}, "fuse", r1->shape, {&na, &nb}, r1->history);
    auto m2 = deriveNamingMap(FeatureId{3}, "fuse", r2->shape, {&nb, &na}, r2->history);
    ASSERT_TRUE(m1) << m1.error().message;
    ASSERT_TRUE(m2) << m2.error().message;
    expectAllTypesDistinct(*m1);
    expectAllTypesDistinct(*m2);

    // Surviving face keeps its name.
    const auto bottom = *na.nameOf(ElementType::Face, faceAt(a, 0, 100));
    EXPECT_EQ(m1->indicesOf(bottom).size(), 1u);

    // A's top face gives two split pieces (B's bottom face also splits into two, x<0 and x>10). Among the
    // A-top pieces the ordinals are distinct and the lower ordinal has the smaller geometric key.
    auto k1 = faceKeysOfOp(*m1, r1->shape, "fuse.split");
    ASSERT_EQ(k1.size(), 4u);
    const auto topName = *na.nameOf(ElementType::Face, faceAt(a, 10, 100));
    std::map<std::uint32_t, std::array<double, 4>> byOrdinal;
    for (const auto& [n, key] : k1)
        if (n.sources == std::vector<ElementName>{topName}) byOrdinal[n.ordinal] = key;
    ASSERT_EQ(byOrdinal.size(), 2u);
    EXPECT_TRUE(byOrdinal.at(0) < byOrdinal.at(1)) << "ordinal 0 must have the smaller geometric key";
    EXPECT_EQ(byOrdinal.at(0)[1], 2.0); // piece y in [0,4] has centroid y = 2
    EXPECT_EQ(byOrdinal.at(1)[1], 8.0); // piece y in [6,10] has centroid y = 8

    // Building the same fuse with operands swapped yields the same names on the same geometry.
    EXPECT_EQ(faceKeysOfOp(*m2, r2->shape, "fuse.split"), k1);
}

TEST(NamingIntegration, SlotSplittingAFaceMakesOldReferenceAmbiguous) {
    const Shape a = boxShape({0, 0, 0}, 10, 10, 10);
    const Shape tool = boxShape({-5, 4, 5}, 20, 2, 10); // 2 mm slot, 5 mm deep, through the top in x
    const NamingMap na = boxNames(1, a);
    const NamingMap nt = boxNames(2, tool);
    auto cutResult = kernel::cut(a, tool);
    ASSERT_TRUE(cutResult) << cutResult.error().message;
    auto m = deriveNamingMap(FeatureId{3}, "cut", cutResult->shape, {&na, &nt}, cutResult->history);
    ASSERT_TRUE(m) << m.error().message;
    expectAllTypesDistinct(*m);

    // The top face is now two pieces: the old reference must not pick either of them.
    EXPECT_EQ(faceKeysOfOp(*m, cutResult->shape, "cut.split").size(), 2u);
    const auto topName = *na.nameOf(ElementType::Face, faceAt(a, 10, 100));
    auto amb = resolve(ElementRef{FeatureId{1}, ElementType::Face, topName}, *m);
    ASSERT_FALSE(amb);
    EXPECT_EQ(amb.error().code, ErrorCode::Ambiguous);

    // An untouched face still resolves exactly.
    const auto bottom = *na.nameOf(ElementType::Face, faceAt(a, 0, 100));
    auto ok = resolve(ElementRef{FeatureId{1}, ElementType::Face, bottom}, *m);
    ASSERT_TRUE(ok) << ok.error().message;
    EXPECT_EQ(ok->quality, ResolveQuality::Exact);
}
