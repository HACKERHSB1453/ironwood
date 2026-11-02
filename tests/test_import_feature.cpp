// SPDX-License-Identifier: GPL-3.0-or-later
// F31: ImportFeature (STEP solids as a base feature): one body per solid with world placement, history-free naming
// that is stable across re-reads, JSON round trip through the registry, clear errors for a missing / changed file,
// document save/load, and a downstream fillet that references an imported edge.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "features/document.hpp"
#include "features/edge_features.hpp"
#include "features/extrude_feature.hpp"
#include "features/import_feature.hpp"
#include "kernel/exchange.hpp"
#include "kernel/ops.hpp"
#include "naming/element_name.hpp"
#include "recompute/evaluator.hpp"

using namespace iw;
using namespace iw::features;
namespace fs = std::filesystem;

namespace {

fs::path tmpDir() {
    static const fs::path d = [] {
        auto p = fs::temp_directory_path() / "ironwood_test_import_feature";
        fs::create_directories(p);
        return p;
    }();
    return d;
}

double volumeOf(const kernel::Shape& s) {
    auto mp = kernel::massProperties(s);
    EXPECT_TRUE(mp.has_value());
    return mp ? mp->volume : 0.0;
}

/// Writes a STEP assembly: a 10x20x30 box at the origin (root) and a r=5 h=10 cylinder placed by a translation of
/// (100, 0, 0) on an assembly component, so the import must apply the placement.
fs::path writeTwoSolidStep(const std::string& name) {
    auto box = kernel::makeBox({0, 0, 0}, 10, 20, 30);
    auto cyl = kernel::makeCylinder({0, 0, 0}, {0, 0, 1}, 5, 10);
    EXPECT_TRUE(box.has_value() && cyl.has_value());
    kernel::ImportedAssembly a;
    kernel::AssemblyNode root;
    root.name = "asm";
    root.children = {1, 2};
    kernel::AssemblyNode b;
    b.name = "block";
    b.shape = box->shape;
    b.parent = 0;
    kernel::AssemblyNode c;
    c.name = "pin";
    c.shape = cyl->shape;
    c.parent = 0;
    c.local.m[3] = 100;
    a.nodes = {root, b, c};
    a.roots = {0};
    a.lengthUnitInFile = "mm";
    const fs::path p = tmpDir() / name;
    EXPECT_TRUE(kernel::exportStep(p, a).has_value());
    return p;
}

fs::path writeBoxStep(const std::string& name, double dx) {
    auto box = kernel::makeBox({0, 0, 0}, dx, 20, 30);
    EXPECT_TRUE(box.has_value());
    const fs::path p = tmpDir() / name;
    EXPECT_TRUE(kernel::exportStep(p, box->shape).has_value());
    return p;
}

recompute::FeatureStatus statusOf(const DocumentEvaluation& ev, FeatureId id) {
    for (const auto& s : ev.statuses)
        if (s.id == id) return s;
    ADD_FAILURE() << "no status for feature " << id.value();
    return {};
}

std::vector<std::string> allNames(const naming::NamingMap& m) {
    std::vector<std::string> out;
    for (auto t : {naming::ElementType::Face, naming::ElementType::Edge, naming::ElementType::Vertex})
        for (std::uint32_t i = 0; i < m.size(t); ++i) {
            auto n = m.nameOf(t, i);
            out.push_back(n ? n->toString() : std::string("<unnamed>"));
        }
    return out;
}

std::shared_ptr<ImportFeature> importOf(Document& doc, const fs::path& p) {
    auto params = makeImportParams(p, [&] { return doc.allocateBodyId(); });
    EXPECT_TRUE(params.has_value()) << (params ? "" : params.error().message);
    return std::make_shared<ImportFeature>(doc.allocateFeatureId(), *params);
}

} // namespace

TEST(ImportFeature, ReadStepSolidsAppliesAssemblyPlacement) {
    const fs::path p = writeTwoSolidStep("placement.step");
    auto solids = readStepSolids(p);
    ASSERT_TRUE(solids.has_value()) << solids.error().message;
    ASSERT_EQ(solids->size(), 2u);
    EXPECT_NEAR(volumeOf((*solids)[0].shape), 6000.0, 1e-6);
    const double cylVol = std::numbers::pi * 25 * 10;
    EXPECT_NEAR(volumeOf((*solids)[1].shape), cylVol, 1e-6);
    const Box3 bb = (*solids)[1].shape.boundingBox();
    EXPECT_NEAR(bb.min.x, 95, 1e-6);
    EXPECT_NEAR(bb.max.x, 105, 1e-6);
    EXPECT_EQ((*solids)[1].name, "pin");
}

TEST(ImportFeature, EachSolidBecomesANamedBody) {
    Document doc;
    const fs::path p = writeTwoSolidStep("two.step");
    auto f = importOf(doc, p);
    ASSERT_EQ(f->params().bodies.size(), 2u);
    EXPECT_EQ(f->params().contentHash.size(), 16u);
    ASSERT_TRUE(doc.addFeature(f).has_value());
    EXPECT_EQ(doc.timeline().at(0)->name(), "Import 1");
    auto ev = doc.evaluate();
    ASSERT_TRUE(ev.has_value());
    const auto st = statusOf(*ev, f->id());
    ASSERT_EQ(st.state, recompute::FeatureStatus::State::Ok) << st.error.message;
    EXPECT_TRUE(st.diagnostics.empty());
    EXPECT_EQ(st.attributes.at("solids"), "2");
    ASSERT_EQ(ev->bodies.size(), 2u);
    const BodyId b0 = f->params().bodies[0], b1 = f->params().bodies[1];
    EXPECT_NEAR(volumeOf(ev->bodies.at(b0)), 6000.0, 1e-6);
    EXPECT_NEAR(ev->bodies.at(b1).boundingBox().min.x, 95, 1e-6);

    // Every element is named, names are unique inside a body and distinct between the two bodies.
    std::set<std::string> seen0, seen1;
    for (const auto& n : allNames(ev->namings.at(b0))) {
        EXPECT_NE(n, "<unnamed>");
        EXPECT_TRUE(seen0.insert(n).second) << "duplicate " << n;
        EXPECT_NE(n.find(":import.solid0.new:"), std::string::npos) << n;
    }
    for (const auto& n : allNames(ev->namings.at(b1))) {
        EXPECT_TRUE(seen1.insert(n).second) << "duplicate " << n;
        EXPECT_EQ(seen0.count(n), 0u);
    }
    EXPECT_EQ(seen0.size(), 6u + 12u + 8u);
}

TEST(ImportFeature, NamesAreStableAcrossReReadsAndIndependentOfPlacementInTheFile) {
    const fs::path p = writeTwoSolidStep("stable.step");
    std::vector<std::string> first;
    for (int run = 0; run < 3; ++run) {
        Document doc; // fresh evaluator each time: the file is really re-read
        auto f = importOf(doc, p);
        ASSERT_TRUE(doc.addFeature(f).has_value());
        auto ev = doc.evaluate();
        ASSERT_TRUE(ev.has_value());
        auto names = allNames(ev->namings.at(f->params().bodies[0]));
        if (run == 0) first = names;
        else EXPECT_EQ(names, first);
    }
}

TEST(ImportFeature, JsonRoundTripThroughTheRegistry) {
    recompute::FeatureRegistry reg;
    registerFeatures(reg);
    ImportParams p{"/some/where/part.step", {BodyId{7}, BodyId{9}}, "0123456789abcdef"};
    ImportFeature f(FeatureId{5}, p);
    auto back = reg.create("import", FeatureId{5}, f.paramsJson());
    ASSERT_TRUE(back.has_value()) << back.error().message;
    EXPECT_EQ((*back)->kind(), "import");
    EXPECT_EQ((*back)->paramsJson(), f.paramsJson());
    EXPECT_EQ((*back)->paramHash(), f.paramHash());
    EXPECT_EQ((*back)->writesBodies(), (std::vector<BodyId>{BodyId{7}, BodyId{9}}));
    // The hash covers path, bodies and content hash.
    ImportParams q = p;
    q.path = "/other.step";
    EXPECT_NE(ImportFeature(FeatureId{5}, q).paramHash(), f.paramHash());
    q = p;
    q.contentHash = "fedcba9876543210";
    EXPECT_NE(ImportFeature(FeatureId{5}, q).paramHash(), f.paramHash());

    for (const char* bad : {"[]", "{}", R"({"path": "", "bodies": [1]})", R"({"path": "a.step", "bodies": []})",
                            R"({"path": "a.step", "bodies": [0]})", R"({"path": "a.step", "bodies": [1, 1]})",
                            R"({"path": "a.step", "bodies": [-2]})", R"({"path": "a.step", "bodies": [1], "content_hash": 3})"}) {
        auto r = reg.create("import", FeatureId{1}, bad);
        ASSERT_FALSE(r.has_value()) << bad;
        EXPECT_EQ(r.error().code, ErrorCode::ParseError) << bad;
    }
    // content_hash is optional.
    EXPECT_TRUE(reg.create("import", FeatureId{1}, R"({"path": "a.step", "bodies": [3]})").has_value());
}

TEST(ImportFeature, MissingFileFailsWithAClearError) {
    Document doc;
    const fs::path p = writeBoxStep("gone.step", 10);
    auto f = importOf(doc, p);
    ASSERT_TRUE(doc.addFeature(f).has_value());
    fs::remove(p);
    auto ev = doc.evaluate();
    ASSERT_TRUE(ev.has_value());
    const auto st = statusOf(*ev, f->id());
    ASSERT_EQ(st.state, recompute::FeatureStatus::State::Failed);
    EXPECT_EQ(st.error.code, ErrorCode::IoError);
    EXPECT_NE(st.error.message.find("gone.step"), std::string::npos) << st.error.message;
    EXPECT_NE(st.error.message.find("missing"), std::string::npos) << st.error.message;
    EXPECT_TRUE(ev->bodies.empty());

    // Creating an import from a missing file fails up front.
    auto params = makeImportParams(tmpDir() / "never.step", [&] { return doc.allocateBodyId(); });
    ASSERT_FALSE(params.has_value());
    EXPECT_EQ(params.error().code, ErrorCode::IoError);
}

TEST(ImportFeature, ChangedFileWarnsAndSolidCountMismatchIsReported) {
    const fs::path p = writeBoxStep("changing.step", 10);
    {
        Document doc;
        auto f = importOf(doc, p);
        ASSERT_TRUE(doc.addFeature(f).has_value());
        writeBoxStep("changing.step", 12); // same solid count, different content
        auto ev = doc.evaluate();
        ASSERT_TRUE(ev.has_value());
        const auto st = statusOf(*ev, f->id());
        ASSERT_EQ(st.state, recompute::FeatureStatus::State::Ok) << st.error.message;
        ASSERT_EQ(st.diagnostics.size(), 1u);
        EXPECT_EQ(st.diagnostics[0].severity, recompute::Diagnostic::Severity::Warning);
        EXPECT_NE(st.diagnostics[0].message.find("changed"), std::string::npos);
        EXPECT_NEAR(volumeOf(ev->bodies.at(f->params().bodies[0])), 12 * 20 * 30, 1e-6);
    }
    {
        // The import expects two solids; the file now has one.
        Document doc;
        ImportParams q{p.string(), {doc.allocateBodyId(), doc.allocateBodyId()}, ""};
        auto f = std::make_shared<ImportFeature>(doc.allocateFeatureId(), q);
        ASSERT_TRUE(doc.addFeature(f).has_value());
        auto ev = doc.evaluate();
        ASSERT_TRUE(ev.has_value());
        const auto st = statusOf(*ev, f->id());
        ASSERT_EQ(st.state, recompute::FeatureStatus::State::Failed);
        EXPECT_EQ(st.error.code, ErrorCode::NotFound);
        EXPECT_NE(st.error.message.find("1 solid"), std::string::npos) << st.error.message;
    }
    {
        // More solids than bodies: the first ones are imported with a warning.
        const fs::path two = writeTwoSolidStep("more.step");
        Document doc;
        ImportParams q{two.string(), {doc.allocateBodyId()}, ""};
        auto f = std::make_shared<ImportFeature>(doc.allocateFeatureId(), q);
        ASSERT_TRUE(doc.addFeature(f).has_value());
        auto ev = doc.evaluate();
        ASSERT_TRUE(ev.has_value());
        const auto st = statusOf(*ev, f->id());
        ASSERT_EQ(st.state, recompute::FeatureStatus::State::Ok) << st.error.message;
        ASSERT_EQ(st.diagnostics.size(), 1u);
        EXPECT_EQ(ev->bodies.size(), 1u);
    }
}

TEST(ImportFeature, DocumentSaveLoadAndDownstreamFilletOnAnImportedEdge) {
    const fs::path p = writeBoxStep("filleted.step", 10);
    const fs::path dir = tmpDir() / "doc_with_import";
    fs::remove_all(dir);
    double filleted = 0;
    {
        Document doc;
        auto f = importOf(doc, p);
        ASSERT_TRUE(doc.addFeature(f).has_value());
        const BodyId body = f->params().bodies[0];
        auto ev = doc.evaluate();
        ASSERT_TRUE(ev.has_value());
        // Pick an edge by type through the kernel query: the first line edge parallel to Z.
        const kernel::Shape& s = ev->bodies.at(body);
        std::optional<naming::ElementName> edge;
        for (std::uint32_t i = 0; i < s.subShapeCount(kernel::SubShapeKind::Edge) && !edge; ++i) {
            const auto g = s.edgeGeometry(i);
            if (g.kind == kernel::EdgeGeometry::Kind::Line && std::abs(g.direction.z) > 0.999)
                edge = ev->namings.at(body).nameOf(naming::ElementType::Edge, i);
        }
        ASSERT_TRUE(edge.has_value());
        FilletParams fp;
        fp.body = body;
        fp.edges = {naming::ElementRef{edge->feature, naming::ElementType::Edge, *edge}};
        fp.radius = "2 mm";
        ASSERT_TRUE(doc.addFeature(std::make_shared<FilletFeature>(doc.allocateFeatureId(), fp)).has_value());
        auto ev2 = doc.evaluate();
        ASSERT_TRUE(ev2.has_value());
        for (const auto& st : ev2->statuses) ASSERT_EQ(st.state, recompute::FeatureStatus::State::Ok) << st.error.message;
        filleted = volumeOf(ev2->bodies.at(body));
        EXPECT_LT(filleted, 6000.0);
        ASSERT_TRUE(doc.save(dir).has_value());
    }
    Document loaded;
    auto r = loaded.load(dir);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    ASSERT_EQ(loaded.timeline().size(), 2u);
    EXPECT_EQ(loaded.timeline().at(0)->kind(), "import");
    auto ev = loaded.evaluate();
    ASSERT_TRUE(ev.has_value());
    for (const auto& st : ev->statuses) ASSERT_EQ(st.state, recompute::FeatureStatus::State::Ok) << st.error.message;
    ASSERT_EQ(ev->bodies.size(), 1u);
    EXPECT_NEAR(volumeOf(ev->bodies.begin()->second), filleted, 1e-6);
}
