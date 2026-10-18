// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <filesystem>
#include <functional>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "core/expr.hpp"
#include "core/params.hpp"
#include "io/document_io.hpp"
#include "kernel/ops.hpp"
#include "naming/naming_map.hpp"
#include "recompute/evaluator.hpp"

namespace {

namespace fs = std::filesystem;
using iw::ErrorCode;
using iw::FeatureId;
using iw::recompute::EvalContext;
using iw::recompute::Feature;
using iw::recompute::FeatureRegistry;
using iw::recompute::FeatureResult;

/// Test-only feature: parameters are stored verbatim (canonical compact JSON, sorted keys).
class TestFeature final : public Feature {
public:
    TestFeature(FeatureId id, std::string kind, std::string params)
        : id_(id), kind_(std::move(kind)), params_(std::move(params)) {}
    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return kind_; }
    std::vector<iw::BodyId> writesBodies() const override { return {}; }
    std::uint64_t paramHash() const override { return std::hash<std::string>{}(params_); }
    std::string paramsJson() const override { return params_; }
    iw::Result<FeatureResult> compute(const EvalContext&) const override { return FeatureResult{}; }

private:
    FeatureId id_;
    std::string kind_;
    std::string params_;
};

std::shared_ptr<TestFeature> makeFeature(std::uint64_t id, const std::string& kind, const std::string& params) {
    return std::make_shared<TestFeature>(FeatureId(id), kind, params);
}

FeatureRegistry makeRegistry() {
    FeatureRegistry reg;
    auto factory = [](std::string_view kind) {
        return [k = std::string(kind)](FeatureId id, std::string_view params)
                   -> iw::Result<std::shared_ptr<Feature>> {
            return std::make_shared<TestFeature>(id, k, std::string(params));
        };
    };
    reg.add("test.box", factory("test.box"));
    reg.add("test.hole", factory("test.hole"));
    return reg;
}

/// Three features, feature 2 suppressed, rollback marker at 2 (feature 3 is beyond it).
iw::io::Document makeDocument() {
    iw::io::Document doc;
    doc.name = "Part1";
    doc.displayUnit = iw::LengthUnit::Inch;
    doc.nextId = 10;
    doc.timeline.append(makeFeature(1, "test.box", R"({"h":3,"w":10})"));
    doc.timeline.append(makeFeature(2, "test.hole", R"({"d":2.5,"name":"a\"b"})"));
    doc.timeline.append(makeFeature(3, "test.box", R"({"h":1,"w":1})"));
    doc.timeline.setSuppressed(FeatureId(2), true);
    doc.timeline.setRollbackMarker(2);
    return doc;
}

std::string readFile(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void writeFile(const fs::path& p, const std::string& text) {
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    out << text;
}

class IoTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::random_device rd;
        root_ = fs::temp_directory_path() / ("iw-io-test-" + std::to_string(rd()));
        fs::create_directories(root_);
    }
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(root_, ec);
    }
    fs::path doc() const { return root_ / "mydoc.ironwood"; }
    fs::path manifest() const { return doc() / "manifest.json"; }

    fs::path root_;
};

} // namespace

TEST_F(IoTest, RoundTripPreservesTimelineAndMetadata) {
    const iw::io::Document original = makeDocument();
    ASSERT_TRUE(iw::io::saveDocument(doc(), original).has_value());

    auto loaded = iw::io::loadDocument(doc(), makeRegistry());
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    const iw::io::Document& d = *loaded;

    EXPECT_EQ(d.name, "Part1");
    EXPECT_EQ(d.displayUnit, iw::LengthUnit::Inch);
    EXPECT_EQ(d.nextId, 10u);
    ASSERT_EQ(d.timeline.size(), 3u);
    EXPECT_EQ(d.timeline.at(0)->id(), FeatureId(1));
    EXPECT_EQ(d.timeline.at(1)->id(), FeatureId(2));
    EXPECT_EQ(d.timeline.at(2)->id(), FeatureId(3));
    EXPECT_EQ(d.timeline.at(0)->kind(), "test.box");
    EXPECT_EQ(d.timeline.at(1)->kind(), "test.hole");
    EXPECT_EQ(d.timeline.at(2)->kind(), "test.box");
    EXPECT_TRUE(d.timeline.isSuppressed(FeatureId(2)));
    EXPECT_FALSE(d.timeline.isSuppressed(FeatureId(1)));
    EXPECT_FALSE(d.timeline.isSuppressed(FeatureId(3)));
    EXPECT_EQ(d.timeline.rollbackMarker(), 2u);
    // Params come back as canonical compact JSON (sorted keys); escapes survive.
    EXPECT_EQ(d.timeline.at(0)->paramsJson(), R"({"h":3,"w":10})");
    EXPECT_EQ(d.timeline.at(1)->paramsJson(), R"({"d":2.5,"name":"a\"b"})");
}

TEST_F(IoTest, RoundTripWithoutRollbackMarkerAndEmptyTimeline) {
    iw::io::Document empty;
    empty.name = "";
    empty.nextId = 1;
    ASSERT_TRUE(iw::io::saveDocument(doc(), empty).has_value());
    auto loaded = iw::io::loadDocument(doc(), makeRegistry());
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    EXPECT_EQ(loaded->timeline.size(), 0u);
    EXPECT_EQ(loaded->timeline.rollbackMarker(), 0u);
    EXPECT_EQ(loaded->displayUnit, iw::LengthUnit::Millimeter);
    EXPECT_EQ(loaded->nextId, 1u);
}

TEST_F(IoTest, UnknownKindIsNotFound) {
    ASSERT_TRUE(iw::io::saveDocument(doc(), makeDocument()).has_value());
    FeatureRegistry reg;
    reg.add("test.box", [](FeatureId, std::string_view) -> iw::Result<std::shared_ptr<Feature>> {
        return std::make_shared<TestFeature>(FeatureId(1), "test.box", "{}");
    });
    auto loaded = iw::io::loadDocument(doc(), reg);
    ASSERT_FALSE(loaded.has_value());
    EXPECT_EQ(loaded.error().code, ErrorCode::NotFound);
}

TEST_F(IoTest, VersionAboveSupportedIsParseError) {
    ASSERT_TRUE(iw::io::saveDocument(doc(), makeDocument()).has_value());
    std::string text = readFile(manifest());
    const std::string current = "\"version\": " + std::to_string(iw::io::kDocumentFormatVersion);
    const auto pos = text.find(current);
    ASSERT_NE(pos, std::string::npos);
    text.replace(pos, current.size(), "\"version\": 99");
    writeFile(manifest(), text);
    auto loaded = iw::io::loadDocument(doc(), makeRegistry());
    ASSERT_FALSE(loaded.has_value());
    EXPECT_EQ(loaded.error().code, ErrorCode::ParseError);
}

TEST_F(IoTest, WrongFormatTagIsParseError) {
    ASSERT_TRUE(iw::io::saveDocument(doc(), makeDocument()).has_value());
    std::string text = readFile(manifest());
    const auto pos = text.find("ironwood-document");
    ASSERT_NE(pos, std::string::npos);
    text.replace(pos, std::string("ironwood-document").size(), "something-else");
    writeFile(manifest(), text);
    auto loaded = iw::io::loadDocument(doc(), makeRegistry());
    ASSERT_FALSE(loaded.has_value());
    EXPECT_EQ(loaded.error().code, ErrorCode::ParseError);
}

TEST_F(IoTest, TruncatedManifestIsParseError) {
    ASSERT_TRUE(iw::io::saveDocument(doc(), makeDocument()).has_value());
    const std::string text = readFile(manifest());
    for (std::size_t cut : {text.size() / 2, text.size() - 3, std::size_t{1}}) {
        writeFile(manifest(), text.substr(0, cut));
        auto loaded = iw::io::loadDocument(doc(), makeRegistry());
        ASSERT_FALSE(loaded.has_value()) << "cut at " << cut;
        EXPECT_EQ(loaded.error().code, ErrorCode::ParseError) << "cut at " << cut;
    }
}

TEST_F(IoTest, GarbageManifestIsParseError) {
    fs::create_directories(doc());
    writeFile(manifest(), "this is not json at all {{{");
    auto loaded = iw::io::loadDocument(doc(), makeRegistry());
    ASSERT_FALSE(loaded.has_value());
    EXPECT_EQ(loaded.error().code, ErrorCode::ParseError);
}

TEST_F(IoTest, MissingManifestIsParseErrorAndMissingDocumentIsNotFound) {
    fs::create_directories(doc());
    auto noManifest = iw::io::loadDocument(doc(), makeRegistry());
    ASSERT_FALSE(noManifest.has_value());
    EXPECT_EQ(noManifest.error().code, ErrorCode::ParseError);

    auto absent = iw::io::loadDocument(root_ / "nope.ironwood", makeRegistry());
    ASSERT_FALSE(absent.has_value());
    EXPECT_EQ(absent.error().code, ErrorCode::NotFound);
}

TEST_F(IoTest, RollbackMarkerOutOfRangeIsParseError) {
    ASSERT_TRUE(iw::io::saveDocument(doc(), makeDocument()).has_value());
    std::string text = readFile(manifest());
    const auto pos = text.find("\"rollbackMarker\": 2");
    ASSERT_NE(pos, std::string::npos);
    text.replace(pos, std::string("\"rollbackMarker\": 2").size(), "\"rollbackMarker\": 7");
    writeFile(manifest(), text);
    auto loaded = iw::io::loadDocument(doc(), makeRegistry());
    ASSERT_FALSE(loaded.has_value());
    EXPECT_EQ(loaded.error().code, ErrorCode::ParseError);
}

TEST_F(IoTest, FailedSaveDoesNotClobberExistingTarget) {
    ASSERT_TRUE(iw::io::saveDocument(doc(), makeDocument()).has_value());
    const std::string before = readFile(manifest());

    // A document whose feature params are not a JSON object is rejected before anything is written.
    iw::io::Document bad = makeDocument();
    bad.timeline.append(makeFeature(4, "test.box", "not json"));
    auto result = iw::io::saveDocument(doc(), bad);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::InvalidArgument);

    // nextId not greater than the feature ids is rejected the same way.
    iw::io::Document badId = makeDocument();
    badId.nextId = 3;
    EXPECT_EQ(iw::io::saveDocument(doc(), badId).error().code, ErrorCode::InvalidArgument);

    EXPECT_EQ(readFile(manifest()), before);
    // No staging leftovers next to the target.
    for (const auto& entry : fs::directory_iterator(root_)) {
        EXPECT_EQ(entry.path().filename().string(), "mydoc.ironwood") << entry.path();
    }
}

TEST_F(IoTest, SaveOverwritesPreviousVersionCleanly) {
    ASSERT_TRUE(iw::io::saveDocument(doc(), makeDocument()).has_value());
    iw::io::Document second = makeDocument();
    second.name = "Renamed";
    second.nextId = 42;
    ASSERT_TRUE(iw::io::saveDocument(doc(), second).has_value());

    auto loaded = iw::io::loadDocument(doc(), makeRegistry());
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    EXPECT_EQ(loaded->name, "Renamed");
    EXPECT_EQ(loaded->nextId, 42u);
    for (const auto& entry : fs::directory_iterator(root_)) {
        EXPECT_EQ(entry.path().filename().string(), "mydoc.ironwood") << entry.path();
    }
}

TEST_F(IoTest, SaveRejectsRegularFileAsTarget) {
    writeFile(doc(), "occupied");
    auto result = iw::io::saveDocument(doc(), makeDocument());
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(readFile(doc()), "occupied");
}

// ---- parameter table persistence (format v1) and stamps after reload ------------------------------------------

namespace {

/// Test feature: a box whose width is an expression, evaluated against the parameter table at compute time.
class ParamBoxFeature final : public Feature {
public:
    ParamBoxFeature(FeatureId id, std::string width) : id_(id), width_(std::move(width)) {}
    FeatureId id() const override { return id_; }
    std::string_view kind() const override { return "test.parambox"; }
    std::vector<iw::BodyId> writesBodies() const override { return {iw::BodyId{1}}; }
    std::vector<std::string> parameterReferences() const override {
        auto ex = iw::Expression::parse(width_);
        return ex ? ex->references() : std::vector<std::string>{};
    }
    std::uint64_t paramHash() const override { return std::hash<std::string>{}(width_); }
    std::string paramsJson() const override { return "{\"width\":\"" + width_ + "\"}"; }
    iw::Result<FeatureResult> compute(const EvalContext& ctx) const override {
        auto ex = iw::Expression::parse(width_);
        if (!ex) return std::unexpected(ex.error());
        auto q = ex->evaluate([&ctx](std::string_view name) -> iw::Result<iw::Quantity> {
            if (!ctx.params) return iw::makeError(ErrorCode::NotFound, "no parameter table");
            return ctx.params->evaluate(name);
        });
        if (!q) return std::unexpected(q.error());
        auto box = iw::kernel::makeBox({0, 0, 0}, q->value, 2, 2);
        if (!box) return std::unexpected(box.error());
        FeatureResult out;
        iw::naming::NamingMap map;
        map.resize(box->shape.subShapeCount(iw::kernel::SubShapeKind::Face),
                   box->shape.subShapeCount(iw::kernel::SubShapeKind::Edge),
                   box->shape.subShapeCount(iw::kernel::SubShapeKind::Vertex));
        out.bodies.emplace(iw::BodyId{1}, std::move(box->shape));
        out.namings.emplace(iw::BodyId{1}, std::move(map));
        return out;
    }

private:
    FeatureId id_;
    std::string width_;
};

FeatureRegistry paramRegistry() {
    FeatureRegistry reg;
    reg.add("test.parambox", [](FeatureId id, std::string_view params) -> iw::Result<std::shared_ptr<Feature>> {
        // Params are {"width":"<expr>"} with no escapes in the expressions used by these tests.
        const std::string key = "\"width\":\"";
        const auto at = std::string(params).find(key);
        if (at == std::string::npos) return iw::makeError(ErrorCode::ParseError, "parambox: bad params");
        const std::string text(params);
        const auto start = at + key.size();
        const auto end = text.find('"', start);
        if (end == std::string::npos) return iw::makeError(ErrorCode::ParseError, "parambox: bad params");
        return std::make_shared<ParamBoxFeature>(id, text.substr(start, end - start));
    });
    return reg;
}

/// Stamp of body 1 for each feature-body pair (what the evaluator hands out).
std::uint64_t bodyStamp(const iw::recompute::EvalResult& r) { return r.bodies.at(iw::BodyId{1}).stamp().value; }

} // namespace

TEST_F(IoTest, ParametersRoundTripWithAllFields) {
    iw::io::Document d;
    d.name = "params";
    d.nextId = 10;
    ASSERT_TRUE(d.parameters.add("d1", "2 in + 5 mm", "mm", "wall thickness"));
    ASSERT_TRUE(d.parameters.add("d2", "d1 * 2", "", ""));
    ASSERT_TRUE(d.parameters.add("ang", "30 deg", "deg", "tilt"));
    ASSERT_TRUE(d.parameters.setFavourite("d1", true));
    ASSERT_TRUE(iw::io::saveDocument(doc(), d).has_value());

    auto loaded = iw::io::loadDocument(doc(), paramRegistry());
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    const auto& ps = loaded->parameters.parameters();
    ASSERT_EQ(ps.size(), 3u);
    EXPECT_EQ(ps[0].name, "d1");
    EXPECT_EQ(ps[0].expression.source(), "2 in + 5 mm");
    EXPECT_EQ(ps[0].unit, "mm");
    EXPECT_EQ(ps[0].comment, "wall thickness");
    EXPECT_TRUE(ps[0].favourite);
    EXPECT_EQ(ps[1].name, "d2");
    EXPECT_EQ(ps[1].expression.source(), "d1 * 2");
    EXPECT_FALSE(ps[1].favourite);
    EXPECT_EQ(ps[2].unit, "deg");
    auto d2 = loaded->parameters.evaluate("d2");
    ASSERT_TRUE(d2.has_value());
    EXPECT_NEAR(d2->value, 2 * 55.8, 1e-9);
}

TEST_F(IoTest, StampsAreIdenticalAfterReload) {
    iw::io::Document d;
    d.name = "stamps";
    d.nextId = 10;
    ASSERT_TRUE(d.parameters.add("w", "12 mm"));
    ASSERT_TRUE(d.parameters.add("unused", "1 mm"));
    d.timeline.append(std::make_shared<ParamBoxFeature>(FeatureId{1}, "w"));
    ASSERT_TRUE(iw::io::saveDocument(doc(), d).has_value());

    auto loaded = iw::io::loadDocument(doc(), paramRegistry());
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;

    iw::recompute::Evaluator before, after;
    auto a = before.evaluate(d.timeline, &d.parameters);
    auto b = after.evaluate(loaded->timeline, &loaded->parameters);
    ASSERT_TRUE(a && b);
    ASSERT_EQ(a->statuses[0].state, iw::recompute::FeatureStatus::State::Ok) << a->statuses[0].error.message;
    EXPECT_EQ(bodyStamp(*a), bodyStamp(*b));
    EXPECT_EQ(d.parameters.hashOfUsed(std::vector<std::string>{"w"}),
              loaded->parameters.hashOfUsed(std::vector<std::string>{"w"}));

    // Editing the used parameter after reload recomputes; the unused one does not.
    auto edited = loaded->parameters;
    ASSERT_TRUE(edited.setExpression("unused", "2 mm"));
    auto same = after.evaluate(loaded->timeline, &edited);
    ASSERT_TRUE(same.has_value());
    EXPECT_EQ(same->computedCount, 0u);
    ASSERT_TRUE(edited.setExpression("w", "20 mm"));
    auto changed = after.evaluate(loaded->timeline, &edited);
    ASSERT_TRUE(changed.has_value());
    EXPECT_EQ(changed->computedCount, 1u);
    EXPECT_NE(bodyStamp(*changed), bodyStamp(*b));
}

TEST_F(IoTest, V0ManifestWithoutParametersLoadsAsEmptyTable) {
    const std::string v0 = R"({
  "format": "ironwood-document",
  "version": 0,
  "name": "old",
  "displayUnit": "mm",
  "nextId": 3,
  "timeline": {
    "rollbackMarker": null,
    "suppressed": [],
    "features": [ { "id": 1, "kind": "test.box", "params": { "h": 3 } } ]
  }
})";
    fs::create_directories(doc());
    writeFile(manifest(), v0);
    auto loaded = iw::io::loadDocument(doc(), makeRegistry());
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    EXPECT_TRUE(loaded->parameters.parameters().empty());
    EXPECT_EQ(loaded->timeline.size(), 1u);
}

TEST_F(IoTest, V1ManifestWithoutParametersIsParseError) {
    fs::create_directories(doc());
    writeFile(manifest(), R"({"format":"ironwood-document","version":1,"name":"x","displayUnit":"mm","nextId":2,)"
                          R"("timeline":{"rollbackMarker":null,"suppressed":[],"features":[]}})");
    auto loaded = iw::io::loadDocument(doc(), makeRegistry());
    EXPECT_EQ(loaded.error().code, ErrorCode::ParseError);
}

TEST_F(IoTest, SavedManifestIsVersion2WithParametersNamesAndCounters) {
    iw::io::Document d = makeDocument();
    ASSERT_TRUE(d.parameters.add("p", "5 mm"));
    d.nameCounters["test.box"] = 4;
    ASSERT_TRUE(iw::io::saveDocument(doc(), d).has_value());
    const std::string text = readFile(manifest());
    EXPECT_NE(text.find("\"version\": 2"), std::string::npos);
    EXPECT_NE(text.find("\"parameters\""), std::string::npos);
    EXPECT_NE(text.find("\"nameCounters\""), std::string::npos);
}

TEST_F(IoTest, FeatureNamesAndNameCountersRoundTrip) {
    iw::io::Document d;
    d.nextId = 10;
    auto named = makeFeature(1, "test.box", R"({"h":3,"w":10})");
    named->setName("Base block");
    d.timeline.append(named);
    d.timeline.append(makeFeature(2, "test.box", R"({"h":1,"w":1})"));
    d.nameCounters["test.box"] = 7;
    ASSERT_TRUE(iw::io::saveDocument(doc(), d).has_value());
    auto loaded = iw::io::loadDocument(doc(), makeRegistry());
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    EXPECT_EQ(loaded->timeline.at(0)->name(), "Base block");
    EXPECT_EQ(loaded->timeline.at(1)->name(), "") << "unnamed features stay unnamed on disk";
    ASSERT_EQ(loaded->nameCounters.count("test.box"), 1u);
    EXPECT_EQ(loaded->nameCounters.at("test.box"), 7u);
}

TEST_F(IoTest, Version1ManifestLoadsWithoutNames) {
    iw::io::Document d = makeDocument();
    ASSERT_TRUE(iw::io::saveDocument(doc(), d).has_value());
    std::string text = readFile(manifest());
    const std::string current = "\"version\": 2";
    const auto pos = text.find(current);
    ASSERT_NE(pos, std::string::npos);
    text.replace(pos, current.size(), "\"version\": 1");
    writeFile(manifest(), text);
    auto loaded = iw::io::loadDocument(doc(), makeRegistry());
    ASSERT_TRUE(loaded.has_value()) << loaded.error().message;
    EXPECT_EQ(loaded->timeline.size(), 3u);
    EXPECT_EQ(loaded->timeline.at(0)->name(), "");
}
