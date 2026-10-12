// SPDX-License-Identifier: GPL-3.0-or-later
// Determinism of the box/extrude/boolean/tessellate pipeline: repeated runs, and concurrent runs from 4 threads.
// Each run builds its own Shapes from the same inputs. A snapshot records every observable quantity (sub-shape
// counts and keys, element names, mesh bytes); on mismatch the first differing entry is reported by name.
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "core/ids.hpp"
#include "kernel/ops.hpp"
#include "kernel/shape.hpp"
#include "naming/element_name.hpp"
#include "naming/naming_map.hpp"
#include "recompute/evaluator.hpp"
#include "features/extrude_feature.hpp"
#include "recompute/timeline.hpp"
#include "tess/tessellate.hpp"

namespace iw {
namespace {

// Ordered (name, value) pairs. Values are byte strings so mesh data compares bit for bit.
struct Snapshot {
    std::vector<std::pair<std::string, std::string>> entries;

    void add(std::string key, std::string value) { entries.emplace_back(std::move(key), std::move(value)); }
    void addCount(const std::string& key, std::uint64_t value) { add(key, std::to_string(value)); }
    template <class T> void addBytes(const std::string& key, const std::vector<T>& v) {
        add(key, std::string(reinterpret_cast<const char*>(v.data()), v.size() * sizeof(T)));
    }
};

// Returns "" when equal, otherwise a description of the first difference.
std::string firstDifference(const Snapshot& a, const Snapshot& b) {
    if (a.entries.size() != b.entries.size()) {
        return "entry count " + std::to_string(a.entries.size()) + " vs " + std::to_string(b.entries.size());
    }
    for (std::size_t i = 0; i < a.entries.size(); ++i) {
        const auto& [ka, va] = a.entries[i];
        const auto& [kb, vb] = b.entries[i];
        if (ka != kb) return "entry #" + std::to_string(i) + " key '" + ka + "' vs '" + kb + "'";
        if (va == vb) continue;
        std::size_t at = 0;
        while (at < va.size() && at < vb.size() && va[at] == vb[at]) ++at;
        return "'" + ka + "' differs: sizes " + std::to_string(va.size()) + " vs " + std::to_string(vb.size()) +
               ", first differing byte " + std::to_string(at);
    }
    return {};
}

recompute::Timeline makeTimeline() {
    recompute::Timeline timeline;
    auto prism = [](std::uint64_t id, double x0, double y0, double w, double h, double height,
                    features::ExtrudeParams::Operation op) {
        features::ExtrudeParams p;
        features::PolygonSource src;
        src.planeOrigin = {0, 0, 0};
        src.planeNormal = {0, 0, 1};
        src.planeXDir = {1, 0, 0};
        src.points = {{x0, y0}, {x0 + w, y0}, {x0 + w, y0 + h}, {x0, y0 + h}};
        src.vector = {0, 0, height};
        p.source = src;
        p.operation = op;
        p.body = BodyId{1};
        return std::make_shared<features::ExtrudeFeature>(FeatureId{id}, p);
    };
    using Op = features::ExtrudeParams::Operation;
    timeline.append(prism(1, 0, 0, 4, 3, 6, Op::NewBody));
    timeline.append(prism(2, 2, 1, 4, 3, 4, Op::Join));
    timeline.append(prism(3, 1, 2, 2, 5, 3, Op::Join));
    return timeline;
}

// One full pipeline run. Any failing step is recorded as an "error" entry so it shows up in comparisons.
Snapshot runPipeline() {
    Snapshot s;
    auto fail = [&](const std::string& step, const Error& e) {
        s.add("error." + step, std::string(errorCodeName(e.code)) + ": " + e.message);
        return s;
    };

    // B-rep operations: box, cylinder, union, then a cut.
    auto box = kernel::makeBox({0, 0, 0}, 20, 10, 5);
    if (!box) return fail("makeBox", box.error());
    auto cyl = kernel::makeCylinder({10, 5, 0}, {0, 0, 1}, 3, 12);
    if (!cyl) return fail("makeCylinder", cyl.error());
    auto fused = kernel::fuse(box->shape, cyl->shape);
    if (!fused) return fail("fuse", fused.error());
    auto tool = kernel::makeBox({15, 2, 2}, 3, 6, 10);
    if (!tool) return fail("makeBox.tool", tool.error());
    auto cut = kernel::cut(fused->shape, tool->shape);
    if (!cut) return fail("cut", cut.error());
    const kernel::Shape& solid = cut->shape;

    const auto faces = solid.subShapeCount(kernel::SubShapeKind::Face);
    const auto edges = solid.subShapeCount(kernel::SubShapeKind::Edge);
    const auto vertices = solid.subShapeCount(kernel::SubShapeKind::Vertex);
    s.addCount("brep.faces", faces);
    s.addCount("brep.edges", edges);
    s.addCount("brep.vertices", vertices);
    for (std::uint32_t i = 0; i < faces; ++i) {
        const auto key = solid.subShapeKey(kernel::SubShapeKind::Face, i);
        s.addBytes("brep.faceKey." + std::to_string(i), std::vector<double>(key.begin(), key.end()));
    }

    // Extrude/join timeline with a fresh evaluator.
    recompute::Evaluator evaluator;
    auto evaluated = evaluator.evaluate(makeTimeline());
    if (!evaluated) return fail("evaluate", evaluated.error());
    for (const auto& st : evaluated->statuses) {
        s.add("eval.status." + std::to_string(st.id.value()), std::to_string(static_cast<int>(st.state)));
    }
    s.addCount("eval.computed", evaluated->computedCount);
    for (const auto& [id, body] : evaluated->bodies) {
        const auto& map = evaluated->namings.at(id);
        const std::string prefix = "eval.body" + std::to_string(id.value());
        s.addCount(prefix + ".faces", body.subShapeCount(kernel::SubShapeKind::Face));
        s.addCount(prefix + ".edges", body.subShapeCount(kernel::SubShapeKind::Edge));
        for (std::uint32_t i = 0; i < map.size(naming::ElementType::Face); ++i) {
            const auto name = map.nameOf(naming::ElementType::Face, i);
            s.add(prefix + ".faceName." + std::to_string(i), name ? name->toString() : std::string("<unnamed>"));
        }
    }

    // Tessellation of the fused/cut solid and of the evaluated body, coarse and fine.
    for (double deflection : {0.5, 0.05}) {
        tess::TessParams params;
        params.linearDeflection = deflection;
        const std::string tag = "tess." + std::to_string(deflection);
        auto mesh = tess::tessellate(solid, params);
        if (!mesh) return fail(tag, mesh.error());
        s.addCount(tag + ".triangles", mesh->indices.size() / 3);
        s.addBytes(tag + ".positions", mesh->positions);
        s.addBytes(tag + ".normals", mesh->normals);
        s.addBytes(tag + ".indices", mesh->indices);
        for (std::size_t i = 0; i < mesh->faceRanges.size(); ++i) {
            s.addBytes(tag + ".faceRange." + std::to_string(i),
                       std::vector<std::uint32_t>{mesh->faceRanges[i].firstIndex, mesh->faceRanges[i].indexCount});
        }
        for (std::size_t i = 0; i < mesh->edgePolylines.size(); ++i)
            s.addBytes(tag + ".edge." + std::to_string(i), mesh->edgePolylines[i].points);
    }
    return s;
}

constexpr int kRepeats = 20;
constexpr int kThreads = 4;
constexpr int kRoundsPerThread = 5;

} // namespace

TEST(Determinism, RepeatedPipelineIsIdentical) {
    const Snapshot reference = runPipeline();
    ASSERT_FALSE(reference.entries.empty());
    for (const auto& [key, value] : reference.entries) {
        ASSERT_NE(key.rfind("error.", 0), 0u) << "reference run failed: " << key << " = " << value;
    }
    for (int i = 1; i < kRepeats; ++i) {
        const std::string diff = firstDifference(reference, runPipeline());
        ASSERT_TRUE(diff.empty()) << "run " << i << " of " << kRepeats << ": " << diff;
    }
}

// First real multi-thread check of the OCCT usage in this pipeline. Each thread builds its own shapes and
// evaluator from the same inputs; all results must match the single-threaded reference bit for bit.
TEST(Determinism, FourConcurrentThreadsMatchReference) {
    const Snapshot reference = runPipeline();
    for (const auto& [key, value] : reference.entries) {
        ASSERT_NE(key.rfind("error.", 0), 0u) << "reference run failed: " << key << " = " << value;
    }

    std::vector<std::vector<Snapshot>> perThread(kThreads);
    std::vector<std::thread> threads;
    threads.reserve(kThreads);
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&perThread, t] {
            for (int r = 0; r < kRoundsPerThread; ++r) perThread[t].push_back(runPipeline());
        });
    }
    for (auto& th : threads) th.join();

    for (int t = 0; t < kThreads; ++t) {
        ASSERT_EQ(perThread[t].size(), static_cast<std::size_t>(kRoundsPerThread));
        for (int r = 0; r < kRoundsPerThread; ++r) {
            const std::string diff = firstDifference(reference, perThread[t][r]);
            EXPECT_TRUE(diff.empty()) << "thread " << t << " round " << r << ": " << diff;
        }
    }
}

} // namespace iw
