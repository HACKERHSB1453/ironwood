// SPDX-License-Identifier: GPL-3.0-or-later
// Benchmark harness. Writes one JSON document to stdout; progress and diagnostics go to stderr.
//   {"version":1,"machine":{"cpu":"...","cores":N,"gpu":null},"results":[{"name":"...","ms":0.0,"n":0}]}
// Every timing is the median of kReps runs. "n" is the count of work items the run covered (nodes, triangles,
// features actually computed); for the re-evaluations it is the number of features recomputed, which shows the
// cache reach directly. Exits non-zero if any feature in the timeline fails, so a broken build cannot produce
// a plausible-looking baseline.
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "core/ids.hpp"
#include "core/math.hpp"
#include "kernel/exchange.hpp"
#include "kernel/ops.hpp"
#include "recompute/evaluator.hpp"
#include "recompute/extrude_feature.hpp"
#include "recompute/timeline.hpp"
#include "tess/tessellate.hpp"

namespace {

using Clock = std::chrono::steady_clock;
namespace kernel = iw::kernel;
namespace recompute = iw::recompute;

constexpr int kReps = 3;
constexpr int kTimelineFeatures = 50;

struct Result {
    std::string name;
    double ms = 0;
    std::size_t n = 0;
};

class Timer {
public:
    Timer() : start_(Clock::now()) {}
    double ms() const { return std::chrono::duration<double, std::milli>(Clock::now() - start_).count(); }
private:
    Clock::time_point start_;
};

double median(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

// Assembly fixture: a group node holding boxes and cylinders at distinct placements, exported by the kernel itself
// so the STEP import benchmark needs no external file.
kernel::ImportedAssembly makeFixtureAssembly() {
    kernel::ImportedAssembly assembly;
    kernel::AssemblyNode group;
    group.name = "Bench assembly";
    group.parent = -1;
    assembly.nodes.push_back(group);
    assembly.roots.push_back(0);

    auto addPart = [&](const std::string& name, const kernel::Shape& shape, double x, double y) {
        kernel::AssemblyNode node;
        node.name = name;
        node.shape = shape;
        node.local.m[3] = x;
        node.local.m[7] = y;
        node.parent = 0;
        const auto index = static_cast<std::uint32_t>(assembly.nodes.size());
        assembly.nodes.push_back(node);
        assembly.nodes[0].children.push_back(index);
    };

    for (int i = 0; i < 8; ++i) {
        auto box = kernel::makeBox({0, 0, 0}, 10 + i, 12, 8 + i);
        if (!box) throw std::runtime_error("fixture makeBox failed: " + box.error().message);
        addPart("box " + std::to_string(i), box->shape, i * 30.0, 0.0);
    }
    for (int i = 0; i < 4; ++i) {
        auto cyl = kernel::makeCylinder({0, 0, 0}, {0, 0, 1}, 4 + i, 20 + i);
        if (!cyl) throw std::runtime_error("fixture makeCylinder failed: " + cyl.error().message);
        addPart("cylinder " + std::to_string(i), cyl->shape, i * 30.0, 60.0);
    }
    return assembly;
}

// Timeline: feature 1 creates body 1 from a 2 x 2 mm square prism; features 2..50 join overlapping prisms
// (each 1 mm wide, stepping 0.5 mm along x) onto it. `editedHeight` replaces the extrusion height of one feature.
std::shared_ptr<const recompute::Feature> makePrism(int index, double height) {
    recompute::ExtrudeParams p;
    p.planeOrigin = {0, 0, 0};
    p.planeNormal = {0, 0, 1};
    p.planeXDir = {1, 0, 0};
    const double x0 = index * 0.5;
    const double x1 = x0 + 1.0;
    p.points = {{x0, 0.0}, {x1, 0.0}, {x1, 2.0}, {x0, 2.0}};
    p.vector = {0, 0, height};
    p.operation = index == 1 ? recompute::ExtrudeParams::Operation::NewBody : recompute::ExtrudeParams::Operation::Join;
    p.body = iw::BodyId{1};
    return std::make_shared<recompute::ExtrudeFeature>(iw::FeatureId{static_cast<std::uint64_t>(index)}, p);
}

recompute::Timeline makeTimeline() {
    recompute::Timeline timeline;
    for (int i = 1; i <= kTimelineFeatures; ++i) timeline.append(makePrism(i, 10.0));
    return timeline;
}

// Returns false (and prints the offending features) if any status is not Ok.
bool allOk(const recompute::EvalResult& r) {
    bool ok = true;
    for (const auto& s : r.statuses) {
        if (s.state == recompute::FeatureStatus::State::Ok) continue;
        ok = false;
        std::fprintf(stderr, "error: feature %llu not Ok (state %d): %s\n",
                     static_cast<unsigned long long>(s.id.value()), static_cast<int>(s.state),
                     s.error.message.c_str());
    }
    return ok;
}

std::string cpuModel() {
    std::ifstream in("/proc/cpuinfo");
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("model name", 0) == 0) {
            const auto colon = line.find(':');
            if (colon != std::string::npos) {
                std::string name = line.substr(colon + 1);
                const auto first = name.find_first_not_of(' ');
                return first == std::string::npos ? name : name.substr(first);
            }
        }
    }
    return "unknown";
}

std::string jsonEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

// OCCT's STEP reader/writer prints progress to the process stdout. To keep stdout pure JSON, the harness points
// fd 1 at stderr for its whole run and writes the document to the original stdout descriptor.
class StdoutGuard {
public:
    StdoutGuard() {
        std::fflush(stdout);
        savedStdout_ = ::dup(STDOUT_FILENO);
        if (savedStdout_ >= 0) ::dup2(STDERR_FILENO, STDOUT_FILENO);
    }
    ~StdoutGuard() {
        std::fflush(stdout);
        if (savedStdout_ >= 0) {
            ::dup2(savedStdout_, STDOUT_FILENO);
            ::close(savedStdout_);
        }
    }
    StdoutGuard(const StdoutGuard&) = delete;
    StdoutGuard& operator=(const StdoutGuard&) = delete;
    int original() const { return savedStdout_; }
private:
    int savedStdout_ = -1;
};

void writeJson(FILE* out, const std::vector<Result>& results) {
    std::fprintf(out, "{\"version\":1,\"machine\":{\"cpu\":\"%s\",\"cores\":%u,\"gpu\":null},\"results\":[",
                 jsonEscape(cpuModel()).c_str(), std::thread::hardware_concurrency());
    for (std::size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        std::fprintf(out, "%s{\"name\":\"%s\",\"ms\":%.3f,\"n\":%zu}", i ? "," : "", r.name.c_str(), r.ms, r.n);
    }
    std::fprintf(out, "]}\n");
    std::fflush(out);
}

} // namespace

int main() {
    StdoutGuard guard;
    if (guard.original() < 0) {
        std::fprintf(stderr, "error: cannot duplicate stdout\n");
        return 1;
    }
    try {
        std::vector<Result> results;
        const auto dir = std::filesystem::temp_directory_path() / "ironwood-bench";
        std::filesystem::create_directories(dir);
        const auto stepPath = dir / "fixture.step";

        std::vector<kernel::Shape> importedShapes; // curved geometry (cylinders) for the tessellation benchmarks

        // 1. STEP import (fixture written by the kernel's own exporter).
        auto fixture = makeFixtureAssembly();
        if (auto w = kernel::exportStep(stepPath, fixture); !w) {
            std::fprintf(stderr, "error: fixture export failed: %s\n", w.error().message.c_str());
            return 1;
        }
        {
            std::vector<double> times;
            std::optional<kernel::ImportedAssembly> imported;
            for (int r = 0; r < kReps; ++r) {
                Timer t;
                auto next = kernel::importStep(stepPath);
                times.push_back(t.ms());
                if (!next) {
                    std::fprintf(stderr, "error: importStep failed: %s\n", next.error().message.c_str());
                    return 1;
                }
                imported = std::move(*next);
            }
            results.push_back({"step_import", median(times), imported->nodes.size()});
            importedShapes.clear();
            for (const auto& node : imported->nodes)
                if (!node.shape.isNull()) importedShapes.push_back(node.shape);
        }

        // 2. Cold evaluation of the 50-feature timeline (fresh Evaluator per repetition).
        const auto timeline = makeTimeline();
        std::optional<recompute::EvalResult> cold;
        {
            std::vector<double> times;
            for (int r = 0; r < kReps; ++r) {
                recompute::Evaluator evaluator;
                Timer t;
                auto evaluated = evaluator.evaluate(timeline);
                times.push_back(t.ms());
                if (!evaluated) {
                    std::fprintf(stderr, "error: evaluate failed: %s\n", evaluated.error().message.c_str());
                    return 1;
                }
                if (!allOk(*evaluated)) return 1;
                cold = std::move(*evaluated);
            }
            results.push_back({"evaluate_cold_50", median(times), cold->computedCount});
        }
        const kernel::Shape& body = cold->bodies.at(iw::BodyId{1});

        // 3. Tessellation, coarse and fine (absolute deflection, mm): the evaluated timeline body (planar, so the
        // deflection barely matters) and the imported STEP fixture (cylinders, where it does).
        const auto tessellateAll = [&](const std::vector<kernel::Shape>& shapes, const std::string& name) -> bool {
            for (const auto& [suffix, deflection] :
                 std::vector<std::pair<std::string, double>>{{"coarse", 0.5}, {"fine", 0.05}}) {
                std::vector<double> times;
                std::size_t triangles = 0;
                for (int r = 0; r < kReps; ++r) {
                    iw::tess::TessParams params;
                    params.linearDeflection = deflection;
                    Timer t;
                    std::size_t count = 0;
                    for (const auto& shape : shapes) {
                        auto mesh = iw::tess::tessellate(shape, params);
                        if (!mesh) {
                            std::fprintf(stderr, "error: tessellate failed: %s\n", mesh.error().message.c_str());
                            return false;
                        }
                        count += mesh->indices.size() / 3;
                    }
                    times.push_back(t.ms());
                    triangles = count;
                }
                results.push_back({name + "_" + suffix, median(times), triangles});
            }
            return true;
        };
        if (!tessellateAll({body}, "tessellate_body")) return 1;
        if (!tessellateAll(importedShapes, "tessellate_step")) return 1;

        // 4. Re-evaluation after editing feature 50, then after editing feature 1. A warm Evaluator is primed once
        // (untimed); each repetition alternates the edited height so every call is a real change, not a cache hit.
        recompute::Evaluator warm;
        if (auto primed = warm.evaluate(timeline); !primed || !allOk(*primed)) {
            std::fprintf(stderr, "error: priming evaluation failed\n");
            return 1;
        }
        auto reevaluate = [&](int editedIndex, const std::string& name) -> bool {
            std::vector<double> times;
            std::size_t computed = 0;
            recompute::Timeline edited = timeline;
            for (int r = 0; r < kReps; ++r) {
                const double height = (r % 2 == 0) ? 12.0 : 14.0;
                if (auto rep = edited.replace(makePrism(editedIndex, height)); !rep) return false;
                Timer t;
                auto evaluated = warm.evaluate(edited);
                times.push_back(t.ms());
                if (!evaluated || !allOk(*evaluated)) {
                    std::fprintf(stderr, "error: re-evaluation (%s) failed\n", name.c_str());
                    return false;
                }
                computed = evaluated->computedCount;
            }
            results.push_back({name, median(times), computed});
            return true;
        };
        if (!reevaluate(kTimelineFeatures, "reevaluate_edit_50")) return 1;
        if (!reevaluate(1, "reevaluate_edit_1")) return 1;

        std::filesystem::remove_all(dir);
        FILE* json = ::fdopen(::dup(guard.original()), "w");
        if (!json) {
            std::fprintf(stderr, "error: cannot open JSON output\n");
            return 1;
        }
        writeJson(json, results);
        std::fclose(json);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
