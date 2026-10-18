// SPDX-License-Identifier: GPL-3.0-or-later
// Benchmark harness. Writes one JSON document to stdout; progress and diagnostics go to stderr.
//   {"version":2,"machine":{"cpu":"...","cores":N,"gpu":"..."|null},"results":[{"name":"...","ms":0.0,"n":0}]}
// "gpu" is the preferred Vulkan physical device name (discrete GPUs first), or null when no loader/ICD is present.
// Every timing is the median of kReps runs. "n" is the count of work items the run covered (nodes, triangles,
// entities, faces, features actually computed); for the re-evaluations it is the number of features recomputed,
// which shows the cache reach directly. Exits non-zero if any feature in a timeline fails, so a broken build cannot
// produce a plausible-looking baseline.
//
// Run ONE benchmark process at a time, in a Release build, with the machine otherwise idle:
//   cmake --preset release -B build/<dir> && cmake --build build/<dir> -j 4 --target ironwood-bench
//   build/<dir>/ironwood-bench > bench/baselines/<date>_<machine>.json
// Geometry validation (boolean / fillet / chamfer result checks, F38/F39) is always on; there is no switch for it.
#include <dlfcn.h>
#include <unistd.h>

// Only the Vulkan types and the PFN_ typedefs are used; the loader is opened with dlopen, so no link dependency.
#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "core/ids.hpp"
#include "core/math.hpp"
#include "core/params.hpp"
#include "features/edge_features.hpp"
#include "features/extrude_feature.hpp"
#include "features/hole_feature.hpp"
#include "kernel/exchange.hpp"
#include "kernel/ops.hpp"
#include "naming/element_name.hpp"
#include "naming/naming_map.hpp"
#include "recompute/evaluator.hpp"
#include "recompute/timeline.hpp"
#include "sketch/profile.hpp"
#include "sketch/sketch.hpp"
#include "sketch/solver/bridge.hpp"
#include "tess/tessellate.hpp"

namespace {

using Clock = std::chrono::steady_clock;
namespace kernel = iw::kernel;
namespace recompute = iw::recompute;
namespace features = iw::features;
namespace naming = iw::naming;
namespace sketch = iw::sketch;

constexpr int kReps = 3;
constexpr int kTimelineFeatures = 50;
constexpr std::uint64_t kBodyId = 1;

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

// Throws with `what` in the message when a Result<T> holds an error; otherwise returns the value.
template <class T>
T take(iw::Result<T> r, const char* what) {
    if (!r) throw std::runtime_error(std::string(what) + ": " + r.error().message);
    return std::move(*r);
}

void check(const iw::Result<void>& r, const char* what) {
    if (!r) throw std::runtime_error(std::string(what) + ": " + r.error().message);
}

// Assembly fixture: a group node holding boxes and cylinders at distinct placements, exported by the kernel itself
// so the STEP import benchmark needs no external file. `boxes` + `cylinders` parts in total.
kernel::ImportedAssembly makeFixtureAssembly(int boxes, int cylinders) {
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

    // Parts are laid out on a 20-column grid with a 15 mm pitch (the 200-part fixture needs ~150 mm x 150 mm).
    for (int i = 0; i < boxes; ++i) {
        auto box = kernel::makeBox({0, 0, 0}, 8 + i % 5, 10 + i % 3, 6 + i % 4);
        if (!box) throw std::runtime_error("fixture makeBox failed: " + box.error().message);
        addPart("box " + std::to_string(i), box->shape, (i % 20) * 15.0, (i / 20) * 15.0);
    }
    for (int i = 0; i < cylinders; ++i) {
        auto cyl = kernel::makeCylinder({0, 0, 0}, {0, 0, 1}, 3 + i % 3, 10 + i % 4);
        if (!cyl) throw std::runtime_error("fixture makeCylinder failed: " + cyl.error().message);
        addPart("cylinder " + std::to_string(i), cyl->shape, (i % 20) * 15.0 + 7.0, (i / 20) * 15.0 + 7.0 + 75.0);
    }
    return assembly;
}

// Timeline: feature 1 creates body 1 from a 2 x 2 mm square prism; features 2..50 join overlapping prisms
// (each 1 mm wide, stepping 0.5 mm along x) onto it. `editedHeight` replaces the extrusion height of one feature.
std::shared_ptr<const recompute::Feature> makePrism(int index, double height) {
    features::ExtrudeParams p;
    features::PolygonSource src;
    src.planeOrigin = {0, 0, 0};
    src.planeNormal = {0, 0, 1};
    src.planeXDir = {1, 0, 0};
    const double x0 = index * 0.5;
    const double x1 = x0 + 1.0;
    src.points = {{x0, 0.0}, {x1, 0.0}, {x1, 2.0}, {x0, 2.0}};
    src.vector = {0, 0, height};
    p.source = src;
    p.operation = index == 1 ? features::ExtrudeParams::Operation::NewBody : features::ExtrudeParams::Operation::Join;
    p.body = iw::BodyId{kBodyId};
    return std::make_shared<features::ExtrudeFeature>(iw::FeatureId{static_cast<std::uint64_t>(index)}, p);
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

// ---- Sketch solve: 200 entities -------------------------------------------------------------------------------
// 200 separate lines on a 20 x 10 grid (15 mm pitch). Even lines are horizontal, odd lines vertical, each with a
// 10 mm distance dimension. The initial geometry is perturbed (about 0.4 mm off the target) so the solver has to
// iterate instead of returning at once.
sketch::Sketch makeSolveSketch() {
    const auto plane = take(sketch::SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0}), "sketch plane");
    sketch::Sketch s(plane);
    for (int i = 0; i < 200; ++i) {
        const double x = (i % 20) * 15.0;
        const double y = (i / 20) * 15.0;
        const bool horizontal = i % 2 == 0;
        const double jitter = 0.2 * std::sin(i * 1.7);
        const sketch::Point2 a{x + jitter, y};
        const sketch::Point2 b = horizontal ? sketch::Point2{x + 10.4, y + 0.2 * std::cos(i)}
                                            : sketch::Point2{x + 0.2 * std::cos(i), y + 10.4};
        const auto id = take(s.addLine(a, b), "solve sketch line");
        using PR = sketch::PointRole;
        check(s.addConstraint(horizontal ? sketch::ConstraintKind::Horizontal : sketch::ConstraintKind::Vertical,
                              {{id, std::nullopt}}).transform([](auto) {}), "solve sketch constraint");
        check(s.addDimension(sketch::DimensionKind::Distance, {{id, PR::Start}, {id, PR::End}}, "10 mm").transform([](auto) {}),
              "solve sketch dimension");
    }
    return s;
}

// ---- Realistic 100-feature part -------------------------------------------------------------------------------
// Feature list (100 features, ids 1..100):
//   1      extrude (new body): 120 x 60 x 12 mm block
//   2      fillet: the four vertical outer edges, R 3 mm
//   3..100 49 pairs of (sketch-profile cut extrude, simple hole), laid out on an 11 x 5 grid over the top face:
//          pocket i is a 4 x 4 mm square cut 'pd' deep; hole i is a 2.5 mm hole 6 mm deep at (x+7, y+2).
// Parameters: 'pd' (pocket depth, used by all 49 pockets: an early edit) and 'hdl' (diameter of the LAST hole only:
// a late edit). Names of the top face and the corner edges are looked up from a scripted evaluation (pass 1) and
// reused by the timed passes, so every timed evaluation starts from the same feature list.
constexpr int kPockets = 49;
constexpr double kPlateZ = 12.0;

struct RealisticPart {
    std::vector<std::shared_ptr<const recompute::Feature>> features;
    iw::ParameterTable table;
};

// Top face: planar, normal +Z, at z = kPlateZ.
struct TopFace {
    naming::ElementRef ref;
    kernel::Plane plane;
};

TopFace findTopFace(const recompute::EvalResult& r) {
    const auto& shape = r.bodies.at(iw::BodyId{kBodyId});
    const auto& names = r.namings.at(iw::BodyId{kBodyId});
    for (std::uint32_t i = 0; i < shape.subShapeCount(kernel::SubShapeKind::Face); ++i) {
        const auto g = shape.faceGeometry(i);
        if (g.kind != kernel::FaceGeometry::Kind::Plane || g.normal.z < 1.0 - 1e-9) continue;
        const auto key = shape.subShapeKey(kernel::SubShapeKind::Face, i);
        if (std::abs(key[2] - kPlateZ) > 1e-6) continue;
        const auto name = names.nameOf(naming::ElementType::Face, i);
        if (!name) throw std::runtime_error("top face has no name");
        return TopFace{naming::ElementRef{name->feature, naming::ElementType::Face, *name},
                       take(kernel::planeFromFace(shape, i), "planeFromFace")};
    }
    throw std::runtime_error("top face not found");
}

// The four vertical outer edges of the block: length = plate height, centroid at mid-height.
std::vector<naming::ElementRef> cornerEdges(const recompute::EvalResult& r) {
    const auto& shape = r.bodies.at(iw::BodyId{kBodyId});
    const auto& names = r.namings.at(iw::BodyId{kBodyId});
    std::vector<naming::ElementRef> out;
    for (std::uint32_t i = 0; i < shape.subShapeCount(kernel::SubShapeKind::Edge); ++i) {
        const auto key = shape.subShapeKey(kernel::SubShapeKind::Edge, i);
        if (std::abs(key[3] - kPlateZ) > 1e-6 || std::abs(key[2] - kPlateZ / 2) > 1e-6) continue;
        const auto name = names.nameOf(naming::ElementType::Edge, i);
        if (!name) throw std::runtime_error("corner edge has no name");
        out.push_back(naming::ElementRef{name->feature, naming::ElementType::Edge, *name});
    }
    if (out.size() != 4) throw std::runtime_error("expected 4 corner edges, found " + std::to_string(out.size()));
    return out;
}

// Square pocket sketch on the top plane (sketch coordinates = world X, Y at z = kPlateZ).
sketch::Sketch pocketSketch(double x, double y) {
    const auto plane = take(sketch::SketchPlane::make({0, 0, kPlateZ}, {0, 0, 1}, {1, 0, 0}), "pocket plane");
    sketch::Sketch s(plane);
    const sketch::Point2 p0{x, y}, p1{x + 4, y}, p2{x + 4, y + 4}, p3{x, y + 4};
    check(s.addLine(p0, p1).transform([](auto) {}), "pocket line");
    check(s.addLine(p1, p2).transform([](auto) {}), "pocket line");
    check(s.addLine(p2, p3).transform([](auto) {}), "pocket line");
    check(s.addLine(p3, p0).transform([](auto) {}), "pocket line");
    return s;
}

// Builds the feature list with a scripted evaluation after every step (the top-face and corner-edge names are only
// known once the features before them have been computed).
RealisticPart buildRealisticPart() {
    RealisticPart part;
    check(part.table.add("pd", "4 mm"), "param pd");
    check(part.table.add("hdl", "2.5 mm"), "param hdl");

    recompute::Timeline timeline;
    recompute::Evaluator evaluator;
    auto push = [&](std::shared_ptr<const recompute::Feature> f) {
        part.features.push_back(f);
        timeline.append(std::move(f));
    };
    auto evaluateAll = [&]() {
        auto r = take(evaluator.evaluate(timeline, &part.table), "realistic evaluate");
        if (!allOk(r)) throw std::runtime_error("realistic part: a feature failed during construction");
        return r;
    };

    features::PolygonSource block;
    block.planeOrigin = {0, 0, 0};
    block.planeNormal = {0, 0, 1};
    block.planeXDir = {1, 0, 0};
    block.points = {{0, 0}, {120, 0}, {120, 60}, {0, 60}};
    block.vector = {0, 0, kPlateZ};
    features::ExtrudeParams baseParams;
    baseParams.source = block;
    baseParams.operation = features::ExtrudeParams::Operation::NewBody;
    baseParams.body = iw::BodyId{kBodyId};
    push(std::make_shared<features::ExtrudeFeature>(iw::FeatureId{1}, baseParams));
    auto afterBase = evaluateAll();

    features::FilletParams fillet;
    fillet.body = iw::BodyId{kBodyId};
    fillet.edges = cornerEdges(afterBase);
    fillet.radius = "3 mm";
    fillet.tangentChain = false;
    push(std::make_shared<features::FilletFeature>(iw::FeatureId{2}, fillet));
    evaluateAll();

    for (int i = 0; i < kPockets; ++i) {
        const double x = 8.0 + 10.0 * (i % 11);
        const double y = 8.0 + 10.0 * (i / 11);

        sketch::Sketch pocket = pocketSketch(x, y);
        const auto profiles = take(sketch::detectProfiles(pocket), "pocket profiles");
        if (profiles.size() != 1) throw std::runtime_error("pocket sketch must have exactly one profile");
        features::ExtrudeParams cut;
        cut.source = features::SketchSource{pocket, profiles.front().name};
        cut.extent = features::ExtrudeParams::Extent::OneSide;
        cut.distance = "-pd";  // negative = backwards along the sketch normal, i.e. down into the plate
        cut.operation = features::ExtrudeParams::Operation::Cut;
        cut.body = iw::BodyId{kBodyId};
        push(std::make_shared<features::ExtrudeFeature>(iw::FeatureId{static_cast<std::uint64_t>(3 + 2 * i)}, cut));
        const TopFace top = findTopFace(evaluateAll());

        const bool last = i == kPockets - 1;
        const auto at = top.plane;
        const double hx = x + 7.0, hy = y + 2.0;
        const iw::Vec3 world{hx, hy, kPlateZ};
        const iw::Vec3 d = world - at.origin;
        features::HoleParams hole;
        hole.body = iw::BodyId{kBodyId};
        hole.face = top.ref;
        hole.u = std::to_string(iw::dot(d, at.xDir)) + " mm";
        hole.v = std::to_string(iw::dot(d, iw::cross(at.normal, at.xDir))) + " mm";
        hole.kind = kernel::HoleKind::Simple;
        hole.diameter = last ? "hdl" : "2.5 mm";
        hole.depth = "6 mm";
        push(std::make_shared<features::HoleFeature>(iw::FeatureId{static_cast<std::uint64_t>(4 + 2 * i)}, hole));
        evaluateAll();
    }
    return part;
}

// Fresh evaluator over the stored feature list (cold: nothing cached).
recompute::Timeline timelineOf(const RealisticPart& part) {
    recompute::Timeline t;
    for (const auto& f : part.features) t.append(f);
    return t;
}

// ---- Machine description --------------------------------------------------------------------------------------

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

// Name of the preferred Vulkan physical device (discrete GPU first), or empty when the loader or no device is
// available. Creates a throwaway instance; loads libvulkan.so.1 at run time so the harness stays link-free.
std::string gpuName() {
    void* lib = ::dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!lib) return {};
    auto getProc = reinterpret_cast<PFN_vkGetInstanceProcAddr>(::dlsym(lib, "vkGetInstanceProcAddr"));
    if (!getProc) return {};
    auto create = reinterpret_cast<PFN_vkCreateInstance>(getProc(VK_NULL_HANDLE, "vkCreateInstance"));
    if (!create) return {};

    VkApplicationInfo app{};
    app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app.pApplicationName = "ironwood-bench";
    app.apiVersion = VK_API_VERSION_1_0;
    VkInstanceCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    info.pApplicationInfo = &app;
    VkInstance instance = VK_NULL_HANDLE;
    if (create(&info, nullptr, &instance) != VK_SUCCESS || instance == VK_NULL_HANDLE) return {};

    std::string name;
    auto enumerate = reinterpret_cast<PFN_vkEnumeratePhysicalDevices>(getProc(instance, "vkEnumeratePhysicalDevices"));
    auto properties = reinterpret_cast<PFN_vkGetPhysicalDeviceProperties>(getProc(instance, "vkGetPhysicalDeviceProperties"));
    auto destroy = reinterpret_cast<PFN_vkDestroyInstance>(getProc(instance, "vkDestroyInstance"));
    std::uint32_t count = 0;
    if (enumerate && properties && enumerate(instance, &count, nullptr) == VK_SUCCESS && count > 0) {
        std::vector<VkPhysicalDevice> devices(count);
        if (enumerate(instance, &count, devices.data()) == VK_SUCCESS) {
            VkPhysicalDeviceProperties best{};
            bool have = false;
            for (const VkPhysicalDevice device : devices) {
                VkPhysicalDeviceProperties p{};
                properties(device, &p);
                const bool discrete = p.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU;
                const bool bestDiscrete = best.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU;
                if (!have || (discrete && !bestDiscrete)) {
                    best = p;
                    have = true;
                }
            }
            if (have) name = best.deviceName;
        }
    }
    if (destroy) destroy(instance, nullptr);
    return name;
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

void writeJson(FILE* out, const std::vector<Result>& results, const std::string& gpu) {
    const std::string gpuJson = gpu.empty() ? "null" : "\"" + jsonEscape(gpu) + "\"";
    std::fprintf(out, "{\"version\":2,\"machine\":{\"cpu\":\"%s\",\"cores\":%u,\"gpu\":%s},\"results\":[",
                 jsonEscape(cpuModel()).c_str(), std::thread::hardware_concurrency(), gpuJson.c_str());
    for (std::size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        std::fprintf(out, "%s{\"name\":\"%s\",\"ms\":%.3f,\"n\":%zu}", i ? "," : "", r.name.c_str(), r.ms, r.n);
    }
    std::fprintf(out, "]}\n");
    std::fflush(out);
}

// Tessellates every shape at a coarse and a fine deflection (mm) and records the median time and triangle count.
bool tessellateAll(const std::vector<kernel::Shape>& shapes, const std::string& name, std::vector<Result>& results) {
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
}

// Imports a STEP file kReps times; returns the last import (nodes and shapes) and records the median time.
kernel::ImportedAssembly importTimed(const std::filesystem::path& path, const std::string& name,
                                     std::vector<Result>& results) {
    std::vector<double> times;
    std::optional<kernel::ImportedAssembly> imported;
    for (int r = 0; r < kReps; ++r) {
        Timer t;
        auto next = kernel::importStep(path);
        times.push_back(t.ms());
        if (!next) throw std::runtime_error("importStep failed: " + next.error().message);
        imported = std::move(*next);
    }
    results.push_back({name, median(times), imported->nodes.size()});
    return std::move(*imported);
}

std::vector<kernel::Shape> nonNullShapes(const kernel::ImportedAssembly& a) {
    std::vector<kernel::Shape> out;
    for (const auto& node : a.nodes)
        if (!node.shape.isNull()) out.push_back(node.shape);
    return out;
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
        const auto step200Path = dir / "fixture200.step";

        std::vector<kernel::Shape> importedShapes; // curved geometry (cylinders) for the tessellation benchmarks

        // 1. STEP import (12-part fixture written by the kernel's own exporter).
        auto fixture = makeFixtureAssembly(8, 4);
        check(kernel::exportStep(stepPath, fixture), "fixture export");
        importedShapes = nonNullShapes(importTimed(stepPath, "step_import", results));

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
        const kernel::Shape& body = cold->bodies.at(iw::BodyId{kBodyId});

        // 3. Tessellation, coarse and fine: the evaluated timeline body (planar, so the deflection barely matters),
        // a curved cylinder (where it does), and the imported 12-part STEP fixture.
        if (!tessellateAll({body}, "tessellate_body", results)) return 1;
        auto curved = kernel::makeCylinder({0, 0, 0}, {0, 0, 1}, 10.0, 20.0);
        if (!curved) {
            std::fprintf(stderr, "error: makeCylinder failed: %s\n", curved.error().message.c_str());
            return 1;
        }
        if (!tessellateAll({curved->shape}, "tessellate_cylinder", results)) return 1;
        if (!tessellateAll(importedShapes, "tessellate_step", results)) return 1;

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

        // 5. STEP import of a generated 200-part assembly (100 boxes, 100 cylinders) and its tessellation.
        check(kernel::exportStep(step200Path, makeFixtureAssembly(100, 100)), "200-part fixture export");
        const auto big = importTimed(step200Path, "step_import_200", results);
        if (!tessellateAll(nonNullShapes(big), "tessellate_step200", results)) return 1;

        // 6. Sketch solve: 200 entities, fresh copy per repetition, time of solveSketch only.
        const sketch::Sketch base = makeSolveSketch();
        {
            std::vector<double> times;
            std::size_t entities = 0;
            for (int r = 0; r < kReps; ++r) {
                sketch::Sketch copy = base;
                Timer t;
                const auto report = sketch::solveSketch(copy, nullptr);
                times.push_back(t.ms());
                if (report.status != sketch::SolverStatus::Solved) {
                    std::fprintf(stderr, "error: sketch solve failed: %s\n", report.message.c_str());
                    return 1;
                }
                entities = copy.entities().size();
            }
            results.push_back({"sketch_solve_200", median(times), entities});
        }

        // 7. Constant fillet of all 12 edges of a box, and a counterbore through-hole on its top face.
        const auto box = take(kernel::makeBox({0, 0, 0}, 40, 30, 20), "bench box");
        {
            std::vector<std::uint32_t> edges(box.shape.subShapeCount(kernel::SubShapeKind::Edge));
            std::iota(edges.begin(), edges.end(), 0u);
            std::vector<double> times;
            std::size_t faces = 0;
            for (int r = 0; r < kReps; ++r) {
                Timer t;
                auto filleted = kernel::filletEdges(box.shape, edges, 2.0);
                times.push_back(t.ms());
                if (!filleted) {
                    std::fprintf(stderr, "error: fillet failed: %s\n", filleted.error().message.c_str());
                    return 1;
                }
                faces = filleted->shape.subShapeCount(kernel::SubShapeKind::Face);
            }
            results.push_back({"fillet_box_12_edges", median(times), faces});
        }
        {
            kernel::HoleSpec spec;
            spec.kind = kernel::HoleKind::Counterbore;
            spec.diameter = 6;
            spec.through = true;
            spec.counterboreDiameter = 10;
            spec.counterboreDepth = 3;
            const kernel::Axis axis{{20, 15, 20}, {0, 0, -1}};
            std::vector<double> times;
            std::size_t faces = 0;
            for (int r = 0; r < kReps; ++r) {
                Timer t;
                auto drilled = kernel::hole(box.shape, axis, spec);
                times.push_back(t.ms());
                if (!drilled) {
                    std::fprintf(stderr, "error: hole failed: %s\n", drilled.error().message.c_str());
                    return 1;
                }
                faces = drilled->shape.subShapeCount(kernel::SubShapeKind::Face);
            }
            results.push_back({"hole_box_counterbore", median(times), faces});
        }

        // 8. The realistic 100-feature part: cold evaluation, then edit-early (pd, every pocket) and edit-late (hdl,
        // the last hole only). Warm timings use one evaluator; the parameter alternates so each call is a real change.
        RealisticPart part = buildRealisticPart();
        {
            std::vector<double> times;
            std::size_t computed = 0;
            for (int r = 0; r < kReps; ++r) {
                recompute::Evaluator evaluator;
                const auto t100 = timelineOf(part);
                Timer t;
                auto evaluated = evaluator.evaluate(t100, &part.table);
                times.push_back(t.ms());
                if (!evaluated || !allOk(*evaluated)) {
                    std::fprintf(stderr, "error: realistic part cold evaluation failed\n");
                    return 1;
                }
                computed = evaluated->computedCount;
            }
            results.push_back({"realistic_100_cold", median(times), computed});
        }
        const auto realisticTimeline = timelineOf(part);
        recompute::Evaluator realisticWarm;
        if (auto primed = realisticWarm.evaluate(realisticTimeline, &part.table); !primed || !allOk(*primed)) {
            std::fprintf(stderr, "error: realistic part priming failed\n");
            return 1;
        }
        auto editRealistic = [&](const std::string& param, const char* a, const char* b, const std::string& name) -> bool {
            std::vector<double> times;
            std::size_t computed = 0;
            for (int r = 0; r < kReps; ++r) {
                if (!part.table.setExpression(param, r % 2 == 0 ? a : b)) return false;
                Timer t;
                auto evaluated = realisticWarm.evaluate(realisticTimeline, &part.table);
                times.push_back(t.ms());
                if (!evaluated || !allOk(*evaluated)) {
                    std::fprintf(stderr, "error: realistic re-evaluation (%s) failed\n", name.c_str());
                    return false;
                }
                computed = evaluated->computedCount;
            }
            results.push_back({name, median(times), computed});
            return true;
        };
        if (!editRealistic("pd", "5 mm", "4 mm", "realistic_edit_early_pd")) return 1;
        if (!editRealistic("hdl", "3 mm", "2.5 mm", "realistic_edit_late_hdl")) return 1;

        std::filesystem::remove_all(dir);
        FILE* json = ::fdopen(::dup(guard.original()), "w");
        if (!json) {
            std::fprintf(stderr, "error: cannot open JSON output\n");
            return 1;
        }
        writeJson(json, results, gpuName());
        std::fclose(json);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
