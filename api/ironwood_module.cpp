// SPDX-License-Identifier: GPL-3.0-or-later
// B5: headless Python bindings. The module `ironwood` wraps features::Document (timeline, parameters, undo, evaluation,
// persistence), sketch building and solving, the feature constructors and kernel measurement / exchange. No Qt.
//
// Conventions: lengths are mm, angles are degrees in the Python API (expressions are passed through as strings).
// Lengths and angles accept either a number (mm / deg) or an expression string such as "w * 2" or "2 in".
// Errors are raised as IronwoodError subclasses carrying the C++ message.

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <format>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "features/construction_features.hpp"
#include "features/document.hpp"
#include "features/edge_features.hpp"
#include "features/extrude_feature.hpp"
#include "features/hole_feature.hpp"
#include "features/import_feature.hpp"
#include "features/move_feature.hpp"
#include "features/revolve_feature.hpp"
#include "features/sketch_feature.hpp"
#include "kernel/exchange.hpp"
#include "kernel/ops.hpp"
#include "kernel/shape.hpp"
#include "naming/element_name.hpp"
#include "naming/naming_map.hpp"
#include "naming/resolve.hpp"
#include "recompute/feature.hpp"
#include "recompute/timeline.hpp"
#include "sketch/profile.hpp"
#include "sketch/sketch.hpp"
#include "sketch/solver/bridge.hpp"

namespace py = pybind11;
namespace fe = iw::features;
namespace sk = iw::sketch;
namespace nm = iw::naming;
namespace kn = iw::kernel;
namespace rc = iw::recompute;
using iw::BodyId;
using iw::ErrorCode;
using iw::FeatureId;
using iw::Result;
using iw::Vec3;

namespace {

// ---- errors ------------------------------------------------------------------------------------------------------

/// A failure carried across the C++ / Python boundary (translated by the exception translator below).
struct ApiError : std::runtime_error {
    ErrorCode code;
    ApiError(ErrorCode c, const std::string& msg) : std::runtime_error(msg), code(c) {}
};

[[noreturn]] void fail(ErrorCode code, const std::string& msg) { throw ApiError(code, msg); }

template <class T> T unwrap(Result<T> r) {
    if (!r) throw ApiError(r.error().code, r.error().message);
    return std::move(*r);
}
void unwrapVoid(const Result<void>& r) {
    if (!r) throw ApiError(r.error().code, r.error().message);
}

/// Python exception class per ErrorCode. Filled at module init; the objects are owned by the module.
std::map<ErrorCode, PyObject*>& excClasses() {
    static std::map<ErrorCode, PyObject*> table;
    return table;
}

// ---- model state -------------------------------------------------------------------------------------------------

/// One open document plus a cache of its last evaluation. Any change through features::Document invalidates the cache
/// via its listener, so body measurements always reflect the current timeline.
struct DocModel {
    fe::Document doc;
    std::optional<fe::DocumentEvaluation> cache;

    DocModel() {
        doc.addListener([this] { cache.reset(); });
    }
    const fe::DocumentEvaluation& evaluated() {
        if (!cache) cache = unwrap(doc.evaluate());
        return *cache;
    }
};
using ModelPtr = std::shared_ptr<DocModel>;

struct BodyH {
    ModelPtr m;
    BodyId id;
};

struct FeatureH {
    ModelPtr m;
    FeatureId id;
};

struct EdgeH {
    nm::ElementRef ref;
    BodyId body;
    std::array<double, 3> centroid{};
    double length = 0;
    std::string kind; // line / circle / ellipse / bspline / other (kernel::EdgeGeometry)
    std::array<double, 3> point{}, direction{};
    double radius = 0, minorRadius = 0;
    bool closed = false;
};

struct VertexH {
    nm::ElementRef ref;
    BodyId body;
    std::array<double, 3> position{};
};

struct FaceH {
    ModelPtr m;
    nm::ElementRef ref;
    BodyId body;
    std::array<double, 3> centroid{};
    double area = 0;
    std::string kind; // plane / cylinder / cone / other
    std::array<double, 3> normal{};
    bool planar = false;
    kn::Plane plane{};
};

struct SketchH {
    ModelPtr m; // optional: used for parameter-aware solving
    sk::Sketch sketch;
    std::optional<fe::PlaneRef> attach;
};

struct SolidH {
    kn::Shape shape;
    std::string name;
};

struct ParamH {
    std::string name, expression, unit, comment, dimension, error;
    bool favourite = false;
    std::optional<double> value;
};

struct StatusH {
    FeatureId id;
    std::string state, error;
    ErrorCode errorCode = ErrorCode::Internal;
    std::vector<std::string> diagnostics;
    std::map<std::string, std::string> attributes;
};

struct EvaluationH {
    ModelPtr m;
    std::vector<StatusH> statuses;
    std::size_t computed = 0, reused = 0;
    bool ok = true;
    std::vector<BodyId> bodies;
};

// ---- small helpers -------------------------------------------------------------------------------------------------

std::string numberOr(const py::handle& v, const char* unit) {
    if (py::isinstance<py::str>(v)) return v.cast<std::string>();
    return std::format("{:.17g} {}", v.cast<double>(), unit);
}
std::string lengthExpr(const py::handle& v) { return numberOr(v, "mm"); }
std::string angleExpr(const py::handle& v) { return numberOr(v, "deg"); }

std::string vecStr(const Vec3& v) { return std::format("({:.6g}, {:.6g}, {:.6g})", v.x, v.y, v.z); }
std::array<double, 3> toArray(const Vec3& v) { return {v.x, v.y, v.z}; }
Vec3 toVec(const std::array<double, 3>& a) { return {a[0], a[1], a[2]}; }

std::string statusName(rc::FeatureStatus::State s) {
    switch (s) {
    case rc::FeatureStatus::State::Ok: return "ok";
    case rc::FeatureStatus::State::Suppressed: return "suppressed";
    case rc::FeatureStatus::State::Failed: return "failed";
    case rc::FeatureStatus::State::Skipped: return "skipped";
    }
    return "unknown";
}

std::string edgeKindName(kn::EdgeGeometry::Kind k) {
    switch (k) {
    case kn::EdgeGeometry::Kind::Line: return "line";
    case kn::EdgeGeometry::Kind::Circle: return "circle";
    case kn::EdgeGeometry::Kind::Ellipse: return "ellipse";
    case kn::EdgeGeometry::Kind::BSpline: return "bspline";
    case kn::EdgeGeometry::Kind::Other: return "other";
    }
    return "other";
}

const kn::Shape& shapeOf(DocModel& m, BodyId id) {
    const auto& ev = m.evaluated();
    auto it = ev.bodies.find(id);
    if (it == ev.bodies.end())
        fail(ErrorCode::NotFound, "body " + std::to_string(id.value()) + " does not exist at the end of the timeline");
    return it->second;
}

const nm::NamingMap& namesOf(DocModel& m, BodyId id) {
    const auto& ev = m.evaluated();
    auto it = ev.namings.find(id);
    if (it == ev.namings.end()) fail(ErrorCode::Internal, "body " + std::to_string(id.value()) + " has no naming map");
    return it->second;
}

std::vector<EdgeH> edgesOf(DocModel& m, BodyId id) {
    const kn::Shape& s = shapeOf(m, id);
    const nm::NamingMap& names = namesOf(m, id);
    std::vector<EdgeH> out;
    for (std::uint32_t i = 0; i < s.subShapeCount(kn::SubShapeKind::Edge); ++i) {
        auto name = names.nameOf(nm::ElementType::Edge, i);
        if (!name) continue;
        const auto key = s.subShapeKey(kn::SubShapeKind::Edge, i);
        EdgeH e;
        e.ref = nm::ElementRef{name->feature, nm::ElementType::Edge, *name};
        e.body = id;
        e.centroid = {key[0], key[1], key[2]};
        e.length = key[3];
        const kn::EdgeGeometry g = s.edgeGeometry(i);
        e.kind = edgeKindName(g.kind);
        e.point = toArray(g.point);
        e.direction = toArray(g.direction);
        e.radius = g.radius;
        e.minorRadius = g.minorRadius;
        e.closed = g.closed;
        out.push_back(std::move(e));
    }
    return out;
}

std::vector<VertexH> verticesOf(DocModel& m, BodyId id) {
    const kn::Shape& s = shapeOf(m, id);
    const nm::NamingMap& names = namesOf(m, id);
    std::vector<VertexH> out;
    for (std::uint32_t i = 0; i < s.subShapeCount(kn::SubShapeKind::Vertex); ++i) {
        auto name = names.nameOf(nm::ElementType::Vertex, i);
        if (!name) continue;
        const auto key = s.subShapeKey(kn::SubShapeKind::Vertex, i);
        out.push_back(VertexH{nm::ElementRef{name->feature, nm::ElementType::Vertex, *name}, id, {key[0], key[1], key[2]}});
    }
    return out;
}

std::vector<FaceH> facesOf(const ModelPtr& mp, BodyId id) {
    const kn::Shape& s = shapeOf(*mp, id);
    const nm::NamingMap& names = namesOf(*mp, id);
    std::vector<FaceH> out;
    for (std::uint32_t i = 0; i < s.subShapeCount(kn::SubShapeKind::Face); ++i) {
        auto name = names.nameOf(nm::ElementType::Face, i);
        if (!name) continue;
        const auto key = s.subShapeKey(kn::SubShapeKind::Face, i);
        FaceH f;
        f.m = mp;
        f.ref = nm::ElementRef{name->feature, nm::ElementType::Face, *name};
        f.body = id;
        f.centroid = {key[0], key[1], key[2]};
        f.area = key[3];
        const kn::FaceGeometry g = s.faceGeometry(i);
        switch (g.kind) {
        case kn::FaceGeometry::Kind::Plane: f.kind = "plane"; break;
        case kn::FaceGeometry::Kind::Cylinder: f.kind = "cylinder"; break;
        case kn::FaceGeometry::Kind::Cone: f.kind = "cone"; break;
        case kn::FaceGeometry::Kind::Other: f.kind = "other"; break;
        }
        f.normal = toArray(g.normal);
        if (auto p = kn::planeFromFace(s, i)) {
            f.planar = true;
            f.plane = *p;
        }
        out.push_back(std::move(f));
    }
    return out;
}

sk::PointRole roleOf(const std::string& r) {
    if (r == "start") return sk::PointRole::Start;
    if (r == "end") return sk::PointRole::End;
    if (r == "center" || r == "centre") return sk::PointRole::Center;
    if (r == "mid") return sk::PointRole::Mid;
    fail(ErrorCode::InvalidArgument, "unknown point role '" + r + "' (use start, end, center, mid)");
}

sk::SketchRef refOf(const py::handle& h) {
    if (py::isinstance<py::tuple>(h)) {
        auto t = h.cast<py::tuple>();
        if (t.size() != 2) fail(ErrorCode::InvalidArgument, "a reference tuple is (entity_id, role)");
        return sk::SketchRef{sk::SketchEntityId{t[0].cast<std::uint64_t>()}, roleOf(t[1].cast<std::string>())};
    }
    return sk::SketchRef{sk::SketchEntityId{h.cast<std::uint64_t>()}, std::nullopt};
}

std::vector<sk::SketchRef> refsOf(const py::iterable& refs) {
    std::vector<sk::SketchRef> out;
    for (const auto& h : refs) out.push_back(refOf(h));
    return out;
}

sk::ConstraintKind constraintKindOf(const std::string& s) {
    static const std::map<std::string, sk::ConstraintKind> table{
        {"coincident", sk::ConstraintKind::Coincident}, {"horizontal", sk::ConstraintKind::Horizontal},
        {"vertical", sk::ConstraintKind::Vertical},     {"parallel", sk::ConstraintKind::Parallel},
        {"perpendicular", sk::ConstraintKind::Perpendicular}, {"tangent", sk::ConstraintKind::Tangent},
        {"equal", sk::ConstraintKind::Equal},           {"concentric", sk::ConstraintKind::Concentric},
        {"midpoint", sk::ConstraintKind::Midpoint},     {"collinear", sk::ConstraintKind::Collinear},
        {"symmetric", sk::ConstraintKind::Symmetric},   {"fix", sk::ConstraintKind::Fix}};
    auto it = table.find(s);
    if (it == table.end()) fail(ErrorCode::InvalidArgument, "unknown constraint kind '" + s + "'");
    return it->second;
}

sk::DimensionKind dimensionKindOf(const std::string& s) {
    static const std::map<std::string, sk::DimensionKind> table{
        {"distance", sk::DimensionKind::Distance}, {"distance_x", sk::DimensionKind::DistanceX},
        {"distance_y", sk::DimensionKind::DistanceY}, {"radius", sk::DimensionKind::Radius},
        {"diameter", sk::DimensionKind::Diameter}, {"angle", sk::DimensionKind::Angle}};
    auto it = table.find(s);
    if (it == table.end()) fail(ErrorCode::InvalidArgument, "unknown dimension kind '" + s + "'");
    return it->second;
}

sk::SketchPlane presetPlane(const std::string& name, const Vec3& origin) {
    // Sketch y = normal x xDir. XY: y = +Y. XZ: y = +Z. YZ: y = +Z.
    Vec3 normal, xDir;
    if (name == "XY") { normal = {0, 0, 1}; xDir = {1, 0, 0}; }
    else if (name == "XZ") { normal = {0, -1, 0}; xDir = {1, 0, 0}; }
    else if (name == "YZ") { normal = {1, 0, 0}; xDir = {0, 1, 0}; }
    else fail(ErrorCode::InvalidArgument, "unknown plane '" + name + "' (use XY, XZ, YZ, or normal= and x_dir=)");
    return unwrap(sk::SketchPlane::make(origin, normal, xDir));
}

std::vector<std::string> profileNamesOf(const sk::Sketch& s) {
    std::vector<std::string> names;
    for (const auto& p : unwrap(sk::detectProfiles(s))) names.push_back(p.name);
    return names;
}

std::string joinNames(const std::vector<std::string>& names) {
    std::string out;
    for (const auto& n : names) out += (out.empty() ? "" : ", ") + n;
    return out.empty() ? "(none)" : out;
}

/// Profile name for an extrude / revolve: explicit, or the only profile of the sketch.
std::string chooseProfile(const sk::Sketch& s, const std::optional<std::string>& profile) {
    const auto names = profileNamesOf(s);
    if (profile) {
        if (std::find(names.begin(), names.end(), *profile) == names.end())
            fail(ErrorCode::NotFound, "sketch has no profile '" + *profile + "' (profiles: " + joinNames(names) + ")");
        return *profile;
    }
    if (names.size() == 1) return names.front();
    fail(ErrorCode::InvalidArgument,
         "pass profile=<name>; the sketch has " + std::to_string(names.size()) + " profiles: " + joinNames(names));
}

fe::ExtrudeParams::Operation extrudeOpOf(const std::string& s) {
    if (s == "new_body") return fe::ExtrudeParams::Operation::NewBody;
    if (s == "join") return fe::ExtrudeParams::Operation::Join;
    if (s == "cut") return fe::ExtrudeParams::Operation::Cut;
    if (s == "intersect") return fe::ExtrudeParams::Operation::Intersect;
    fail(ErrorCode::InvalidArgument, "operation must be new_body, join, cut or intersect");
}

fe::common::BodyOp bodyOpOf(const std::string& s) {
    if (s == "new_body") return fe::common::BodyOp::NewBody;
    if (s == "join") return fe::common::BodyOp::Join;
    if (s == "cut") return fe::common::BodyOp::Cut;
    if (s == "intersect") return fe::common::BodyOp::Intersect;
    fail(ErrorCode::InvalidArgument, "operation must be new_body, join, cut or intersect");
}

/// Refuses to add a feature that needs an existing body when none was given.
BodyId targetBody(DocModel& m, const std::string& op, const std::optional<BodyH>& body) {
    if (op == "new_body") return m.doc.allocateBodyId();
    if (!body) fail(ErrorCode::InvalidArgument, "operation '" + op + "' needs body=<Body> to combine with");
    return body->id;
}

/// Sketch-feature handle: the solved-or-stored sketch of a SketchFeature on the timeline.
const fe::SketchFeature& sketchFeatureOf(DocModel& m, const FeatureH& f) {
    auto idx = m.doc.timeline().indexOf(f.id);
    if (!idx) fail(ErrorCode::NotFound, "feature " + std::to_string(f.id.value()) + " is not on the timeline");
    const auto* sf = dynamic_cast<const fe::SketchFeature*>(m.doc.timeline().at(*idx).get());
    if (!sf) fail(ErrorCode::InvalidArgument, "feature " + std::to_string(f.id.value()) + " is not a sketch");
    return *sf;
}

/// Applies a Body.edges() / faces() filter: None keeps everything, a string keeps elements of that kind (several kinds
/// separated by '|', e.g. "line|circle"), a callable is called with the reference.
template <class H> bool passesFilter(const py::object& filter, const H& h, const char* what, const std::vector<std::string>& kinds) {
    if (filter.is_none()) return true;
    if (py::isinstance<py::str>(filter)) {
        const std::string spec = filter.cast<std::string>();
        bool match = false;
        std::size_t start = 0;
        for (;;) {
            const std::size_t bar = spec.find('|', start);
            const std::string k = spec.substr(start, bar == std::string::npos ? std::string::npos : bar - start);
            if (std::find(kinds.begin(), kinds.end(), k) == kinds.end())
                fail(ErrorCode::InvalidArgument, std::string("unknown ") + what + " kind '" + k + "' (use " + joinNames(kinds) + ")");
            if (k == h.kind) match = true;
            if (bar == std::string::npos) break;
            start = bar + 1;
        }
        return match;
    }
    return filter(h).template cast<bool>();
}

/// A plane input of a construction feature: a planar FaceRef or a construction Plane feature.
fe::PlaneRef planeRefOf(const py::object& base, const char* what) {
    if (py::isinstance<FaceH>(base)) {
        const FaceH f = base.cast<FaceH>();
        if (!f.planar) fail(ErrorCode::InvalidArgument, std::string(what) + " must be a planar face");
        return fe::PlaneRef{true, f.body, f.ref, FeatureId{}};
    }
    if (py::isinstance<FeatureH>(base)) return fe::PlaneRef{false, BodyId{}, nm::ElementRef{}, base.cast<FeatureH>().id};
    fail(ErrorCode::InvalidArgument, std::string(what) + " must be a planar FaceRef or a construction Plane feature");
}

/// Three length expressions from a 3-sequence of numbers (mm) or expression strings.
std::array<std::string, 3> pointExprs(const py::handle& h) {
    const auto seq = h.cast<py::sequence>();
    if (seq.size() != 3) fail(ErrorCode::InvalidArgument, "a point is (x, y, z)");
    return {lengthExpr(seq[0]), lengthExpr(seq[1]), lengthExpr(seq[2])};
}

/// Adds `feature` at the end of the timeline and returns its handle.
FeatureH append(const ModelPtr& m, std::shared_ptr<const rc::Feature> feature) {
    const FeatureId id = feature->id();
    unwrapVoid(m->doc.addFeature(std::move(feature)));
    return FeatureH{m, id};
}

}  // namespace

namespace {

/// Parses a 3-sequence (point / vector) argument.
std::array<double, 3> vec3Arg(const py::handle& h) { return h.cast<std::array<double, 3>>(); }

const std::uint64_t kSketchOriginId = sk::kOriginPointId.value();
const std::uint64_t kSketchXAxisId = sk::kXAxisId.value();
const std::uint64_t kSketchYAxisId = sk::kYAxisId.value();

/// Shape / size options shared by Document.hole and Document.hole_from_sketch.
void setHoleOptions(fe::HoleParams& p, py::object diameter, py::object depth, bool through, const std::string& kind,
                    py::object drillAngle, py::object cbDiameter, py::object cbDepth, py::object csDiameter,
                    py::object csAngle, const std::string& thread) {
    if (kind == "simple") p.kind = kn::HoleKind::Simple;
    else if (kind == "counterbore") p.kind = kn::HoleKind::Counterbore;
    else if (kind == "countersink") p.kind = kn::HoleKind::Countersink;
    else fail(ErrorCode::InvalidArgument, "kind must be simple, counterbore or countersink");
    p.diameter = lengthExpr(diameter);
    if (!through) {
        if (depth.is_none()) fail(ErrorCode::InvalidArgument, "give depth=... or through=True");
        p.depth = lengthExpr(depth);
    }
    p.through = through;
    p.drillAngle = angleExpr(drillAngle);
    p.counterboreDiameter = lengthExpr(cbDiameter);
    p.counterboreDepth = lengthExpr(cbDepth);
    p.countersinkDiameter = lengthExpr(csDiameter);
    p.countersinkAngle = angleExpr(csAngle);
    p.thread = thread;
}

/// The body that all edges belong to (fillet / chamfer); fails on an empty or mixed selection.
BodyId commonBodyOf(const std::vector<EdgeH>& edges) {
    if (edges.empty()) fail(ErrorCode::InvalidArgument, "select at least one edge");
    for (const auto& e : edges)
        if (e.body != edges.front().body) fail(ErrorCode::InvalidArgument, "all edges must belong to the same body");
    return edges.front().body;
}

std::vector<nm::ElementRef> edgeRefs(const std::vector<EdgeH>& edges) {
    std::vector<nm::ElementRef> refs;
    for (const auto& e : edges) refs.push_back(e.ref);
    return refs;
}

struct SolveReportH {
    sk::SolverStatus status = sk::SolverStatus::Failed;
    std::string message;
    int dof = -1;
    std::vector<std::uint64_t> conflictingConstraints, conflictingDimensions, redundantConstraints, redundantDimensions;
};

template <class C, class G> void addMeasures(py::class_<C>& cls, G get) {
    cls.def_property_readonly("volume", [get](const C& c) { return unwrap(kn::massProperties(get(c))).volume; },
                              "Volume in mm^3.")
        .def_property_readonly("area", [get](const C& c) { return unwrap(kn::massProperties(get(c))).surfaceArea; },
                               "Surface area in mm^2.")
        .def_property_readonly(
            "centre_of_mass",
            [get](const C& c) { return toArray(unwrap(kn::massProperties(get(c))).centerOfMass); },
            "Centre of mass (x, y, z) in mm, unit density.")
        .def_property_readonly(
            "bounding_box",
            [get](const C& c) {
                const iw::Box3 b = get(c).boundingBox();
                return py::make_tuple(toArray(b.min), toArray(b.max));
            },
            "((min_x, min_y, min_z), (max_x, max_y, max_z)) in mm.")
        .def_property_readonly("face_count", [get](const C& c) { return get(c).subShapeCount(kn::SubShapeKind::Face); })
        .def_property_readonly("edge_count", [get](const C& c) { return get(c).subShapeCount(kn::SubShapeKind::Edge); })
        .def_property_readonly("vertex_count",
                               [get](const C& c) { return get(c).subShapeCount(kn::SubShapeKind::Vertex); });
}

/// Feature object on the timeline.
const rc::Feature& featureOf(DocModel& m, FeatureId id) {
    auto idx = m.doc.timeline().indexOf(id);
    if (!idx) fail(ErrorCode::NotFound, "feature " + std::to_string(id.value()) + " is not on the timeline");
    return *m.doc.timeline().at(*idx);
}

std::string featureStatusOf(DocModel& m, FeatureId id) {
    if (!m.cache) return "unevaluated";
    for (const auto& s : m.cache->statuses)
        if (s.id == id) return statusName(s.state);
    return "unknown";
}

/// Writes every body in `bodies` (default: all bodies at the end of the timeline) to `path`; the format is taken from
/// the extension: .step/.stp, .stl (exactly one body), .3mf.
void exportBodies(DocModel& m, const std::filesystem::path& path, std::vector<BodyId> bodies) {
    const auto& ev = m.evaluated();
    if (bodies.empty())
        for (const auto& entry : ev.bodies) bodies.push_back(entry.first);
    if (bodies.empty()) fail(ErrorCode::InvalidArgument, "there are no bodies to export");
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext == ".step" || ext == ".stp") {
        kn::ImportedAssembly asmb;
        for (BodyId id : bodies) {
            kn::AssemblyNode node;
            node.name = "body_" + std::to_string(id.value());
            node.shape = shapeOf(m, id);
            asmb.roots.push_back(static_cast<std::uint32_t>(asmb.nodes.size()));
            asmb.nodes.push_back(std::move(node));
        }
        asmb.lengthUnitInFile = "mm";
        unwrapVoid(kn::exportStep(path, asmb));
    } else if (ext == ".stl") {
        if (bodies.size() != 1) fail(ErrorCode::InvalidArgument, "STL export takes exactly one body; pass bodies=[...]");
        unwrapVoid(kn::exportStl(path, shapeOf(m, bodies.front())));
    } else if (ext == ".3mf") {
        std::vector<kn::MeshBody> meshes;
        for (BodyId id : bodies) meshes.push_back(kn::MeshBody{"body_" + std::to_string(id.value()), shapeOf(m, id), std::nullopt});
        unwrapVoid(kn::export3mf(path, meshes));
    } else {
        fail(ErrorCode::InvalidArgument, "unsupported export format '" + ext + "' (use .step, .stp, .stl or .3mf)");
    }
}

}  // namespace

PYBIND11_MODULE(ironwood, m) {
    m.doc() = R"doc(Headless Python API for Ironwood (no GUI).

Build a model with a Document: parameters, sketches (add_sketch), features (extrude, revolve, fillet, chamfer, hole,
move, offset_plane, midplane, plane_through_points, plane_at_angle, axis_*, point_at_*, import_step), then evaluate() for statuses and bodies. Bodies expose volume, area, centre_of_mass, bounding_box,
face_count, edge_count, vertex_count, and faces()/edges() queries returning references usable by features.
Lengths are mm and angles are degrees; pass numbers or expression strings such as "w * 2" or "2 in".)doc";

    // ---- exceptions: IronwoodError is the base; the C++ message is the Python message ----
    static auto* keep = new std::vector<py::object>();  // lives as long as the interpreter
    auto makeExc = [&m](const char* name, py::object base) {
        const std::string qualified = std::string("ironwood.") + name;
        py::object cls = py::reinterpret_steal<py::object>(PyErr_NewException(qualified.c_str(), base.ptr(), nullptr));
        if (!cls) throw py::error_already_set();
        m.attr(name) = cls;
        keep->push_back(cls);
        return cls;
    };
    py::object baseErr = makeExc("IronwoodError", py::reinterpret_borrow<py::object>(PyExc_Exception));
    py::object valueErr = py::make_tuple(baseErr, py::reinterpret_borrow<py::object>(PyExc_ValueError));
    py::object lookupErr = py::make_tuple(baseErr, py::reinterpret_borrow<py::object>(PyExc_LookupError));
    excClasses()[ErrorCode::NotImplemented] = baseErr.ptr();
    excClasses()[ErrorCode::Internal] = baseErr.ptr();
    excClasses()[ErrorCode::InvalidArgument] = makeExc("InvalidArgumentError", valueErr).ptr();
    excClasses()[ErrorCode::ParseError] = makeExc("ParseError", valueErr).ptr();
    excClasses()[ErrorCode::NotFound] = makeExc("NotFoundError", lookupErr).ptr();
    excClasses()[ErrorCode::Ambiguous] = makeExc("AmbiguousError", baseErr).ptr();
    excClasses()[ErrorCode::IoError] = makeExc("IoError", baseErr).ptr();
    excClasses()[ErrorCode::Cancelled] = makeExc("CancelledError", baseErr).ptr();
    excClasses()[ErrorCode::KernelFailure] = makeExc("KernelError", baseErr).ptr();
    py::register_exception_translator([](std::exception_ptr p) {
        try {
            if (p) std::rethrow_exception(p);
        } catch (const ApiError& e) {
            auto it = excClasses().find(e.code);
            PyErr_SetString(it != excClasses().end() ? it->second : PyExc_RuntimeError, e.what());
        }
    });

    // ---- Document ----
    py::class_<DocModel, ModelPtr>(m, "Document", "A parametric document: parameters, sketches, timeline, undo, persistence.")
        .def(py::init([](const std::string& name) {
                 auto d = std::make_shared<DocModel>();
                 if (!name.empty()) d->doc.setName(name);
                 return d;
             }),
             py::arg("name") = "")
        .def_static("open", [](const std::string& path) {
            auto d = std::make_shared<DocModel>();
            unwrapVoid(d->doc.load(path));
            d->cache.reset();
            return d;
        }, py::arg("path"), "Open a document directory written by save().")
        .def("save", [](DocModel& d, const std::string& path) { unwrapVoid(d.doc.save(path)); }, py::arg("path"),
             "Write the document directory (marks it clean).")
        .def_property("name", [](const DocModel& d) { return d.doc.name(); },
                      [](DocModel& d, const std::string& n) { d.doc.setName(n); })
        .def_property_readonly("is_clean", [](const DocModel& d) { return d.doc.isClean(); })
        // parameters
        .def("add_parameter", [](DocModel& d, const std::string& name, const std::string& expression, const std::string& unit,
                                 const std::string& comment) { unwrapVoid(d.doc.addParameter(name, expression, unit, comment)); },
             py::arg("name"), py::arg("expression"), py::arg("unit") = "", py::arg("comment") = "",
             "Add a parameter; expression is e.g. '40 mm' or '2 * w'.")
        .def("set_parameter", [](DocModel& d, const std::string& name, const std::string& expression) {
                 unwrapVoid(d.doc.setParameterExpression(name, expression));
             },
             py::arg("name"), py::arg("expression"), "Change a parameter's expression (undoable).")
        .def_property_readonly("parameters",
                               [](DocModel& d) {
                                   std::vector<ParamH> out;
                                   for (const auto& p : d.doc.parameters().parameters()) {
                                       ParamH h;
                                       h.name = p.name;
                                       h.expression = p.expression.source();
                                       h.unit = p.unit;
                                       h.comment = p.comment;
                                       h.favourite = p.favourite;
                                       auto q = d.doc.parameters().evaluate(p.name);
                                       if (q) {
                                           h.value = q->value;
                                           h.dimension = q->dim == iw::Dimension::Length ? "length"
                                                         : q->dim == iw::Dimension::Angle ? "angle"
                                                                                          : "unitless";
                                       } else {
                                           h.error = q.error().message;
                                       }
                                       out.push_back(std::move(h));
                                   }
                                   return out;
                               },
                               "All parameters with their evaluated value (mm, rad or unitless).")
        // sketches
        .def("add_sketch", [](const ModelPtr& d, const SketchH& s) {
                 fe::SketchFeatureParams p{s.sketch, s.attach};
                 return append(d, std::make_shared<fe::SketchFeature>(d->doc.allocateFeatureId(), p));
             },
             py::arg("sketch"), "Add a Sketch to the end of the timeline; returns its Feature.")
        .def("edit_sketch", [](DocModel& d, const FeatureH& f, const SketchH& s) {
                 fe::SketchFeatureParams p{s.sketch, s.attach};
                 unwrapVoid(d.doc.editFeature(std::make_shared<fe::SketchFeature>(f.id, p)));
                 return f;
             },
             py::arg("feature"), py::arg("sketch"), "Replace the sketch of a sketch feature (undoable).")
        // features
        .def("extrude", [](const ModelPtr& mp, const FeatureH& sketchFeature, std::optional<std::string> profile,
                           py::object distance, const std::string& extent, py::object distance2, const std::string& operation,
                           std::optional<BodyH> body) {
                 DocModel& d = *mp;
                 const std::string prof = chooseProfile(sketchFeatureOf(d, sketchFeature).params().sketch, profile);
                 fe::ExtrudeParams p;
                 p.source = fe::SketchReferenceSource{sketchFeature.id, prof};
                 if (extent == "one_side") p.extent = fe::ExtrudeParams::Extent::OneSide;
                 else if (extent == "two_sides") p.extent = fe::ExtrudeParams::Extent::TwoSides;
                 else if (extent == "symmetric") p.extent = fe::ExtrudeParams::Extent::Symmetric;
                 else fail(ErrorCode::InvalidArgument, "extent must be one_side, two_sides or symmetric");
                 p.distance = lengthExpr(distance);
                 p.distance2 = lengthExpr(distance2);
                 p.operation = extrudeOpOf(operation);
                 p.body = targetBody(d, operation, body);
                 return append(mp, std::make_shared<fe::ExtrudeFeature>(d.doc.allocateFeatureId(), p));
             },
             py::arg("sketch"), py::arg("profile") = py::none(), py::arg("distance") = py::str("10 mm"),
             py::kw_only(), py::arg("extent") = "one_side", py::arg("distance2") = py::str("0 mm"),
             py::arg("operation") = "new_body", py::arg("body") = py::none(),
             "Extrude a sketch profile. extent: one_side, two_sides or symmetric. operation: new_body, join, cut or intersect.")
        .def("revolve", [](const ModelPtr& mp, const FeatureH& sketchFeature, int axisLine, std::optional<std::string> profile,
                           py::object angle, const std::string& extent, py::object angle2, const std::string& operation,
                           std::optional<BodyH> body) {
                 DocModel& d = *mp;
                 const std::string prof = chooseProfile(sketchFeatureOf(d, sketchFeature).params().sketch, profile);
                 fe::RevolveParams p;
                 p.source = fe::SketchReferenceSource{sketchFeature.id, prof};
                 p.axis.kind = fe::RevolveAxis::Kind::SketchLine;
                 p.axis.line = sk::SketchEntityId{static_cast<std::uint64_t>(axisLine)};
                 if (extent == "one_side") p.extent = fe::RevolveParams::Extent::OneSide;
                 else if (extent == "two_sides") p.extent = fe::RevolveParams::Extent::TwoSides;
                 else if (extent == "symmetric") p.extent = fe::RevolveParams::Extent::Symmetric;
                 else if (extent == "full") p.extent = fe::RevolveParams::Extent::Full;
                 else fail(ErrorCode::InvalidArgument, "extent must be one_side, two_sides, symmetric or full");
                 p.angle = angleExpr(angle);
                 p.angle2 = angleExpr(angle2);
                 p.operation = bodyOpOf(operation);
                 p.body = targetBody(d, operation, body);
                 return append(mp, std::make_shared<fe::RevolveFeature>(d.doc.allocateFeatureId(), p));
             },
             py::arg("sketch"), py::arg("axis_line"), py::arg("profile") = py::none(), py::arg("angle") = py::str("360 deg"),
             py::kw_only(), py::arg("extent") = "full", py::arg("angle2") = py::str("0 deg"),
             py::arg("operation") = "new_body", py::arg("body") = py::none(),
             "Revolve a sketch profile about a sketch line (axis_line = its entity id). extent defaults to a full turn.")
        .def("fillet", [](const ModelPtr& mp, const std::vector<EdgeH>& edges, py::object radius, bool tangentChain) {
                 fe::FilletParams p;
                 p.body = commonBodyOf(edges);
                 p.edges = edgeRefs(edges);
                 p.radius = lengthExpr(radius);
                 p.tangentChain = tangentChain;
                 return append(mp, std::make_shared<fe::FilletFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("edges"), py::arg("radius") = py::str("1 mm"), py::kw_only(), py::arg("tangent_chain") = true,
             "Constant-radius fillet of edges (EdgeRef list from Body.edges()).")
        .def("chamfer", [](const ModelPtr& mp, const std::vector<EdgeH>& edges, py::object distance, py::object distance2,
                           py::object angle, bool tangentChain) {
                 fe::ChamferParams p;
                 p.body = commonBodyOf(edges);
                 p.edges = edgeRefs(edges);
                 p.distance = lengthExpr(distance);
                 p.tangentChain = tangentChain;
                 if (!angle.is_none()) {
                     p.mode = fe::ChamferParams::Mode::DistanceAngle;
                     p.angle = angleExpr(angle);
                 } else if (!distance2.is_none()) {
                     p.mode = fe::ChamferParams::Mode::TwoDistances;
                     p.distance2 = lengthExpr(distance2);
                 }
                 return append(mp, std::make_shared<fe::ChamferFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("edges"), py::arg("distance") = py::str("1 mm"), py::arg("distance2") = py::none(),
             py::arg("angle") = py::none(), py::kw_only(), py::arg("tangent_chain") = true,
             "Chamfer of edges: equal distance, two distances (distance2) or distance + angle.")
        .def("hole", [](const ModelPtr& mp, const FaceH& face, py::object diameter, py::object depth, bool through,
                        py::tuple at, const std::string& kind, py::object drillAngle, py::object cbDiameter,
                        py::object cbDepth, py::object csDiameter, py::object csAngle, const std::string& thread) {
                 fe::HoleParams p;
                 p.body = face.body;
                 p.face = face.ref;
                 p.u = lengthExpr(at[0]);
                 p.v = lengthExpr(at[1]);
                 setHoleOptions(p, diameter, depth, through, kind, drillAngle, cbDiameter, cbDepth, csDiameter, csAngle, thread);
                 return append(mp, std::make_shared<fe::HoleFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("face"), py::arg("diameter") = py::str("6 mm"), py::arg("depth") = py::none(), py::kw_only(),
             py::arg("through") = false, py::arg("at") = py::make_tuple(0.0, 0.0), py::arg("kind") = "simple",
             py::arg("drill_angle") = py::str("118 deg"), py::arg("counterbore_diameter") = py::str("10 mm"),
             py::arg("counterbore_depth") = py::str("3 mm"), py::arg("countersink_diameter") = py::str("12 mm"),
             py::arg("countersink_angle") = py::str("90 deg"), py::arg("thread") = "",
             "Hole on a planar face. at=(u, v) is the entry point in the face's plane (see FaceRef.local()).")
        .def("hole_from_sketch", [](const ModelPtr& mp, const FeatureH& sketchFeature, const std::vector<std::uint64_t>& points,
                                    py::object diameter, py::object depth, std::optional<BodyH> body, bool through, bool flip,
                                    const std::string& kind, py::object drillAngle, py::object cbDiameter, py::object cbDepth,
                                    py::object csDiameter, py::object csAngle, const std::string& thread) {
                 DocModel& d = *mp;
                 (void)sketchFeatureOf(d, sketchFeature); // must be a sketch on the timeline
                 if (points.empty()) fail(ErrorCode::InvalidArgument, "give at least one sketch point id");
                 fe::HoleParams p;
                 if (body) {
                     p.body = body->id;
                 } else {
                     const auto& bodies = mp->evaluated().bodies;
                     if (bodies.size() != 1)
                         fail(ErrorCode::InvalidArgument,
                              "the document has " + std::to_string(bodies.size()) + " bodies; pass body=<Body> to cut");
                     p.body = bodies.begin()->first;
                 }
                 fe::HoleSketchPlacement placement;
                 placement.sketch = sketchFeature.id;
                 for (auto id : points) placement.points.push_back(sk::SketchEntityId{id});
                 placement.flip = flip;
                 p.sketchPoints = std::move(placement);
                 setHoleOptions(p, diameter, depth, through, kind, drillAngle, cbDiameter, cbDepth, csDiameter, csAngle, thread);
                 return append(mp, std::make_shared<fe::HoleFeature>(d.doc.allocateFeatureId(), p));
             },
             py::arg("sketch"), py::arg("points"), py::arg("diameter") = py::str("6 mm"), py::arg("depth") = py::none(),
             py::kw_only(), py::arg("body") = py::none(), py::arg("through") = false, py::arg("flip") = false,
             py::arg("kind") = "simple", py::arg("drill_angle") = py::str("118 deg"),
             py::arg("counterbore_diameter") = py::str("10 mm"), py::arg("counterbore_depth") = py::str("3 mm"),
             py::arg("countersink_diameter") = py::str("12 mm"), py::arg("countersink_angle") = py::str("90 deg"),
             py::arg("thread") = "",
             "One hole per sketch point of a sketch Feature (point ids, circle/arc ids for their centre, or "
             "Sketch.ORIGIN), all in one feature. The holes follow the points when the sketch changes. Depth is from the "
             "sketch plane along its normal, into the body (flip=True reverses). body= defaults to the document's only body.")
        .def("move", [](const ModelPtr& mp, const BodyH& body, py::tuple translate, py::object angle, py::object axisOrigin,
                        py::object axisDirection, bool copy) {
                 fe::MoveParams p;
                 p.body = body.id;
                 p.copy = copy;
                 if (copy) p.newBody = mp->doc.allocateBodyId();
                 for (std::size_t i = 0; i < 3; ++i) p.translate[i] = lengthExpr(translate[i]);
                 p.angle = angleExpr(angle);
                 p.axisOrigin = toVec(vec3Arg(axisOrigin));
                 p.axisDirection = toVec(vec3Arg(axisDirection));
                 return append(mp, std::make_shared<fe::MoveFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("body"), py::arg("translate") = py::make_tuple(0.0, 0.0, 0.0), py::kw_only(),
             py::arg("angle") = py::str("0 deg"), py::arg("axis_origin") = py::make_tuple(0.0, 0.0, 0.0),
             py::arg("axis_direction") = py::make_tuple(0.0, 0.0, 1.0), py::arg("copy") = false,
             "Move (or copy) a body: rotate by angle about the axis, then translate.")
        .def("offset_plane", [](const ModelPtr& mp, py::object base, py::object distance) {
                 fe::ConstructionPlaneParams p;
                 p.kind = fe::ConstructionPlaneParams::Kind::Offset;
                 p.distance = lengthExpr(distance);
                 p.base = planeRefOf(base, "offset_plane base");
                 return append(mp, std::make_shared<fe::ConstructionPlaneFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("base"), py::arg("distance"), "Construction plane offset from a planar face or another plane.")
        .def("midplane", [](const ModelPtr& mp, py::object a, py::object b) {
                 fe::ConstructionPlaneParams p;
                 p.kind = fe::ConstructionPlaneParams::Kind::MidPlane;
                 p.base = planeRefOf(a, "midplane a");
                 p.other = planeRefOf(b, "midplane b");
                 return append(mp, std::make_shared<fe::ConstructionPlaneFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("a"), py::arg("b"), "Construction plane midway between two parallel planes (planar faces or Plane features).")
        .def("plane_through_points", [](const ModelPtr& mp, py::object a, py::object b, py::object c) {
                 fe::ConstructionPlaneParams p;
                 p.kind = fe::ConstructionPlaneParams::Kind::ThreePoints;
                 p.points = {pointExprs(a), pointExprs(b), pointExprs(c)};
                 return append(mp, std::make_shared<fe::ConstructionPlaneFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("a"), py::arg("b"), py::arg("c"),
             "Construction plane through three points (x, y, z); normal = (b - a) x (c - a). Coordinates are mm or expressions.")
        .def("plane_at_angle", [](const ModelPtr& mp, py::object base, const FeatureH& axis, py::object angle) {
                 fe::ConstructionPlaneParams p;
                 p.kind = fe::ConstructionPlaneParams::Kind::AngleAxis;
                 p.base = planeRefOf(base, "plane_at_angle base");
                 p.axis = axis.id;
                 p.angle = angleExpr(angle);
                 return append(mp, std::make_shared<fe::ConstructionPlaneFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("base"), py::arg("axis"), py::arg("angle"),
             "Construction plane through a construction Axis feature, rotated by `angle` from the base plane's orientation.")
        .def("plane_tangent", [](const ModelPtr& mp, const FaceH& face, py::object angle) {
                 if (face.kind != "cylinder" && face.kind != "cone")
                     fail(ErrorCode::InvalidArgument, "plane_tangent needs a cylindrical or conical face (got '" + face.kind + "')");
                 fe::ConstructionPlaneParams p;
                 p.kind = fe::ConstructionPlaneParams::Kind::Tangent;
                 p.base = fe::PlaneRef{true, face.body, face.ref, FeatureId{}};
                 p.angle = angleExpr(angle);
                 return append(mp, std::make_shared<fe::ConstructionPlaneFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("face"), py::arg("angle") = py::str("0 deg"),
             "Construction plane tangent to a cylindrical or conical face along the ruling at `angle` around the face's "
             "axis (for a Z-axis cylinder 0 deg touches the +X side, 90 deg the +Y side). The normal points away from the axis.")
        .def("axis_through_cylinder", [](const ModelPtr& mp, const FaceH& face) {
                 fe::ConstructionAxisParams p;
                 p.kind = fe::ConstructionAxisParams::Kind::Cylinder;
                 p.body = face.body;
                 p.ref = face.ref;
                 return append(mp, std::make_shared<fe::ConstructionAxisFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("face"), "Construction axis along a cylindrical or conical face.")
        .def("axis_through_points", [](const ModelPtr& mp, py::object a, py::object b) {
                 fe::ConstructionAxisParams p;
                 p.kind = fe::ConstructionAxisParams::Kind::TwoPoints;
                 p.a = pointExprs(a);
                 p.b = pointExprs(b);
                 return append(mp, std::make_shared<fe::ConstructionAxisFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("a"), py::arg("b"), "Construction axis from point a towards point b (mm or expressions).")
        .def("axis_along_edge", [](const ModelPtr& mp, const EdgeH& edge) {
                 fe::ConstructionAxisParams p;
                 p.kind = fe::ConstructionAxisParams::Kind::Edge;
                 p.body = edge.body;
                 p.ref = edge.ref;
                 return append(mp, std::make_shared<fe::ConstructionAxisFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("edge"), "Construction axis along a linear edge.")
        .def("axis_two_planes", [](const ModelPtr& mp, py::object a, py::object b) {
                 fe::ConstructionAxisParams p;
                 p.kind = fe::ConstructionAxisParams::Kind::TwoPlanes;
                 p.planeA = planeRefOf(a, "axis_two_planes a");
                 p.planeB = planeRefOf(b, "axis_two_planes b");
                 return append(mp, std::make_shared<fe::ConstructionAxisFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("a"), py::arg("b"),
             "Construction axis on the intersection line of two planes (planar faces or Plane features); direction "
             "a.normal x b.normal. Parallel planes fail the feature.")
        .def("point_at_vertex", [](const ModelPtr& mp, const VertexH& vertex) {
                 fe::ConstructionPointParams p;
                 p.kind = fe::ConstructionPointParams::Kind::Vertex;
                 p.body = vertex.body;
                 p.ref = vertex.ref;
                 return append(mp, std::make_shared<fe::ConstructionPointFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("vertex"), "Construction point at a vertex (VertexRef from Body.vertices()).")
        .def("point_at_circle_centre", [](const ModelPtr& mp, const EdgeH& edge) {
                 fe::ConstructionPointParams p;
                 p.kind = fe::ConstructionPointParams::Kind::CircleCentre;
                 p.body = edge.body;
                 p.ref = edge.ref;
                 return append(mp, std::make_shared<fe::ConstructionPointFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("edge"), "Construction point at the centre of a circular edge (full circle or arc).")
        .def("point_edge_plane", [](const ModelPtr& mp, const EdgeH& edge, py::object plane) {
                 fe::ConstructionPointParams p;
                 p.kind = fe::ConstructionPointParams::Kind::EdgePlane;
                 p.body = edge.body;
                 p.ref = edge.ref;
                 p.plane = planeRefOf(plane, "point_edge_plane plane");
                 return append(mp, std::make_shared<fe::ConstructionPointFeature>(mp->doc.allocateFeatureId(), p));
             },
             py::arg("edge"), py::arg("plane"),
             "Construction point where an edge crosses a plane (planar face or Plane feature; the plane is infinite). "
             "With several crossings the one nearest the edge start is used; none fails the feature.")
        // import
        .def("import_step", [](const ModelPtr& mp, const std::string& path) {
                 auto params = unwrap(fe::makeImportParams(path, [&] { return mp->doc.allocateBodyId(); }));
                 return append(mp, std::make_shared<fe::ImportFeature>(mp->doc.allocateFeatureId(), std::move(params)));
             },
             py::arg("path"),
             "Add an Import feature for a STEP file: one body per solid (Feature.bodies). The file is re-read on "
             "recompute (geometry is not embedded in the document yet), so it must stay available.")
        // timeline
        .def_property_readonly("timeline",
                               [](const ModelPtr& mp) {
                                   std::vector<FeatureH> out;
                                   const auto& t = mp->doc.timeline();
                                   for (std::size_t i = 0; i < t.size(); ++i) out.push_back(FeatureH{mp, t.at(i)->id()});
                                   return out;
                               },
                               "Features in timeline order.")
        .def("suppress", [](DocModel& d, const FeatureH& f, bool suppressed) { unwrapVoid(d.doc.setSuppressed(f.id, suppressed)); },
             py::arg("feature"), py::arg("suppressed") = true, "Suppress (or unsuppress) a feature (undoable).")
        .def("delete", [](DocModel& d, const FeatureH& f) { unwrapVoid(d.doc.deleteFeature(f.id)); }, py::arg("feature"),
             "Delete a feature (undoable).")
        .def("rollback", [](DocModel& d, std::optional<std::size_t> index) { unwrapVoid(d.doc.setRollbackMarker(index)); },
             py::arg("index") = py::none(), "Rollback marker: only features before index are active; None = end.")
        .def("undo", [](DocModel& d) { unwrapVoid(d.doc.undo()); })
        .def("redo", [](DocModel& d) { unwrapVoid(d.doc.redo()); })
        .def_property_readonly("can_undo", [](const DocModel& d) { return d.doc.canUndo(); })
        .def_property_readonly("can_redo", [](const DocModel& d) { return d.doc.canRedo(); })
        .def_property_readonly("undo_text", [](const DocModel& d) { return d.doc.undoText(); })
        .def_property_readonly("redo_text", [](const DocModel& d) { return d.doc.redoText(); })
        // evaluation and results
        .def("evaluate", [](const ModelPtr& mp) {
                 const auto& ev = mp->evaluated();
                 EvaluationH out;
                 out.m = mp;
                 out.computed = ev.computedCount;
                 out.reused = ev.reusedCount;
                 for (const auto& s : ev.statuses) {
                     StatusH h;
                     h.id = s.id;
                     h.state = statusName(s.state);
                     h.error = s.error.message;
                     h.errorCode = s.error.code;
                     for (const auto& dg : s.diagnostics) h.diagnostics.push_back(dg.message);
                     h.attributes = s.attributes;
                     if (s.state == rc::FeatureStatus::State::Failed) out.ok = false;
                     out.statuses.push_back(std::move(h));
                 }
                 for (const auto& entry : ev.bodies) out.bodies.push_back(entry.first);
                 return out;
             },
             "Evaluate the timeline (cached until the next change). Failures are reported per feature, not raised.")
        .def_property_readonly("bodies",
                               [](const ModelPtr& mp) {
                                   std::vector<BodyH> out;
                                   for (const auto& entry : mp->evaluated().bodies) out.push_back(BodyH{mp, entry.first});
                                   return out;
                               },
                               "Bodies at the end of the timeline.")
        .def("export", [](DocModel& d, const std::string& path, std::optional<std::vector<BodyH>> bodies) {
                 std::vector<BodyId> ids;
                 if (bodies)
                     for (const auto& b : *bodies) ids.push_back(b.id);
                 exportBodies(d, path, ids);
             },
             py::arg("path"), py::arg("bodies") = py::none(),
             "Export bodies (all by default). Format from the extension: .step/.stp, .stl (one body), .3mf.")
        .def("__repr__", [](const DocModel& d) {
            return std::format("Document(name='{}', features={}, parameters={})", d.doc.name(), d.doc.timeline().size(),
                               d.doc.parameters().size());
        });

    // ---- Evaluation / status ----
    py::class_<StatusH>(m, "FeatureStatus", "Per-feature result of Document.evaluate().")
        .def_property_readonly("feature_id", [](const StatusH& s) { return s.id.value(); })
        .def_property_readonly("state", [](const StatusH& s) { return s.state; }, "ok, suppressed, failed or skipped.")
        .def_property_readonly("error", [](const StatusH& s) { return s.error; })
        .def_property_readonly("diagnostics", [](const StatusH& s) { return s.diagnostics; })
        .def_property_readonly("attributes", [](const StatusH& s) { return s.attributes; })
        .def("__repr__", [](const StatusH& s) { return std::format("FeatureStatus(id={}, state='{}')", s.id.value(), s.state); });

    py::class_<EvaluationH>(m, "Evaluation", "Result of Document.evaluate().")
        .def_property_readonly("ok", [](const EvaluationH& e) { return e.ok; }, "True when no feature failed.")
        .def_property_readonly("statuses", [](const EvaluationH& e) { return e.statuses; })
        .def_property_readonly("computed_count", [](const EvaluationH& e) { return e.computed; })
        .def_property_readonly("reused_count", [](const EvaluationH& e) { return e.reused; })
        .def_property_readonly("bodies",
                               [](const EvaluationH& e) {
                                   std::vector<BodyH> out;
                                   for (BodyId id : e.bodies) out.push_back(BodyH{e.m, id});
                                   return out;
                               })
        .def("raise_for_errors", [](const EvaluationH& e) {
            for (const auto& s : e.statuses)
                if (s.state == "failed") throw ApiError(s.errorCode, s.error);
        }, "Raise the first failed feature's error as an IronwoodError subclass.")
        .def("__repr__", [](const EvaluationH& e) { return std::format("Evaluation(ok={}, features={})", e.ok ? "True" : "False", e.statuses.size()); });

    // ---- Parameter snapshot ----
    py::class_<ParamH>(m, "Parameter")
        .def_property_readonly("name", [](const ParamH& p) { return p.name; })
        .def_property_readonly("expression", [](const ParamH& p) { return p.expression; })
        .def_property_readonly("unit", [](const ParamH& p) { return p.unit; })
        .def_property_readonly("comment", [](const ParamH& p) { return p.comment; })
        .def_property_readonly("favourite", [](const ParamH& p) { return p.favourite; })
        .def_property_readonly("value", [](const ParamH& p) { return p.value; }, "Value in mm, rad or unitless; None on error.")
        .def_property_readonly("dimension", [](const ParamH& p) { return p.dimension; })
        .def_property_readonly("error", [](const ParamH& p) { return p.error; })
        .def("__repr__", [](const ParamH& p) { return std::format("Parameter('{}' = '{}')", p.name, p.expression); });

    // ---- Feature (timeline item) ----
    py::class_<FeatureH>(m, "Feature", "A timeline item: a feature id plus its status and produced body or sketch.")
        .def_property_readonly("id", [](const FeatureH& f) { return f.id.value(); })
        .def_property_readonly("kind", [](const FeatureH& f) { return std::string(featureOf(*f.m, f.id).kind()); })
        .def_property_readonly("index", [](const FeatureH& f) { return f.m->doc.timeline().indexOf(f.id); })
        .def_property_readonly("suppressed", [](const FeatureH& f) { return f.m->doc.timeline().isSuppressed(f.id); })
        .def_property_readonly("status", [](const FeatureH& f) { return featureStatusOf(*f.m, f.id); },
                               "ok, failed, suppressed, skipped, unknown, or unevaluated.")
        .def_property_readonly("error", [](const FeatureH& f) {
            if (f.m->cache)
                for (const auto& s : f.m->cache->statuses)
                    if (s.id == f.id) return s.error.message;
            return std::string();
        })
        .def_property_readonly("body", [](const FeatureH& f) -> std::optional<BodyH> {
            auto w = featureOf(*f.m, f.id).writesBodies();
            if (w.empty()) return std::nullopt;
            return BodyH{f.m, w.front()};
        }, "The body this feature writes (None for sketches and construction features).")
        .def_property_readonly("bodies", [](const FeatureH& f) {
            std::vector<BodyH> out;
            for (BodyId id : featureOf(*f.m, f.id).writesBodies()) out.push_back(BodyH{f.m, id});
            return out;
        }, "All bodies this feature writes (an Import writes one per solid).")
        .def_property_readonly("construction", [](const FeatureH& f) -> std::optional<py::dict> {
            for (const auto& s : f.m->evaluated().statuses) {
                if (s.id != f.id || !s.construction) continue;
                const rc::ConstructionGeometry& g = *s.construction;
                py::dict d;
                switch (g.kind) {
                case rc::ConstructionGeometry::Kind::Plane:
                    d["kind"] = "plane";
                    d["normal"] = toArray(g.normal);
                    d["x_dir"] = toArray(g.xDir);
                    break;
                case rc::ConstructionGeometry::Kind::Axis:
                    d["kind"] = "axis";
                    d["direction"] = toArray(g.direction);
                    break;
                case rc::ConstructionGeometry::Kind::Point: d["kind"] = "point"; break;
                }
                d["origin"] = toArray(g.origin);
                return d;
            }
            return std::nullopt;
        }, "Geometry of a construction feature after evaluation: {'kind', 'origin', ...}; None for other features or on failure.")
        .def_property_readonly("sketch", [](const FeatureH& f) -> std::optional<SketchH> {
            const auto* sf = dynamic_cast<const fe::SketchFeature*>(&featureOf(*f.m, f.id));
            if (!sf) return std::nullopt;
            return SketchH{f.m, sf->params().sketch, sf->params().attach};
        }, "The sketch of a sketch feature (a copy; use Document.edit_sketch to change it).")
        .def("__repr__", [](const FeatureH& f) {
            return std::format("Feature(id={}, kind='{}')", f.id.value(), std::string(featureOf(*f.m, f.id).kind()));
        });

    // ---- Body ----
    py::class_<BodyH> body(m, "Body", "A solid body at the end of the timeline. Measurements reflect the current evaluation.");
    body.def_property_readonly("id", [](const BodyH& b) { return b.id.value(); })
        .def("edges", [](const BodyH& b, py::object filter) {
                 static const std::vector<std::string> kinds{"line", "circle", "ellipse", "bspline", "other"};
                 std::vector<EdgeH> out;
                 for (auto& e : edgesOf(*b.m, b.id))
                     if (passesFilter(filter, e, "edge", kinds)) out.push_back(std::move(e));
                 return out;
             },
             py::arg("filter") = py::none(),
             "Edges of the body, optionally filtered by kind ('line', 'circle', 'ellipse', 'bspline', 'other', or several "
             "joined by '|') or by a callable taking an EdgeRef (e.g. lambda e: e.kind == 'circle' and e.radius > 5).")
        .def("faces", [](const BodyH& b, py::object filter) {
                 static const std::vector<std::string> kinds{"plane", "cylinder", "cone", "other"};
                 std::vector<FaceH> out;
                 for (auto& f : facesOf(b.m, b.id))
                     if (passesFilter(filter, f, "face", kinds)) out.push_back(std::move(f));
                 return out;
             },
             py::arg("filter") = py::none(),
             "Faces of the body, optionally filtered by kind ('plane', 'cylinder', 'cone', 'other', or several joined by "
             "'|') or by a callable taking a FaceRef.")
        .def("vertices", [](const BodyH& b, py::object filter) {
                 std::vector<VertexH> out;
                 for (auto& v : verticesOf(*b.m, b.id))
                     if (filter.is_none() || filter(v).cast<bool>()) out.push_back(std::move(v));
                 return out;
             },
             py::arg("filter") = py::none(), "Vertices of the body, optionally filtered by a callable taking a VertexRef.")
        .def("__repr__", [](const BodyH& b) { return std::format("Body(id={})", b.id.value()); });
    addMeasures(body, [](const BodyH& b) { return shapeOf(*b.m, b.id); });

    // ---- Edge / face references ----
    py::class_<EdgeH>(m, "EdgeRef", "A reference to an edge of a body, with its geometry.")
        .def_property_readonly("centroid", [](const EdgeH& e) { return e.centroid; }, "Edge midpoint (mm).")
        .def_property_readonly("length", [](const EdgeH& e) { return e.length; }, "Edge length (mm).")
        .def_property_readonly("body_id", [](const EdgeH& e) { return e.body.value(); })
        .def_property_readonly("kind", [](const EdgeH& e) { return e.kind; }, "line, circle, ellipse, bspline or other.")
        .def_property_readonly("closed", [](const EdgeH& e) { return e.closed; }, "True for a full circle / ellipse / closed curve.")
        .def_property_readonly("direction", [](const EdgeH& e) -> std::optional<std::array<double, 3>> {
            if (e.kind != "line") return std::nullopt;
            return e.direction;
        }, "Unit direction of a line edge (its sign follows the edge's curve and is not meaningful); None for other kinds.")
        .def_property_readonly("center", [](const EdgeH& e) -> std::optional<std::array<double, 3>> {
            if (e.kind != "circle" && e.kind != "ellipse") return std::nullopt;
            return e.point;
        }, "Centre of a circle / ellipse edge (mm); None for other kinds.")
        .def_property_readonly("axis", [](const EdgeH& e) -> std::optional<std::array<double, 3>> {
            if (e.kind != "circle" && e.kind != "ellipse") return std::nullopt;
            return e.direction;
        }, "Unit axis (plane normal) of a circle / ellipse edge; None for other kinds.")
        .def_property_readonly("radius", [](const EdgeH& e) -> std::optional<double> {
            if (e.kind != "circle" && e.kind != "ellipse") return std::nullopt;
            return e.radius;
        }, "Radius of a circle edge (major radius of an ellipse) in mm; None for other kinds.")
        .def_property_readonly("minor_radius", [](const EdgeH& e) -> std::optional<double> {
            if (e.kind != "ellipse") return std::nullopt;
            return e.minorRadius;
        }, "Minor radius of an ellipse edge (mm); None for other kinds.")
        .def("__repr__", [](const EdgeH& e) {
            return std::format("EdgeRef(kind='{}', centroid={}, length={:.6g})", e.kind, vecStr(toVec(e.centroid)), e.length);
        });
    py::class_<VertexH>(m, "VertexRef", "A reference to a vertex of a body.")
        .def_property_readonly("position", [](const VertexH& v) { return v.position; }, "Vertex position (mm).")
        .def_property_readonly("body_id", [](const VertexH& v) { return v.body.value(); })
        .def("__repr__", [](const VertexH& v) { return std::format("VertexRef(position={})", vecStr(toVec(v.position))); });
    py::class_<FaceH>(m, "FaceRef", "A reference to a face of a body, with its geometry.")
        .def_property_readonly("centroid", [](const FaceH& f) { return f.centroid; }, "Face centroid (mm).")
        .def_property_readonly("area", [](const FaceH& f) { return f.area; }, "Face area (mm^2).")
        .def_property_readonly("kind", [](const FaceH& f) { return f.kind; }, "plane, cylinder, cone or other.")
        .def_property_readonly("normal", [](const FaceH& f) { return f.normal; }, "Outward normal (planar faces).")
        .def_property_readonly("planar", [](const FaceH& f) { return f.planar; })
        .def_property_readonly("body_id", [](const FaceH& f) { return f.body.value(); })
        .def("local", [](const FaceH& f, py::object point) {
                 if (!f.planar) fail(ErrorCode::InvalidArgument, "local() needs a planar face");
                 const auto p = vec3Arg(point);
                 const Vec3 d = toVec(p) - f.plane.origin;
                 const Vec3 yDir = iw::cross(f.plane.normal, f.plane.xDir);
                 return std::make_tuple(iw::dot(d, f.plane.xDir), iw::dot(d, yDir));
             },
             py::arg("point"), "(u, v) in the face's plane of a world point, for hole(at=...).")
        .def("__repr__", [](const FaceH& f) { return std::format("FaceRef(kind='{}', area={:.6g})", f.kind, f.area); });

    // ---- Sketch ----
    py::class_<SolveReportH>(m, "SolveReport", "Outcome of Sketch.solve(): status, DOF, and the conflicting record ids.")
        .def_property_readonly("status", [](const SolveReportH& r) {
            switch (r.status) {
            case sk::SolverStatus::Solved: return std::string("solved");
            case sk::SolverStatus::Failed: return std::string("failed");
            case sk::SolverStatus::InvalidInput: return std::string("invalid_input");
            }
            return std::string("unknown");
        })
        .def_property_readonly("ok", [](const SolveReportH& r) { return r.status == sk::SolverStatus::Solved; })
        .def_property_readonly("dof", [](const SolveReportH& r) { return r.dof; }, "Degrees of freedom (0 = fully constrained).")
        .def_property_readonly("message", [](const SolveReportH& r) { return r.message; })
        .def_property_readonly("conflicting_constraints", [](const SolveReportH& r) { return r.conflictingConstraints; })
        .def_property_readonly("conflicting_dimensions", [](const SolveReportH& r) { return r.conflictingDimensions; })
        .def_property_readonly("redundant_constraints", [](const SolveReportH& r) { return r.redundantConstraints; })
        .def_property_readonly("redundant_dimensions", [](const SolveReportH& r) { return r.redundantDimensions; });

    py::class_<SketchH>(m, "Sketch", "A 2D sketch on a plane: entities, constraints and dimensions (expression strings).")
        .def(py::init([](const std::string& plane, py::object origin, py::object normal, py::object xDir) {
                 const auto o = vec3Arg(origin);
                 if (!normal.is_none()) {
                     if (xDir.is_none()) fail(ErrorCode::InvalidArgument, "a custom plane needs x_dir=(x, y, z)");
                     return SketchH{nullptr, sk::Sketch(unwrap(sk::SketchPlane::make(toVec(o), toVec(vec3Arg(normal)), toVec(vec3Arg(xDir))))), std::nullopt};
                 }
                 return SketchH{nullptr, sk::Sketch(presetPlane(plane, toVec(o))), std::nullopt};
             }),
             py::arg("plane") = "XY", py::arg("origin") = py::make_tuple(0.0, 0.0, 0.0), py::kw_only(),
             py::arg("normal") = py::none(), py::arg("x_dir") = py::none(),
             "A sketch on XY, XZ or YZ (or a custom plane with normal= and x_dir=). Sketch y = normal x x_dir.")
        .def_static("on_face", [](const FaceH& f) {
                 if (!f.planar) fail(ErrorCode::InvalidArgument, "a sketch needs a planar face");
                 return SketchH{f.m, sk::Sketch(unwrap(sk::SketchPlane::make(f.plane.origin, f.plane.normal, f.plane.xDir))),
                                fe::PlaneRef{true, f.body, f.ref, FeatureId{}}};
             },
             py::arg("face"), "A sketch on a planar face. The sketch follows the face (its feature re-derives the plane).")
        .def("line", [](SketchH& s, py::object start, py::object end, bool construction) {
                 const auto a = start.cast<std::array<double, 2>>();
                 const auto b = end.cast<std::array<double, 2>>();
                 return unwrap(s.sketch.addLine({a[0], a[1]}, {b[0], b[1]}, construction)).value();
             },
             py::arg("start"), py::arg("end"), py::arg("construction") = false, "Line between two points; returns its entity id.")
        .def("rect", [](SketchH& s, py::object a, py::object b, bool construction) {
                 const auto p = a.cast<std::array<double, 2>>();
                 const auto q = b.cast<std::array<double, 2>>();
                 const sk::Point2 c0{p[0], p[1]}, c1{q[0], p[1]}, c2{q[0], q[1]}, c3{p[0], q[1]};
                 const auto bottom = unwrap(s.sketch.addLine(c0, c1, construction));
                 const auto right = unwrap(s.sketch.addLine(c1, c2, construction));
                 const auto top = unwrap(s.sketch.addLine(c2, c3, construction));
                 const auto left = unwrap(s.sketch.addLine(c3, c0, construction));
                 using PR = sk::PointRole;
                 auto coin = [&](sk::SketchEntityId x, PR rx, sk::SketchEntityId y, PR ry) {
                     unwrap(s.sketch.addConstraint(sk::ConstraintKind::Coincident, {{x, rx}, {y, ry}}));
                 };
                 coin(bottom, PR::End, right, PR::Start);
                 coin(right, PR::End, top, PR::Start);
                 coin(top, PR::End, left, PR::Start);
                 coin(left, PR::End, bottom, PR::Start);
                 unwrap(s.sketch.addConstraint(sk::ConstraintKind::Horizontal, {{bottom, std::nullopt}}));
                 unwrap(s.sketch.addConstraint(sk::ConstraintKind::Horizontal, {{top, std::nullopt}}));
                 unwrap(s.sketch.addConstraint(sk::ConstraintKind::Vertical, {{right, std::nullopt}}));
                 unwrap(s.sketch.addConstraint(sk::ConstraintKind::Vertical, {{left, std::nullopt}}));
                 return py::make_tuple(bottom.value(), right.value(), top.value(), left.value());
             },
             py::arg("corner_a"), py::arg("corner_b"), py::arg("construction") = false,
             "Closed rectangle of four lines with coincident corners and H/V constraints. Returns the four line ids.")
        .def("circle", [](SketchH& s, py::object center, double radius, bool construction) {
                 const auto c = center.cast<std::array<double, 2>>();
                 return unwrap(s.sketch.addCircle({c[0], c[1]}, radius, construction)).value();
             },
             py::arg("center"), py::arg("radius"), py::arg("construction") = false, "Circle; returns its entity id.")
        .def("arc", [](SketchH& s, py::object center, double radius, double startDeg, double sweepDeg, bool construction) {
                 const auto c = center.cast<std::array<double, 2>>();
                 constexpr double deg = 3.14159265358979323846 / 180.0;
                 return unwrap(s.sketch.addArc({c[0], c[1]}, radius, startDeg * deg, sweepDeg * deg, construction)).value();
             },
             py::arg("center"), py::arg("radius"), py::arg("start_deg"), py::arg("sweep_deg"), py::arg("construction") = false,
             "Counter-clockwise arc; returns its entity id.")
        .def("point", [](SketchH& s, py::object p, bool construction) {
                 const auto a = p.cast<std::array<double, 2>>();
                 return unwrap(s.sketch.addPoint({a[0], a[1]}, construction)).value();
             },
             py::arg("p"), py::arg("construction") = false, "Point; returns its entity id.")
        .def("remove", [](SketchH& s, std::uint64_t id) { unwrapVoid(s.sketch.removeEntity(sk::SketchEntityId{id})); },
             py::arg("entity_id"), "Remove an entity and every record that references it.")
        .def("constrain", [](SketchH& s, const std::string& kind, const py::iterable& refs) {
                 return unwrap(s.sketch.addConstraint(constraintKindOf(kind), refsOf(refs))).value();
             },
             py::arg("kind"), py::arg("refs"),
             "Constraint record. kind: coincident, horizontal, vertical, parallel, perpendicular, tangent, equal, "
             "concentric, midpoint, collinear, symmetric, fix. refs: entity ids or (entity_id, 'start'|'end'|'center'|'mid').")
        .def("dimension", [](SketchH& s, const std::string& kind, const py::iterable& refs, py::object expression) {
                 const auto dk = dimensionKindOf(kind);
                 const std::string unit = dk == sk::DimensionKind::Angle ? "deg" : "mm";
                 return unwrap(s.sketch.addDimension(dk, refsOf(refs), numberOr(expression, unit.c_str()))).value();
             },
             py::arg("kind"), py::arg("refs"), py::arg("expression"),
             "Dimension record with an expression (e.g. 'w', '2 * h', 25). kind: distance, distance_x, distance_y, radius, "
             "diameter, angle.")
        .def("solve", [](SketchH& s, std::optional<ModelPtr> document) {
                 const ModelPtr doc = document ? *document : s.m;
                 const iw::ParameterTable* params = doc ? &doc->doc.parameters() : nullptr;
                 const sk::SketchSolveReport rep = sk::solveSketch(s.sketch, params);
                 SolveReportH out;
                 out.status = rep.status;
                 out.message = rep.message;
                 out.dof = rep.dof;
                 for (auto id : rep.conflictingConstraints) out.conflictingConstraints.push_back(id.value());
                 for (auto id : rep.conflictingDimensions) out.conflictingDimensions.push_back(id.value());
                 for (auto id : rep.redundantConstraints) out.redundantConstraints.push_back(id.value());
                 for (auto id : rep.redundantDimensions) out.redundantDimensions.push_back(id.value());
                 return out;
             },
             py::arg("document") = py::none(),
             "Solve the constraints (parameters from `document`, or the sketch's own document). Updates the geometry on success.")
        .def("profile_names", [](const SketchH& s) { return profileNamesOf(s.sketch); },
             "Names of the closed regions (usable as extrude profile=).")
        .def_property_readonly("entity_count", [](const SketchH& s) { return s.sketch.entities().size(); })
        .def_property_readonly("constraint_count", [](const SketchH& s) { return s.sketch.constraints().size(); })
        .def_property_readonly("dimension_count", [](const SketchH& s) { return s.sketch.dimensions().size(); })
        .def_property_readonly("plane_origin", [](const SketchH& s) { return toArray(s.sketch.plane().origin); })
        .def_property_readonly("plane_normal", [](const SketchH& s) { return toArray(s.sketch.plane().normal); })
        .def_property_readonly("plane_x_dir", [](const SketchH& s) { return toArray(s.sketch.plane().xDir); })
        .def("__repr__", [](const SketchH& s) {
            return std::format("Sketch(entities={}, constraints={}, dimensions={})", s.sketch.entities().size(),
                               s.sketch.constraints().size(), s.sketch.dimensions().size());
        })
        // Reference geometry every sketch holds (fixed): usable in constraints, dimensions and hole_from_sketch.
        .def_readonly_static("ORIGIN", &kSketchOriginId, "Entity id of the fixed sketch origin point (0, 0).")
        .def_readonly_static("X_AXIS", &kSketchXAxisId, "Entity id of the sketch x axis (construction line).")
        .def_readonly_static("Y_AXIS", &kSketchYAxisId, "Entity id of the sketch y axis (construction line).");

    // ---- Standalone solids (imported files; not part of a Document) ----
    py::class_<SolidH> solid(m, "Solid", "A solid read from a file (not part of a Document's timeline).");
    solid.def_property_readonly("name", [](const SolidH& s) { return s.name; })
        .def("__repr__", [](const SolidH& s) { return std::format("Solid(name='{}')", s.name); });
    addMeasures(solid, [](const SolidH& s) { return s.shape; });

    m.def("import_step", [](const std::string& path) {
        std::vector<SolidH> out;
        for (const auto& node : unwrap(kn::importStep(path)).nodes)
            if (!node.shape.isNull()) out.push_back(SolidH{node.shape, node.name});
        return out;
    }, py::arg("path"),
       "Read a STEP file; returns a Solid per geometric node (not added to any Document). To import into a Document's "
       "timeline use Document.import_step(path).");
    m.def("import_stl", [](const std::string& path) {
        return SolidH{unwrap(kn::importStl(path)), std::filesystem::path(path).stem().string()};
    }, py::arg("path"), "Read an STL file as one Solid (not added to any Document).");
}
