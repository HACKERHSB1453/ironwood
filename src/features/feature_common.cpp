// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/feature_common.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <optional>
#include <utility>

#include <nlohmann/json.hpp>

#include "core/expr.hpp"
#include "features/feature_json.hpp"
#include "core/ids.hpp"
#include "kernel/ops.hpp"
#include "naming/element_name.hpp"
#include "naming/naming_map.hpp"
#include "naming/resolve.hpp"

namespace iw::features::common {
namespace {
using json = nlohmann::json;
using recompute::Diagnostic;
using naming::ElementName;
using naming::ElementType;
using naming::NamingMap;
} // namespace

void Fnv::bytes(std::string_view s) {
    for (unsigned char c : s) {
        h ^= c;
        h *= 0x100000001b3ULL;
    }
}

Vec3 normalized(const Vec3& v) { return v * (1.0 / length(v)); }
bool finite(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// ---- expressions -----------------------------------------------------------------------------------------------

/// Evaluates an expression against the parameter table. A missing table only fails for parameter references.
Result<Quantity> evalExpr(const ParameterTable* table, const std::string& text) {
    auto ex = Expression::parse(text);
    if (!ex) return std::unexpected(ex.error());
    const ParameterLookup lookup = [table](std::string_view name) -> Result<Quantity> {
        if (!table) return makeError(ErrorCode::NotFound, "unknown parameter '" + std::string(name) + "' (no parameter table)");
        return table->evaluate(name);
    };
    return ex->evaluate(lookup);
}

Result<double> lengthOf(const ParameterTable* table, const std::string& text, const char* what) {
    auto q = evalExpr(table, text);
    if (!q) return std::unexpected(q.error());
    if (q->dim != Dimension::Length)
        return makeError(ErrorCode::InvalidArgument, std::string("extrude: ") + what + " must be a length, got " + dimensionName(q->dim));
    return q->value;
}

Result<double> angleOf(const ParameterTable* table, const std::string& text, const char* what) {
    auto q = evalExpr(table, text);
    if (!q) return std::unexpected(q.error());
    if (q->dim != Dimension::Angle)
        return makeError(ErrorCode::InvalidArgument, std::string("extrude: ") + what + " must be an angle, got " + dimensionName(q->dim));
    return q->value;
}

std::vector<std::string> referencesOf(const std::string& text) {
    auto ex = Expression::parse(text);
    if (!ex) return {};
    return ex->references();
}

// ---- naming ----------------------------------------------------------------------------------------------------

/// P0 polygon profile naming: positional, no sketch identity.
Result<NamingMap> polygonProfileNaming(FeatureId feature, const kernel::Shape& profile) {
    NamingMap map;
    const auto nf = profile.subShapeCount(kernel::SubShapeKind::Face);
    const auto ne = profile.subShapeCount(kernel::SubShapeKind::Edge);
    const auto nv = profile.subShapeCount(kernel::SubShapeKind::Vertex);
    map.resize(nf, ne, nv);
    auto fill = [&](ElementType t, std::uint32_t n, const char* op) -> Result<void> {
        for (std::uint32_t i = 0; i < n; ++i) {
            ElementName name;
            name.feature = feature;
            name.op = op;
            name.type = t;
            name.ordinal = i;
            if (auto r = map.set(t, i, std::move(name)); !r) return std::unexpected(r.error());
        }
        return {};
    };
    if (auto r = fill(ElementType::Face, nf, "profile.face"); !r) return std::unexpected(r.error());
    if (auto r = fill(ElementType::Edge, ne, "profile.edge"); !r) return std::unexpected(r.error());
    if (auto r = fill(ElementType::Vertex, nv, "profile.vertex"); !r) return std::unexpected(r.error());
    return map;
}

/// World-space key of a profile edge, comparable with kernel::Shape::subShapeKey (centre of mass, length).
std::array<double, 4> edgeKey(const sketch::SketchPlane& plane, const sketch::ProfileEdge& e) {
    using sketch::Point2;
    Point2 local;
    double len = 0;
    if (!e.isArc) {
        local = {(e.start.x + e.end.x) * 0.5, (e.start.y + e.end.y) * 0.5};
        len = sketch::distance(e.start, e.end);
    } else {
        const double absSweep = std::abs(e.sweep);
        if (absSweep >= 2.0 * std::numbers::pi - 1e-12) {
            local = e.center;
            len = 2.0 * std::numbers::pi * e.radius;
        } else {
            const double half = absSweep * 0.5;
            const double bis = e.startAngle + e.sweep * 0.5;
            const double r = e.radius * (half > 1e-15 ? std::sin(half) / half : 1.0);
            local = {e.center.x + r * std::cos(bis), e.center.y + r * std::sin(bis)};
            len = e.radius * absSweep;
        }
    }
    const Vec3 w = plane.toWorld(local);
    return {w.x, w.y, w.z, len};
}

constexpr double kMatchTol = 1e-6;

bool keysMatch(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    for (std::size_t i = 0; i < 4; ++i)
        if (std::abs(a[i] - b[i]) > kMatchTol * (1.0 + std::abs(b[i]))) return false;
    return true;
}

/// Names of a sketch-derived profile face. Every element is named from sketch entity ids, never from kernel order:
///   face    -> "sketch.region"
///   edge    -> "sketch.e<entity>.<piece>" (the bounding sketch piece it came from)
///   vertex  -> "sketch.vertex" with the sorted names of the edges meeting there as sources.
/// A dimension edit that keeps the entity ids therefore keeps every name. Kernel elements that cannot be matched to a
/// sketch piece get "sketch.unmatched" names and a warning.
Result<NamingMap> sketchProfileNaming(FeatureId feature, const kernel::Shape& face, const sketch::SketchProfile& prof,
                                      const sketch::SketchPlane& plane, std::vector<Diagnostic>& diags) {
    std::vector<const sketch::ProfileEdge*> pieces;
    for (const auto& e : prof.outer.edges) pieces.push_back(&e);
    for (const auto& h : prof.holes)
        for (const auto& e : h.edges) pieces.push_back(&e);

    std::vector<ElementName> pieceNames(pieces.size());
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        pieceNames[i].feature = feature;
        pieceNames[i].type = ElementType::Edge;
        pieceNames[i].op = "sketch.e" + std::to_string(pieces[i]->entity.value()) + "." + std::to_string(pieces[i]->subIndex);
    }
    std::vector<std::array<double, 4>> pieceKeys;
    for (const auto* p : pieces) pieceKeys.push_back(edgeKey(plane, *p));

    const auto nf = face.subShapeCount(kernel::SubShapeKind::Face);
    const auto ne = face.subShapeCount(kernel::SubShapeKind::Edge);
    const auto nv = face.subShapeCount(kernel::SubShapeKind::Vertex);
    NamingMap map;
    map.resize(nf, ne, nv);

    // Edges: greedy match of each kernel edge to an unused piece by (centre of mass, length).
    std::vector<bool> used(pieces.size(), false);
    std::vector<std::optional<ElementName>> edgeNames(ne);
    for (std::uint32_t i = 0; i < ne; ++i) {
        const auto k = face.subShapeKey(kernel::SubShapeKind::Edge, i);
        for (std::size_t j = 0; j < pieces.size(); ++j) {
            if (used[j] || !keysMatch(k, pieceKeys[j])) continue;
            used[j] = true;
            edgeNames[i] = pieceNames[j];
            break;
        }
        if (!edgeNames[i]) {
            ElementName n;
            n.feature = feature;
            n.type = ElementType::Edge;
            n.op = "sketch.unmatched";
            n.ordinal = i;
            edgeNames[i] = n;
            diags.push_back({Diagnostic::Severity::Warning, "extrude: profile edge could not be matched to a sketch piece", std::nullopt});
        }
        if (auto r = map.set(ElementType::Edge, i, *edgeNames[i]); !r) return std::unexpected(r.error());
    }

    for (std::uint32_t i = 0; i < nf; ++i) {
        ElementName n;
        n.feature = feature;
        n.type = ElementType::Face;
        n.op = "sketch.region";
        n.ordinal = i;
        if (auto r = map.set(ElementType::Face, i, std::move(n)); !r) return std::unexpected(r.error());
    }

    // Vertices: named by the sorted names of the kernel edges that meet at them.
    std::map<std::vector<ElementName>, std::uint32_t> seenVertex;
    for (std::uint32_t i = 0; i < nv; ++i) {
        const auto k = face.subShapeKey(kernel::SubShapeKind::Vertex, i);
        std::vector<ElementName> srcs;
        for (std::size_t j = 0; j < pieces.size(); ++j) {
            for (const sketch::Point2 q : {pieces[j]->start, pieces[j]->end}) {
                const Vec3 w = plane.toWorld(q);
                if (std::abs(w.x - k[0]) <= kMatchTol * (1 + std::abs(k[0])) && std::abs(w.y - k[1]) <= kMatchTol * (1 + std::abs(k[1])) &&
                    std::abs(w.z - k[2]) <= kMatchTol * (1 + std::abs(k[2]))) {
                    srcs.push_back(pieceNames[j]);
                }
            }
        }
        std::sort(srcs.begin(), srcs.end());
        srcs.erase(std::unique(srcs.begin(), srcs.end()), srcs.end());
        ElementName n;
        n.feature = feature;
        n.type = ElementType::Vertex;
        n.op = srcs.empty() ? "sketch.unmatched" : "sketch.vertex";
        n.sources = srcs;
        if (srcs.empty()) diags.push_back({Diagnostic::Severity::Warning, "extrude: profile vertex could not be matched to a sketch piece", std::nullopt});
        n.ordinal = seenVertex[srcs]++; // identical source sets (e.g. both ends of a split circle) -> sibling ordinals
        if (auto r = map.set(ElementType::Vertex, i, std::move(n)); !r) return std::unexpected(r.error());
    }
    return map;
}


Result<std::array<double, 3>> readVec(const json& obj, const char* key) {
    auto it = obj.find(key);
    if (it == obj.end() || !it->is_array() || it->size() != 3)
        return makeError(ErrorCode::ParseError, std::string("extrude: '") + key + "' must be a 3-number array");
    std::array<double, 3> out{};
    for (std::size_t i = 0; i < 3; ++i) {
        if (!(*it)[i].is_number()) return makeError(ErrorCode::ParseError, std::string("extrude: '") + key + "' has a non-number");
        out[i] = (*it)[i].get<double>();
    }
    return out;
}

Result<std::string> readString(const json& obj, const char* key, std::optional<std::string> fallback) {
    auto it = obj.find(key);
    if (it == obj.end()) {
        if (fallback) return *fallback;
        return makeError(ErrorCode::ParseError, std::string("extrude: missing '") + key + "'");
    }
    if (!it->is_string()) return makeError(ErrorCode::ParseError, std::string("extrude: '") + key + "' must be a string");
    return it->get<std::string>();
}

Result<void> checkExpression(const std::string& text, const char* key) {
    auto ex = Expression::parse(text);
    if (!ex) return makeError(ErrorCode::ParseError, std::string("extrude: '") + key + "': " + ex.error().message);
    return {};
}


/// Rigid rotation of `angle` radians about `axis` (Rodrigues), as a row-major 4x4 transform.
Transform rotationAbout(const kernel::Axis& axis, double angle) {
    const Vec3 k = axis.direction;
    const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
    const double r[3][3] = {
        {t * k.x * k.x + c, t * k.x * k.y - s * k.z, t * k.x * k.z + s * k.y},
        {t * k.x * k.y + s * k.z, t * k.y * k.y + c, t * k.y * k.z - s * k.x},
        {t * k.x * k.z - s * k.y, t * k.y * k.z + s * k.x, t * k.z * k.z + c}};
    const Vec3 o = axis.origin;
    const double tx = o.x - (r[0][0] * o.x + r[0][1] * o.y + r[0][2] * o.z);
    const double ty = o.y - (r[1][0] * o.x + r[1][1] * o.y + r[1][2] * o.z);
    const double tz = o.z - (r[2][0] * o.x + r[2][1] * o.y + r[2][2] * o.z);
    Transform m;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m.m[static_cast<std::size_t>(i * 4 + j)] = r[i][j];
    m.m[3] = tx;
    m.m[7] = ty;
    m.m[11] = tz;
    return m;
}

const char* bodyOpName(BodyOp op) {
    switch (op) {
    case BodyOp::NewBody: return "new_body";
    case BodyOp::Join: return "join";
    case BodyOp::Cut: return "cut";
    case BodyOp::Intersect: return "intersect";
    }
    return "new_body";
}

Result<BodyOp> bodyOpFrom(const std::string& s) {
    if (s == "new_body") return BodyOp::NewBody;
    if (s == "join") return BodyOp::Join;
    if (s == "cut") return BodyOp::Cut;
    if (s == "intersect") return BodyOp::Intersect;
    return makeError(ErrorCode::ParseError, "unknown operation '" + s + "'");
}

Result<std::uint32_t> resolveRef(const recompute::EvalContext& ctx, BodyId body, const naming::ElementRef& ref,
                                 naming::ElementType expected, std::vector<Diagnostic>& diags) {
    const std::string what = ref.name.toString();
    if (ref.type != expected) return makeError(ErrorCode::InvalidArgument, "reference " + what + " is not of the expected element type");
    const kernel::Shape* shape = ctx.body(body);
    const naming::NamingMap* map = ctx.naming(body);
    if (!shape || !map) return makeError(ErrorCode::NotFound, "body " + std::to_string(body.value()) + " does not exist at this position");
    auto r = naming::resolve(ref, *map);
    if (!r) {
        const auto code = r.error().code == ErrorCode::Ambiguous ? ErrorCode::Ambiguous : ErrorCode::NotFound;
        return makeError(code, "reference " + what + " could not be resolved: " + r.error().message);
    }
    for (const auto& w : r->warnings) diags.push_back({Diagnostic::Severity::Warning, w, ref});
    return r->index;
}

Result<recompute::FeatureResult> applyBodyOp(FeatureId feature, BodyOp op, BodyId body, const recompute::EvalContext& ctx,
                                             const kernel::ShapeResult& tool, const naming::NamingMap& toolNaming,
                                             std::string_view toolTag, std::vector<Diagnostic> diags) {
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "feature cancelled");
    auto toolNames = naming::deriveNamingMap(feature, toolTag, tool.shape, {&toolNaming}, tool.history);
    if (!toolNames) return std::unexpected(toolNames.error());
    recompute::FeatureResult out;
    out.diagnostics = std::move(diags);
    if (op == BodyOp::NewBody) {
        out.bodies.emplace(body, tool.shape);
        out.namings.emplace(body, std::move(*toolNames));
        return out;
    }
    const kernel::Shape* target = ctx.body(body);
    const naming::NamingMap* targetNaming = ctx.naming(body);
    if (!target || !targetNaming) return makeError(ErrorCode::NotFound, "target body does not exist at this position");

    Result<kernel::ShapeResult> combined = makeError(ErrorCode::Internal, "not combined");
    const char* opTag = "fuse";
    switch (op) {
    case BodyOp::Join: combined = kernel::fuse(*target, tool.shape, ctx.cancel); opTag = "fuse"; break;
    case BodyOp::Cut: combined = kernel::cut(*target, tool.shape, ctx.cancel); opTag = "cut"; break;
    case BodyOp::Intersect: combined = kernel::common(*target, tool.shape, ctx.cancel); opTag = "common"; break;
    case BodyOp::NewBody: break;
    }
    if (!combined) return std::unexpected(combined.error());
    if (combined->shape.isNull() || combined->shape.subShapeCount(kernel::SubShapeKind::Face) == 0)
        return makeError(ErrorCode::KernelFailure, std::string(bodyOpName(op)) + " produced no solid");
    auto names = naming::deriveNamingMap(feature, opTag, combined->shape, {targetNaming, &*toolNames}, combined->history);
    if (!names) return std::unexpected(names.error());
    out.bodies.emplace(body, std::move(combined->shape));
    out.namings.emplace(body, std::move(*names));
    return out;
}

Result<recompute::FeatureResult> modifyBody(FeatureId feature, BodyId body, const recompute::EvalContext& ctx,
                                            const kernel::ShapeResult& result, std::string_view opTag,
                                            std::vector<Diagnostic> diags, const naming::NamingMap* toolNaming,
                                            const naming::SiblingFrame* frame) {
    const kernel::Shape* input = ctx.body(body);
    const naming::NamingMap* inputNaming = ctx.naming(body);
    if (!input || !inputNaming) return makeError(ErrorCode::NotFound, "body " + std::to_string(body.value()) + " does not exist at this position");
    if (ctx.cancel.cancelled()) return makeError(ErrorCode::Cancelled, "feature cancelled");
    // Without a tool naming only the target body is an input: history entries about other operands carry names of their
    // own, so they are dropped rather than rejected by deriveNamingMap.
    const std::uint32_t operands = toolNaming ? 2 : 1;
    kernel::ShapeHistory own;
    for (const auto& [in, out] : result.history.generated)
        if (in.operand < operands) own.generated.insert({in, out});
    for (const auto& [in, out] : result.history.modified)
        if (in.operand < operands) own.modified.insert({in, out});
    for (const auto& [in, out] : result.history.unchanged)
        if (in.operand < operands) own.unchanged.insert({in, out});
    for (const auto& in : result.history.deleted)
        if (in.operand < operands) own.deleted.insert(in);
    std::vector<const naming::NamingMap*> inputs{inputNaming};
    if (toolNaming) inputs.push_back(toolNaming);
    auto names = naming::deriveNamingMap(feature, opTag, result.shape, inputs, own, frame, toolNaming ? 1u : UINT32_MAX);
    if (!names) return std::unexpected(names.error());
    recompute::FeatureResult out;
    out.diagnostics = std::move(diags);
    out.bodies.emplace(body, result.shape);
    out.namings.emplace(body, std::move(*names));
    return out;
}

nlohmann::json refToJson(const naming::ElementRef& ref) {
    static constexpr const char* kTypes[] = {"F", "E", "V"};
    return json{{"feature", ref.feature.value()}, {"type", kTypes[static_cast<int>(ref.type)]}, {"name", ref.name.toString()}};
}

Result<naming::ElementRef> refFromJson(const json& obj, const char* key) {
    auto it = obj.find(key);
    if (it == obj.end() || !it->is_object()) return makeError(ErrorCode::ParseError, std::string("missing element reference '") + key + "'");
    const json& r = *it;
    if (!r.contains("feature") || !r["feature"].is_number_unsigned() || !r.contains("type") || !r["type"].is_string() || !r.contains("name") || !r["name"].is_string())
        return makeError(ErrorCode::ParseError, std::string("malformed element reference '") + key + "'");
    naming::ElementRef out;
    out.feature = FeatureId{r["feature"].get<std::uint64_t>()};
    const std::string t = r["type"].get<std::string>();
    if (t == "F") out.type = naming::ElementType::Face;
    else if (t == "E") out.type = naming::ElementType::Edge;
    else if (t == "V") out.type = naming::ElementType::Vertex;
    else return makeError(ErrorCode::ParseError, "unknown element type '" + t + "'");
    auto name = naming::ElementName::parse(r["name"].get<std::string>());
    if (!name) return std::unexpected(name.error());
    out.name = std::move(*name);
    if (out.name.type != out.type) return makeError(ErrorCode::ParseError, "element reference type mismatch");
    return out;
}

} // namespace iw::features::common
