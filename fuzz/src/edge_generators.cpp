// SPDX-License-Identifier: GPL-3.0-or-later
// Generators + predictors for fillet and chamfer (features/edge_features.hpp).
//
// Both pick edges BY NAME from the model state at the insertion position (half of the time preferring the rim of a
// hole: the F27 scenario "fillet a hole's rim, then edit the hole"; the rim is chosen by geometry, see holeRimPick).
// Elements the operation generates are
// checked against what it must produce: a fillet face is a cylinder of the requested radius (or a blend patch) lying
// next to its source edge; a chamfer face is a plane lying next to its source edge.
#include <algorithm>
#include <cmath>
#include <tuple>

#include "features/edge_features.hpp"
#include "features/hole_feature.hpp"
#include "fuzz/generator.hpp"
#include "gen_util.hpp"
#include "kernel/ops.hpp"

namespace iw::fuzz {
namespace {

using features::ChamferFeature;
using features::ChamferParams;
using features::FilletFeature;
using features::FilletParams;
using naming::ElementName;
using naming::ElementType;

/// Rim of a hole (F27): for each hole of the timeline whose entry face is in the state, the hole-created non-straight edge
/// lying in the plane of that face (the wall meets the entry face there). Per hole the rim is the longest such edge, ties
/// broken by centroid; the hole is then chosen at random. Geometry decides the rim, so no name order enters the pick.
std::optional<util::Pick> holeRimPick(Rng& rng, const GenContext& ctx) {
    std::vector<util::Pick> rims;
    for (std::size_t i = 0; i < ctx.timeline.size(); ++i) {
        const auto& f = ctx.timeline.at(i);
        if (f->kind() != "hole") continue;
        const auto& hp = static_cast<const features::HoleFeature&>(*f).params();
        auto shapeIt = ctx.state.bodies.find(hp.body);
        auto namesIt = ctx.state.namings.find(hp.body);
        if (shapeIt == ctx.state.bodies.end() || namesIt == ctx.state.namings.end()) continue;
        const auto faceIdx = namesIt->second.indicesOf(hp.face.name);
        if (faceIdx.size() != 1) continue;
        auto plane = kernel::planeFromFace(shapeIt->second, faceIdx.front());
        if (!plane) continue;
        std::optional<util::Pick> best;
        GeomSig bestSig;
        for (const auto& [name, idx] : ctx.state.namedElements(hp.body, ElementType::Edge)) {
            if (name.feature != f->id() || !util::opStartsWith(name, "hole.")) continue;
            const GeomSig sig = readSig(shapeIt->second, ElementType::Edge, idx);
            if (sig.isLine || sig.measure() < 1e-9) continue;
            if (std::abs(dot(plane->normal, sig.centroid() - plane->origin)) > 1e-3) continue; // not on the entry face
            const bool longer = !best || sig.measure() > bestSig.measure();
            const bool tieBreak = best && sig.measure() == bestSig.measure() &&
                                  std::tie(sig.key[0], sig.key[1], sig.key[2]) < std::tie(bestSig.key[0], bestSig.key[1], bestSig.key[2]);
            if (longer || tieBreak) {
                best = util::Pick{hp.body, name, idx};
                bestSig = sig;
            }
        }
        if (best) rims.push_back(*best);
    }
    if (rims.empty()) return std::nullopt;
    return rims[rng.below(rims.size())];
}

std::vector<naming::ElementRef> pickEdges(Rng& rng, const GenContext& ctx, BodyId& bodyOut) {
    const auto anyEdge = [](BodyId, const kernel::Shape&, std::uint32_t, const ElementName&) { return true; };
    std::optional<util::Pick> first;
    // Prefer the rim of a hole (F27).
    if (rng.chance(0.5)) first = holeRimPick(rng, ctx);
    if (!first) first = util::pickElement(rng, ctx, ElementType::Edge, anyEdge);
    if (!first) return {};
    bodyOut = first->body;
    std::vector<naming::ElementRef> edges{util::refOf(ctx, *first)};
    if (rng.chance(0.35)) {
        const BodyId b = first->body;
        auto second = util::pickElement(rng, ctx, ElementType::Edge,
                                        [&](BodyId body, const kernel::Shape&, std::uint32_t, const ElementName& n) {
                                            return body == b && !(n == first->name);
                                        });
        if (second) edges.push_back(util::refOf(ctx, *second));
    }
    return edges;
}

/// Edge-feature expectations shared by fillet and chamfer.
///  * generated face ("<tag>.side", source = one edge of the input): lies next to that edge; for a fillet it is a
///    cylinder of the requested radius (or a blend patch), for a chamfer a plane or cone.
///  * any other generated element (edges, vertices, "<tag>.new", corner faces): lies within reach of an edge of the
///    selection (expanded by tangent continuity when the feature asks for it).
std::optional<std::vector<Expected>> predictEdgeOp(const std::vector<naming::ElementRef>& edges, bool chain, BodyId body,
                                                   double reach, double radius, bool filletFaces, const ElementName& name,
                                                   const PredictContext& ctx) {
    const std::string role = name.op.substr(name.op.find('.') + 1);
    if (role == "split" || role == "merge") return std::nullopt; // handled generically from the sources
    auto shapeIt = ctx.in.bodies.find(body);
    if (shapeIt == ctx.in.bodies.end()) return std::nullopt;
    const kernel::Shape& shape = shapeIt->second;
    std::vector<GeomSig> near;
    if (name.sources.size() == 1 && name.sources[0].type == ElementType::Edge) {
        if (auto s = ctx.lookup(body, name.sources[0])) near = {*s}; // the exact source edge
    }
    if (near.empty()) {
        std::vector<std::uint32_t> idx;
        for (const auto& e : edges)
            if (auto i = ctx.indexOf(body, e.name)) idx.push_back(*i);
        if (idx.empty()) return std::nullopt;
        std::sort(idx.begin(), idx.end());
        idx.erase(std::unique(idx.begin(), idx.end()), idx.end());
        // OCCT's contour builder propagates along tangent-continuous edges whatever the feature's chain flag says.
        (void)chain;
        if (auto ex = kernel::tangentChain(shape, idx)) idx = *ex;
        for (std::uint32_t i : idx) near.push_back(readSig(shape, ElementType::Edge, i));
        // A blend also reshapes the edges meeting the selection at its ends (vertex blends, end caps): add the input edges
        // within reach of a selected edge (one hop).
        if (name.type != ElementType::Face || role != "side") {
            std::vector<GeomSig> hop = near;
            const std::uint32_t ne = shape.subShapeCount(kernel::SubShapeKind::Edge);
            for (std::uint32_t i = 0; i < ne; ++i) {
                if (std::binary_search(idx.begin(), idx.end(), i)) continue;
                const GeomSig e = readSig(shape, ElementType::Edge, i);
                for (const GeomSig& n : near)
                    if (length(e.centroid() - n.centroid()) <= (e.measure() + n.measure()) * 0.5 + reach) {
                        hop.push_back(e);
                        break;
                    }
            }
            near = std::move(hop);
        }
    }

    Expected x;
    x.sig.type = name.type;
    x.note = "next to a selected edge:";
    for (std::size_t i = 0; i < std::min<std::size_t>(near.size(), 3); ++i) x.note += " " + describe(near[i]);
    x.extra = [near, reach, radius, filletFaces, type = name.type, role](const GeomSig& got) {
        if (type == ElementType::Edge && got.measure() < 1e-9) return true; // degenerate edge (cone apex, ...)
        bool close = false;
        for (const GeomSig& e : near) {
            // A blend element can reach beyond the end of the edge by the blend width and run along its length.
            const double extent = type == ElementType::Face ? std::sqrt(std::max(0.0, got.measure()))
                                  : type == ElementType::Edge ? got.measure() * 0.5 : 0.0;
            if (length(got.centroid() - e.centroid()) <= e.measure() * 0.5 + extent + reach + 1e-6) close = true;
        }
        if (!close) return false;
        if (type == ElementType::Face && role == "side") {
            if (filletFaces) {
                if (got.surface == SurfaceKind::Cylinder) return std::abs(got.radius - radius) < 1e-6 + 1e-6 * radius;
                return got.surface != SurfaceKind::Plane; // vertex blends: sphere / torus / B-spline patch
            }
            return true; // chamfer of a line -> plane, of a circle -> cone, of a curve -> B-spline
        }
        return true;
    };
    return std::vector<Expected>{x};
}

class FilletGenerator final : public FeatureGenerator {
public:
    std::string_view kind() const override { return "fillet"; }
    int weight() const override { return 4; }

    std::shared_ptr<const recompute::Feature> generate(Rng& rng, FeatureId id, const GenContext& ctx) const override {
        FilletParams p;
        p.edges = pickEdges(rng, ctx, p.body);
        if (p.edges.empty()) return nullptr;
        p.radius = mmText(util::round1(rng.real(0.3, 1.6)));
        p.tangentChain = rng.chance(0.7);
        return std::make_shared<FilletFeature>(id, std::move(p));
    }

    std::shared_ptr<const recompute::Feature> editDimension(Rng& rng, const recompute::Feature& f) const override {
        if (f.kind() != "fillet") return nullptr;
        FilletParams p = static_cast<const FilletFeature&>(f).params();
        const double r = parseMm(p.radius).value_or(1.0);
        static const double kScale[] = {0.5, 0.75, 1.3, 1.6};
        p.radius = mmText(util::round1(std::max(0.2, r * kScale[rng.below(4)])));
        return std::make_shared<FilletFeature>(f.id(), std::move(p));
    }

    std::optional<BodyId> inheritsFrom(const recompute::Feature& f) const override {
        return static_cast<const FilletFeature&>(f).params().body;
    }

    /// ADVISORY (counted, not a failure): a name that is only distinguished by a sibling ordinal ("<tag>.new#k", ...) must keep denoting the same KIND of element
    /// across an edit of this feature's dimension.
    std::optional<std::string> advisoryRoleChanged(const recompute::Feature& before, const recompute::Feature& after,
                                           const ElementName& name, const GeomSig& pre, const GeomSig& post) const override {
        if (before.kind() != kind() || after.kind() != kind()) return std::nullopt;
        if (name.op.size() < 4 || name.op.compare(name.op.size() - 4, 4, ".new") != 0) return std::nullopt;
        if (name.type == ElementType::Edge && pre.isLine != post.isLine && pre.measure() > 1e-9 && post.measure() > 1e-9)
            return "changed between a straight edge and a curve after an edit of its feature";
        if (name.type == ElementType::Face && pre.surface != post.surface && pre.surface != SurfaceKind::Other &&
            post.surface != SurfaceKind::Other)
            return "changed from one surface kind to another after an edit of its feature";
        return std::nullopt;
    }


    std::optional<std::vector<Expected>> predict(const recompute::Feature& f, const ElementName& name,
                                                 const PredictContext& ctx) const override {
        const FilletParams& p = static_cast<const FilletFeature&>(f).params();
        const double r = parseMm(p.radius).value_or(0);
        if (r <= 0) return std::nullopt;
        (void)f;
        return predictEdgeOp(p.edges, p.tangentChain, p.body, 3.0 * r + 1e-3, r, true, name, ctx);
    }
};

class ChamferGenerator final : public FeatureGenerator {
public:
    std::string_view kind() const override { return "chamfer"; }
    int weight() const override { return 3; }

    std::shared_ptr<const recompute::Feature> generate(Rng& rng, FeatureId id, const GenContext& ctx) const override {
        ChamferParams p;
        p.edges = pickEdges(rng, ctx, p.body);
        if (p.edges.empty()) return nullptr;
        switch (rng.below(3)) {
        case 0: p.mode = ChamferParams::Mode::Equal; break;
        case 1: p.mode = ChamferParams::Mode::TwoDistances; break;
        default: p.mode = ChamferParams::Mode::DistanceAngle; break;
        }
        p.distance = mmText(util::round1(rng.real(0.3, 1.5)));
        p.distance2 = mmText(util::round1(rng.real(0.3, 1.5)));
        p.angle = degText(static_cast<double>(rng.intIn(3, 6) * 10));
        p.tangentChain = rng.chance(0.7);
        return std::make_shared<ChamferFeature>(id, std::move(p));
    }

    std::shared_ptr<const recompute::Feature> editDimension(Rng& rng, const recompute::Feature& f) const override {
        if (f.kind() != "chamfer") return nullptr;
        ChamferParams p = static_cast<const ChamferFeature&>(f).params();
        const double d = parseMm(p.distance).value_or(1.0);
        static const double kScale[] = {0.5, 0.75, 1.3, 1.6};
        p.distance = mmText(util::round1(std::max(0.2, d * kScale[rng.below(4)])));
        return std::make_shared<ChamferFeature>(f.id(), std::move(p));
    }

    std::optional<BodyId> inheritsFrom(const recompute::Feature& f) const override {
        return static_cast<const ChamferFeature&>(f).params().body;
    }

    /// ADVISORY (counted, not a failure): a name that is only distinguished by a sibling ordinal ("<tag>.new#k", ...) must keep denoting the same KIND of element
    /// across an edit of this feature's dimension.
    std::optional<std::string> advisoryRoleChanged(const recompute::Feature& before, const recompute::Feature& after,
                                           const ElementName& name, const GeomSig& pre, const GeomSig& post) const override {
        if (before.kind() != kind() || after.kind() != kind()) return std::nullopt;
        if (name.op.size() < 4 || name.op.compare(name.op.size() - 4, 4, ".new") != 0) return std::nullopt;
        if (name.type == ElementType::Edge && pre.isLine != post.isLine && pre.measure() > 1e-9 && post.measure() > 1e-9)
            return "changed between a straight edge and a curve after an edit of its feature";
        if (name.type == ElementType::Face && pre.surface != post.surface && pre.surface != SurfaceKind::Other &&
            post.surface != SurfaceKind::Other)
            return "changed from one surface kind to another after an edit of its feature";
        return std::nullopt;
    }


    std::optional<std::vector<Expected>> predict(const recompute::Feature& f, const ElementName& name,
                                                 const PredictContext& ctx) const override {
        const ChamferParams& p = static_cast<const ChamferFeature&>(f).params();
        const double d1 = parseMm(p.distance).value_or(0);
        const double d2 = p.mode == ChamferParams::Mode::TwoDistances ? parseMm(p.distance2).value_or(0) : d1;
        // Distance + angle: the second leg is d * tan(angle) (angle < 90 deg here), at most ~ 6 d.
        const double dm = std::max({d1, d2, p.mode == ChamferParams::Mode::DistanceAngle ? 6.0 * d1 : 0.0});
        if (dm <= 0) return std::nullopt;
        return predictEdgeOp(p.edges, p.tangentChain, p.body, 4.0 * dm + 1e-3, 0, false, name, ctx);
    }
};

} // namespace

std::shared_ptr<const FeatureGenerator> makeFilletGenerator() { return std::make_shared<FilletGenerator>(); }
std::shared_ptr<const FeatureGenerator> makeChamferGenerator() { return std::make_shared<ChamferGenerator>(); }

} // namespace iw::fuzz
