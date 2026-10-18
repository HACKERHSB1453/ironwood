// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/sketch/session.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <cmath>
#include <memory>
#include <numbers>
#include <set>
#include <type_traits>
#include <utility>
#include <variant>

namespace iw::ui {
namespace {

using sketch::ConstraintKind;
using sketch::EntityGeometry;
using sketch::EntityKind;
using sketch::LineGeom;
using sketch::Point2;
using sketch::PointRole;
using sketch::Sketch;
using sketch::SketchEntityId;
using sketch::SketchRef;

constexpr double kEps = 1e-7;
constexpr double kTwoPi = 2.0 * std::numbers::pi;
constexpr double kAxisTolRad = 1.5 * std::numbers::pi / 180.0;   ///< Near-horizontal / vertical auto-constraint.

/// Everything an undo step restores: the sketch and the session's driven (reference) dimensions.
struct DocSnapshot {
    Sketch sketch;
    std::vector<DrivenDimension> driven;
};

/// One undo step: the whole document state before and after the edit. Snapshots keep the entity ids, so undo and
/// redo restore exactly the same ids and the id counter never hands out a retired id again.
class SketchEditCommand final : public Command {
public:
    SketchEditCommand(std::string text, Sketch* target, std::vector<DrivenDimension>* drivenTarget,
                      std::shared_ptr<const DocSnapshot> before, std::shared_ptr<const DocSnapshot> after)
        : text_(std::move(text)), target_(target), drivenTarget_(drivenTarget), before_(std::move(before)),
          after_(std::move(after)) {}

    Result<void> execute() override {
        *target_ = after_->sketch;
        *drivenTarget_ = after_->driven;
        return {};
    }
    void undo() override {
        *target_ = before_->sketch;
        *drivenTarget_ = before_->driven;
    }
    std::string text() const override { return text_; }

private:
    std::string text_;
    Sketch* target_;
    std::vector<DrivenDimension>* drivenTarget_;
    std::shared_ptr<const DocSnapshot> before_;
    std::shared_ptr<const DocSnapshot> after_;
};

// ---- small vector helpers -----------------------------------------------------------------------------------
double dotp(Point2 a, Point2 b) { return a.x * b.x + a.y * b.y; }
double crossp(Point2 a, Point2 b) { return a.x * b.y - a.y * b.x; }
double lenp(Point2 a) { return std::hypot(a.x, a.y); }
Point2 unitp(Point2 a) {
    const double l = lenp(a);
    return l > 0 ? a * (1.0 / l) : Point2{0, 0};
}
Point2 rot90(Point2 a) { return {-a.y, a.x}; }
double wrapAngle(double a) {
    a = std::fmod(a, kTwoPi);
    return a < 0 ? a + kTwoPi : a;
}
Point2 polar(Point2 c, double r, double a) { return {c.x + r * std::cos(a), c.y + r * std::sin(a)}; }
double angleOf(Point2 c, Point2 p) { return std::atan2(p.y - c.y, p.x - c.x); }
double segmentDistance(Point2 p, Point2 a, Point2 b) {
    const Point2 ab = b - a;
    const double len2 = dotp(ab, ab);
    double t = 0;
    if (len2 > 0) t = std::clamp(dotp(p - a, ab) / len2, 0.0, 1.0);
    return sketch::distance(p, a + ab * t);
}

/// Geometry of a curve for the intersection and projection maths. A line is `p + t*dir` (bounded: t in [0,1]).
/// A circle is `c, r`; with `arc` only the CCW angular range a0..a0+sweep counts.
struct Geo {
    bool circle = false;
    bool bounded = false;
    bool arc = false;
    Point2 p{}, dir{};
    Point2 c{};
    double r = 0;
    double a0 = 0, sweep = kTwoPi;
};

std::optional<Geo> geoOf(const EntityGeometry& g) {
    return std::visit(
        [](const auto& x) -> std::optional<Geo> {
            using T = std::decay_t<decltype(x)>;
            Geo o;
            if constexpr (std::is_same_v<T, sketch::PointGeom>) {
                return std::nullopt;
            } else if constexpr (std::is_same_v<T, LineGeom>) {
                o.p = x.start;
                o.dir = x.end - x.start;
                o.bounded = true;
            } else if constexpr (std::is_same_v<T, sketch::ArcGeom>) {
                o.circle = o.arc = true;
                o.c = x.center;
                o.r = x.radius;
                o.a0 = x.startAngle;
                o.sweep = x.sweep;
            } else {
                o.circle = true;
                o.c = x.center;
                o.r = x.radius;
            }
            return o;
        },
        g);
}

bool inArc(const Geo& g, Point2 q) {
    if (!g.arc) return true;
    const double rel = wrapAngle(angleOf(g.c, q) - g.a0);
    return rel <= g.sweep + 1e-9 || rel >= kTwoPi - 1e-9;
}

double lineParam(const Geo& g, Point2 q) { return dotp(q - g.p, g.dir) / dotp(g.dir, g.dir); }

/// Point of `g` nearest to q that lies on the curve (bounded lines clamp; arcs need the nearest point in range).
std::optional<Point2> nearestOn(const Geo& g, Point2 q) {
    if (!g.circle) {
        double t = lineParam(g, q);
        if (g.bounded) t = std::clamp(t, 0.0, 1.0);
        return g.p + g.dir * t;
    }
    const Point2 v = q - g.c;
    const double ang = (lenp(v) > kEps) ? std::atan2(v.y, v.x) : g.a0;
    const Point2 at = polar(g.c, g.r, ang);
    if (!inArc(g, at)) return std::nullopt;
    return at;
}

/// Intersection points of two curves that lie on both (bounded segments, arc ranges). Tangency gives one point.
std::vector<Point2> roots(const Geo& A, const Geo& B) {
    std::vector<Point2> out;
    auto inBounds = [](const Geo& g, double t) { return !g.bounded || (t >= -1e-9 && t <= 1 + 1e-9); };
    if (!A.circle && !B.circle) {
        const double den = crossp(A.dir, B.dir);
        if (std::abs(den) <= kEps * lenp(A.dir) * lenp(B.dir)) return out;   // parallel
        const Point2 w = B.p - A.p;
        const double t = crossp(w, B.dir) / den;
        const double u = crossp(w, A.dir) / den;
        if (inBounds(A, t) && inBounds(B, u)) out.push_back(A.p + A.dir * t);
        return out;
    }
    if (A.circle && B.circle) {
        const Point2 d = B.c - A.c;
        const double dd = lenp(d);
        if (dd < kEps || dd > A.r + B.r + 1e-9 || dd < std::abs(A.r - B.r) - 1e-9) return out;
        const double a = (A.r * A.r - B.r * B.r + dd * dd) / (2 * dd);
        const double h = std::sqrt(std::max(0.0, A.r * A.r - a * a));
        const Point2 m = A.c + d * (a / dd);
        const Point2 perp = rot90(d) * (1.0 / dd);
        for (double s : {1.0, -1.0}) {
            if (h < 1e-12 && s < 0) break;
            const Point2 q = m + perp * (s * h);
            if (inArc(A, q) && inArc(B, q)) out.push_back(q);
        }
        return out;
    }
    const Geo& L = A.circle ? B : A;
    const Geo& C = A.circle ? A : B;
    const Point2 f = L.p - C.c;
    const double qa = dotp(L.dir, L.dir), qb = 2 * dotp(f, L.dir), qc = dotp(f, f) - C.r * C.r;
    const double disc = qb * qb - 4 * qa * qc;
    if (disc < 0) return out;
    const double sq = std::sqrt(disc);
    for (double t : {(-qb - sq) / (2 * qa), (-qb + sq) / (2 * qa)}) {
        if (!inBounds(L, t)) continue;
        const Point2 q = L.p + L.dir * t;
        if (!inArc(C, q)) continue;
        if (!out.empty() && sketch::distance(out.back(), q) < 1e-9) continue;
        out.push_back(q);
    }
    return out;
}

/// Tangent points on a circle/arc seen from external point `from`.
std::vector<Point2> tangentPoints(const Geo& C, Point2 from) {
    std::vector<Point2> out;
    const double d = lenp(from - C.c);
    if (d <= C.r + kEps) return out;
    const double th = angleOf(C.c, from), al = std::acos(C.r / d);
    for (double a : {th + al, th - al}) {
        const Point2 t = polar(C.c, C.r, a);
        if (inArc(C, t)) out.push_back(t);
    }
    return out;
}

/// Circle through three points: centre, radius; nullopt when collinear.
struct Circumcircle {
    Point2 c;
    double r;
};
std::optional<Circumcircle> circumcircle(Point2 a, Point2 b, Point2 c) {
    const double d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if (std::abs(d) < kEps) return std::nullopt;
    const double a2 = dotp(a, a), b2 = dotp(b, b), c2 = dotp(c, c);
    const Point2 u{(a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d,
                   (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d};
    return Circumcircle{u, sketch::distance(u, a)};
}

/// Arc through a, b, c (b on the arc). `aIsStart`: the arc starts at a and ends at c; otherwise it starts at c.
struct ThreePointArc {
    sketch::ArcGeom geom;
    bool aIsStart = true;
};
std::optional<ThreePointArc> threePointArc(Point2 a, Point2 b, Point2 c) {
    const auto cc = circumcircle(a, b, c);
    if (!cc || cc->r < kEps) return std::nullopt;
    const double aa = angleOf(cc->c, a), ab = angleOf(cc->c, b), ac = angleOf(cc->c, c);
    const double sweepAC = wrapAngle(ac - aa), sweepAB = wrapAngle(ab - aa);
    ThreePointArc out;
    if (sweepAB < sweepAC) {
        out.geom = {cc->c, cc->r, aa, sweepAC};
        out.aIsStart = true;
    } else {
        out.geom = {cc->c, cc->r, ac, wrapAngle(aa - ac)};
        out.aIsStart = false;
    }
    return out;
}

/// Tangent arc from `s` continuing direction `t` (unit) to `e`. farIsStart: the arc starts at e (it runs CW from s).
struct TangentArc {
    sketch::ArcGeom geom;
    bool farIsStart = false;
    Point2 farDir{};   ///< Direction of travel at e, for the next tangent continuation.
};
std::optional<TangentArc> tangentArc(Point2 s, Point2 t, Point2 e) {
    const Point2 n = rot90(t);
    const double side = dotp(e - s, n);
    if (std::abs(side) < kEps) return std::nullopt;   // e on the tangent line: a straight continuation
    const double r = dotp(e - s, e - s) / (2 * std::abs(side));
    TangentArc out;
    if (side > 0) {
        const Point2 c = s + n * r;
        const double a0 = angleOf(c, s);
        out.geom = {c, r, a0, wrapAngle(angleOf(c, e) - a0)};
        out.farIsStart = false;
        out.farDir = rot90(e - c) * (1.0 / r);
    } else {
        const Point2 c = s - n * r;
        const double a0 = angleOf(c, e);
        out.geom = {c, r, a0, wrapAngle(angleOf(c, s) - a0)};
        out.farIsStart = true;
        out.farDir = rot90(e - c) * (-1.0 / r);
    }
    return out;
}

/// Adds the four sides of the rectangle through corners a and b. `hitA`/`hitB` are the snaps of the two corners
/// (constrained only when `autoOn`); side 0 starts at a, side 1 ends at b.
Result<void> addRectangle(Sketch& s, Point2 a, Point2 b, std::array<SketchEntityId, 4>& out,
                          const std::optional<SnapHit>& hitA, const std::optional<SnapHit>& hitB,
                          bool autoOn);

} // namespace

// ---- snapping and constraint helpers shared by the tools ------------------------------------------------------
namespace {

/// Constraint a snapped point implies for the point `mine` of a new entity: coincident, midpoint, or PointOnObject for
/// on-curve and intersection snaps. Other snap kinds imply nothing for a single point.
void constrainPoint(Sketch& s, SketchRef mine, const SnapHit& hit) {
    switch (hit.kind) {
    case SnapKind::Endpoint:
    case SnapKind::Center:
        if (hit.ref) (void)s.addConstraint(ConstraintKind::Coincident, {mine, *hit.ref});
        break;
    case SnapKind::Midpoint:
        if (hit.ref) {
            const sketch::SketchEntity* e = s.findEntity(hit.ref->entity);
            if (e && e->kind() == EntityKind::Line) (void)s.addConstraint(ConstraintKind::Midpoint, {mine, *hit.ref});
        }
        break;
    case SnapKind::OnCurve:
        if (hit.ref) (void)s.addConstraint(ConstraintKind::PointOnObject, {mine, *hit.ref});
        break;
    case SnapKind::Intersection:
        if (hit.ref) (void)s.addConstraint(ConstraintKind::PointOnObject, {mine, *hit.ref});
        if (hit.other) (void)s.addConstraint(ConstraintKind::PointOnObject, {mine, *hit.other});
        break;
    default:
        break;
    }
}

Result<void> addRectangle(Sketch& s, Point2 a, Point2 b, std::array<SketchEntityId, 4>& out,
                          const std::optional<SnapHit>& hitA, const std::optional<SnapHit>& hitB, bool autoOn) {
    const Point2 p0 = a, p1{b.x, a.y}, p2 = b, p3{a.x, b.y};
    const std::array<std::pair<Point2, Point2>, 4> sides{{{p0, p1}, {p1, p2}, {p2, p3}, {p3, p0}}};
    for (std::size_t i = 0; i < 4; ++i) {
        auto id = s.addLine(sides[i].first, sides[i].second);
        if (!id) return std::unexpected(id.error());
        out[i] = *id;
    }
    // Corner i joins the end of side i and the start of side i+1 (mod 4).
    for (std::size_t i = 0; i < 4; ++i) {
        auto c = s.addConstraint(ConstraintKind::Coincident,
                                 {SketchRef{out[i], PointRole::End}, SketchRef{out[(i + 1) % 4], PointRole::Start}});
        if (!c) return std::unexpected(c.error());
    }
    for (std::size_t i : {0u, 2u}) {
        auto c = s.addConstraint(ConstraintKind::Horizontal, {SketchRef{out[i], std::nullopt}});
        if (!c) return std::unexpected(c.error());
    }
    for (std::size_t i : {1u, 3u}) {
        auto c = s.addConstraint(ConstraintKind::Vertical, {SketchRef{out[i], std::nullopt}});
        if (!c) return std::unexpected(c.error());
    }
    if (autoOn) {
        // Corner a is the start of side 0, corner b the end of side 1.
        if (hitA) constrainPoint(s, SketchRef{out[0], PointRole::Start}, *hitA);
        if (hitB) constrainPoint(s, SketchRef{out[1], PointRole::End}, *hitB);
    }
    return {};
}

/// Adds a curve equal to `g` (same kind) to `s`. Used when trimming splits an entity.
Result<SketchEntityId> addPiece(Sketch& s, const EntityGeometry& g, bool construction) {
    return std::visit(
        [&](const auto& x) -> Result<SketchEntityId> {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, LineGeom>) return s.addLine(x.start, x.end, construction);
            else if constexpr (std::is_same_v<T, sketch::ArcGeom>)
                return s.addArc(x.center, x.radius, x.startAngle, x.sweep, construction);
            else return makeError(ErrorCode::InvalidArgument, "piece kind not supported");
        },
        g);
}

/// Constraint kinds that still describe a sub-piece when they reference the whole curve it came from.
bool keepsOnPiece(ConstraintKind k) {
    switch (k) {
    case ConstraintKind::Horizontal:
    case ConstraintKind::Vertical:
    case ConstraintKind::Parallel:
    case ConstraintKind::Perpendicular:
    case ConstraintKind::Tangent:
    case ConstraintKind::Collinear:
    case ConstraintKind::Concentric:
    case ConstraintKind::Fix:
        return true;
    default:
        return false;
    }
}

} // namespace

SketchSession::SketchSession(sketch::SketchPlane plane) : sketch_(std::move(plane)) { refresh(); }

SketchSession::SketchSession(sketch::Sketch existing) : sketch_(std::move(existing)) {
    // Reference dimensions leave the solver's sketch (it would solve them) and wait in driven_ under their own ids.
    std::vector<sketch::SketchDimensionId> drivenIds;
    for (const auto& [id, d] : sketch_.dimensions())
        if (d.driven) {
            driven_.push_back(DrivenDimension{id.value(), d.kind, d.refs});
            drivenIds.push_back(id);
        }
    for (sketch::SketchDimensionId id : drivenIds) (void)sketch_.removeDimension(id);
    refresh();
}

Result<Sketch> SketchSession::toSketch() const {
    Sketch out = sketch_;
    for (const DrivenDimension& d : driven_) {
        auto r = out.restoreDimension(sketch::SketchDimensionId{d.id}, d.kind, d.refs, std::string{}, true);
        if (!r) return std::unexpected(r.error());
    }
    return out;
}

SketchSession::~SketchSession() = default;

namespace {

/// Driven dimensions whose geometry still exists.
std::vector<DrivenDimension> pruneDriven(const Sketch& s, std::vector<DrivenDimension> d) {
    std::erase_if(d, [&](const DrivenDimension& dd) {
        return std::any_of(dd.refs.begin(), dd.refs.end(), [&](const SketchRef& r) { return !s.findEntity(r.entity); });
    });
    return d;
}

} // namespace

Result<void> SketchSession::commit(std::string text, Sketch next, std::vector<DrivenDimension> drivenNext) {
    auto before = std::make_shared<const DocSnapshot>(DocSnapshot{sketch_, driven_});
    auto after = std::make_shared<const DocSnapshot>(DocSnapshot{std::move(next), std::move(drivenNext)});
    auto cmd = std::make_unique<SketchEditCommand>(std::move(text), &sketch_, &driven_, std::move(before),
                                                   std::move(after));
    if (auto r = undo_.push(std::move(cmd)); !r) return std::unexpected(r.error());
    refresh();
    return {};
}

Result<void> SketchSession::apply(std::string text, const std::function<Result<void>(Sketch&)>& edit) {
    Sketch next = sketch_;
    if (auto r = edit(next); !r) return std::unexpected(r.error());
    const sketch::SketchSolveReport solved = sketch::solveSketch(next, params_);
    if (!solved.ok()) {
        return makeError(ErrorCode::InvalidArgument,
                         solved.message.empty() ? std::string("the sketch constraints conflict") : solved.message);
    }
    std::vector<DrivenDimension> drivenNext = pruneDriven(next, driven_);
    return commit(std::move(text), std::move(next), std::move(drivenNext));
}

Result<void> SketchSession::applyAuto(std::string text, const std::function<Result<void>(Sketch&, bool)>& edit) {
    auto first = apply(text, [&](Sketch& s) { return edit(s, true); });
    if (first) return first;
    return apply(std::move(text), [&](Sketch& s) { return edit(s, false); });
}

Result<SketchEntityId> SketchSession::addLine(Point2 a, Point2 b) {
    SketchEntityId created;
    auto r = apply("Create Line", [&](Sketch& s) -> Result<void> {
        auto id = s.addLine(a, b);
        if (!id) return std::unexpected(id.error());
        created = *id;
        return {};
    });
    if (!r) return std::unexpected(r.error());
    return created;
}

Result<std::array<SketchEntityId, 4>> SketchSession::addRect2pt(Point2 a, Point2 b) {
    std::array<SketchEntityId, 4> ids{};
    auto r = apply("Create Rectangle", [&](Sketch& s) { return addRectangle(s, a, b, ids, {}, {}, false); });
    if (!r) return std::unexpected(r.error());
    return ids;
}

Result<std::array<SketchEntityId, 4>> SketchSession::addRectCenter(Point2 center, Point2 corner) {
    return addRect2pt(center * 2.0 - corner, corner);
}

Result<SketchEntityId> SketchSession::addCircleCenter(Point2 center, double radius) {
    SketchEntityId created;
    auto r = apply("Create Circle", [&](Sketch& s) -> Result<void> {
        auto id = s.addCircle(center, radius);
        if (!id) return std::unexpected(id.error());
        created = *id;
        return {};
    });
    if (!r) return std::unexpected(r.error());
    return created;
}

Result<SketchEntityId> SketchSession::addPoint(Point2 p) {
    SketchEntityId created;
    auto r = apply("Create Point", [&](Sketch& s) -> Result<void> {
        auto id = s.addPoint(p);
        if (!id) return std::unexpected(id.error());
        created = *id;
        return {};
    });
    if (!r) return std::unexpected(r.error());
    return created;
}

Result<SketchEntityId> SketchSession::addArc3(Point2 a, Point2 through, Point2 b) {
    const auto g = threePointArc(a, through, b);
    if (!g) return makeError(ErrorCode::InvalidArgument, "the three points of an arc must not be collinear");
    SketchEntityId created;
    auto r = apply("Create Arc", [&](Sketch& s) -> Result<void> {
        auto id = s.addArc(g->geom.center, g->geom.radius, g->geom.startAngle, g->geom.sweep);
        if (!id) return std::unexpected(id.error());
        created = *id;
        return {};
    });
    if (!r) return std::unexpected(r.error());
    return created;
}

Result<void> SketchSession::toggleConstruction(SketchEntityId id) {
    const sketch::SketchEntity* e = sketch_.findEntity(id);
    if (!e) return makeError(ErrorCode::NotFound, "no such sketch entity");
    const bool next = !e->construction;
    return apply("Toggle Construction", [&](Sketch& s) -> Result<void> { return s.setConstruction(id, next); });
}

void SketchSession::setTool(SketchTool t) {
    clearPending();
    tool_ = t;
}

// ---- inference ------------------------------------------------------------------------------------------------

void SketchSession::setInference(InferenceSettings s) {
    inference_ = s;
    snapHint_ = inference_.enabled && cursor_ ? snapAt(*cursor_) : std::nullopt;
}

std::optional<SnapHit> SketchSession::snapAt(Point2 p) const {
    if (!inference_.enabled) return std::nullopt;
    const double R = inference_.radiusMm;

    struct Candidate {
        int tier;     ///< Index into the priority order (lower wins).
        double dist;
        SnapHit hit;
    };
    std::vector<Candidate> cands;
    auto addPoint = [&](int tier, Point2 at, SnapHit h) {
        const double d = sketch::distance(p, at);
        if (d > R + 1e-12) return;
        h.point = at;
        cands.push_back({tier, d, std::move(h)});
    };

    // Tier 0 endpoint, 1 centre, 2 midpoint. Also collect the points used for alignment.
    std::vector<Point2> alignPts;
    std::vector<std::pair<SketchEntityId, Geo>> curves;
    for (const auto& [id, e] : sketch_.entities()) {
        if (auto g = geoOf(e.geometry)) curves.emplace_back(id, *g);
        std::visit(
            [&](const auto& x) {
                using T = std::decay_t<decltype(x)>;
                if constexpr (std::is_same_v<T, sketch::PointGeom>) {
                    addPoint(0, x.p, {SnapKind::Endpoint, x.p, SketchRef{id, std::nullopt}, {}, {}});
                    alignPts.push_back(x.p);
                } else if constexpr (std::is_same_v<T, LineGeom>) {
                    addPoint(0, x.start, {SnapKind::Endpoint, x.start, SketchRef{id, PointRole::Start}, {}, {}});
                    addPoint(0, x.end, {SnapKind::Endpoint, x.end, SketchRef{id, PointRole::End}, {}, {}});
                    const Point2 mid = (x.start + x.end) * 0.5;
                    addPoint(2, mid, {SnapKind::Midpoint, mid, SketchRef{id, std::nullopt}, {}, {}});
                    alignPts.push_back(x.start);
                    alignPts.push_back(x.end);
                } else if constexpr (std::is_same_v<T, sketch::ArcGeom>) {
                    const Point2 st = sketch::arcStart(x), en = sketch::arcEnd(x), md = sketch::arcMid(x);
                    addPoint(0, st, {SnapKind::Endpoint, st, SketchRef{id, PointRole::Start}, {}, {}});
                    addPoint(0, en, {SnapKind::Endpoint, en, SketchRef{id, PointRole::End}, {}, {}});
                    addPoint(2, md, {SnapKind::Midpoint, md, SketchRef{id, std::nullopt}, {}, {}});
                    addPoint(1, x.center, {SnapKind::Center, x.center, SketchRef{id, PointRole::Center}, {}, {}});
                    alignPts.push_back(st);
                    alignPts.push_back(en);
                    alignPts.push_back(x.center);
                } else if constexpr (std::is_same_v<T, sketch::CircleGeom>) {
                    addPoint(1, x.center, {SnapKind::Center, x.center, SketchRef{id, PointRole::Center}, {}, {}});
                    alignPts.push_back(x.center);
                }
            },
            e.geometry);
    }

    // Tier 3: intersections of pairs of curves.
    for (std::size_t i = 0; i < curves.size(); ++i)
        for (std::size_t j = i + 1; j < curves.size(); ++j)
            for (Point2 q : roots(curves[i].second, curves[j].second))
                addPoint(3, q, {SnapKind::Intersection, q, SketchRef{curves[i].first, std::nullopt},
                                SketchRef{curves[j].first, std::nullopt}, {}});

    // Tiers 4-5 need the pending start of the polyline.
    if (lineStart_) {
        const Point2 from = *lineStart_;
        for (const auto& [id, g] : curves) {
            if (g.circle) {
                for (Point2 t : tangentPoints(g, from))
                    addPoint(4, t, {SnapKind::Tangent, t, std::nullopt, SketchRef{id, std::nullopt},
                                    {{from, t}}});
            } else {
                const Point2 n = unitp(rot90(g.dir));
                const Point2 q = from + n * dotp(p - from, n);
                if (lenp(q - from) > kEps)
                    addPoint(5, q, {SnapKind::Perpendicular, q, std::nullopt, SketchRef{id, std::nullopt},
                                    {{from, q}}});
            }
        }
        alignPts.push_back(from);
    }
    if (arcStart_) alignPts.push_back(*arcStart_);

    // Tier 6: nearest point on a curve.
    for (const auto& [id, g] : curves) {
        if (auto q = nearestOn(g, p)) addPoint(6, *q, {SnapKind::OnCurve, *q, SketchRef{id, std::nullopt}, {}, {}});
    }

    // Tier 7: horizontal / vertical alignment with an existing point; the x or y of the cursor is kept otherwise.
    const double big = 1e4;
    std::optional<Point2> alignH, alignV;
    double bestDy = R, bestDx = R;
    for (Point2 q : alignPts) {
        if (std::abs(p.y - q.y) <= bestDy) {
            bestDy = std::abs(p.y - q.y);
            alignH = q;
        }
        if (std::abs(p.x - q.x) <= bestDx) {
            bestDx = std::abs(p.x - q.x);
            alignV = q;
        }
    }
    if (alignH || alignV) {
        SnapHit h;
        h.kind = alignH ? SnapKind::Horizontal : SnapKind::Vertical;
        h.point = {alignV ? alignV->x : p.x, alignH ? alignH->y : p.y};
        if (alignH) h.guides.push_back({{alignH->x - big, alignH->y}, {alignH->x + big, alignH->y}});
        if (alignV) h.guides.push_back({{alignV->x, alignV->y - big}, {alignV->x, alignV->y + big}});
        cands.push_back({7, sketch::distance(p, h.point), std::move(h)});
    }

    if (cands.empty()) return std::nullopt;
    const auto best = std::min_element(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
        return a.tier != b.tier ? a.tier < b.tier : a.dist < b.dist;
    });
    return best->hit;
}

Point2 SketchSession::cursorPoint() const {
    if (snapHint_) return snapHint_->point;
    return cursor_.value_or(Point2{0, 0});
}

void SketchSession::setCursor(std::optional<Point2> p) {
    cursor_ = p;
    snapHint_ = inference_.enabled && p ? snapAt(*p) : std::nullopt;
}

// ---- tangent arc -----------------------------------------------------------------------------------------------

bool SketchSession::canTangentArcAt(Point2 p, double tolMm) const {
    return tool_ == SketchTool::Line && lineStart_ && joint_ && sketch::distance(p, *lineStart_) <= tolMm;
}

Result<SketchEntityId> SketchSession::tangentArcTo(Point2 end) {
    if (!joint_ || !lineStart_) return makeError(ErrorCode::InvalidArgument, "no polyline to continue tangentially");
    const Joint jt = *joint_;
    std::optional<SnapHit> hit = snapAt(end);
    const Point2 target = hit ? hit->point : end;
    const auto ta = tangentArc(jt.p, jt.dir, target);
    if (!ta) return makeError(ErrorCode::InvalidArgument, "the end point lies on the tangent line");

    SketchEntityId created;
    auto r = applyAuto("Create Tangent Arc", [&](Sketch& s, bool autoOn) -> Result<void> {
        auto id = s.addArc(ta->geom.center, ta->geom.radius, ta->geom.startAngle, ta->geom.sweep);
        if (!id) return std::unexpected(id.error());
        created = *id;
        const PointRole nearRole = ta->farIsStart ? PointRole::End : PointRole::Start;
        const PointRole farRole = ta->farIsStart ? PointRole::Start : PointRole::End;
        auto c = s.addConstraint(ConstraintKind::Coincident, {SketchRef{jt.entity, jt.role}, SketchRef{*id, nearRole}});
        if (!c) return std::unexpected(c.error());
        auto t = s.addConstraint(ConstraintKind::Tangent, {SketchRef{jt.entity, std::nullopt}, SketchRef{*id, std::nullopt}});
        if (!t) return std::unexpected(t.error());
        if (autoOn && hit) constrainPoint(s, SketchRef{*id, farRole}, *hit);
        return {};
    });
    if (!r) return std::unexpected(r.error());

    const PointRole farRole = ta->farIsStart ? PointRole::Start : PointRole::End;
    lineStart_ = target;
    lineStartHit_.reset();
    joint_ = Joint{created, farRole, target, ta->farDir};
    return created;
}

// ---- tools -----------------------------------------------------------------------------------------------------

Result<void> SketchSession::click(Point2 p, double pickToleranceMm) {
    switch (tool_) {
    case SketchTool::Select:
        selectAt(p, pickToleranceMm);
        return {};

    case SketchTool::Dimension:
        return clickDimension(p, pickToleranceMm);

    case SketchTool::Trim:
        return trimAt(p, pickToleranceMm);

    case SketchTool::Offset: {
        auto r = offsetAt(p, pickToleranceMm);
        if (!r) return std::unexpected(r.error());
        return {};
    }

    case SketchTool::Line: {
        const std::optional<SnapHit> hit = snapAt(p);
        return clickLine(hit ? hit->point : p, hit);
    }

    case SketchTool::Arc: {
        const std::optional<SnapHit> hit = snapAt(p);
        return clickArc(hit ? hit->point : p, hit);
    }

    case SketchTool::Rectangle:
    case SketchTool::CenterRectangle: {
        const std::optional<SnapHit> hit = snapAt(p);
        const Point2 q = hit ? hit->point : p;
        if (!firstCorner_) {
            firstCorner_ = q;
            firstCornerHit_ = hit;
            return {};
        }
        const bool centred = tool_ == SketchTool::CenterRectangle;
        const Point2 a = centred ? *firstCorner_ * 2.0 - q : *firstCorner_;
        const std::optional<SnapHit> hitA = centred ? std::nullopt : firstCornerHit_;
        std::array<SketchEntityId, 4> ids{};
        auto r = applyAuto(centred ? "Create Centre Rectangle" : "Create Rectangle", [&](Sketch& s, bool autoOn) {
            return addRectangle(s, a, q, ids, hitA, hit, autoOn);
        });
        if (!r) return std::unexpected(r.error());   // degenerate second corner: keep waiting for one
        firstCorner_.reset();
        firstCornerHit_.reset();
        return {};
    }

    case SketchTool::Circle: {
        const std::optional<SnapHit> hit = snapAt(p);
        const Point2 q = hit ? hit->point : p;
        if (!circleCenter_) {
            circleCenter_ = q;
            circleCenterHit_ = hit;
            return {};
        }
        const Point2 center = *circleCenter_;
        const double radius = sketch::distance(center, q);
        const std::optional<SnapHit> cHit = circleCenterHit_;
        SketchEntityId created;
        auto r = applyAuto("Create Circle", [&](Sketch& s, bool autoOn) -> Result<void> {
            auto id = s.addCircle(center, radius);
            if (!id) return std::unexpected(id.error());
            created = *id;
            if (autoOn && cHit) constrainPoint(s, SketchRef{*id, PointRole::Center}, *cHit);
            return {};
        });
        if (!r) return std::unexpected(r.error());
        circleCenter_.reset();
        circleCenterHit_.reset();
        return {};
    }

    case SketchTool::Point: {
        const std::optional<SnapHit> hit = snapAt(p);
        const Point2 q = hit ? hit->point : p;
        SketchEntityId created;
        auto r = applyAuto("Create Point", [&](Sketch& s, bool autoOn) -> Result<void> {
            auto id = s.addPoint(q);
            if (!id) return std::unexpected(id.error());
            created = *id;
            if (autoOn && hit) constrainPoint(s, SketchRef{*id, std::nullopt}, *hit);
            return {};
        });
        if (!r) return std::unexpected(r.error());
        return {};
    }
    }
    return {};
}

Result<void> SketchSession::clickLine(Point2 q, const std::optional<SnapHit>& hit) {
    if (!lineStart_) {
        lineStart_ = q;
        lineStartHit_ = hit;
        joint_.reset();
        return {};
    }
    const Point2 from = *lineStart_;
    const std::optional<Joint> prev = joint_;
    // A joint already connects the start of this segment, so the first point's snap is only used without one.
    const std::optional<SnapHit> fromHit = prev ? std::nullopt : lineStartHit_;
    SketchEntityId created;
    auto r = applyAuto("Create Line", [&](Sketch& s, bool autoOn) -> Result<void> {
        auto id = s.addLine(from, q);
        if (!id) return std::unexpected(id.error());
        created = *id;
        if (prev) {
            auto c = s.addConstraint(ConstraintKind::Coincident,
                                     {SketchRef{prev->entity, prev->role}, SketchRef{*id, PointRole::Start}});
            if (!c) return std::unexpected(c.error());
        }
        if (!autoOn || !inference_.enabled) return {};
        if (fromHit) constrainPoint(s, SketchRef{*id, PointRole::Start}, *fromHit);
        constrainPoint(s, SketchRef{*id, PointRole::End}, hit.value_or(SnapHit{}));
        if (hit && (hit->kind == SnapKind::Perpendicular || hit->kind == SnapKind::Tangent) && hit->other) {
            const ConstraintKind k = hit->kind == SnapKind::Perpendicular ? ConstraintKind::Perpendicular
                                                                          : ConstraintKind::Tangent;
            auto c = s.addConstraint(k, {SketchRef{*id, std::nullopt}, *hit->other});
            if (!c) return std::unexpected(c.error());
            if (hit->kind == SnapKind::Tangent) {
                // The tangent point is on the circle or arc: pin the end there (tangency alone only fixes the distance).
                auto on = s.addConstraint(ConstraintKind::PointOnObject, {SketchRef{*id, PointRole::End}, *hit->other});
                if (!on) return std::unexpected(on.error());
            }
        } else {
            // Near-axis segments become exactly horizontal or vertical.
            const Point2 d = q - from;
            if (std::abs(d.y) <= std::abs(d.x) * std::tan(kAxisTolRad)) {
                (void)s.addConstraint(ConstraintKind::Horizontal, {SketchRef{*id, std::nullopt}});
            } else if (std::abs(d.x) <= std::abs(d.y) * std::tan(kAxisTolRad)) {
                (void)s.addConstraint(ConstraintKind::Vertical, {SketchRef{*id, std::nullopt}});
            }
        }
        return {};
    });
    if (!r) return std::unexpected(r.error());
    lineStart_ = q;
    lineStartHit_ = hit;
    joint_ = Joint{created, PointRole::End, q, unitp(q - from)};
    return {};
}

Result<void> SketchSession::clickArc(Point2 q, const std::optional<SnapHit>& hit) {
    if (!arcStart_) {
        arcStart_ = q;
        arcStartHit_ = hit;
        return {};
    }
    if (!arcMid_) {
        arcMid_ = q;
        return {};
    }
    const Point2 a = *arcStart_, m = *arcMid_;
    const std::optional<SnapHit> aHit = arcStartHit_;
    auto g = threePointArc(a, m, q);
    if (!g) return makeError(ErrorCode::InvalidArgument, "the three points of an arc must not be collinear");
    auto r = applyAuto("Create Arc", [&](Sketch& s, bool autoOn) -> Result<void> {
        auto id = s.addArc(g->geom.center, g->geom.radius, g->geom.startAngle, g->geom.sweep);
        if (!id) return std::unexpected(id.error());
        if (!autoOn || !inference_.enabled) return {};
        const PointRole aRole = g->aIsStart ? PointRole::Start : PointRole::End;
        const PointRole qRole = g->aIsStart ? PointRole::End : PointRole::Start;
        if (aHit) constrainPoint(s, SketchRef{*id, aRole}, *aHit);
        if (hit) constrainPoint(s, SketchRef{*id, qRole}, *hit);
        return {};
    });
    if (!r) return std::unexpected(r.error());
    arcStart_.reset();
    arcStartHit_.reset();
    arcMid_.reset();
    return {};
}

void SketchSession::endPolyline() {
    lineStart_.reset();
    lineStartHit_.reset();
    joint_.reset();
    tangentDrag_.reset();
}

bool SketchSession::cancel() {
    if (tool_ == SketchTool::Select) return false;
    clearPending();
    tool_ = SketchTool::Select;
    return true;
}

bool SketchSession::hasPending() const noexcept {
    return lineStart_ || firstCorner_ || circleCenter_ || arcStart_ || !dimPicks_.empty();
}

std::vector<EntityGeometry> SketchSession::preview() const {
    std::vector<EntityGeometry> out;
    if (!cursor_) return out;
    const Point2 c = cursorPoint();
    if (tangentDrag_ && joint_) {
        if (auto ta = tangentArc(joint_->p, joint_->dir, *tangentDrag_)) out.emplace_back(ta->geom);
        return out;
    }
    if (arcStart_) {
        if (arcMid_) {
            if (auto g = threePointArc(*arcStart_, *arcMid_, c)) out.emplace_back(g->geom);
        } else if (sketch::distance(*arcStart_, c) > kEps) {
            out.emplace_back(LineGeom{*arcStart_, c});
        }
        return out;
    }
    if (lineStart_ && sketch::distance(*lineStart_, c) > kEps) {
        out.emplace_back(LineGeom{*lineStart_, c});
    } else if (firstCorner_ && sketch::distance(*firstCorner_, c) > kEps) {
        const Point2 a = *firstCorner_;
        if (tool_ == SketchTool::CenterRectangle) {
            const Point2 o = a * 2.0 - c;
            const Point2 p0{o.x, o.y}, p1{c.x, o.y}, p3{o.x, c.y};
            out.emplace_back(LineGeom{p0, p1});
            out.emplace_back(LineGeom{p1, c});
            out.emplace_back(LineGeom{c, p3});
            out.emplace_back(LineGeom{p3, p0});
        } else {
            const Point2 p1{c.x, a.y}, p3{a.x, c.y};
            out.emplace_back(LineGeom{a, p1});
            out.emplace_back(LineGeom{p1, c});
            out.emplace_back(LineGeom{c, p3});
            out.emplace_back(LineGeom{p3, a});
        }
    } else if (circleCenter_ && sketch::distance(*circleCenter_, c) > kEps) {
        out.emplace_back(sketch::CircleGeom{*circleCenter_, sketch::distance(*circleCenter_, c)});
    }
    return out;
}

std::optional<SketchEntityId> SketchSession::nearestEntity(Point2 p, double tolMm) const {
    std::optional<SketchEntityId> best;
    double bestDist = tolMm;
    for (const auto& [id, e] : sketch_.entities()) {
        double d = 0;
        if (auto g = geoOf(e.geometry)) {
            auto q = nearestOn(*g, p);
            if (!q) {
                // Arc: distance to its ends when the projection falls outside the arc.
                const auto& arc = std::get<sketch::ArcGeom>(e.geometry);
                d = std::min(sketch::distance(p, sketch::arcStart(arc)), sketch::distance(p, sketch::arcEnd(arc)));
            } else if (g->circle) {
                d = sketch::distance(p, *q);
            } else {
                d = segmentDistance(p, g->p, g->p + g->dir);
            }
        } else {
            d = sketch::distance(p, std::get<sketch::PointGeom>(e.geometry).p);
        }
        if (d <= bestDist && (!best || d < bestDist)) {
            best = id;
            bestDist = d;
        }
    }
    return best;
}

// ---- selection --------------------------------------------------------------------------------------------------

void SketchSession::select(std::optional<SketchEntityId> id) {
    selection_.clear();
    if (id) selection_.push_back(SketchRef{*id, std::nullopt});
    selConstraint_.reset();
    selDimension_.reset();
}

void SketchSession::clearSelection() noexcept {
    selection_.clear();
    selConstraint_.reset();
    selDimension_.reset();
}

void SketchSession::setSelection(std::vector<SketchRef> items) {
    std::erase_if(items, [&](const SketchRef& r) { return !sketch_.findEntity(r.entity); });
    selection_ = std::move(items);
    selConstraint_.reset();
    selDimension_.reset();
}

void SketchSession::selectConstraint(std::optional<sketch::SketchConstraintId> id) {
    selection_.clear();
    selDimension_.reset();
    selConstraint_ = id;
}

void SketchSession::selectDimension(std::optional<DimensionRef> d) {
    selection_.clear();
    selConstraint_.reset();
    selDimension_ = d;
}

std::optional<SketchRef> SketchSession::pickPointAt(Point2 p, double tolMm) const {
    std::optional<SketchRef> best;
    double bestDist = tolMm;
    auto consider = [&](SketchRef ref, Point2 at) {
        const double d = sketch::distance(p, at);
        if (d <= bestDist && (!best || d < bestDist)) {
            best = ref;
            bestDist = d;
        }
    };
    for (const auto& [id, e] : sketch_.entities()) {
        std::visit(
            [&](const auto& x) {
                using T = std::decay_t<decltype(x)>;
                if constexpr (std::is_same_v<T, sketch::PointGeom>) {
                    consider(SketchRef{id, std::nullopt}, x.p);
                } else if constexpr (std::is_same_v<T, LineGeom>) {
                    consider(SketchRef{id, PointRole::Start}, x.start);
                    consider(SketchRef{id, PointRole::End}, x.end);
                } else if constexpr (std::is_same_v<T, sketch::ArcGeom>) {
                    consider(SketchRef{id, PointRole::Start}, sketch::arcStart(x));
                    consider(SketchRef{id, PointRole::End}, sketch::arcEnd(x));
                    consider(SketchRef{id, PointRole::Center}, x.center);
                } else {
                    consider(SketchRef{id, PointRole::Center}, x.center);
                }
            },
            e.geometry);
    }
    return best;
}

std::optional<SketchEntityId> SketchSession::selectAt(Point2 p, double tolMm, bool additive) {
    std::optional<SketchRef> item = pickPointAt(p, tolMm);
    if (!item) {
        if (const auto e = nearestEntity(p, tolMm)) item = SketchRef{*e, std::nullopt};
    }
    selConstraint_.reset();
    selDimension_.reset();
    if (!item) {
        if (!additive) selection_.clear();
        return std::nullopt;
    }
    if (!additive) {
        selection_ = {*item};
    } else if (const auto it = std::find(selection_.begin(), selection_.end(), *item); it != selection_.end()) {
        selection_.erase(it);
    } else {
        selection_.push_back(*item);
    }
    return item->entity;
}

// ---- constraints --------------------------------------------------------------------------------------------------
namespace {

std::string fmtNum(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.4f", v);
    std::string t = buf;
    while (!t.empty() && t.back() == '0') t.pop_back();
    if (!t.empty() && t.back() == '.') t.pop_back();
    if (t == "-0" || t.empty()) t = "0";
    return t;
}

template <class T> bool contains(const std::vector<T>& v, const T& x) { return std::find(v.begin(), v.end(), x) != v.end(); }

/// Where a curve's glyph sits: line / arc middle, a point itself, a circle on its upper right.
Point2 entityAnchor(const EntityGeometry& g) {
    return std::visit(
        [](const auto& x) -> Point2 {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, sketch::PointGeom>) return x.p;
            else if constexpr (std::is_same_v<T, LineGeom>) return (x.start + x.end) * 0.5;
            else if constexpr (std::is_same_v<T, sketch::ArcGeom>) return sketch::arcMid(x);
            else return polar(x.center, x.radius, std::numbers::pi / 4);
        },
        g);
}

const char* glyphLabel(ConstraintKind k) {
    switch (k) {
    case ConstraintKind::Horizontal: return "H";
    case ConstraintKind::Vertical: return "V";
    case ConstraintKind::Parallel: return "//";
    case ConstraintKind::Perpendicular: return "_|_";
    case ConstraintKind::Tangent: return "T";
    case ConstraintKind::Equal: return "=";
    case ConstraintKind::Concentric: return "O";
    case ConstraintKind::Midpoint: return "M";
    case ConstraintKind::Collinear: return "CL";
    case ConstraintKind::Symmetric: return "S";
    case ConstraintKind::Fix: return "F";
    case ConstraintKind::PointOnObject: return "P";
    case ConstraintKind::Coincident: return "";
    }
    return "";
}

} // namespace

Result<std::vector<sketch::SketchConstraintId>> SketchSession::applyConstraint(ConstraintKind kind) {
    if (selection_.empty()) return makeError(ErrorCode::InvalidArgument, "select geometry first");
    std::vector<SketchEntityId> curves;   // whole non-point entities, click order
    std::vector<SketchRef> points;        // sub-points and point entities
    for (const SketchRef& r : selection_) {
        const sketch::SketchEntity* e = sketch_.findEntity(r.entity);
        if (!e) continue;
        if (r.role || e->kind() == EntityKind::Point) {
            points.push_back(r.role ? r : SketchRef{r.entity, std::nullopt});
        } else if (!contains(curves, r.entity)) {
            curves.push_back(r.entity);
        }
    }
    auto kindOf = [&](SketchEntityId id) { return sketch_.findEntity(id)->kind(); };
    auto isLine = [&](SketchEntityId id) { return kindOf(id) == EntityKind::Line; };
    auto isRound = [&](SketchEntityId id) { return kindOf(id) == EntityKind::Circle || kindOf(id) == EntityKind::Arc; };
    auto whole = [](SketchEntityId id) { return SketchRef{id, std::nullopt}; };

    std::vector<std::pair<ConstraintKind, std::vector<SketchRef>>> adds;
    const auto fail = [](const char* text) {
        return makeError(ErrorCode::InvalidArgument, std::string(text));
    };
    switch (kind) {
    case ConstraintKind::Horizontal:
    case ConstraintKind::Vertical:
        if (!curves.empty()) {
            for (SketchEntityId id : curves) {
                if (!isLine(id)) return fail("horizontal and vertical apply to lines or to two points");
                adds.push_back({kind, {whole(id)}});
            }
        } else if (points.size() == 2) {
            adds.push_back({kind, {points[0], points[1]}});
        } else {
            return fail("select lines, or two points");
        }
        break;
    case ConstraintKind::Parallel:
    case ConstraintKind::Perpendicular:
    case ConstraintKind::Collinear:
        if (curves.size() != 2 || !points.empty() || !isLine(curves[0]) || !isLine(curves[1]))
            return fail("select two lines");
        adds.push_back({kind, {whole(curves[0]), whole(curves[1])}});
        break;
    case ConstraintKind::Equal:
        if (curves.size() != 2 || !points.empty() || (isLine(curves[0]) != isLine(curves[1])))
            return fail("select two lines, or two circles / arcs");
        adds.push_back({kind, {whole(curves[0]), whole(curves[1])}});
        break;
    case ConstraintKind::Tangent:
        if (curves.size() != 2 || !points.empty() || (isLine(curves[0]) && isLine(curves[1])))
            return fail("select a line and a circle / arc, or two circles / arcs");
        adds.push_back({kind, {whole(curves[0]), whole(curves[1])}});
        break;
    case ConstraintKind::Concentric:
        if (curves.size() != 2 || !points.empty() || !isRound(curves[0]) || !isRound(curves[1]))
            return fail("select two circles / arcs");
        adds.push_back({kind, {whole(curves[0]), whole(curves[1])}});
        break;
    case ConstraintKind::Coincident:
        if (points.size() != 2 || !curves.empty() || points[0] == points[1]) return fail("select two points");
        adds.push_back({kind, {points[0], points[1]}});
        break;
    case ConstraintKind::Midpoint:
        if (points.size() != 1 || curves.size() != 1 || !isLine(curves[0])) return fail("select a point and a line");
        adds.push_back({kind, {points[0], whole(curves[0])}});
        break;
    case ConstraintKind::PointOnObject:
        if (points.size() != 1 || curves.size() != 1 || points[0].entity == curves[0])
            return fail("select a point and a line, circle or arc");
        adds.push_back({kind, {points[0], whole(curves[0])}});
        break;
    case ConstraintKind::Symmetric:
        if (points.size() == 2 && curves.size() == 1 && isLine(curves[0])) {
            adds.push_back({kind, {points[0], points[1], whole(curves[0])}});
        } else if (points.size() == 3 && curves.empty()) {
            adds.push_back({kind, {points[0], points[1], points[2]}});
        } else {
            return fail("select two points and a line, or three points");
        }
        break;
    case ConstraintKind::Fix: {
        std::vector<SketchEntityId> ents = curves;
        for (const SketchRef& r : points)
            if (!contains(ents, r.entity)) ents.push_back(r.entity);
        for (SketchEntityId id : ents) {
            const bool already = std::any_of(sketch_.constraints().begin(), sketch_.constraints().end(), [&](const auto& kv) {
                return kv.second.kind == ConstraintKind::Fix && kv.second.refs[0].entity == id;
            });
            if (!already) adds.push_back({kind, {whole(id)}});
        }
        if (adds.empty()) return fail("already fixed");
        break;
    }
    }

    Sketch next = sketch_;
    std::vector<sketch::SketchConstraintId> ids;
    for (auto& [k, refs] : adds) {
        auto id = next.addConstraint(k, refs);
        if (!id) return std::unexpected(id.error());
        ids.push_back(*id);
    }
    const sketch::SketchSolveReport rep = sketch::solveSketch(next, params_);
    if (rep.status == sketch::SolverStatus::InvalidInput) return makeError(ErrorCode::InvalidArgument, rep.message);
    if (!rep.ok()) return makeError(ErrorCode::InvalidArgument, "the constraint conflicts with the existing ones");
    for (auto id : ids) {
        if (contains(rep.redundantConstraints, id) || contains(rep.partiallyRedundantConstraints, id))
            return makeError(ErrorCode::InvalidArgument, "the sketch already implies this constraint");
    }
    if (auto r = commit("Add Constraint", std::move(next), pruneDriven(sketch_, driven_)); !r) return std::unexpected(r.error());
    return ids;
}

Result<void> SketchSession::toggleFix() {
    std::vector<SketchEntityId> ents;
    for (const SketchRef& r : selection_)
        if (sketch_.findEntity(r.entity) && !contains(ents, r.entity)) ents.push_back(r.entity);
    if (ents.empty()) return makeError(ErrorCode::InvalidArgument, "select geometry first");
    auto fixOf = [&](SketchEntityId id) -> std::optional<sketch::SketchConstraintId> {
        for (const auto& [cid, c] : sketch_.constraints())
            if (c.kind == ConstraintKind::Fix && c.refs[0].entity == id) return cid;
        return std::nullopt;
    };
    const bool allFixed = std::all_of(ents.begin(), ents.end(), [&](SketchEntityId id) { return fixOf(id).has_value(); });
    if (!allFixed) {
        auto r = applyConstraint(ConstraintKind::Fix);
        if (!r) return std::unexpected(r.error());
        return {};
    }
    std::vector<sketch::SketchConstraintId> ids;
    for (SketchEntityId id : ents) ids.push_back(*fixOf(id));
    return apply("Unfix", [&](Sketch& s) -> Result<void> {
        for (auto cid : ids)
            if (auto r = s.removeConstraint(cid); !r) return r;
        return {};
    });
}

Result<void> SketchSession::removeConstraint(sketch::SketchConstraintId id) {
    return apply("Delete Constraint", [&](Sketch& s) { return s.removeConstraint(id); });
}

Result<void> SketchSession::removeDimension(std::uint64_t id, bool drivenDim) {
    if (drivenDim) {
        std::vector<DrivenDimension> next = driven_;
        const auto n = std::erase_if(next, [&](const DrivenDimension& d) { return d.id == id; });
        if (n == 0) return makeError(ErrorCode::NotFound, "no such dimension");
        return commit("Delete Dimension", sketch_, std::move(next));
    }
    return apply("Delete Dimension", [&](Sketch& s) { return s.removeDimension(sketch::SketchDimensionId{id}); });
}

Result<void> SketchSession::deleteSelection() {
    if (selConstraint_) return removeConstraint(*selConstraint_);
    if (selDimension_) return removeDimension(selDimension_->id, selDimension_->driven);
    std::vector<SketchEntityId> ents;
    for (const SketchRef& r : selection_)
        if (!contains(ents, r.entity)) ents.push_back(r.entity);
    if (ents.empty()) return makeError(ErrorCode::NotFound, "nothing selected");
    return apply("Delete", [&](Sketch& s) -> Result<void> {
        for (SketchEntityId id : ents)
            if (auto r = s.removeEntity(id); !r) return r;
        return {};
    });
}

std::vector<ConstraintGlyph> SketchSession::glyphs() const {
    std::vector<ConstraintGlyph> out;
    std::vector<std::pair<Point2, int>> used;   // anchors already taken, with their slot counts
    for (const auto& [id, c] : sketch_.constraints()) {
        if (c.refs.empty()) continue;
        std::optional<Point2> at;
        auto entityAt = [&](const SketchRef& r) -> std::optional<Point2> {
            const sketch::SketchEntity* e = sketch_.findEntity(r.entity);
            if (!e) return std::nullopt;
            return entityAnchor(e->geometry);
        };
        auto pointAt = [&](const SketchRef& r) -> std::optional<Point2> {
            auto q = sketch_.pointOf(r);
            if (!q) return std::nullopt;
            return *q;
        };
        switch (c.kind) {
        case ConstraintKind::Horizontal:
        case ConstraintKind::Vertical:
            if (c.refs.size() == 1) {
                at = entityAt(c.refs[0]);
            } else {
                const auto a = pointAt(c.refs[0]), b = pointAt(c.refs[1]);
                if (a && b) at = (*a + *b) * 0.5;
            }
            break;
        case ConstraintKind::Coincident:
        case ConstraintKind::Midpoint:
        case ConstraintKind::PointOnObject:
        case ConstraintKind::Symmetric:
            at = pointAt(c.refs[0]);
            break;
        case ConstraintKind::Concentric: {
            const sketch::SketchEntity* e = sketch_.findEntity(c.refs[0].entity);
            if (e) at = pointAt(SketchRef{c.refs[0].entity, PointRole::Center});
            break;
        }
        case ConstraintKind::Tangent: {
            // The tangent point: foot of the round curve's centre on the line, or on the first round curve.
            const sketch::SketchEntity* a = sketch_.findEntity(c.refs[0].entity);
            const sketch::SketchEntity* b = c.refs.size() > 1 ? sketch_.findEntity(c.refs[1].entity) : nullptr;
            if (a && b) {
                const sketch::SketchEntity* ln = a->kind() == EntityKind::Line ? a : (b->kind() == EntityKind::Line ? b : nullptr);
                const sketch::SketchEntity* rd = ln == a ? b : a;
                const auto gr = geoOf(rd->geometry);
                if (ln && gr) {
                    const Geo gl = *geoOf(ln->geometry);
                    at = gl.p + gl.dir * lineParam(gl, gr->c);
                } else if (gr) {
                    const auto g2 = geoOf((rd == a ? b : a)->geometry);
                    if (g2) at = gr->c + unitp(g2->c - gr->c) * gr->r;
                }
            }
            break;
        }
        default:
            at = entityAt(c.refs[0]);
            break;
        }
        if (!at) continue;
        int slot = 0;
        for (auto& [pt, n] : used) {
            if (sketch::distance(pt, *at) < 1e-6) {
                slot = n++;
                break;
            }
        }
        if (slot == 0) used.emplace_back(*at, 1);
        ConstraintGlyph g;
        g.id = id;
        g.kind = c.kind;
        g.anchor = *at;
        g.slot = slot;
        g.label = glyphLabel(c.kind);
        g.conflicting = contains(report_.conflictingConstraints, id) || contains(report_.redundantConstraints, id) ||
                        contains(report_.partiallyRedundantConstraints, id);
        out.push_back(std::move(g));
    }
    return out;
}

ConstraintState SketchSession::constraintState() const {
    const bool bad = report_.status == sketch::SolverStatus::Failed || !report_.conflictingConstraints.empty() ||
                     !report_.conflictingDimensions.empty() || !report_.redundantConstraints.empty() ||
                     !report_.redundantDimensions.empty();
    if (bad) return ConstraintState::Conflicting;
    if (report_.ok() && report_.dof == 0 && !sketch_.entities().empty()) return ConstraintState::Fully;
    return ConstraintState::Under;
}

// ---- dimensions ---------------------------------------------------------------------------------------------------

Result<double> SketchSession::measure(sketch::DimensionKind kind, const std::vector<SketchRef>& refs) const {
    using DK = sketch::DimensionKind;
    auto pt = [&](std::size_t i) { return sketch_.pointOf(refs[i]); };
    switch (kind) {
    case DK::Distance:
    case DK::DistanceX:
    case DK::DistanceY: {
        if (refs.size() != 2) return makeError(ErrorCode::InvalidArgument, "a distance needs two points");
        auto a = pt(0), b = pt(1);
        if (!a) return std::unexpected(a.error());
        if (!b) return std::unexpected(b.error());
        if (kind == DK::Distance) return sketch::distance(*a, *b);
        return kind == DK::DistanceX ? b->x - a->x : b->y - a->y;
    }
    case DK::Radius:
    case DK::Diameter: {
        if (refs.size() != 1) return makeError(ErrorCode::InvalidArgument, "a radius needs one circle or arc");
        const sketch::SketchEntity* e = sketch_.findEntity(refs[0].entity);
        if (!e) return makeError(ErrorCode::NotFound, "no such entity");
        double r = 0;
        if (const auto* c = std::get_if<sketch::CircleGeom>(&e->geometry)) r = c->radius;
        else if (const auto* a = std::get_if<sketch::ArcGeom>(&e->geometry)) r = a->radius;
        else return makeError(ErrorCode::InvalidArgument, "a radius needs a circle or arc");
        return kind == DK::Radius ? r : 2 * r;
    }
    case DK::Angle: {
        if (refs.size() != 2) return makeError(ErrorCode::InvalidArgument, "an angle needs two lines");
        const sketch::SketchEntity* a = sketch_.findEntity(refs[0].entity);
        const sketch::SketchEntity* b = sketch_.findEntity(refs[1].entity);
        if (!a || !b) return makeError(ErrorCode::NotFound, "no such entity");
        const auto* la = std::get_if<LineGeom>(&a->geometry);
        const auto* lb = std::get_if<LineGeom>(&b->geometry);
        if (!la || !lb) return makeError(ErrorCode::InvalidArgument, "an angle needs two lines");
        const Point2 d1 = la->end - la->start, d2 = lb->end - lb->start;
        return std::atan2(crossp(d1, d2), dotp(d1, d2)) * 180.0 / std::numbers::pi;
    }
    }
    return makeError(ErrorCode::InvalidArgument, "unknown dimension kind");
}

Result<SketchSession::DimensionPlan> SketchSession::planDimension(const std::vector<SketchRef>& picks,
                                                                  Point2 mouse) const {
    using DK = sketch::DimensionKind;
    const auto bad = [](const char* t) { return makeError(ErrorCode::InvalidArgument, std::string(t)); };
    auto entityOf = [&](const SketchRef& r) { return sketch_.findEntity(r.entity); };
    auto isPointRef = [&](const SketchRef& r) {
        const sketch::SketchEntity* e = entityOf(r);
        return e && (r.role || e->kind() == EntityKind::Point);
    };
    DimensionPlan plan;
    if (picks.size() == 1) {
        const sketch::SketchEntity* e = entityOf(picks[0]);
        if (!e || isPointRef(picks[0])) return bad("pick a line, circle or arc, or two points");
        switch (e->kind()) {
        case EntityKind::Line:
            plan.kind = DK::Distance;
            plan.refs = {SketchRef{picks[0].entity, PointRole::Start}, SketchRef{picks[0].entity, PointRole::End}};
            break;
        case EntityKind::Circle:
            plan.kind = DK::Diameter;
            plan.refs = {SketchRef{picks[0].entity, std::nullopt}};
            break;
        case EntityKind::Arc:
            plan.kind = DK::Radius;
            plan.refs = {SketchRef{picks[0].entity, std::nullopt}};
            break;
        default:
            return bad("pick a line, circle or arc, or two points");
        }
        return plan;
    }
    if (picks.size() != 2) return bad("pick one or two things to dimension");
    if (isPointRef(picks[0]) && isPointRef(picks[1])) {
        auto a = sketch_.pointOf(picks[0]), b = sketch_.pointOf(picks[1]);
        if (!a || !b) return bad("bad point");
        if (sketch::distance(*a, *b) < kEps) return bad("the two points coincide");
        const double x0 = std::min(a->x, b->x), x1 = std::max(a->x, b->x);
        const double y0 = std::min(a->y, b->y), y1 = std::max(a->y, b->y);
        const bool inX = mouse.x >= x0 && mouse.x <= x1, inY = mouse.y >= y0 && mouse.y <= y1;
        plan.refs = {picks[0], picks[1]};
        if (inX && !inY) {
            plan.kind = DK::DistanceX;
            if (b->x < a->x) std::swap(plan.refs[0], plan.refs[1]);
        } else if (inY && !inX) {
            plan.kind = DK::DistanceY;
            if (b->y < a->y) std::swap(plan.refs[0], plan.refs[1]);
        } else {
            plan.kind = DK::Distance;
        }
        return plan;
    }
    const sketch::SketchEntity* ea = entityOf(picks[0]);
    const sketch::SketchEntity* eb = entityOf(picks[1]);
    if (ea && eb && !isPointRef(picks[0]) && !isPointRef(picks[1]) && ea->kind() == EntityKind::Line &&
        eb->kind() == EntityKind::Line && picks[0].entity != picks[1].entity) {
        plan.kind = DK::Angle;
        plan.refs = {SketchRef{picks[0].entity, std::nullopt}, SketchRef{picks[1].entity, std::nullopt}};
        auto m = measure(DK::Angle, plan.refs);
        if (m && *m < 0) std::swap(plan.refs[0], plan.refs[1]);
        return plan;
    }
    return bad("pick two points or two lines");
}

Result<Quantity> SketchSession::evaluateExpression(sketch::DimensionKind kind, const std::string& expression) const {
    auto parsed = Expression::parse(expression);
    if (!parsed) return std::unexpected(parsed.error());
    ParameterLookup lookup = [this](std::string_view name) -> Result<Quantity> {
        if (!params_) return makeError(ErrorCode::NotFound, "unknown parameter '" + std::string(name) + "'");
        return params_->evaluate(name);
    };
    auto q = parsed->evaluate(lookup);
    if (!q) return std::unexpected(q.error());
    const bool angle = kind == sketch::DimensionKind::Angle;
    if (q->dim == Dimension::Unitless)
        return angle ? Quantity::radians(q->value * std::numbers::pi / 180.0) : Quantity::mm(q->value);
    if (angle ? q->dim == Dimension::Angle : q->dim == Dimension::Length) return *q;
    return makeError(ErrorCode::InvalidArgument, "the expression is a " + dimensionName(q->dim) + ", expected " +
                                                     (angle ? "an angle" : "a length"));
}

Result<SketchSession::DimensionTry> SketchSession::addDimension(sketch::DimensionKind kind, std::vector<SketchRef> refs,
                                                                std::string expression) {
    if (expression.empty()) {
        auto m = measure(kind, refs);
        if (!m) return std::unexpected(m.error());
        expression = fmtNum(*m);
        if (kind == sketch::DimensionKind::Angle) expression += " deg";
    } else if (auto v = evaluateExpression(kind, expression); !v) {
        return std::unexpected(v.error());
    }
    Sketch next = sketch_;
    auto id = next.addDimension(kind, std::move(refs), expression);
    if (!id) return std::unexpected(id.error());
    const sketch::SketchSolveReport rep = sketch::solveSketch(next, params_);
    DimensionTry out;
    if (rep.status == sketch::SolverStatus::InvalidInput) return makeError(ErrorCode::InvalidArgument, rep.message);
    if (!rep.ok()) {
        out.overConstrains = true;
        out.message = "the dimension conflicts with the existing constraints";
        return out;
    }
    if (contains(rep.redundantDimensions, *id) || contains(rep.partiallyRedundantDimensions, *id)) {
        out.overConstrains = true;
        out.message = "the sketch already fixes this measure";
        return out;
    }
    if (auto r = commit("Add Dimension", std::move(next), driven_); !r) return std::unexpected(r.error());
    out.id = *id;
    return out;
}

Result<std::uint64_t> SketchSession::addDrivenDimension(sketch::DimensionKind kind, std::vector<SketchRef> refs) {
    if (auto m = measure(kind, refs); !m) return std::unexpected(m.error());
    for (const SketchRef& r : refs)
        if (!sketch_.findEntity(r.entity)) return makeError(ErrorCode::NotFound, "no such entity");
    // The id comes from the sketch's counter (shared with every other record); the counter is advanced past it.
    Sketch next = sketch_;
    const std::uint64_t id = next.nextId();
    if (auto r = next.restoreNextId(id + 1); !r) return std::unexpected(r.error());
    std::vector<DrivenDimension> nextDriven = driven_;
    nextDriven.push_back(DrivenDimension{id, kind, std::move(refs)});
    if (auto r = commit("Add Reference Dimension", std::move(next), std::move(nextDriven)); !r)
        return std::unexpected(r.error());
    return id;
}

Result<void> SketchSession::setDimensionExpression(sketch::SketchDimensionId id, std::string expression) {
    const auto it = sketch_.dimensions().find(id);
    if (it == sketch_.dimensions().end()) return makeError(ErrorCode::NotFound, "no such dimension");
    if (auto v = evaluateExpression(it->second.kind, expression); !v) return std::unexpected(v.error());
    return apply("Edit Dimension", [&](Sketch& s) { return s.setDimensionExpression(id, expression); });
}

std::vector<DimensionView> SketchSession::dimensionViews() const {
    std::vector<DimensionView> out;
    auto prefix = [](sketch::DimensionKind k) -> std::string {
        return k == sketch::DimensionKind::Radius ? "R" : (k == sketch::DimensionKind::Diameter ? "\xC3\x98" : "");
    };
    auto suffix = [](sketch::DimensionKind k) -> std::string { return k == sketch::DimensionKind::Angle ? "\xC2\xB0" : ""; };
    for (const auto& [id, d] : sketch_.dimensions()) {
        DimensionView v;
        v.id = id.value();
        v.kind = d.kind;
        v.refs = d.refs;
        v.expression = d.expression;
        v.value = std::numeric_limits<double>::quiet_NaN();
        v.conflicting = contains(report_.conflictingDimensions, id) || contains(report_.redundantDimensions, id) ||
                        contains(report_.partiallyRedundantDimensions, id);
        if (auto q = evaluateExpression(d.kind, d.expression)) {
            v.value = d.kind == sketch::DimensionKind::Angle ? q->value * 180.0 / std::numbers::pi : q->value;
            bool literal = false;
            if (auto parsed = Expression::parse(d.expression)) literal = parsed->root().kind == ExprNode::Kind::Literal;
            v.text = prefix(d.kind) + (literal ? "" : d.expression + " = ") + fmtNum(v.value) + suffix(d.kind);
        } else {
            v.text = "?";
            v.conflicting = true;
        }
        out.push_back(std::move(v));
    }
    for (const DrivenDimension& d : driven_) {
        DimensionView v;
        v.id = d.id;
        v.driven = true;
        v.kind = d.kind;
        v.refs = d.refs;
        if (auto m = measure(d.kind, d.refs)) {
            v.value = std::abs(*m);
            v.text = "(" + prefix(d.kind) + fmtNum(v.value) + suffix(d.kind) + ")";
        } else {
            v.value = std::numeric_limits<double>::quiet_NaN();
            v.text = "(?)";
        }
        out.push_back(std::move(v));
    }
    return out;
}

std::optional<DimensionEvent> SketchSession::takeDimensionEvent() {
    auto e = std::move(dimEvent_);
    dimEvent_.reset();
    return e;
}

Result<void> SketchSession::clickDimension(Point2 p, double tolMm) {
    const auto pt = pickPointAt(p, tolMm);
    const auto ent = nearestEntity(p, tolMm);
    auto isPointRef = [&](const SketchRef& r) {
        const sketch::SketchEntity* e = sketch_.findEntity(r.entity);
        return e && (r.role || e->kind() == EntityKind::Point);
    };
    auto place = [&]() -> Result<void> {
        auto plan = planDimension(dimPicks_, p);
        dimPicks_.clear();
        if (!plan) return std::unexpected(plan.error());
        auto t = addDimension(plan->kind, plan->refs);
        if (!t) return std::unexpected(t.error());
        DimensionEvent ev;
        ev.kind = plan->kind;
        ev.refs = plan->refs;
        if (t->overConstrains) {
            ev.type = DimensionEvent::Type::OverConstrains;
            ev.message = t->message;
        } else {
            ev.type = DimensionEvent::Type::Created;
            ev.id = t->id->value();
        }
        dimEvent_ = std::move(ev);
        return {};
    };

    if (dimPicks_.empty()) {
        if (pt) {
            dimPicks_.push_back(*pt);
        } else if (ent) {
            dimPicks_.push_back(SketchRef{*ent, std::nullopt});
        } else {
            return makeError(ErrorCode::InvalidArgument, "pick a line, circle, arc or point to dimension");
        }
        return {};
    }
    if (dimPicks_.size() == 1) {
        const SketchRef first = dimPicks_[0];
        if (isPointRef(first)) {
            if (!pt || *pt == first) return makeError(ErrorCode::InvalidArgument, "pick a second point");
            dimPicks_.push_back(*pt);
            return {};
        }
        const sketch::SketchEntity* fe = sketch_.findEntity(first.entity);
        if (fe && fe->kind() == EntityKind::Line && ent && *ent != first.entity &&
            sketch_.findEntity(*ent)->kind() == EntityKind::Line) {
            dimPicks_.push_back(SketchRef{*ent, std::nullopt});   // two lines: an angle
            return {};
        }
        return place();
    }
    return place();
}

// ---- live drag ----------------------------------------------------------------------------------------------------

Result<void> SketchSession::beginDrag(const SketchRef& point) {
    if (drag_) endDrag();
    const sketch::SketchEntity* e = sketch_.findEntity(point.entity);
    if (!e) return makeError(ErrorCode::NotFound, "no such entity");
    const bool ok = e->kind() == EntityKind::Point ||
                    (point.role && (*point.role == PointRole::Start || *point.role == PointRole::End ||
                                    *point.role == PointRole::Center));
    if (!ok) return makeError(ErrorCode::InvalidArgument, "only points can be dragged");
    if (auto q = sketch_.pointOf(point); !q) return std::unexpected(q.error());
    drag_ = DragState{point, std::make_shared<const Sketch>(sketch_), driven_, false};
    return {};
}

Result<void> SketchSession::dragTo(Point2 target) {
    if (!drag_) return makeError(ErrorCode::InvalidArgument, "no drag in progress");
    const auto cur = sketch_.pointOf(drag_->point);
    if (!cur) return std::unexpected(cur.error());
    // The dragged point is held at the target by a temporary fixed point it is made coincident with. When the
    // constraints cannot follow all the way, retry with half the step.
    for (int attempt = 0; attempt < 5; ++attempt) {
        const Point2 goal = *cur + (target - *cur) * std::pow(0.5, attempt);
        Sketch trial = sketch_;
        auto pin = trial.addPoint(goal, true);
        if (!pin) return std::unexpected(pin.error());
        (void)trial.addConstraint(ConstraintKind::Fix, {SketchRef{*pin, std::nullopt}});
        (void)trial.addConstraint(ConstraintKind::Coincident, {drag_->point, SketchRef{*pin, std::nullopt}});
        if (!sketch::solveSketch(trial, params_).ok()) continue;
        std::vector<SketchEntityId> ids;
        for (const auto& [id, e] : sketch_.entities()) ids.push_back(id);
        for (SketchEntityId id : ids) (void)sketch_.setGeometry(id, trial.findEntity(id)->geometry);
        auto profiles = sketch::detectProfiles(sketch_);
        profiles_ = profiles ? std::move(*profiles) : std::vector<sketch::SketchProfile>{};
        drag_->moved = true;
        return {};
    }
    return makeError(ErrorCode::InvalidArgument, "the constraints do not let the point move there");
}

void SketchSession::endDrag() {
    if (!drag_) return;
    DragState d = std::move(*drag_);
    drag_.reset();
    if (d.moved) {
        auto before = std::make_shared<const DocSnapshot>(DocSnapshot{*d.before, d.beforeDriven});
        auto after = std::make_shared<const DocSnapshot>(DocSnapshot{sketch_, driven_});
        auto cmd = std::make_unique<SketchEditCommand>("Drag", &sketch_, &driven_, std::move(before), std::move(after));
        (void)undo_.push(std::move(cmd));
    }
    refresh();
}

Result<void> SketchSession::resolve() {
    Sketch next = sketch_;
    const sketch::SketchSolveReport rep = sketch::solveSketch(next, params_);
    if (!rep.ok()) {
        return makeError(ErrorCode::InvalidArgument,
                         rep.message.empty() ? std::string("the sketch constraints conflict") : rep.message);
    }
    bool moved = false;
    for (const auto& [id, e] : sketch_.entities()) {
        const sketch::SketchEntity* n = next.findEntity(id);
        if (!n) continue;
        const auto flat = [](const EntityGeometry& g) {
            return std::visit(
                [](const auto& x) -> std::array<double, 5> {
                    using T = std::decay_t<decltype(x)>;
                    if constexpr (std::is_same_v<T, sketch::PointGeom>) return {x.p.x, x.p.y, 0, 0, 0};
                    else if constexpr (std::is_same_v<T, LineGeom>) return {x.start.x, x.start.y, x.end.x, x.end.y, 0};
                    else if constexpr (std::is_same_v<T, sketch::ArcGeom>) return {x.center.x, x.center.y, x.radius, x.startAngle, x.sweep};
                    else return {x.center.x, x.center.y, x.radius, 0, 0};
                },
                g);
        };
        const auto a = flat(e.geometry), b = flat(n->geometry);
        for (std::size_t i = 0; i < a.size(); ++i) moved = moved || std::abs(a[i] - b[i]) > 1e-9;
    }
    if (!moved) {
        refresh();
        return {};
    }
    return commit("Update Parameters", std::move(next), driven_);
}

Result<void> SketchSession::trimAt(Point2 p, double tolMm) {
    const auto target = nearestEntity(p, tolMm);
    if (!target) return makeError(ErrorCode::InvalidArgument, "nothing to trim at that point");
    const sketch::SketchEntity& e = *sketch_.findEntity(*target);
    if (e.kind() == EntityKind::Circle) return trimCircle(*target, p);
    if (e.kind() != EntityKind::Line && e.kind() != EntityKind::Arc)
        return makeError(ErrorCode::InvalidArgument, "only lines, arcs and circles can be trimmed");
    const Geo geo = *geoOf(e.geometry);
    auto paramOf = [&](Point2 q) {
        if (!geo.circle) return lineParam(geo, q);
        return wrapAngle(angleOf(geo.c, q) - geo.a0) / geo.sweep;
    };

    // Cut parameters (0..1 along the target) where other curves cross it strictly inside.
    std::vector<double> cuts;
    for (const auto& [id, o] : sketch_.entities()) {
        if (id == *target) continue;
        auto og = geoOf(o.geometry);
        if (!og) continue;
        for (Point2 q : roots(geo, *og)) {
            const double t = paramOf(q);
            if (t > 1e-6 && t < 1 - 1e-6) cuts.push_back(t);
        }
    }
    const auto tcPoint = nearestOn(geo, p);
    if (!tcPoint) return makeError(ErrorCode::InvalidArgument, "nothing to trim at that point");
    const double tc = paramOf(*tcPoint);

    if (cuts.empty()) {
        return apply("Trim", [&](Sketch& s) { return s.removeEntity(*target); });
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end(), [](double a, double b) { return std::abs(a - b) < 1e-9; }),
               cuts.end());
    double lo = 0, hi = 1;
    for (double c : cuts) {
        if (c <= tc) {
            lo = c;
        } else {
            hi = c;
            break;
        }
    }
    const bool keepLeft = lo > 0, keepRight = hi < 1;

    auto pieceGeom = [&](double t0, double t1) -> EntityGeometry {
        if (!geo.circle) return LineGeom{geo.p + geo.dir * t0, geo.p + geo.dir * t1};
        return sketch::ArcGeom{geo.c, geo.r, geo.a0 + geo.sweep * t0, geo.sweep * (t1 - t0)};
    };
    const bool construction = e.construction;

    return apply("Trim", [&](Sketch& s) -> Result<void> {
        // Dimensions on the trimmed curve no longer describe a piece: they go.
        std::vector<sketch::SketchDimensionId> dims;
        for (const auto& [did, d] : s.dimensions())
            for (const SketchRef& r : d.refs)
                if (r.entity == *target) dims.push_back(did);
        for (auto did : dims)
            if (auto r = s.removeDimension(did); !r) return std::unexpected(r.error());

        // Constraints that touch the curve are taken out and re-created per surviving piece.
        std::vector<sketch::SketchConstraintId> cids;
        std::vector<std::pair<ConstraintKind, std::vector<SketchRef>>> olds;
        for (const auto& [cid, c] : s.constraints()) {
            bool touches = false;
            for (const SketchRef& r : c.refs) touches = touches || r.entity == *target;
            if (!touches) continue;
            cids.push_back(cid);
            olds.emplace_back(c.kind, c.refs);
        }
        for (auto cid : cids)
            if (auto r = s.removeConstraint(cid); !r) return std::unexpected(r.error());

        struct Piece {
            SketchEntityId id;
            double t0, t1;
        };
        std::vector<Piece> pieces;
        if (keepLeft) {
            if (auto r = s.setGeometry(*target, pieceGeom(0, lo)); !r) return std::unexpected(r.error());
            pieces.push_back({*target, 0, lo});
            if (keepRight) {
                auto id = addPiece(s, pieceGeom(hi, 1), construction);
                if (!id) return std::unexpected(id.error());
                pieces.push_back({*id, hi, 1});
            }
        } else if (keepRight) {
            if (auto r = s.setGeometry(*target, pieceGeom(hi, 1)); !r) return std::unexpected(r.error());
            pieces.push_back({*target, hi, 1});
        } else {
            if (auto r = s.removeEntity(*target); !r) return std::unexpected(r.error());
        }

        for (const auto& [kind, refs] : olds) {
            for (const Piece& pc : pieces) {
                std::vector<SketchRef> mapped;
                bool ok = true;
                for (const SketchRef& r : refs) {
                    if (r.entity != *target) {
                        mapped.push_back(r);
                    } else if (!r.role) {
                        if (keepsOnPiece(kind)) {
                            mapped.push_back({pc.id, std::nullopt});
                        } else if (kind == ConstraintKind::PointOnObject && &r == &refs[1] && refs[0].entity != *target) {
                            // The point stays on the curve only if it falls within this piece.
                            const auto q = s.pointOf(refs[0]);
                            const double t = q ? paramOf(*q) : -1.0;
                            if (q && t >= pc.t0 - 1e-9 && t <= pc.t1 + 1e-9) mapped.push_back({pc.id, std::nullopt});
                            else ok = false;
                        } else {
                            ok = false;
                        }
                    } else if (*r.role == PointRole::Start && pc.t0 <= 1e-12) {
                        mapped.push_back({pc.id, PointRole::Start});
                    } else if (*r.role == PointRole::End && pc.t1 >= 1 - 1e-12) {
                        mapped.push_back({pc.id, PointRole::End});
                    } else if (*r.role == PointRole::Center && geo.circle) {
                        mapped.push_back({pc.id, PointRole::Center});
                    } else {
                        ok = false;
                    }
                }
                if (ok) (void)s.addConstraint(kind, mapped);   // a constraint the piece cannot carry is dropped
            }
        }
        return {};
    });
}

Result<void> SketchSession::trimCircle(SketchEntityId target, Point2 p) {
    const sketch::SketchEntity& e = *sketch_.findEntity(target);
    const auto& circ = std::get<sketch::CircleGeom>(e.geometry);
    const Geo geo = *geoOf(e.geometry);
    auto paramOf = [&](Point2 q) {
        double t = wrapAngle(angleOf(circ.center, q)) / kTwoPi;
        return t >= 1 - 1e-12 ? 0.0 : t;
    };

    // Crossings (parameter 0..1 around the circle, CCW from angle 0) with the other curves, and who made them.
    std::vector<std::pair<double, SketchEntityId>> cuts;
    for (const auto& [id, o] : sketch_.entities()) {
        if (id == target) continue;
        auto og = geoOf(o.geometry);
        if (!og) continue;
        for (Point2 q : roots(geo, *og)) cuts.emplace_back(paramOf(q), id);
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end(),
                           [](const auto& a, const auto& b) { return std::abs(a.first - b.first) < 1e-9; }),
               cuts.end());
    if (cuts.empty()) return apply("Trim", [&](Sketch& s) { return s.removeEntity(target); });
    if (cuts.size() == 1)
        return makeError(ErrorCode::InvalidArgument, "a circle needs two crossing curves to be trimmed");

    const double tc = paramOf(p);
    // The piece to remove runs from `lo` CCW to `hi`, the crossings around the click.
    const std::size_t n = cuts.size();
    std::size_t loI = n - 1, hiI = 0;
    double lo = cuts[n - 1].first - 1.0, hi = cuts[0].first;
    for (std::size_t i = 0; i < n; ++i) {
        if (cuts[i].first > tc) break;
        loI = i;
        lo = cuts[i].first;
        hiI = (i + 1) % n;
        hi = i + 1 < n ? cuts[i + 1].first : cuts[0].first + 1.0;
    }
    const double startAngle = wrapAngle(kTwoPi * hi);
    const double sweep = kTwoPi * (1.0 - (hi - lo));
    if (!(sweep > 1e-9)) return makeError(ErrorCode::InvalidArgument, "nothing to trim at that point");
    const SketchEntityId startCutter = cuts[hiI].second, endCutter = cuts[loI].second;
    const bool construction = e.construction;
    const double r = circ.radius;
    const Point2 c = circ.center;

    auto build = [&](Sketch& s, bool autoOn) -> Result<void> {
        struct Old {
            ConstraintKind kind;
            std::vector<SketchRef> refs;
        };
        std::vector<Old> olds;
        for (const auto& [cid, cc] : s.constraints()) {
            if (std::any_of(cc.refs.begin(), cc.refs.end(), [&](const SketchRef& rf) { return rf.entity == target; }))
                olds.push_back({cc.kind, cc.refs});
        }
        struct OldDim {
            sketch::DimensionKind kind;
            std::string expression;
        };
        std::vector<OldDim> dims;
        for (const auto& [did, d] : s.dimensions()) {
            const bool onCircle = std::any_of(d.refs.begin(), d.refs.end(), [&](const SketchRef& rf) { return rf.entity == target; });
            if (onCircle && (d.kind == sketch::DimensionKind::Radius || d.kind == sketch::DimensionKind::Diameter))
                dims.push_back({d.kind, d.expression});
        }
        if (auto rm = s.removeEntity(target); !rm) return std::unexpected(rm.error());   // also drops its records
        auto arc = s.addArc(c, r, startAngle, sweep, construction);
        if (!arc) return std::unexpected(arc.error());
        const SketchEntityId a = *arc;
        for (const Old& o : olds) {
            std::vector<SketchRef> mapped;
            bool ok = true;
            for (const SketchRef& rf : o.refs) {
                if (rf.entity != target) {
                    mapped.push_back(rf);
                } else if (rf.role && *rf.role == PointRole::Center) {
                    mapped.push_back({a, PointRole::Center});
                } else if (!rf.role && (keepsOnPiece(o.kind) || o.kind == ConstraintKind::Equal)) {
                    mapped.push_back({a, std::nullopt});
                } else if (!rf.role && o.kind == ConstraintKind::PointOnObject && &rf == &o.refs[1]) {
                    const auto q = s.pointOf(o.refs[0]);
                    const double rel = q ? wrapAngle(angleOf(c, *q) - startAngle) : kTwoPi;
                    if (q && rel <= sweep + 1e-9) mapped.push_back({a, std::nullopt});
                    else ok = false;
                } else {
                    ok = false;
                }
            }
            if (ok) (void)s.addConstraint(o.kind, mapped);
        }
        for (const OldDim& d : dims) (void)s.addDimension(d.kind, {SketchRef{a, std::nullopt}}, d.expression);
        if (autoOn) {   // the arc's ends stay on the curves that cut it
            (void)s.addConstraint(ConstraintKind::PointOnObject, {SketchRef{a, PointRole::Start}, SketchRef{startCutter, std::nullopt}});
            (void)s.addConstraint(ConstraintKind::PointOnObject, {SketchRef{a, PointRole::End}, SketchRef{endCutter, std::nullopt}});
        }
        return {};
    };
    return applyAuto("Trim", build);
}

std::optional<SketchEntityId> SketchSession::offsetSeedAt(Point2 p, double tolMm) const {
    return nearestEntity(p, tolMm);
}

Result<std::vector<SketchEntityId>> SketchSession::offsetAt(Point2 p, double tolMm) {
    const auto seed = nearestEntity(p, tolMm);
    if (!seed) return makeError(ErrorCode::InvalidArgument, "nothing to offset at that point");
    return offsetLoop(*seed, offsetDistance_);
}

Result<std::vector<SketchEntityId>> SketchSession::offsetLoop(SketchEntityId seed, double d) {
    if (!std::isfinite(d) || std::abs(d) < kEps) return makeError(ErrorCode::InvalidArgument, "offset distance must be non-zero");

    const sketch::ProfileLoop* loop = nullptr;
    for (const sketch::SketchProfile& prof : profiles_) {
        auto has = [&](const sketch::ProfileLoop& l) {
            return std::any_of(l.edges.begin(), l.edges.end(), [&](const sketch::ProfileEdge& e) { return e.entity == seed; });
        };
        if (has(prof.outer)) loop = &prof.outer;
        for (const sketch::ProfileLoop& h : prof.holes)
            if (!loop && has(h)) loop = &h;
        if (loop) break;
    }
    if (!loop) return makeError(ErrorCode::NotFound, "the entity is not on a closed profile");

    // Every edge must be a whole line, arc or circle, each used once.
    std::set<SketchEntityId> seen;
    for (const sketch::ProfileEdge& e : loop->edges) {
        const sketch::SketchEntity* ent = sketch_.findEntity(e.entity);
        if (!ent || !seen.insert(e.entity).second)
            return makeError(ErrorCode::InvalidArgument, "select a closed chain of whole lines and arcs");
        auto near = [](Point2 a, Point2 b) { return sketch::distance(a, b) < 1e-6; };
        bool whole = false;
        if (ent->kind() == EntityKind::Line) {
            const auto& g = std::get<LineGeom>(ent->geometry);
            whole = !e.isArc && ((near(e.start, g.start) && near(e.end, g.end)) || (near(e.start, g.end) && near(e.end, g.start)));
        } else if (ent->kind() == EntityKind::Arc) {
            whole = e.isArc && std::abs(std::abs(e.sweep) - std::get<sketch::ArcGeom>(ent->geometry).sweep) < 1e-6;
        } else if (ent->kind() == EntityKind::Circle) {
            whole = e.isArc && std::abs(std::abs(e.sweep) - kTwoPi) < 1e-6;
        }
        if (!whole) return makeError(ErrorCode::InvalidArgument, "select a closed chain of whole lines and arcs");
    }

    // Edge k: its offset curve, plus the outward normal (away from the loop interior: right of travel for a CCW
    // loop, left for a CW loop) that decides how an arc's radius moves.
    const bool ccw = loop->area > 0;
    struct Edge {
        SketchEntityId id;
        bool line = false;
        Point2 s, e;        ///< Traversal start and end.
        Point2 t;           ///< Line: unit direction of travel.
        Point2 nrm;         ///< Outward normal at the start (lines).
        Geo off;            ///< Offset curve.
        Point2 c;           ///< Arc/circle centre.
        double sweep = 0;   ///< Signed traversal sweep (arcs).
    };
    std::vector<Edge> edges;
    for (const sketch::ProfileEdge& pe : loop->edges) {
        const sketch::SketchEntity* ent = sketch_.findEntity(pe.entity);
        Edge ed;
        ed.id = pe.entity;
        ed.s = pe.start;
        ed.e = pe.end;
        if (ent->kind() == EntityKind::Line) {
            ed.line = true;
            ed.t = unitp(pe.end - pe.start);
            ed.nrm = ccw ? Point2{ed.t.y, -ed.t.x} : Point2{-ed.t.y, ed.t.x};
            ed.off = Geo{};
            ed.off.p = pe.start + ed.nrm * d;
            ed.off.dir = ed.t;
        } else {
            // A circle is traversed as a full turn: it has the same centre and radius as an arc with sweep 2*pi.
            sketch::ArcGeom cg{};
            if (const auto* arc = std::get_if<sketch::ArcGeom>(&ent->geometry)) {
                cg = *arc;
            } else {
                const auto& circ = std::get<sketch::CircleGeom>(ent->geometry);
                cg = sketch::ArcGeom{circ.center, circ.radius, 0, kTwoPi};
            }
            ed.c = cg.center;
            ed.sweep = pe.sweep;
            const double am = angleOf(cg.center, pe.start) + pe.sweep / 2;
            const Point2 rad = unitp(polar(cg.center, 1.0, am));
            const Point2 tv = pe.sweep > 0 ? rot90(rad) : rot90(rad) * -1.0;
            const Point2 nrm = ccw ? Point2{tv.y, -tv.x} : Point2{-tv.y, tv.x};
            const double rNew = cg.radius + (dotp(nrm, rad) >= 0 ? d : -d);
            if (rNew <= kEps) return makeError(ErrorCode::InvalidArgument, "the offset collapses an arc");
            ed.off = Geo{};
            ed.off.circle = true;
            ed.off.c = cg.center;
            ed.off.r = rNew;
        }
        edges.push_back(ed);
    }

    const std::size_t n = edges.size();
    std::vector<Point2> joints(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Edge& prev = edges[(i + n - 1) % n];
        const Edge& cur = edges[i];
        const Point2 P = cur.s;
        if (n == 1) {
            joints[i] = P;   // full circle: no joints
            continue;
        }
        if (prev.line && cur.line && std::abs(crossp(prev.off.dir, cur.off.dir)) < 1e-9) {
            // Parallel neighbours: they join at the common offset point, which must lie on both.
            const Point2 j = P + cur.nrm * d;
            if (std::abs(crossp(prev.off.dir, j - prev.off.p)) > 1e-6 * std::max(1.0, std::abs(d)))
                return makeError(ErrorCode::InvalidArgument, "the offset cannot join opposite parallel edges");
            joints[i] = j;
            continue;
        }
        const auto rs = roots(prev.off, cur.off);
        if (rs.empty()) return makeError(ErrorCode::InvalidArgument, "the offset is too large for this profile");
        joints[i] = *std::min_element(rs.begin(), rs.end(), [&](Point2 a, Point2 b) {
            return sketch::distance(a, P) < sketch::distance(b, P);
        });
    }

    std::vector<SketchEntityId> created;
    auto r = apply("Offset", [&](Sketch& s) -> Result<void> {
        struct Placed {
            SketchEntityId id;
            PointRole startRole, endRole;
        };
        std::vector<Placed> placed;
        for (std::size_t i = 0; i < n; ++i) {
            const Edge& ed = edges[i];
            const Point2 j0 = joints[i], j1 = joints[(i + 1) % n];
            if (ed.line) {
                auto id = s.addLine(j0, j1);
                if (!id) return std::unexpected(id.error());
                placed.push_back({*id, PointRole::Start, PointRole::End});
                if (std::abs(ed.t.y) < 1e-9)
                    (void)s.addConstraint(ConstraintKind::Horizontal, {SketchRef{*id, std::nullopt}});
                else if (std::abs(ed.t.x) < 1e-9)
                    (void)s.addConstraint(ConstraintKind::Vertical, {SketchRef{*id, std::nullopt}});
            } else if (n == 1) {
                auto id = s.addCircle(ed.off.c, ed.off.r);
                if (!id) return std::unexpected(id.error());
                placed.push_back({*id, PointRole::Start, PointRole::End});
            } else {
                const double as = angleOf(ed.off.c, j0), ae = angleOf(ed.off.c, j1);
                const bool fwd = ed.sweep > 0;
                const double start = fwd ? as : ae;
                const double sw = fwd ? wrapAngle(ae - as) : wrapAngle(as - ae);
                if (sw < 1e-9 || sw >= kTwoPi) return makeError(ErrorCode::InvalidArgument, "the offset collapses an arc");
                auto id = s.addArc(ed.off.c, ed.off.r, start, sw);
                if (!id) return std::unexpected(id.error());
                placed.push_back({*id, fwd ? PointRole::Start : PointRole::End, fwd ? PointRole::End : PointRole::Start});
            }
        }
        if (n > 1) {
            for (std::size_t i = 0; i < n; ++i) {
                const Placed& a = placed[i];
                const Placed& b = placed[(i + 1) % n];
                auto c = s.addConstraint(ConstraintKind::Coincident,
                                         {SketchRef{a.id, a.endRole}, SketchRef{b.id, b.startRole}});
                if (!c) return std::unexpected(c.error());
            }
        }
        created.clear();
        for (const Placed& pl : placed) created.push_back(pl.id);
        return {};
    });
    if (!r) return std::unexpected(r.error());
    return created;
}

void SketchSession::clearPending() noexcept {
    lineStart_.reset();
    lineStartHit_.reset();
    joint_.reset();
    firstCorner_.reset();
    firstCornerHit_.reset();
    circleCenter_.reset();
    circleCenterHit_.reset();
    arcStart_.reset();
    arcStartHit_.reset();
    arcMid_.reset();
    tangentDrag_.reset();
    dimPicks_.clear();
}

Result<void> SketchSession::undo() {
    auto r = undo_.undo();
    clearPending();
    refresh();
    return r;
}

Result<void> SketchSession::redo() {
    auto r = undo_.redo();
    clearPending();
    refresh();
    return r;
}

void SketchSession::refresh() {
    auto profiles = sketch::detectProfiles(sketch_);
    profiles_ = profiles ? std::move(*profiles) : std::vector<sketch::SketchProfile>{};
    report_ = sketch::diagnoseSketch(sketch_, params_);
    std::erase_if(selection_, [&](const SketchRef& r) { return !sketch_.findEntity(r.entity); });
    if (selConstraint_ && !sketch_.constraints().count(*selConstraint_)) selConstraint_.reset();
    if (selDimension_) {
        const bool alive = selDimension_->driven
                               ? std::any_of(driven_.begin(), driven_.end(),
                                             [&](const DrivenDimension& d) { return d.id == selDimension_->id; })
                               : sketch_.dimensions().count(sketch::SketchDimensionId{selDimension_->id}) > 0;
        if (!alive) selDimension_.reset();
    }
    std::erase_if(dimPicks_, [&](const SketchRef& r) { return !sketch_.findEntity(r.entity); });
    snapHint_ = inference_.enabled && cursor_ ? snapAt(*cursor_) : std::nullopt;
}

} // namespace iw::ui
