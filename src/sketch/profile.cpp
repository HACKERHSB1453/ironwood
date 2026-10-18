// SPDX-License-Identifier: GPL-3.0-or-later
#include "sketch/profile.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <numbers>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace iw::sketch {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kVertexTol = 1e-7;        // mm: vertex merge and on-curve tolerance (task contract)
constexpr double kAngleTol = 1e-9;         // rad
constexpr double kProbeOffset = 1e-5;      // mm: containment probe distance from its edge
constexpr double kDegenerateArea = 1e-12;  // mm^2: loops below this are discarded

// ---- curves ---------------------------------------------------------------------------------------------------

/// Geometry of one non-construction sketch entity, prepared for intersection. Lines use a..b; arcs and circles
/// use centre c, radius r, start angle a0 and sweep (circles: a0 = 0, sweep = 2*pi, a = b = seam point).
struct Curve {
    SketchEntityId id;
    bool isLine = false;
    bool isCircle = false;
    Point2 a, b;
    Point2 c;
    double r = 0;
    double a0 = 0;
    double sweep = 0;
};

double norm(Point2 p) { return std::hypot(p.x, p.y); }

double normAngle(double t) {
    t = std::fmod(t, kTwoPi);
    if (t < 0) t += kTwoPi;
    return t >= kTwoPi ? 0.0 : t;
}

/// Angle of p about the curve centre, measured from the curve start, in [0, 2*pi). Near-2pi maps to 0.
double relAngle(const Curve& c, Point2 p) {
    const double t = normAngle(std::atan2(p.y - c.c.y, p.x - c.c.x) - c.a0);
    return t > kTwoPi - kAngleTol ? 0.0 : t;
}

/// Curve parameter of a point known to lie on the curve: t in [0,1] for lines, angle for arcs and circles.
double paramOf(const Curve& c, Point2 p) {
    if (c.isLine) {
        const Point2 d = c.b - c.a;
        return ((p.x - c.a.x) * d.x + (p.y - c.a.y) * d.y) / (d.x * d.x + d.y * d.y);
    }
    return relAngle(c, p);
}

bool onCurve(const Curve& c, Point2 p) {
    if (c.isLine) {
        const Point2 d = c.b - c.a;
        const double t = std::clamp(((p.x - c.a.x) * d.x + (p.y - c.a.y) * d.y) / (d.x * d.x + d.y * d.y), 0.0, 1.0);
        return distance(p, c.a + d * t) <= kVertexTol;
    }
    if (std::abs(distance(p, c.c) - c.r) > kVertexTol) return false;
    if (c.isCircle) return true;
    return relAngle(c, p) <= c.sweep + kAngleTol;
}

/// Analytic candidates (filtered later by onCurve on both curves).
void lineLine(const Curve& A, const Curve& B, std::vector<Point2>& out) {
    const Point2 d1 = A.b - A.a, d2 = B.b - B.a;
    const double den = d1.x * d2.y - d1.y * d2.x;
    if (std::abs(den) <= 1e-12 * norm(d1) * norm(d2)) return; // parallel: handled by endpoint tests
    const Point2 w = B.a - A.a;
    const double t = (w.x * d2.y - w.y * d2.x) / den;
    out.push_back(A.a + d1 * t);
}

void lineCircle(const Curve& L, const Curve& C, std::vector<Point2>& out) {
    const Point2 d = L.b - L.a;
    const Point2 f = L.a - C.c;
    const double A = d.x * d.x + d.y * d.y;
    const double B = 2.0 * (f.x * d.x + f.y * d.y);
    const double K = f.x * f.x + f.y * f.y - C.r * C.r;
    const double sq = std::sqrt(std::max(B * B - 4.0 * A * K, 0.0));
    for (const double t : {(-B - sq) / (2.0 * A), (-B + sq) / (2.0 * A)}) {
        Point2 p = L.a + d * t;
        // Snap onto the circle radially: removes most of the sqrt amplification near tangency.
        const double dc = distance(p, C.c);
        if (dc > 0) p = C.c + (p - C.c) * (C.r / dc);
        out.push_back(p);
    }
}

void circleCircle(const Curve& A, const Curve& B, std::vector<Point2>& out) {
    const Point2 dv = B.c - A.c;
    const double d = norm(dv);
    if (d < kVertexTol) return; // concentric: no isolated intersection points
    const double a = (A.r * A.r - B.r * B.r + d * d) / (2.0 * d);
    const double h = std::sqrt(std::max(A.r * A.r - a * a, 0.0));
    const Point2 base = A.c + dv * (a / d);
    const Point2 perp{-dv.y / d, dv.x / d};
    out.push_back(base + perp * h);
    out.push_back(base - perp * h);
}

void candidates(const Curve& A, const Curve& B, std::vector<Point2>& out) {
    if (A.isLine && B.isLine) lineLine(A, B, out);
    else if (A.isLine) lineCircle(A, B, out);
    else if (B.isLine) lineCircle(B, A, out);
    else circleCircle(A, B, out);
}

void endpoints(const Curve& c, std::vector<Point2>& out) {
    out.push_back(c.a);
    if (!c.isCircle) out.push_back(c.b);
}

// ---- vertices -------------------------------------------------------------------------------------------------

/// Merges points closer than kVertexTol. Hash grid with cell size kVertexTol: neighbours are checked 3x3.
class VertexStore {
public:
    int add(Point2 p) {
        const auto [cx, cy] = cell(p);
        for (long long dx = -1; dx <= 1; ++dx)
            for (long long dy = -1; dy <= 1; ++dy) {
                auto it = grid_.find({cx + dx, cy + dy});
                if (it == grid_.end()) continue;
                for (const int id : it->second)
                    if (distance(pts_[static_cast<std::size_t>(id)], p) <= kVertexTol) return id;
            }
        const int id = static_cast<int>(pts_.size());
        pts_.push_back(p);
        grid_[{cx, cy}].push_back(id);
        return id;
    }
    Point2 at(int id) const { return pts_[static_cast<std::size_t>(id)]; }

private:
    static std::pair<long long, long long> cell(Point2 p) {
        return {static_cast<long long>(std::floor(p.x / kVertexTol)), static_cast<long long>(std::floor(p.y / kVertexTol))};
    }
    std::vector<Point2> pts_;
    std::map<std::pair<long long, long long>, std::vector<int>> grid_;
};

// ---- planar graph ---------------------------------------------------------------------------------------------

/// Undirected edge between two vertices. Forward direction is u -> v; half-edge 2e is forward, 2e+1 is reverse.
struct Edge {
    int u = 0, v = 0;
    SketchEntityId id;
    std::uint32_t sub = 0;
    bool isLine = false;
    Point2 c;        ///< arc centre
    double r = 0;    ///< arc radius
    double ta = 0;   ///< forward start angle (absolute, arcs)
    double tb = 0;   ///< forward end angle (absolute, tb - ta > 0)
};

/// A directed piece of boundary: from -> to, arcs travelled from angle ta to tb (signed).
struct Directed {
    bool isArc = false;
    Point2 from, to;
    Point2 c;
    double r = 0;
    double ta = 0, tb = 0;
};

Directed directed(const Edge& e, bool reverse, const VertexStore& store) {
    Directed d;
    d.isArc = !e.isLine;
    d.c = e.c;
    d.r = e.r;
    const Point2 pu = store.at(e.u), pv = store.at(e.v);
    if (!reverse) {
        d.from = pu;
        d.to = pv;
        d.ta = e.ta;
        d.tb = e.tb;
    } else {
        d.from = pv;
        d.to = pu;
        d.ta = e.tb;
        d.tb = e.ta;
    }
    return d;
}

/// Signed area enclosed by a directed piece (Green's theorem; closed form for arcs).
double pieceArea(const Directed& d) {
    if (!d.isArc) return 0.5 * (d.from.x * d.to.y - d.to.x * d.from.y);
    const double R = d.r;
    const double cs = std::cos(d.ta), ss = std::sin(d.ta), ce = std::cos(d.tb), se = std::sin(d.tb);
    return 0.5 * (d.c.x * R * (se - ss) + d.c.y * R * (cs - ce) + R * R * (d.tb - d.ta));
}

/// Signed first moments (integral of x dA and y dA over the piece's Green contribution).
void pieceMoments(const Directed& d, double& mx, double& my) {
    if (!d.isArc) {
        const double dx = d.to.x - d.from.x, dy = d.to.y - d.from.y;
        mx = 0.5 * dy * (d.from.x * d.from.x + d.from.x * dx + dx * dx / 3.0);
        my = -0.5 * dx * (d.from.y * d.from.y + d.from.y * dy + dy * dy / 3.0);
        return;
    }
    const double R = d.r, Cx = d.c.x, Cy = d.c.y;
    const double sa = std::sin(d.ta), sb = std::sin(d.tb), ca = std::cos(d.ta), cb = std::cos(d.tb);
    const double s2a = std::sin(2 * d.ta), s2b = std::sin(2 * d.tb);
    const double dt = d.tb - d.ta;
    const double I1c = sb - sa;
    const double I2c = dt / 2.0 + (s2b - s2a) / 4.0;
    const double I3c = (sb - sb * sb * sb / 3.0) - (sa - sa * sa * sa / 3.0);
    const double I1s = ca - cb;
    const double I2s = dt / 2.0 - (s2b - s2a) / 4.0;
    const double I3s = (-cb + cb * cb * cb / 3.0) - (-ca + ca * ca * ca / 3.0);
    mx = 0.5 * R * (Cx * Cx * I1c + 2.0 * Cx * R * I2c + R * R * I3c);
    my = 0.5 * R * (Cy * Cy * I1s + 2.0 * Cy * R * I2s + R * R * I3s);
}

/// Crossing contribution (+1 / -1 / 0) of a directed piece to the winding number of `q`, ray towards +x.
int pieceWinding(const Directed& d, Point2 q) {
    if (!d.isArc) {
        const bool aboveA = d.from.y > q.y, aboveB = d.to.y > q.y;
        if (aboveA == aboveB) return 0;
        const double t = (q.y - d.from.y) / (d.to.y - d.from.y);
        const double x = d.from.x + t * (d.to.x - d.from.x);
        if (x <= q.x) return 0;
        return aboveB ? 1 : -1;
    }
    // Split the arc at its y-extrema (theta = pi/2 + k*pi) so every sub-arc is y-monotone.
    const double lo = std::min(d.ta, d.tb), hi = std::max(d.ta, d.tb);
    std::vector<double> bps;
    const double k0 = std::ceil((lo - kPi / 2) / kPi);
    for (double th = kPi / 2 + k0 * kPi; th < hi; th += kPi)
        if (th > lo) bps.push_back(th);
    std::vector<double> seq{d.ta};
    if (d.tb >= d.ta) seq.insert(seq.end(), bps.begin(), bps.end());
    else seq.insert(seq.end(), bps.rbegin(), bps.rend());
    seq.push_back(d.tb);

    const auto g = [&](double th) { return d.c.y + d.r * std::sin(th) - q.y; };
    int total = 0;
    for (std::size_t i = 0; i + 1 < seq.size(); ++i) {
        double p = seq[i], e = seq[i + 1];
        const bool aboveP = g(p) > 0, aboveE = g(e) > 0;
        if (aboveP == aboveE) continue;
        // Monotone on [p, e]: bisection on the sign of g.
        const bool gpPositive = g(p) > 0;
        for (int it = 0; it < 200; ++it) {
            const double mid = 0.5 * (p + e);
            if ((g(mid) > 0) == gpPositive) p = mid;
            else e = mid;
        }
        const double th = 0.5 * (p + e);
        const double x = d.c.x + d.r * std::cos(th);
        if (x <= q.x) continue;
        total += aboveE ? 1 : -1;
    }
    return total;
}

/// Unit tangent (direction of travel) and signed curvature (positive = turning left) at the start of a piece.
Point2 startTangent(const Directed& d, double& curvature) {
    if (!d.isArc) {
        curvature = 0;
        const Point2 t = d.to - d.from;
        const double n = norm(t);
        return {t.x / n, t.y / n};
    }
    const double s = d.tb >= d.ta ? 1.0 : -1.0;
    curvature = s / d.r;
    return {-std::sin(d.ta) * s, std::cos(d.ta) * s};
}

/// Point and unit left normal at the middle of a piece (left of travel direction).
Point2 midpointAndLeft(const Directed& d, Point2& left) {
    if (!d.isArc) {
        const Point2 m = (d.from + d.to) * 0.5;
        const Point2 t = d.to - d.from;
        const double n = norm(t);
        left = {-t.y / n, t.x / n};
        return m;
    }
    const double tm = 0.5 * (d.ta + d.tb);
    const double s = d.tb >= d.ta ? 1.0 : -1.0;
    const Point2 tan{-std::sin(tm) * s, std::cos(tm) * s};
    left = {-tan.y, tan.x};
    return {d.c.x + d.r * std::cos(tm), d.c.y + d.r * std::sin(tm)};
}

/// Splits a closed walk into simple cycles at repeated vertices (stack method). Cycles are sub-walks, so each
/// returned cycle visits every vertex at most once; back-and-forth parts of dangling trees become 2-cycles.
std::vector<std::vector<int>> splitWalk(const std::vector<int>& walk, const std::vector<int>& origin,
                                        const std::vector<int>& dest) {
    std::vector<std::vector<int>> cycles;
    std::vector<int> path;
    std::map<int, std::size_t> startPos; // vertex -> index in `path` of the half-edge starting there
    for (const int h : walk) {
        const int b = dest[static_cast<std::size_t>(h)];
        if (auto it = startPos.find(b); it != startPos.end()) {
            const std::size_t j = it->second;
            std::vector<int> cyc(path.begin() + static_cast<std::ptrdiff_t>(j), path.end());
            cyc.push_back(h);
            for (std::size_t k = j; k < path.size(); ++k) startPos.erase(origin[static_cast<std::size_t>(path[k])]);
            path.resize(j);
            cycles.push_back(std::move(cyc));
        } else {
            startPos[origin[static_cast<std::size_t>(h)]] = path.size();
            path.push_back(h);
        }
    }
    if (!path.empty()) cycles.push_back(std::move(path));
    return cycles;
}

struct Loop {
    std::vector<int> halfEdges;
    double area = 0;
    double mx = 0, my = 0;
};

using Token = std::pair<std::uint64_t, std::uint32_t>; // (entity id, sub index)

std::string tokenName(const std::vector<Token>& tokens) {
    std::string s;
    for (const auto& [id, sub] : tokens) {
        if (!s.empty()) s += '+';
        s += 'e' + std::to_string(id) + '.' + std::to_string(sub);
    }
    return s;
}

} // namespace

Result<std::vector<SketchProfile>> detectProfiles(const Sketch& sketch) {
    // 1. Curves of non-construction line / arc / circle entities (points and construction geometry are skipped).
    std::vector<Curve> curves;
    for (const auto& [id, e] : sketch.entities()) {
        if (e.construction) continue;
        Curve c;
        c.id = id;
        if (const auto* l = std::get_if<LineGeom>(&e.geometry)) {
            c.isLine = true;
            c.a = l->start;
            c.b = l->end;
        } else if (const auto* a = std::get_if<ArcGeom>(&e.geometry)) {
            c.c = a->center;
            c.r = a->radius;
            c.a0 = a->startAngle;
            c.sweep = a->sweep;
            c.a = arcStart(*a);
            c.b = arcEnd(*a);
        } else if (const auto* ci = std::get_if<CircleGeom>(&e.geometry)) {
            c.isCircle = true;
            c.c = ci->center;
            c.r = ci->radius;
            c.a0 = 0;
            c.sweep = kTwoPi;
            c.a = c.b = {ci->center.x + ci->radius, ci->center.y};
        } else {
            continue; // points
        }
        curves.push_back(c);
    }
    if (curves.empty()) return std::vector<SketchProfile>{};

    // 2. Split points: endpoints, pairwise intersections that lie on both curves, and endpoints lying on others.
    const std::size_t n = curves.size();
    std::vector<std::vector<Point2>> splits(n);
    for (std::size_t i = 0; i < n; ++i) endpoints(curves[i], splits[i]);
    std::vector<Point2> cand;
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 1; j < n; ++j) {
            const Curve& A = curves[i];
            const Curve& B = curves[j];
            cand.clear();
            candidates(A, B, cand);
            for (const Point2 p : cand)
                if (onCurve(A, p) && onCurve(B, p)) {
                    splits[i].push_back(p);
                    splits[j].push_back(p);
                }
            std::vector<Point2> ends;
            endpoints(A, ends);
            for (const Point2 p : ends)
                if (onCurve(B, p)) splits[j].push_back(p);
            ends.clear();
            endpoints(B, ends);
            for (const Point2 p : ends)
                if (onCurve(A, p)) splits[i].push_back(p);
        }

    // 3. Vertices and per-curve ordered split lists.
    VertexStore store;
    std::vector<std::vector<int>> curveVerts(n);
    for (std::size_t i = 0; i < n; ++i) {
        std::vector<std::pair<double, int>> pv;
        std::set<int> seen;
        for (const Point2 p : splits[i]) {
            const int vid = store.add(p);
            if (seen.insert(vid).second) pv.emplace_back(paramOf(curves[i], store.at(vid)), vid);
        }
        std::sort(pv.begin(), pv.end());
        for (const auto& entry : pv) curveVerts[i].push_back(entry.second);
    }

    // 4. Edges: consecutive split points along each curve (circles wrap around the seam).
    std::vector<Edge> edges;
    for (std::size_t i = 0; i < n; ++i) {
        const Curve& c = curves[i];
        const auto& vs = curveVerts[i];
        const std::size_t m = vs.size();
        std::vector<double> ps;
        for (const int vid : vs) ps.push_back(paramOf(c, store.at(vid)));
        const auto addEdge = [&](std::size_t k, int u, int v, double ta, double tb) {
            Edge e;
            e.u = u;
            e.v = v;
            e.id = c.id;
            e.sub = static_cast<std::uint32_t>(k);
            e.isLine = c.isLine;
            e.c = c.c;
            e.r = c.r;
            e.ta = c.isLine ? 0 : c.a0 + ta;
            e.tb = c.isLine ? 0 : c.a0 + tb;
            edges.push_back(e);
        };
        if (c.isLine || !c.isCircle) {
            for (std::size_t k = 0; k + 1 < m; ++k) addEdge(k, vs[k], vs[k + 1], ps[k], ps[k + 1]);
        } else if (m == 1) {
            addEdge(0, vs[0], vs[0], ps[0], ps[0] + kTwoPi);
        } else {
            for (std::size_t k = 0; k < m; ++k) {
                const std::size_t nx = (k + 1) % m;
                const double tb = nx == 0 ? ps[0] + kTwoPi : ps[nx];
                addEdge(k, vs[k], vs[nx], ps[k], tb);
            }
        }
    }

    // Drop geometric duplicates (identical piece listed twice, e.g. overlapping collinear lines): same vertex
    // pair and same midpoint.
    {
        std::vector<Edge> kept;
        std::vector<Point2> keptMid;
        for (const Edge& e : edges) {
            Point2 left;
            const Point2 mid = midpointAndLeft(directed(e, false, store), left);
            const int lo = std::min(e.u, e.v), hi = std::max(e.u, e.v);
            bool dup = false;
            for (std::size_t k = 0; k < kept.size() && !dup; ++k)
                dup = std::min(kept[k].u, kept[k].v) == lo && std::max(kept[k].u, kept[k].v) == hi &&
                      distance(mid, keptMid[k]) <= kVertexTol;
            if (!dup) {
                kept.push_back(e);
                keptMid.push_back(mid);
            }
        }
        edges = std::move(kept);
    }

    // 5. Half-edges, origin/destination, angular order of outgoing half-edges per vertex.
    const std::size_t H = edges.size() * 2;
    std::vector<int> origin(H), dest(H);
    std::vector<Directed> dir(H);
    std::vector<double> ang(H), curv(H);
    for (std::size_t e = 0; e < edges.size(); ++e)
        for (int rev = 0; rev < 2; ++rev) {
            const std::size_t h = 2 * e + static_cast<std::size_t>(rev);
            origin[h] = rev ? edges[e].v : edges[e].u;
            dest[h] = rev ? edges[e].u : edges[e].v;
            dir[h] = directed(edges[e], rev != 0, store);
            const Point2 t = startTangent(dir[h], curv[h]);
            ang[h] = std::atan2(t.y, t.x);
        }
    std::map<int, std::vector<int>> outgoing;
    for (std::size_t h = 0; h < H; ++h) outgoing[origin[h]].push_back(static_cast<int>(h));
    std::vector<std::size_t> posInOut(H, 0);
    for (auto& [v, list] : outgoing) {
        std::sort(list.begin(), list.end(), [&](int a, int b) {
            const auto ka = std::make_tuple(ang[static_cast<std::size_t>(a)], curv[static_cast<std::size_t>(a)], a);
            const auto kb = std::make_tuple(ang[static_cast<std::size_t>(b)], curv[static_cast<std::size_t>(b)], b);
            return ka < kb;
        });
        for (std::size_t k = 0; k < list.size(); ++k) posInOut[static_cast<std::size_t>(list[k])] = k;
    }
    // Face on the LEFT: next(h) is the first outgoing edge clockwise from twin(h) at dest(h).
    const auto nextOf = [&](int h) {
        const int v = dest[static_cast<std::size_t>(h)];
        const auto& list = outgoing[v];
        const std::size_t deg = list.size();
        const std::size_t idx = posInOut[static_cast<std::size_t>(h ^ 1)];
        return list[(idx + deg - 1) % deg];
    };

    // 6. Orbits -> simple loops (twin 2-cycles and zero-area loops are dropped).
    std::vector<bool> visited(H, false);
    std::vector<Loop> loops;
    for (std::size_t h0 = 0; h0 < H; ++h0) {
        if (visited[h0]) continue;
        std::vector<int> orbit;
        int h = static_cast<int>(h0);
        do {
            visited[static_cast<std::size_t>(h)] = true;
            orbit.push_back(h);
            h = nextOf(h);
        } while (static_cast<std::size_t>(h) != h0);
        for (auto& cyc : splitWalk(orbit, origin, dest)) {
            if (cyc.size() == 2 && cyc[1] == (cyc[0] ^ 1)) continue; // bridge / dangling spur
            Loop L;
            L.halfEdges = std::move(cyc);
            for (const int hh : L.halfEdges) {
                const Directed& d = dir[static_cast<std::size_t>(hh)];
                L.area += pieceArea(d);
                double mx = 0, my = 0;
                pieceMoments(d, mx, my);
                L.mx += mx;
                L.my += my;
            }
            if (std::abs(L.area) < kDegenerateArea) continue;
            loops.push_back(std::move(L));
        }
    }

    // 7. Classification (even-odd nesting). Positive loops bound regions; a negative loop is a hole of the
    //    smallest positive loop that contains the point just outside it. Negative loops with no such container
    //    are the outer walks of unbounded parts and are dropped.
    std::vector<std::size_t> positive, negative;
    for (std::size_t i = 0; i < loops.size(); ++i) (loops[i].area > 0 ? positive : negative).push_back(i);

    std::map<std::size_t, std::vector<std::size_t>> holesOf; // positive loop index -> negative loop indices
    for (const std::size_t ni : negative) {
        const Loop& N = loops[ni];
        Point2 left;
        const Point2 mid = midpointAndLeft(dir[static_cast<std::size_t>(N.halfEdges.front())], left);
        const Point2 probe = mid + left * kProbeOffset;
        // `best` holds a LOOP index (not a position in `positive`), so the "none" sentinel must be outside the
        // loop index range. Using positive.size() here silently dropped holes whose container happened to have
        // loop index == positive.size() (F28: rect-in-rect lost its hole depending on insertion order).
        std::size_t best = loops.size();
        double bestArea = 0;
        for (const std::size_t pi : positive) {
            int w = 0;
            for (const int hh : loops[pi].halfEdges) w += pieceWinding(dir[static_cast<std::size_t>(hh)], probe);
            if (w == 0) continue;
            if (best == loops.size() || loops[pi].area < bestArea) {
                best = pi;
                bestArea = loops[pi].area;
            }
        }
        if (best != loops.size()) holesOf[best].push_back(ni);
    }

    // 8. Regions. Each piece is named by (entity id, sub index).
    const auto tokensOf = [&](const Loop& L) {
        std::vector<Token> t;
        for (const int hh : L.halfEdges) {
            const Edge& e = edges[static_cast<std::size_t>(hh / 2)];
            t.emplace_back(e.id.value(), e.sub);
        }
        return t;
    };
    const auto loopOut = [&](const Loop& L) {
        ProfileLoop pl;
        pl.area = L.area;
        for (const int hh : L.halfEdges) {
            const Edge& e = edges[static_cast<std::size_t>(hh / 2)];
            const Directed d = dir[static_cast<std::size_t>(hh)];
            ProfileEdge pe;
            pe.entity = e.id;
            pe.subIndex = e.sub;
            pe.isArc = d.isArc;
            pe.start = d.from;
            pe.end = d.to;
            if (d.isArc) {
                pe.center = d.c;
                pe.radius = d.r;
                pe.startAngle = d.ta;
                pe.sweep = d.tb - d.ta;
            }
            pl.edges.push_back(pe);
        }
        return pl;
    };

    std::vector<SketchProfile> profiles;
    for (const std::size_t pi : positive) {
        const Loop& O = loops[pi];
        SketchProfile sp;
        sp.outer = loopOut(O);
        double area = O.area, mx = O.mx, my = O.my;
        std::vector<Token> tokens = tokensOf(O);
        std::vector<std::pair<std::string, ProfileLoop>> holes;
        if (auto it = holesOf.find(pi); it != holesOf.end()) {
            for (const std::size_t ni : it->second) {
                const Loop& N = loops[ni];
                area += N.area;
                mx += N.mx;
                my += N.my;
                auto t = tokensOf(N);
                tokens.insert(tokens.end(), t.begin(), t.end());
                auto tn = t;
                std::sort(tn.begin(), tn.end());
                holes.emplace_back(tokenName(tn), loopOut(N));
            }
        }
        std::sort(holes.begin(), holes.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        for (auto& [name, hl] : holes) sp.holes.push_back(std::move(hl));
        std::sort(tokens.begin(), tokens.end());
        tokens.erase(std::unique(tokens.begin(), tokens.end()), tokens.end());
        sp.name = tokenName(tokens);
        sp.area = area;
        sp.centroid = {mx / area, my / area};
        profiles.push_back(std::move(sp));
    }
    std::sort(profiles.begin(), profiles.end(),
              [](const SketchProfile& a, const SketchProfile& b) { return a.name < b.name; });
    return profiles;
}

} // namespace iw::sketch
