// SPDX-License-Identifier: GPL-3.0-or-later
#include "fuzz/geom.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "kernel/ops.hpp"

namespace iw::fuzz {
namespace {

constexpr double kCentroidTol = 1e-4;  // mm
constexpr double kSurfaceTol = 1e-4;   // mm (boolean results carry ~1e-5 numerical noise)
constexpr double kDirTol = 1e-4;

bool parallel(const Vec3& a, const Vec3& b) { return length(cross(a, b)) < kDirTol; }

bool sameMeasure(double a, double b) {
    return std::abs(a - b) <= 1e-6 + 1e-6 * std::max(std::abs(a), std::abs(b));
}

bool sameCentroid(const GeomSig& a, const GeomSig& b) { return length(a.centroid() - b.centroid()) <= kCentroidTol * 1.8; }

Vec3 rotate(const Transform& t, const Vec3& v) {
    return {t.m[0] * v.x + t.m[1] * v.y + t.m[2] * v.z, t.m[4] * v.x + t.m[5] * v.y + t.m[6] * v.z,
            t.m[8] * v.x + t.m[9] * v.y + t.m[10] * v.z};
}

} // namespace

GeomSig readSig(const kernel::Shape& shape, naming::ElementType type, std::uint32_t index) {
    GeomSig s;
    s.type = type;
    s.key = shape.subShapeKey(naming::toSubShapeKind(type), index);
    if (type == naming::ElementType::Face) {
        const kernel::FaceGeometry g = shape.faceGeometry(index);
        s.surface = g.kind;
        switch (g.kind) {
        case SurfaceKind::Plane:
            s.normal = unit(g.normal);
            s.point = g.point;
            break;
        case SurfaceKind::Cylinder:
            s.normal = unit(g.direction);
            s.point = g.point;
            s.radius = g.radius;
            break;
        case SurfaceKind::Cone:
            s.normal = unit(g.direction);
            s.point = g.point;
            break;
        case SurfaceKind::Other: break;
        }
    } else if (type == naming::ElementType::Edge) {
        if (auto ax = kernel::axisFromEdge(shape, index)) {
            s.isLine = true;
            s.point = ax->origin;
            s.dir = unit(ax->direction);
        }
    }
    return s;
}

bool sameSurface(const GeomSig& a, const GeomSig& b, bool signedNormal) {
    // Kinds the kernel does not classify (surfaces of revolution, B-splines) carry no comparable data.
    if (a.surface == SurfaceKind::Other || b.surface == SurfaceKind::Other) return true;
    if (a.surface != b.surface) return false;
    switch (a.surface) {
    case SurfaceKind::Plane: {
        Vec3 nb = b.normal;
        double db = dot(b.normal, b.point);
        if (dot(a.normal, nb) < 0) {
            if (signedNormal) return false;
            nb = nb * -1.0;
            db = -db;
        }
        return length(a.normal - nb) < kDirTol && std::abs(dot(a.normal, a.point) - db) < kSurfaceTol;
    }
    case SurfaceKind::Cylinder:
        if (std::abs(a.radius - b.radius) > 1e-6 + 1e-6 * a.radius) return false;
        [[fallthrough]];
    case SurfaceKind::Cone:
        return parallel(a.normal, b.normal) && distToLine(b.point, a.point, a.normal) < kSurfaceTol;
    case SurfaceKind::Other: return true;
    }
    return true;
}

bool sameLine(const GeomSig& a, const GeomSig& b) {
    if (!a.isLine || !b.isLine) return true;
    // Edges that a boolean left on nearly coincident faces carry ~1e-3 of direction noise (a swapped edge differs by far more).
    return length(cross(a.dir, b.dir)) < 2e-3 && distToLine(b.point, a.point, a.dir) < 1e-3;
}

bool sameGeometry(const GeomSig& a, const GeomSig& b) {
    if (a.type != b.type) return false;
    if (!sameCentroid(a, b) || !sameMeasure(a.measure(), b.measure())) return false;
    if (a.type == naming::ElementType::Face) return sameSurface(a, b);
    if (a.type == naming::ElementType::Edge) return sameLine(a, b);
    return true;
}

std::string describe(const GeomSig& s) {
    static constexpr const char* kTypes[] = {"F", "E", "V"};
    char buf[320];
    int n = std::snprintf(buf, sizeof buf, "%s(%.6g, %.6g, %.6g; m=%.6g", kTypes[static_cast<int>(s.type)], s.key[0],
                          s.key[1], s.key[2], s.key[3]);
    if (s.type == naming::ElementType::Face) {
        switch (s.surface) {
        case SurfaceKind::Plane:
            n += std::snprintf(buf + n, sizeof buf - static_cast<std::size_t>(n), "; plane n=(%.4g,%.4g,%.4g) d=%.6g",
                               s.normal.x, s.normal.y, s.normal.z, dot(s.normal, s.point));
            break;
        case SurfaceKind::Cylinder:
            n += std::snprintf(buf + n, sizeof buf - static_cast<std::size_t>(n), "; cyl axis=(%.4g,%.4g,%.4g) r=%.6g",
                               s.normal.x, s.normal.y, s.normal.z, s.radius);
            break;
        case SurfaceKind::Cone:
            n += std::snprintf(buf + n, sizeof buf - static_cast<std::size_t>(n), "; cone axis=(%.4g,%.4g,%.4g)",
                               s.normal.x, s.normal.y, s.normal.z);
            break;
        case SurfaceKind::Other: n += std::snprintf(buf + n, sizeof buf - static_cast<std::size_t>(n), "; other"); break;
        }
    } else if (s.type == naming::ElementType::Edge && s.isLine) {
        n += std::snprintf(buf + n, sizeof buf - static_cast<std::size_t>(n), "; line dir=(%.4g,%.4g,%.4g)", s.dir.x,
                           s.dir.y, s.dir.z);
    }
    std::snprintf(buf + n, sizeof buf - static_cast<std::size_t>(n), ")");
    return buf;
}

GeomSig transformed(const GeomSig& s, const Transform& t) {
    GeomSig o = s;
    const Vec3 c = t.apply(s.centroid());
    o.key = {c.x, c.y, c.z, s.key[3]};
    o.normal = rotate(t, s.normal);
    o.point = t.apply(s.point);
    o.dir = rotate(t, s.dir);
    return o;
}

bool matches(const Expected& e, const GeomSig& got) {
    if (e.sig.type != got.type) return false;
    const unsigned m = e.mask;
    if ((m & mask::Centroid) && !sameCentroid(e.sig, got)) return false;
    if ((m & mask::Measure) && !sameMeasure(e.sig.measure(), got.measure())) return false;
    if ((m & mask::MeasureMax) && got.measure() > e.sig.measure() + 1e-6 + 1e-6 * e.sig.measure()) return false;
    if (m & mask::Surface) {
        if (got.type != naming::ElementType::Face || !sameSurface(e.sig, got, true)) return false;
    }
    if (m & mask::SurfaceFlip) {
        if (got.type != naming::ElementType::Face || !sameSurface(e.sig, got, false)) return false;
    }
    if ((m & mask::Line) && !sameLine(e.sig, got)) return false;
    if (m & mask::Overlap) {
        const double reach = (e.sig.isLine && got.isLine) ? (e.sig.measure() + got.measure()) * 0.5 + kCentroidTol
                                          : e.sig.measure() + got.measure() + kCentroidTol;
        if (length(e.sig.centroid() - got.centroid()) > reach) return false;
    }
    if (e.extra && !e.extra(got)) return false;
    return true;
}

std::string describe(const Expected& e) {
    std::string out = describe(e.sig);
    if (!e.note.empty()) out += " [" + e.note + "]";
    out += " mask=" + std::to_string(e.mask);
    return out;
}

Expected exactly(const GeomSig& s) {
    Expected e;
    e.sig = s;
    e.mask = mask::Centroid | mask::Measure;
    if (s.type == naming::ElementType::Face && s.surface != SurfaceKind::Other) e.mask |= mask::Surface;
    if (s.type == naming::ElementType::Edge && s.isLine) e.mask |= mask::Line;
    return e;
}

Expected surfaceOf(const GeomSig& s, bool signedNormal) {
    Expected e;
    e.sig = s;
    if (s.type == naming::ElementType::Face && s.surface != SurfaceKind::Other)
        e.mask = signedNormal ? mask::Surface : mask::SurfaceFlip;
    return e;
}

Expected relaxed(const Expected& in) {
    Expected e;
    e.sig = in.sig;
    e.note = in.note;
    if (in.relaxedExtra) {
        e.extra = in.relaxedExtra;
        e.relaxedExtra = in.relaxedExtra; // idempotent: relaxing twice (tool prediction, then split derivation)
        return e;
    }
    switch (in.sig.type) {
    case naming::ElementType::Face:
        if (in.sig.surface != SurfaceKind::Other)
            e.mask = (in.mask & mask::SurfaceFlip) ? mask::SurfaceFlip : mask::Surface;
        break;
    case naming::ElementType::Edge:
        e.mask = mask::Overlap | (in.sig.isLine ? mask::Line : 0u);
        break;
    case naming::ElementType::Vertex: e.mask = mask::Centroid; break;
    }
    return e;
}

Expected inheritedFrom(const GeomSig& in) { return relaxed(exactly(in)); }

ModelState ModelState::from(const recompute::EvalResult& r) {
    ModelState s;
    s.bodies = r.bodies;
    s.namings = r.namings;
    for (const auto& st : r.statuses)
        if (st.construction) s.constructions[st.id] = *st.construction;
    return s;
}

std::optional<GeomSig> ModelState::sigOfName(BodyId body, const naming::ElementName& name) const {
    auto b = bodies.find(body);
    auto n = namings.find(body);
    if (b == bodies.end() || n == namings.end()) return std::nullopt;
    const auto idx = n->second.indicesOf(name);
    if (idx.size() != 1) return std::nullopt;
    return readSig(b->second, name.type, idx.front());
}

std::vector<std::pair<naming::ElementName, std::uint32_t>> ModelState::namedElements(BodyId body,
                                                                                      naming::ElementType type) const {
    std::vector<std::pair<naming::ElementName, std::uint32_t>> out;
    auto n = namings.find(body);
    if (n == namings.end()) return out;
    const auto count = n->second.size(type);
    for (std::uint32_t i = 0; i < count; ++i)
        if (auto nm = n->second.nameOf(type, i)) out.emplace_back(*nm, i);
    return out;
}

std::optional<double> parseMm(const std::string& text) {
    char* end = nullptr;
    const double v = std::strtod(text.c_str(), &end);
    if (end == text.c_str()) return std::nullopt;
    std::string rest(end);
    if (rest != " mm") return std::nullopt;
    return v;
}

std::optional<double> parseDeg(const std::string& text) {
    char* end = nullptr;
    const double v = std::strtod(text.c_str(), &end);
    if (end == text.c_str()) return std::nullopt;
    std::string rest(end);
    if (rest != " deg") return std::nullopt;
    return v;
}

std::string mmText(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.10g mm", v);
    return buf;
}

std::string degText(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.10g deg", v);
    return buf;
}

} // namespace iw::fuzz
