// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Geometric signatures and expectations used by the naming oracle (F19).
//
// A signature is: element type + centroid + measure (kernel::Shape::subShapeKey) and, where the kernel exposes it,
//  * planar faces: outward unit normal + a point on the plane (kernel::Shape::faceGeometry);
//  * cylindrical / conical faces: unit axis direction + a point on the axis (+ radius for cylinders);
//  * straight edges: the supporting line (kernel::axisFromEdge).
// An Expected is what a predictor says an element must look like: a partial signature plus a mask of the aspects that
// must match, so that e.g. a face trimmed by a fillet is only required to lie on the same surface.

#include <array>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "core/math.hpp"
#include "kernel/shape.hpp"
#include "naming/element_name.hpp"
#include "naming/naming_map.hpp"
#include "recompute/evaluator.hpp"
#include "recompute/feature.hpp"
#include "recompute/timeline.hpp"

namespace iw::fuzz {

using SurfaceKind = kernel::FaceGeometry::Kind;

struct GeomSig {
    naming::ElementType type = naming::ElementType::Face;
    std::array<double, 4> key{}; ///< centroid x, y, z (mm), measure (mm^2 / mm / 0)
    // Faces.
    SurfaceKind surface = SurfaceKind::Other;
    Vec3 normal;  ///< Plane: outward unit normal. Cylinder/Cone: unit axis direction.
    Vec3 point;   ///< Plane: a point on it. Cylinder/Cone: a point on the axis. Straight edge: a point on the line.
    double radius = 0; ///< Cylinder radius.
    // Edges.
    bool isLine = false;
    Vec3 dir; ///< Straight edge: unit direction.

    Vec3 centroid() const { return {key[0], key[1], key[2]}; }
    double measure() const { return key[3]; }
};

/// Reads the signature of element `index` of `type` from `shape`.
GeomSig readSig(const kernel::Shape& shape, naming::ElementType type, std::uint32_t index);

/// Full comparison of two signatures of the same element: centroid, measure and, when both carry one, the surface /
/// supporting line. Used for "the geometry must not change" checks.
bool sameGeometry(const GeomSig& a, const GeomSig& b);
std::string describe(const GeomSig& s);

/// Same supporting surface (faces): kind, plane (normal, offset) or axis line (+ radius for cylinders). `signedNormal`
/// false accepts a flipped plane normal. Kinds without surface data (sphere, torus, ...) compare equal.
bool sameSurface(const GeomSig& a, const GeomSig& b, bool signedNormal = true);
/// Same supporting line (straight edges); true when either is not a straight edge.
bool sameLine(const GeomSig& a, const GeomSig& b);

/// Rigid transform of a signature (centroid, normals/axes, points).
GeomSig transformed(const GeomSig& s, const Transform& t);

namespace mask {
inline constexpr unsigned Centroid = 1;       ///< centroid within 1e-4 mm
inline constexpr unsigned Measure = 2;        ///< measure within 1e-6 relative
inline constexpr unsigned Surface = 4;        ///< same supporting surface (signed normal)
inline constexpr unsigned SurfaceFlip = 8;    ///< same supporting surface, either orientation
inline constexpr unsigned Line = 16;          ///< straight edges: same supporting line
inline constexpr unsigned Overlap = 32;       ///< edges: centroid distance <= (len_exp + len_got) / 2  (nested / overlapping)
inline constexpr unsigned MeasureMax = 64;    ///< got.measure <= exp.measure (+tol)
inline constexpr unsigned Point = Centroid;   ///< vertices: same point
} // namespace mask

struct Expected {
    GeomSig sig;
    unsigned mask = 0;
    std::function<bool(const GeomSig&)> extra; ///< Optional additional predicate (custom checks).
    /// Predicate that survives a boolean / trim / split / merge (used by relaxed() instead of `extra`); empty = none.
    std::function<bool(const GeomSig&)> relaxedExtra;
    std::string note;                           ///< Shown in failure messages.
};

bool matches(const Expected& e, const GeomSig& got);
std::string describe(const Expected& e);

/// Expected requiring everything the signature carries (the geometry of an element that the feature produced itself).
Expected exactly(const GeomSig& s);
/// Only the supporting surface / line of a (possibly trimmed, split or merged) element.
Expected surfaceOf(const GeomSig& s, bool signedNormal = true);
/// Relaxes an expectation to what survives a boolean / trim / split / merge: the supporting surface or line and, for
/// edges, overlap; vertices keep their point.
Expected relaxed(const Expected& e);
/// Expectation for an element inherited unchanged in kind from `in` through a modifying operation.
Expected inheritedFrom(const GeomSig& in);

/// Model state at one timeline position.
struct ModelState {
    std::map<BodyId, kernel::Shape> bodies;
    std::map<BodyId, naming::NamingMap> namings;
    std::map<FeatureId, recompute::ConstructionGeometry> constructions;

    static ModelState from(const recompute::EvalResult& r);
    /// Signature of the unique element carrying `name`; nullopt if the body or the name is absent.
    std::optional<GeomSig> sigOfName(BodyId body, const naming::ElementName& name) const;
    /// All named elements of `type` in `body`.
    std::vector<std::pair<naming::ElementName, std::uint32_t>> namedElements(BodyId body, naming::ElementType type) const;
};

/// "12.5 mm" -> 12.5; nullopt when the text is not "<number> mm".
std::optional<double> parseMm(const std::string& text);
std::string mmText(double v);
std::string degText(double v);
std::optional<double> parseDeg(const std::string& text);

inline Vec3 unit(const Vec3& v) {
    const double l = length(v);
    return l > 0 ? v * (1.0 / l) : v;
}
/// Distance from point `p` to the infinite line (o, d unit).
inline double distToLine(const Vec3& p, const Vec3& o, const Vec3& d) { return length(cross(p - o, d)); }

} // namespace iw::fuzz
