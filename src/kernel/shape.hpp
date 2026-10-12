// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "core/math.hpp"

namespace iw::kernel {

/// Topological type of a shape (mirrors the B-rep hierarchy; independent of OCCT enums).
enum class ShapeType { Null, Compound, CompSolid, Solid, Shell, Face, Wire, Edge, Vertex };

/// Sub-shape categories that can be addressed by index (see Shape::subShapeCount).
enum class SubShapeKind : std::uint8_t { Face, Edge, Vertex };

/// Version stamp identifying the *content lineage* of a shape (ARCHITECTURE section 4): a hash of
/// (feature id + input stamp chain + parameter hash). Equal stamps imply equal geometry AND equal
/// sub-shape ordering. Cheap and exact for cache keys; it is NOT a geometric content hash.
struct ShapeStamp {
    std::uint64_t value = 0; ///< 0 = unstamped
    friend constexpr bool operator==(ShapeStamp, ShapeStamp) = default;
    /// Deterministically mixes `salt` into `s` (order-sensitive; non-commutative). Pure, thread-safe.
    static constexpr ShapeStamp combine(ShapeStamp s, std::uint64_t salt) noexcept {
        std::uint64_t h = s.value ^ (salt + 0x9e3779b97f4a7c15ULL + (s.value << 6) + (s.value >> 2));
        h ^= h >> 30;
        h *= 0xbf58476d1ce4e5b9ULL;
        h ^= h >> 27;
        return ShapeStamp{h};
    }
};

/// Immutable value handle to a B-rep shape. Copies are O(1) and share the underlying (never mutated)
/// geometry, so a Shape may be read concurrently from any number of threads. No OCCT type is exposed.
///
/// SUB-SHAPE ORDERING (contract for naming/tess/history): Face, edge and vertex indices are 0-based and
/// follow a deterministic traversal of the shape (first-occurrence order of a depth-first topological
/// walk that ignores orientation, i.e. one entry per unique TShape+Location). The same Shape (same stamp)
/// always yields the same ordering. Different kernel operations are NOT guaranteed to preserve ordering
/// between input and output; that is what ShapeHistory is for.
class Shape {
public:
    /// Null shape: isNull() == true.
    Shape();
    ~Shape();
    Shape(const Shape&);
    Shape(Shape&&) noexcept;
    Shape& operator=(const Shape&);
    Shape& operator=(Shape&&) noexcept;

    bool isNull() const noexcept;
    ShapeType type() const noexcept;

    /// Lineage stamp; unstamped (0) for freshly imported shapes until the owner assigns one.
    ShapeStamp stamp() const noexcept;
    /// Returns a copy carrying `stamp` (geometry shared). Used by the evaluator after compute().
    Shape withStamp(ShapeStamp stamp) const;

    /// Number of unique sub-shapes of `kind` (see ordering contract). 0 for a null shape.
    std::uint32_t subShapeCount(SubShapeKind kind) const;

    /// Deterministic geometric key of sub-shape `index` (ordering contract above): {centroid x, y, z, measure}
    /// in mm / mm^2 / mm, measure = area (Face), length (Edge), 0 (Vertex; centroid is its point). Each value is
    /// rounded to 1e-9 so it is stable across builds; used to order split siblings. Pure/read-only on the
    /// immutable shape (concurrent calls on a const Shape are expected safe; UNVERIFIED by stress test).
    /// Returns {0,0,0,0} for a null shape or out-of-range index.
    std::array<double, 4> subShapeKey(SubShapeKind kind, std::size_t index) const;

    /// Axis-aligned bounding box in mm (may be slightly enlarged by tolerance). Empty for null shape.
    Box3 boundingBox() const;

    /// True when both handles refer to the same underlying geometry (TShape + Location + orientation).
    bool isSame(const Shape& other) const noexcept;

    struct Impl;
    /// Kernel-internal access (defined in kernel/internal/, private to iw_kernel). Not for other modules.
    const Impl* impl() const noexcept { return impl_.get(); }
    explicit Shape(std::shared_ptr<const Impl> impl) noexcept;

private:
    std::shared_ptr<const Impl> impl_;
};

/// Returns the linked OCCT version string, e.g. "7.9.3". Thread-safe.
std::string occtVersion();

} // namespace iw::kernel
