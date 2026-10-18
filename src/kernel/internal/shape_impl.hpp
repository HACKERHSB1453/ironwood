// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// PRIVATE to iw_kernel: the only place OCCT types are bound to Shape. Never include from other modules.
//
// SUB-SHAPE ORDERING (implementation of the contract in kernel/shape.hpp)
// ------------------------------------------------------------------------
// For each SubShapeKind the index table is built ONCE at wrap time with TopExp::MapShapes(shape, kind, map),
// i.e. an explicit depth-first walk by TopExp_Explorer:
//   * children are visited in the order they are stored in their parent (TopoDS_Iterator order), which is a
//     property of the topology graph, never of pointer values or hash-bucket order;
//   * an entry is created at the FIRST occurrence of a (TShape, Location) pair; orientation is ignored;
//   * the 1-based position in TopTools_IndexedMapOfShape (insertion order) minus one is the public index.
// The hash inside the indexed map is only used for look-up (find index of a known sub-shape), never for
// iteration order. Two builds of the same operation on the same inputs therefore give identical orderings,
// and the tables are immutable after construction (safe for concurrent readers).

#include <atomic>
#include <limits>
#include <memory>

#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Shape.hxx>

#include "kernel/shape.hpp"

namespace iw::kernel {

/// Immutable per-shape sub-shape index tables (see ordering notes above).
struct SubShapeIndex {
    TopTools_IndexedMapOfShape faces, edges, vertices;

    explicit SubShapeIndex(const TopoDS_Shape& s) {
        TopExp::MapShapes(s, TopAbs_FACE, faces);
        TopExp::MapShapes(s, TopAbs_EDGE, edges);
        TopExp::MapShapes(s, TopAbs_VERTEX, vertices);
    }
    const TopTools_IndexedMapOfShape& map(SubShapeKind k) const {
        switch (k) {
        case SubShapeKind::Face: return faces;
        case SubShapeKind::Edge: return edges;
        default: return vertices;
        }
    }
};

struct Shape::Impl {
    TopoDS_Shape shape;
    ShapeStamp stamp;
    std::shared_ptr<const SubShapeIndex> index; ///< never null for a non-null shape
    /// Memoised volume in mm^3 (NaN = not yet computed). Geometry is immutable, so a computed value never changes.
    mutable std::atomic<double> volumeCache{std::numeric_limits<double>::quiet_NaN()};
    /// Memoised structural verdict (-1 = not yet checked, 0 = invalid, 1 = valid). Immutable geometry, so a verdict
    /// never changes. Only ever written by the kernel's validation (kernel/ops.cpp).
    mutable std::atomic<signed char> validCache{-1};

    Impl() = default;
    Impl(const Impl& o)
        : shape(o.shape), stamp(o.stamp), index(o.index), volumeCache(o.volumeCache.load(std::memory_order_relaxed)),
          validCache(o.validCache.load(std::memory_order_relaxed)) {}
    Impl& operator=(const Impl&) = delete;
};

/// Wraps an OCCT shape (builds the index tables). Null OCCT shape -> null Shape.
inline Shape wrapShape(const TopoDS_Shape& s, ShapeStamp stamp = {}) {
    if (s.IsNull()) return {};
    auto impl = std::make_shared<Shape::Impl>();
    impl->shape = s;
    impl->stamp = stamp;
    impl->index = std::make_shared<const SubShapeIndex>(s);
    return Shape(std::move(impl));
}

/// OCCT shape behind `s` (null TopoDS_Shape for a null Shape).
inline TopoDS_Shape occtShape(const Shape& s) { return s.impl() ? s.impl()->shape : TopoDS_Shape(); }

} // namespace iw::kernel
