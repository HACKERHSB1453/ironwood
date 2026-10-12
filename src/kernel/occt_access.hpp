// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Ironwood-internal; may only be included by modules that already link OCCT (tess, future features); never from
// public headers. Apps and UI code must not include this header. Consumers link the INTERFACE target
// `iw_kernel_occt` (PRIVATE) to receive the OCCT include directories and toolkit libraries.
//
// This is the sanctioned bridge from kernel::Shape to its OCCT shape. The Shape handle itself stays OCCT-free;
// the binding lives in kernel/internal/shape_impl.hpp and is only reachable through the functions below.

#include <TopoDS_Shape.hxx>

#include "kernel/shape.hpp"

namespace iw::kernel::occt {

/// OCCT shape behind `s`. Returns a null TopoDS_Shape for a null Shape. The reference stays valid for the
/// lifetime of `s` (or, for a null Shape, for the program lifetime). Read-only: the shape is immutable.
const TopoDS_Shape& get(const Shape& s);

/// Wraps an OCCT shape as an immutable kernel::Shape (builds its sub-shape index tables once). A null
/// TopoDS_Shape yields a null Shape. `stamp` is the lineage stamp to attach (default: unstamped).
Shape wrap(const TopoDS_Shape& s, ShapeStamp stamp = {});

} // namespace iw::kernel::occt
