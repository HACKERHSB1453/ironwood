// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <compare>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include "kernel/shape.hpp"

namespace iw::kernel {

/// Address of a sub-shape of one INPUT of an operation. `operand` is the 0-based position of the input
/// shape in the operation's argument list (e.g. fuse(a,b): a=0, b=1; extrude profile=0).
/// `index` follows the Shape ordering contract of that input.
struct InputSubShape {
    std::uint32_t operand = 0;
    SubShapeKind kind = SubShapeKind::Face;
    std::uint32_t index = 0;
    friend auto operator<=>(const InputSubShape&, const InputSubShape&) = default;
};

/// Address of a sub-shape of the OUTPUT of an operation (Shape ordering contract of the result).
struct OutputSubShape {
    SubShapeKind kind = SubShapeKind::Face;
    std::uint32_t index = 0;
    friend auto operator<=>(const OutputSubShape&, const OutputSubShape&) = default;
};

/// What an operation did to the sub-shapes of its inputs, in OCCT History terms
/// (BRepTools_History: Generated / Modified / IsRemoved). Value type; safe to read from many threads.
///
/// * generated: an input sub-shape *produced* new output sub-shapes of possibly different kind
///   (edge -> side face when extruding).
/// * modified: an input sub-shape survives in changed form (split, trimmed, re-parametrised); one input
///   may map to several outputs (split) and several inputs to one output (merge).
/// * deleted: an input sub-shape has no counterpart in the output.
/// Input sub-shapes absent from all three are *unchanged* and appear in `unchanged` iff the kernel could
/// establish identity (same TShape); otherwise treat absence as "unknown" (naming must fall back).
struct ShapeHistory {
    std::map<InputSubShape, std::vector<OutputSubShape>> generated;
    std::map<InputSubShape, std::vector<OutputSubShape>> modified;
    std::set<InputSubShape> deleted;
    std::map<InputSubShape, OutputSubShape> unchanged;
};

/// Output of a modelling operation: the new shape plus how it relates to the inputs.
struct ShapeResult {
    Shape shape;           ///< Result; stamp is unset (the evaluator stamps it).
    ShapeHistory history;  ///< Input -> output relation (may be empty for primitives).
};

} // namespace iw::kernel
