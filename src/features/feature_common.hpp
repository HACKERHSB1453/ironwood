// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Helpers shared by the B1 feature implementations (expression evaluation, unit checks, sketch-profile naming,
// polygon naming). Extracted verbatim from extrude_feature.cpp in B1b; no behaviour change. JSON helpers live in
// feature_json.hpp (kept out of public headers).

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "core/math.hpp"
#include "core/params.hpp"
#include "core/result.hpp"
#include "kernel/ops.hpp"
#include "kernel/shape.hpp"
#include "naming/naming_map.hpp"
#include "naming/resolve.hpp"
#include "recompute/feature.hpp"
#include "sketch/profile.hpp"
#include "sketch/sketch.hpp"

namespace iw::features::common {

/// FNV-1a 64-bit accumulator used for paramHash() of the features.
struct Fnv {
    std::uint64_t h = 0xcbf29ce484222325ULL;
    void bytes(std::string_view s);
};

Vec3 normalized(const Vec3& v);
bool finite(const Vec3& v);

/// Evaluates an expression against the parameter table. A missing table only fails for parameter references.
Result<Quantity> evalExpr(const ParameterTable* table, const std::string& text);
/// Expression that must evaluate to a length (mm) / an angle (rad).
Result<double> lengthOf(const ParameterTable* table, const std::string& text, const char* what);
Result<double> angleOf(const ParameterTable* table, const std::string& text, const char* what);
/// Parameter names referenced by an expression (empty if it does not parse).
std::vector<std::string> referencesOf(const std::string& text);

/// Positional naming of a polygon profile face (P0 polygon sources).
Result<naming::NamingMap> polygonProfileNaming(FeatureId feature, const kernel::Shape& profile);
/// Sketch-derived naming of a profile face (entity-id based; see extrude_feature.cpp history).
Result<naming::NamingMap> sketchProfileNaming(FeatureId feature, const kernel::Shape& face, const sketch::SketchProfile& prof,
                                              const sketch::SketchPlane& plane, std::vector<recompute::Diagnostic>& diags);

/// Rigid rotation of `angle` radians about `axis` (right-hand rule), as a row-major 4x4 transform (includes the
/// translation that keeps the axis fixed).
Transform rotationAbout(const kernel::Axis& axis, double angle);

/// How a feature's result meets the body it targets (B1 operations; NewBody creates `body`).
enum class BodyOp { NewBody, Join, Cut, Intersect };
const char* bodyOpName(BodyOp op);
Result<BodyOp> bodyOpFrom(const std::string& s);

/// Resolves `ref` against the naming map of `body` at this timeline position (naming::resolve). A history-derived
/// rebinding adds a Warning to `diags`. Failure is a Fusion-style message naming the reference: NotFound when the body
/// or the element does not exist, Ambiguous when the name cannot be bound uniquely. Returns the element index.
Result<std::uint32_t> resolveRef(const recompute::EvalContext& ctx, BodyId body, const naming::ElementRef& ref,
                                 naming::ElementType expected, std::vector<recompute::Diagnostic>& diags);

/// Completes a feature whose geometry is `tool` (a solid with its history from its own profile naming):
/// NewBody stores it as `body`; Join / Cut / Intersect combine it with the current `body`. Errors as extrude.
Result<recompute::FeatureResult> applyBodyOp(FeatureId feature, BodyOp op, BodyId body, const recompute::EvalContext& ctx,
                                             const kernel::ShapeResult& tool, const naming::NamingMap& toolNaming,
                                             std::string_view toolTag, std::vector<recompute::Diagnostic> diags);

/// Result of an in-place modification of `body` (fillet, chamfer, hole, move): the new shape with names derived from
/// the current body's names and the kernel history. `opTag` is the operation tag used for new names.
Result<recompute::FeatureResult> modifyBody(FeatureId feature, BodyId body, const recompute::EvalContext& ctx,
                                            const kernel::ShapeResult& result, std::string_view opTag,
                                            std::vector<recompute::Diagnostic> diags);

} // namespace iw::features::common
