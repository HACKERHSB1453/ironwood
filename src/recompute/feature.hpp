// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/cancel.hpp"
#include "core/ids.hpp"
#include "core/math.hpp"
#include "core/params.hpp"
#include "core/result.hpp"
#include "kernel/shape.hpp"
#include "naming/element_name.hpp"
#include "naming/naming_map.hpp"

namespace iw::recompute {

/// Non-fatal message attached to a feature result (shown on the timeline item).
struct Diagnostic {
    enum class Severity { Info, Warning, Error };
    Severity severity = Severity::Info;
    std::string message;
    std::optional<naming::ElementRef> ref; ///< Offending reference, if any.
};

/// Geometry a construction feature (Plane / Axis / Point, B1) exposes to later features. It is a side channel: the
/// feature produces no bodies. Fields not used by a kind are zero. Plane: origin, normal, xDir (orthonormal).
/// Axis: origin, direction. Point: origin.
struct ConstructionGeometry {
    enum class Kind : std::uint8_t { Plane, Axis, Point };
    Kind kind = Kind::Point;
    Vec3 origin;
    Vec3 normal{0, 0, 1};
    Vec3 xDir{1, 0, 0};
    Vec3 direction{0, 0, 1};
};

/// State visible to a feature at its timeline position: the bodies produced by all earlier, unsuppressed
/// features (order-dependent semantics, ARCHITECTURE section 4). Immutable for the duration of compute().
struct EvalContext {
    const std::map<BodyId, kernel::Shape>& bodies;
    const std::map<BodyId, naming::NamingMap>& namings;
    CancelToken cancel;
    /// Document parameter table (A1). nullptr when the evaluation has none; expressions that reference parameters
    /// then fail with NotFound.
    const ParameterTable* params = nullptr;
    /// Construction geometry produced by earlier features (B1). nullptr when the evaluation has none.
    const std::map<FeatureId, ConstructionGeometry>* constructions = nullptr;

    /// nullptr if `id` is not a construction feature at this timeline position.
    const ConstructionGeometry* construction(FeatureId id) const {
        if (!constructions) return nullptr;
        auto it = constructions->find(id);
        return it == constructions->end() ? nullptr : &it->second;
    }

    /// nullptr if `id` does not exist at this timeline position.
    const kernel::Shape* body(BodyId id) const {
        auto it = bodies.find(id);
        return it == bodies.end() ? nullptr : &it->second;
    }
    const naming::NamingMap* naming(BodyId id) const {
        auto it = namings.find(id);
        return it == namings.end() ? nullptr : &it->second;
    }
};

/// What a feature changes. Bodies listed in `bodies` are created or replaced; those in `removed` are deleted.
/// Every produced body MUST have a NamingMap entry sized to its sub-shape counts.
struct FeatureResult {
    std::map<BodyId, kernel::Shape> bodies;
    std::map<BodyId, naming::NamingMap> namings;
    std::vector<BodyId> removed;
    std::vector<Diagnostic> diagnostics;
    /// Set by construction features (B1); the evaluator publishes it to later features and to FeatureStatus.
    std::optional<ConstructionGeometry> constructionGeometry;
    /// Non-geometric attributes of the feature (e.g. the cosmetic thread of a Hole: "thread" -> "M6x1.0"). Published on
    /// FeatureStatus::attributes; they never affect geometry or naming.
    std::map<std::string, std::string> attributes;
};

/// A timeline entry. Implementations are immutable after construction (an edit creates a new instance, so
/// cached results keyed by `paramHash()` stay valid). `compute` is a pure function of (parameters, context)
/// and may run on any worker thread; it must not touch shared mutable state and should poll ctx.cancel.
class Feature {
public:
    virtual ~Feature() = default;

    /// User-visible display name ("Extrude 1"). Not part of paramHash() or geometry: renaming never invalidates
    /// cached results. Empty until the owner (features::Document) assigns one. Features are otherwise immutable; the
    /// owner sets the name only on a freshly created instance, before it is shared.
    const std::string& name() const noexcept { return name_; }
    void setName(std::string name) { name_ = std::move(name); }

    virtual FeatureId id() const = 0;
    /// Stable machine tag used for (de)serialisation and the registry, e.g. "extrude".
    virtual std::string_view kind() const = 0;
    /// Element references the feature's parameters hold (for dependency tracking and broken-ref reporting).
    virtual std::vector<naming::ElementRef> explicitRefs() const { return {}; }
    /// Names of the document parameters this feature's expressions reference (directly; the table expands them
    /// transitively). The evaluator mixes ParameterTable::hashOfUsed(these) into the stamp chain, so editing a
    /// parameter invalidates exactly the features that use it (and what follows them on the timeline).
    virtual std::vector<std::string> parameterReferences() const { return {}; }
    /// Bodies whose current state compute() reads / may modify or replace (implicit order dependency).
    virtual std::vector<BodyId> readsBodies() const { return {}; }
    virtual std::vector<BodyId> writesBodies() const = 0;
    /// Deterministic hash of all parameters + explicit refs (NOT of inputs). Part of the stamp chain.
    virtual std::uint64_t paramHash() const = 0;
    /// Serialised parameters (JSON object text) for the native format; round-trips via FeatureRegistry.
    virtual std::string paramsJson() const = 0;

    /// Errors: any ErrorCode; ErrorCode::Cancelled if ctx.cancel fired. A failing feature leaves the
    /// timeline state unchanged for features after it (they are evaluated against the previous state, marked broken).
    virtual Result<FeatureResult> compute(const EvalContext& ctx) const = 0;

private:
    std::string name_;
};

/// Creates features by kind (used by io::loadDocument). Not thread-safe for registration; register at start-up.
class FeatureRegistry {
public:
    using Factory = std::function<Result<std::shared_ptr<Feature>>(FeatureId, std::string_view paramsJson)>;
    void add(std::string kind, Factory factory);
    /// Errors: NotFound (unknown kind) or whatever the factory returns.
    Result<std::shared_ptr<Feature>> create(std::string_view kind, FeatureId id, std::string_view paramsJson) const;

private:
    std::map<std::string, Factory, std::less<>> factories_;
};

} // namespace iw::recompute
