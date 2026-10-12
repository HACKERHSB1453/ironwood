// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/ids.hpp"
#include "fuzz/rng.hpp"
#include "naming/element_name.hpp"
#include "recompute/feature.hpp"
#include "recompute/timeline.hpp"

namespace iw::fuzz {

/// Geometric signature of one sub-shape: its type plus the kernel's {centroid x, y, z, measure} key
/// (kernel::Shape::subShapeKey). Normals/axes are not exposed by the public kernel API, so the signature is
/// centroid + measure only; that is enough to tell apart the faces/edges/vertices of the prismatic and
/// revolved solids P1a builds, but it is weaker than a full surface check (see the report's oracle notes).
struct GeomSig {
    naming::ElementType type = naming::ElementType::Face;
    std::array<double, 4> key{}; ///< centroid x, y, z (mm), measure (mm^2 / mm / 0)
};

/// True when two signatures describe the same element within tolerance: same type, centroid within 1e-4 mm,
/// measure within 1e-6 relative (+1e-6 absolute).
bool sameGeometry(const GeomSig& a, const GeomSig& b);
std::string describe(const GeomSig& s);

/// Read-only context a generator may use when it creates a feature.
struct GenContext {
    const recompute::Timeline& timeline; ///< Timeline the new feature is inserted into.
    std::vector<BodyId> bodies;          ///< Bodies written by features already in the timeline (may be empty).
};

/// Plug-in point: one implementation per feature kind. The fuzzer never names concrete kinds; it asks each
/// registered generator to produce features, edit their dimensions and (optionally) predict their geometry.
/// Adding a kind (B1: revolve, fillet, chamfer, hole) means: implement this interface and register it in
/// GeneratorRegistry::defaults() (or call GeneratorRegistry::add from the caller).
class FeatureGenerator {
public:
    virtual ~FeatureGenerator() = default;

    /// Must equal Feature::kind() of every feature this generator produces.
    virtual std::string_view kind() const = 0;

    /// Random, valid feature with id `id`. nullptr when this kind cannot be generated in `ctx`.
    virtual std::shared_ptr<const recompute::Feature> generate(Rng& rng, FeatureId id, const GenContext& ctx) const = 0;

    /// Copy of `f` (same kind and id) with one dimension changed (a "dimension edit"). nullptr = not editable.
    virtual std::shared_ptr<const recompute::Feature> editDimension(Rng& rng, const recompute::Feature& f) const = 0;

    /// Independent prediction of the geometry of element `name` that `f` produces from its OWN parameters, valid
    /// when `f` is evaluated on a body it fully determines (no upstream dependence). The result is a set of
    /// candidate signatures; the resolved element must match one of them. nullopt = no prediction for this name.
    virtual std::optional<std::vector<GeomSig>> predict(const recompute::Feature& f,
                                                        const naming::ElementName& name) const {
        (void)f;
        (void)name;
        return std::nullopt;
    }
};

/// Registry of feature generators keyed by kind. Iteration order is the kind order (deterministic).
class GeneratorRegistry {
public:
    void add(std::shared_ptr<const FeatureGenerator> generator);
    /// nullptr if no generator is registered for `kind`.
    const FeatureGenerator* find(std::string_view kind) const;
    std::vector<const FeatureGenerator*> all() const;

    /// Every generator shipped with the fuzzer (today: extrude).
    static GeneratorRegistry defaults();

private:
    std::map<std::string, std::shared_ptr<const FeatureGenerator>, std::less<>> byKind_;
};

/// Generator for recompute::ExtrudeFeature (kind "extrude"): NewBody and Join, with predictions for NewBody.
std::shared_ptr<const FeatureGenerator> makeExtrudeGenerator();

} // namespace iw::fuzz
