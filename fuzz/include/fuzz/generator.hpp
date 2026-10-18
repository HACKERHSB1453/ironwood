// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/ids.hpp"
#include "core/result.hpp"
#include "fuzz/geom.hpp"
#include "fuzz/rng.hpp"
#include "naming/element_name.hpp"
#include "naming/resolve.hpp"
#include "recompute/feature.hpp"
#include "recompute/timeline.hpp"

namespace iw::fuzz {

/// Resolver under test. The default is naming::resolve; tests inject deliberately broken resolvers to prove the
/// oracle catches silent wrong picks.
using ResolveFn = std::function<Result<naming::Resolution>(const naming::ElementRef&, const naming::NamingMap&)>;

/// Read-only context a generator may use when it creates a feature.
struct GenContext {
    const recompute::Timeline& timeline; ///< Timeline the new feature is inserted into.
    std::vector<BodyId> bodies;          ///< Bodies written by features already in the timeline (may be empty).
    const ModelState& state;             ///< Model state at the insertion position (bodies, names, construction axes).
    /// Writer of `body` at the insertion position (last feature before it that writes it), or an invalid id.
    FeatureId writerOf(BodyId body) const;
    /// A body id not used by any body of `state` or by any timeline feature.
    BodyId freshBody() const;
};

/// What a predictor may look at when it predicts the geometry of a name produced by a feature.
struct PredictContext {
    const ModelState& in; ///< State BEFORE the feature (on the edited timeline).
    ResolveFn resolve;    ///< Resolver used to look names up in `in` (the one under test).
    /// Signature of the element `in` binds `name` to in `body`: exact match first, then the resolver (history).
    std::optional<GeomSig> lookup(BodyId body, const naming::ElementName& name) const;
    /// Index `in` binds `name` to in `body` (exact, then the resolver).
    std::optional<std::uint32_t> indexOf(BodyId body, const naming::ElementName& name) const;
};

/// Plug-in point: one implementation per feature kind. The fuzzer never names concrete kinds; it asks each registered
/// generator to produce features, edit their dimensions and predict their geometry.
class FeatureGenerator {
public:
    virtual ~FeatureGenerator() = default;

    /// Must equal Feature::kind() of every feature this generator produces.
    virtual std::string_view kind() const = 0;
    /// Relative frequency among generators (integer weight).
    virtual int weight() const { return 3; }

    /// Random, valid feature with id `id`. nullptr when this kind cannot be generated in `ctx`.
    virtual std::shared_ptr<const recompute::Feature> generate(Rng& rng, FeatureId id, const GenContext& ctx) const = 0;

    /// Copy of `f` (same kind and id) with one dimension changed (a "dimension edit"). nullptr = not editable.
    virtual std::shared_ptr<const recompute::Feature> editDimension(Rng& rng, const recompute::Feature& f) const = 0;

    /// Independent prediction of the geometry of element `name` (name.feature == f.id()) as produced by `f`. Any-of
    /// semantics: the resolved element must satisfy one of the returned expectations. nullopt = no prediction.
    virtual std::optional<std::vector<Expected>> predict(const recompute::Feature& f, const naming::ElementName& name,
                                                         const PredictContext& ctx) const {
        (void)f; (void)name; (void)ctx;
        return std::nullopt;
    }

    /// Volume invariant of `f` on the body it modifies: +1 the volume must not decrease (join), -1 it must not increase (cut, hole),
    /// 2 it must be preserved (move), 0 no invariant. A violation is a kernel / boolean fault, independent of naming.
    virtual int volumeRelation(const recompute::Feature& f) const {
        (void)f;
        return 0;
    }

    /// Body whose elements `f` inherits (modifies in place, copies, ...). nullopt: `f` builds its body from scratch.
    virtual std::optional<BodyId> inheritsFrom(const recompute::Feature& f) const {
        (void)f;
        return std::nullopt;
    }

    /// Role-stability check across an edit of this feature: a name that the generator produced must keep naming the same
    /// kind of element. Returns a message when `post` (resolved after the edit) plays a different role than `pre`.
    virtual std::optional<std::string> roleChanged(const recompute::Feature& before, const recompute::Feature& after,
                                                   const naming::ElementName& name, const GeomSig& pre,
                                                   const GeomSig& post) const {
        (void)before; (void)after; (void)name; (void)pre; (void)post;
        return std::nullopt;
    }

    /// Like roleChanged() but only counted (Counters::advisoryRoleChanges), never a failure: names of elements the kernel
    /// reports no history for ("<tag>.new#k") are ordered by geometry and cannot be fully stable across topology edits.
    virtual std::optional<std::string> advisoryRoleChanged(const recompute::Feature& before, const recompute::Feature& after,
                                                           const naming::ElementName& name, const GeomSig& pre,
                                                           const GeomSig& post) const {
        (void)before; (void)after; (void)name; (void)pre; (void)post;
        return std::nullopt;
    }

    /// Expectation for the output element that inherits input element `in` (same name). Default: same supporting
    /// surface / line, overlapping, same vertex point.
    virtual Expected inherited(const recompute::Feature& f, const GeomSig& in) const {
        (void)f;
        return inheritedFrom(in);
    }
};

/// Registry of feature generators keyed by kind. Iteration order is the kind order (deterministic).
class GeneratorRegistry {
public:
    void add(std::shared_ptr<const FeatureGenerator> generator);
    /// nullptr if no generator is registered for `kind`.
    const FeatureGenerator* find(std::string_view kind) const;
    std::vector<const FeatureGenerator*> all() const;

    /// Every generator shipped with the fuzzer: extrude (new/join/cut/coplanar join), revolve, axis, fillet, chamfer,
    /// hole, move/copy.
    static GeneratorRegistry defaults();

private:
    std::map<std::string, std::shared_ptr<const FeatureGenerator>, std::less<>> byKind_;
};

std::shared_ptr<const FeatureGenerator> makeExtrudeGenerator();
std::shared_ptr<const FeatureGenerator> makeRevolveGenerator();
std::shared_ptr<const FeatureGenerator> makeAxisGenerator();
std::shared_ptr<const FeatureGenerator> makeFilletGenerator();
std::shared_ptr<const FeatureGenerator> makeChamferGenerator();
std::shared_ptr<const FeatureGenerator> makeHoleGenerator();
std::shared_ptr<const FeatureGenerator> makeMoveGenerator();

} // namespace iw::fuzz
