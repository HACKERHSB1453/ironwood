// SPDX-License-Identifier: GPL-3.0-or-later
#include "recompute/evaluator.hpp"

#include <algorithm>
#include <exception>
#include <mutex>
#include <set>

#include "core/profile.hpp"

namespace iw::recompute {
namespace {

/// Model state after one timeline position (immutable once cached; shared between EvalResults of different calls).
struct State {
    std::map<BodyId, kernel::Shape> bodies;
    std::map<BodyId, naming::NamingMap> namings;
    std::map<FeatureId, ConstructionGeometry> constructions; ///< Published by Ok construction features.
    std::set<BodyId> poisoned;          ///< Bodies a failed/skipped feature should have written.
    std::set<FeatureId> broken;         ///< Failed or Skipped features (explicit refs into them are dead).
};

struct Entry {
    std::uint64_t chain = 0;            ///< Stamp chain value after this feature.
    std::shared_ptr<const State> state;
    FeatureStatus status;               ///< `reused` is filled per call.
};

std::uint64_t chainStep(std::uint64_t prev, const Feature& f, std::uint64_t paramTableHash, bool suppressed) {
    kernel::ShapeStamp s{prev};
    s = kernel::ShapeStamp::combine(s, f.id().value());
    s = kernel::ShapeStamp::combine(s, f.paramHash());
    s = kernel::ShapeStamp::combine(s, paramTableHash);
    s = kernel::ShapeStamp::combine(s, suppressed ? 0x5355505052455353ULL : 0x41435449564521ULL);
    return s.value;
}

Entry computeEntry(const Feature& f, bool suppressed, std::uint64_t chain, const State& prev, const ParameterTable* params,
                   CancelToken cancel, Error* cancelled) {
    Entry e;
    e.chain = chain;
    e.status.id = f.id();
    auto st = std::make_shared<State>(prev);

    if (suppressed) {
        e.status.state = FeatureStatus::State::Suppressed;
        e.state = std::move(st);
        return e;
    }

    // Skip when the feature depends on something a failed/skipped feature should have provided.
    std::string why;
    for (BodyId b : f.readsBodies())
        if (prev.poisoned.contains(b)) { why = "reads body produced by a failed feature"; break; }
    if (why.empty())
        for (const auto& r : f.explicitRefs())
            if (prev.broken.contains(r.feature)) { why = "references a failed feature"; break; }
    if (!why.empty()) {
        e.status.state = FeatureStatus::State::Skipped;
        e.status.error = Error{ErrorCode::NotFound, why};
        e.status.diagnostics.push_back({Diagnostic::Severity::Error, why, std::nullopt});
        st->broken.insert(f.id());
        for (BodyId b : f.writesBodies()) st->poisoned.insert(b);
        e.state = std::move(st);
        return e;
    }

    auto fail = [&](Error err) {
        e.status.state = FeatureStatus::State::Failed;
        e.status.diagnostics.push_back({Diagnostic::Severity::Error, err.message, std::nullopt});
        e.status.error = std::move(err);
        auto failedState = std::make_shared<State>(prev);
        failedState->broken.insert(f.id());
        for (BodyId b : f.writesBodies()) failedState->poisoned.insert(b);
        e.state = std::move(failedState);
    };

    Result<FeatureResult> res = makeError(ErrorCode::Internal, "not computed");
    try {
        EvalContext ctx{prev.bodies, prev.namings, cancel, params, &prev.constructions};
        res = f.compute(ctx);
    } catch (const std::exception& ex) {
        res = makeError(ErrorCode::Internal, std::string("feature threw: ") + ex.what());
    } catch (...) {
        res = makeError(ErrorCode::Internal, "feature threw a non-standard exception");
    }
    if (!res) {
        if (res.error().code == ErrorCode::Cancelled) {
            *cancelled = res.error();
            return e;
        }
        fail(std::move(res.error()));
        return e;
    }

    // Validate the contract: only declared bodies, each with a naming map.
    const auto writes = f.writesBodies();
    auto declared = [&](BodyId b) { return std::find(writes.begin(), writes.end(), b) != writes.end(); };
    for (const auto& [id, shape] : res->bodies) {
        if (!declared(id)) { fail({ErrorCode::Internal, "feature produced a body not in writesBodies()"}); return e; }
        if (!res->namings.contains(id)) { fail({ErrorCode::Internal, "feature produced a body without a NamingMap"}); return e; }
    }
    for (BodyId id : res->removed)
        if (!declared(id)) { fail({ErrorCode::Internal, "feature removed a body not in writesBodies()"}); return e; }

    for (BodyId id : res->removed) {
        st->bodies.erase(id);
        st->namings.erase(id);
        st->poisoned.erase(id);
    }
    for (auto& [id, shape] : res->bodies) {
        const kernel::ShapeStamp s = kernel::ShapeStamp::combine(kernel::ShapeStamp{chain}, id.value());
        st->bodies.insert_or_assign(id, shape.withStamp(s));
        st->namings.insert_or_assign(id, std::move(res->namings.at(id)));
        st->poisoned.erase(id);
    }
    st->constructions.erase(f.id());
    if (res->constructionGeometry) {
        st->constructions.insert_or_assign(f.id(), *res->constructionGeometry);
        e.status.construction = *res->constructionGeometry;
    }
    e.status.state = FeatureStatus::State::Ok;
    e.status.attributes = std::move(res->attributes);
    e.status.diagnostics = std::move(res->diagnostics);
    e.state = std::move(st);
    return e;
}

} // namespace

struct Evaluator::Impl {
    std::mutex mutex;
    std::vector<Entry> cache;   ///< cache[k] = state after timeline feature k, valid while the chain matches.
};

Evaluator::Evaluator() : impl_(std::make_unique<Impl>()) {}
Evaluator::~Evaluator() = default;

Result<EvalResult> Evaluator::evaluate(const Timeline& timeline, CancelToken cancel) {
    return evaluate(timeline, nullptr, cancel);
}

Result<EvalResult> Evaluator::evaluate(const Timeline& timeline, const ParameterTable* params, CancelToken cancel) {
    IW_ZONE("recompute::evaluate");
    std::lock_guard lock(impl_->mutex);
    auto& cache = impl_->cache;
    const std::size_t marker = timeline.rollbackMarker();
    static const State empty;

    EvalResult out;
    std::uint64_t prevChain = 0;
    const State* prev = &empty;
    bool valid = true; // cache[0..k) still consistent with the timeline
    for (std::size_t k = 0; k < marker; ++k) {
        const Feature& f = *timeline.at(k);
        const bool suppressed = timeline.isSuppressed(f.id());
        const std::uint64_t paramStamp = params ? params->hashOfUsed(f.parameterReferences()) : 0;
        const std::uint64_t chain = chainStep(prevChain, f, paramStamp, suppressed);

        if (valid && k < cache.size() && cache[k].chain == chain) {
            ++out.reusedCount;
            FeatureStatus s = cache[k].status;
            s.reused = true;
            out.statuses.push_back(std::move(s));
        } else {
            if (valid) { if (cache.size() > k) cache.erase(cache.begin() + static_cast<std::ptrdiff_t>(k), cache.end()); valid = false; } // first mismatch: later entries derive from stale state
            if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "evaluation cancelled");
            Error cancelledErr;
            Entry e = computeEntry(f, suppressed, chain, *prev, params, cancel, &cancelledErr);
            if (!e.state) return std::unexpected(cancelledErr); // feature observed cancellation; cache holds [0,k)
            ++out.computedCount;
            cache.push_back(std::move(e));
            out.statuses.push_back(cache.back().status);
        }
        prev = cache[k].state.get();
        prevChain = chain;
    }
    // Results are copies of cached state (Shapes share geometry; NamingMaps are small tables).
    out.bodies = prev->bodies;
    out.namings = prev->namings;
    return out;
}

void Evaluator::clearCache() {
    std::lock_guard lock(impl_->mutex);
    impl_->cache.clear();
}

} // namespace iw::recompute
