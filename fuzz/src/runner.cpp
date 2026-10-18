// SPDX-License-Identifier: GPL-3.0-or-later
#include "fuzz/runner.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <optional>
#include <set>

#include <nlohmann/json.hpp>

#include <sys/wait.h>
#include <unistd.h>

#include "features/revolve_feature.hpp"
#include "kernel/ops.hpp"
#include "recompute/evaluator.hpp"
#include "recompute/feature.hpp"
#include "recompute/timeline.hpp"

namespace iw::fuzz {
namespace {

using recompute::Evaluator;
using recompute::EvalResult;
using recompute::FeatureStatus;
using recompute::Timeline;
using naming::ElementType;

constexpr std::array<ElementType, 3> kTypes{ElementType::Face, ElementType::Edge, ElementType::Vertex};
constexpr const char* kTypeLetter[] = {"F", "E", "V"};

/// One captured reference: a name the owner feature assigned to an element of its own result, plus the element's
/// geometry before the edit.
struct Captured {
    naming::ElementRef ref;
    BodyId body;
    GeomSig sig;
    std::size_t pos = 0; ///< Owner position in the pre-edit timeline.
};

const char* qualityName(naming::ResolveQuality q) {
    switch (q) {
    case naming::ResolveQuality::Exact: return "Exact";
    case naming::ResolveQuality::HistoryDerived: return "HistoryDerived";
    case naming::ResolveQuality::GeometricFallback: return "GeometricFallback";
    }
    return "?";
}

std::string nameText(const naming::ElementRef& ref) {
    return "F" + std::to_string(ref.feature.value()) + " " + ref.name.toString();
}

/// Timeline copy evaluated up to (not including) `upto`.
Result<EvalResult> evaluatePrefix(Evaluator& ev, const Timeline& t, std::size_t upto) {
    Timeline c = t;
    c.setRollbackMarker(upto);
    return ev.evaluate(c);
}

/// True when features [0, upto] are identical (id, kind, parameters, suppression) in both timelines. The geometry
/// of an element of feature k depends only on that prefix, so an identical prefix means an unchanged result.
bool prefixIdentical(const Timeline& a, const Timeline& b, std::size_t upto) {
    if (a.size() <= upto || b.size() <= upto) return false;
    for (std::size_t k = 0; k <= upto; ++k) {
        const auto& fa = a.at(k);
        const auto& fb = b.at(k);
        if (fa->id() != fb->id() || fa->kind() != fb->kind() || fa->paramHash() != fb->paramHash()) return false;
        if (a.isSuppressed(fa->id()) != b.isSuppressed(fb->id())) return false;
    }
    return true;
}

std::optional<std::size_t> positionOf(const Timeline& t, FeatureId id) {
    for (std::size_t k = 0; k < t.size(); ++k)
        if (t.at(k)->id() == id) return k;
    return std::nullopt;
}

bool usable(const std::vector<Expected>& v) {
    return std::any_of(v.begin(), v.end(), [](const Expected& e) { return e.mask != 0 || static_cast<bool>(e.extra); });
}

bool anyMatch(const std::vector<Expected>& candidates, const GeomSig& got) {
    return std::any_of(candidates.begin(), candidates.end(), [&](const Expected& c) { return matches(c, got); });
}

std::string candidatesText(const std::vector<Expected>& candidates) {
    std::string out;
    const std::size_t shown = std::min<std::size_t>(candidates.size(), 3);
    for (std::size_t i = 0; i < shown; ++i) out += (i ? ", " : "") + describe(candidates[i]);
    if (candidates.size() > shown) out += ", ... (" + std::to_string(candidates.size()) + " candidates)";
    return out;
}

bool endsWith(std::string_view s, std::string_view suf) {
    return s.size() >= suf.size() && s.substr(s.size() - suf.size()) == suf;
}

/// Everything needed to predict names of one feature at one timeline position of one timeline.
struct PositionModel {
    ModelState in;                         ///< State before the feature.
    std::shared_ptr<EvalResult> post;      ///< State after the feature (prefix up to and including it).
};

/// Edges and vertices a boolean generates from input faces / edges ("fuse.edge", "cut.vertex", ...) lie on the surfaces /
/// lines they derive from. Only sources that resolve in the input state and carry usable geometry contribute.
std::optional<std::vector<Expected>> derivedFromSources(const FeatureGenerator& g, const recompute::Feature& f,
                                                        const naming::ElementName& name, const PredictContext& pc) {
    if (name.type == ElementType::Face) return std::nullopt;
    const std::string role = name.op.substr(name.op.find('.') + 1);
    if (role != "edge" && role != "vertex") return std::nullopt;
    const auto body = g.inheritsFrom(f);
    if (!body) return std::nullopt;
    std::vector<GeomSig> srcs;
    for (const auto& s : name.sources)
        if (s.feature != f.id())
            if (auto sig = pc.lookup(*body, s)) srcs.push_back(*sig);
    std::vector<std::function<bool(const GeomSig&)>> checks;
    for (const GeomSig& s : srcs) {
        if (s.type == ElementType::Face && s.surface == SurfaceKind::Plane) {
            checks.push_back([s](const GeomSig& got) {
                // Tolerances are loose on purpose: booleans on nearly coincident faces leave ~1e-2 rad / 1e-3 mm of noise.
                if (std::abs(dot(s.normal, got.centroid() - s.point)) > 1e-3) return false;
                return !got.isLine || std::abs(dot(s.normal, got.dir)) < 2e-2;
            });
        } else if (s.type == ElementType::Face && s.surface == SurfaceKind::Cylinder && name.type == ElementType::Vertex) {
            checks.push_back([s](const GeomSig& got) { return std::abs(distToLine(got.centroid(), s.point, s.normal) - s.radius) < 1e-4; });
        } else if (s.type == ElementType::Edge && s.isLine && name.type == ElementType::Vertex) {
            checks.push_back([s](const GeomSig& got) { return distToLine(got.centroid(), s.point, s.dir) < 1e-4; });
        }
    }
    if (checks.empty()) return std::nullopt;
    Expected e;
    e.sig.type = name.type;
    e.note = "on the surfaces it derives from";
    e.extra = [checks](const GeomSig& got) {
        if (got.type == ElementType::Edge && got.measure() < 1e-9) return true;
        return std::all_of(checks.begin(), checks.end(), [&](const auto& c) { return c(got); });
    };
    return std::vector<Expected>{e};
}

/// Prediction of the geometry of `name` as resolved in the output of feature `f`: its own parametric prediction, a
/// relaxed derivation from the sources of a split / merge name, or inheritance from the input element of the same name.
std::optional<std::vector<Expected>> predictFor(const FeatureGenerator& g, const recompute::Feature& f,
                                                const naming::ElementName& name, const PredictContext& pc) {
    if (name.feature == f.id()) {
        if (auto own = g.predict(f, name, pc)) return own;
        if ((endsWith(name.op, ".split") || endsWith(name.op, ".merge")) && !name.sources.empty()) {
            const auto body = g.inheritsFrom(f);
            std::vector<Expected> out;
            for (const auto& src : name.sources) {
                if (src.type != name.type) continue;
                if (src.feature == f.id()) {
                    auto p = g.predict(f, src, pc);
                    if (!p) return std::nullopt;
                    for (const Expected& e : *p) out.push_back(relaxed(e));
                } else {
                    if (!body) return std::nullopt;
                    auto sig = pc.lookup(*body, src);
                    if (!sig) return std::nullopt;
                    out.push_back(relaxed(g.inherited(f, *sig)));
                }
            }
            if (out.empty()) return std::nullopt;
            return out;
        }
        return derivedFromSources(g, f, name, pc);
    }
    const auto body = g.inheritsFrom(f);
    if (!body) return std::nullopt;
    auto sig = pc.lookup(*body, name);
    if (!sig) return std::nullopt;
    return std::vector<Expected>{g.inherited(f, *sig)};
}

/// Checks one edit (or the baseline when before == after). Captures refs on `before`, resolves them on `after`.
void checkStep(Evaluator& ev, const Timeline& before, const Timeline& after, int stepIndex,
               const GeneratorRegistry& reg, const ResolveFn& resolve, ScenarioOutcome& out) {
    auto fail = [&](std::string category, std::string detail, const std::string& kind = {}) {
        out.failures.push_back({std::move(category), stepIndex, std::move(detail)});
        if (!kind.empty()) ++out.counts.perKind[kind].failures;
    };

    auto preFull = ev.evaluate(before);
    auto postFull = ev.evaluate(after);
    if (!preFull || !postFull) {
        fail("internal_evaluation_error", !preFull ? preFull.error().message : postFull.error().message);
        return;
    }
    for (const auto& st : postFull->statuses) {
        if (st.state == FeatureStatus::State::Failed && st.error.code == ErrorCode::Internal) {
            ++out.counts.internalFeatureErrors;
            out.internalErrors.push_back("F" + std::to_string(st.id.value()) + ": " + st.error.message);
        }
    }

    // Per-position models, evaluated lazily (the evaluator caches prefixes).
    auto modelOf = [&](const Timeline& t, std::map<std::size_t, std::shared_ptr<PositionModel>>& cache,
                       std::size_t p) -> std::shared_ptr<PositionModel> {
        auto it = cache.find(p);
        if (it != cache.end()) return it->second;
        auto m = std::make_shared<PositionModel>();
        if (auto in = evaluatePrefix(ev, t, p)) m->in = ModelState::from(*in);
        if (auto post = evaluatePrefix(ev, t, p + 1)) m->post = std::make_shared<EvalResult>(std::move(*post));
        cache.emplace(p, m);
        return m;
    };
    std::map<std::size_t, std::shared_ptr<PositionModel>> beforeModels, afterModels;

    // Legit loss: a reference of feature `consumer` (at position q in `after`) to an element of body `body` that its owner
    // (position p < q) created. The loss is forced by the edit, not a naming fault, when an edit-ordered feature k with
    // p < k < q (a) overwrites `body` without deriving from it (a new body written over it), or (b) modifies `body` in place
    // and consumes the element: the element is in k's input names, gone from its output names, and k's output names record
    // it among their sources (the fillet / chamfer / cut that ate the edge). Both facts come from the evaluated prefixes.
    // (c) removed_by_owner_edit: the edit changed the owner of the reference (its parameters, suppression) or removed it, and
    // the owner's new output no longer carries the element. Forced only when the new output has no element with the same
    // geometry (sameGeometry) and its topology changed. An element with the same geometry under another name is a naming
    // failure, not a legit loss. A removed owner has no output, so the consumer's input state is the scope of that check.
    auto removedByOwnerEdit = [&](const naming::ElementRef& ref, BodyId body, std::size_t q) -> bool {
        const auto pb = positionOf(before, ref.name.feature);
        if (!pb || preFull->statuses[*pb].state != FeatureStatus::State::Ok) return false;
        const auto mb = modelOf(before, beforeModels, *pb)->post;
        if (!mb) return false;
        auto bNames = mb->namings.find(body);
        auto bShape = mb->bodies.find(body);
        if (bNames == mb->namings.end() || bShape == mb->bodies.end()) return false;
        const auto preIdx = bNames->second.indicesOf(ref.name);
        if (preIdx.empty()) return false;
        const GeomSig lost = readSig(bShape->second, ref.type, preIdx.front());

        const auto pa = positionOf(after, ref.name.feature);
        const bool ownerRemoved = !pa;
        const kernel::Shape* scope = nullptr;
        if (!ownerRemoved) {
            const auto& ownerBefore = before.at(*pb);
            const auto& ownerAfter = after.at(*pa);
            const bool edited = ownerBefore->kind() != ownerAfter->kind() || ownerBefore->paramHash() != ownerAfter->paramHash() ||
                                before.isSuppressed(ownerBefore->id()) != after.isSuppressed(ownerAfter->id());
            if (!edited) return false;
            auto ma = modelOf(after, afterModels, *pa)->post;
            if (!ma) return false;
            auto aNames = ma->namings.find(body);
            auto aShape = ma->bodies.find(body);
            if (aNames != ma->namings.end() && !aNames->second.indicesOf(ref.name).empty()) return false; // still named
            if (aShape != ma->bodies.end()) scope = &aShape->second;
            // Topology must have changed: some sub-shape count of the body differs between the old and the new output.
            bool changed = aShape == ma->bodies.end();
            if (!changed)
                for (ElementType t : kTypes)
                    if (bShape->second.subShapeCount(naming::toSubShapeKind(t)) != aShape->second.subShapeCount(naming::toSubShapeKind(t)))
                        changed = true;
            if (!changed) return false;
        } else {
            const auto mIn = modelOf(after, afterModels, q); // kept alive by the cache: `scope` points into it
            auto inShape = mIn->in.bodies.find(body);
            if (inShape != mIn->in.bodies.end()) scope = &inShape->second;
        }
        if (scope)
            for (std::uint32_t i = 0; i < scope->subShapeCount(naming::toSubShapeKind(ref.type)); ++i)
                if (sameGeometry(readSig(*scope, ref.type, i), lost)) return false; // same geometry survives under another name
        return true;
    };

    auto legitLoss = [&](const naming::ElementRef& ref, BodyId body, std::size_t q) -> std::optional<std::string> {
        const auto p = positionOf(after, ref.name.feature);
        if (p && *p < q)
        for (std::size_t k = *p + 1; k < q; ++k) {
            const auto& fk = after.at(k);
            if (postFull->statuses[k].state != FeatureStatus::State::Ok || after.isSuppressed(fk->id())) continue;
            const auto writes = fk->writesBodies();
            if (std::find(writes.begin(), writes.end(), body) == writes.end()) continue;
            const FeatureGenerator* gk = reg.find(fk->kind());
            const std::optional<BodyId> derived = gk ? gk->inheritsFrom(*fk) : std::nullopt;
            const std::string who = "F" + std::to_string(fk->id().value()) + " (" + std::string(fk->kind()) + ")";
            if (derived != std::optional<BodyId>{body})
                return std::string("overwritten_body");
            auto m = modelOf(after, afterModels, k);
            if (!m->post) continue;
            auto in = m->in.namings.find(body);
            auto outIt = m->post->namings.find(body);
            if (in == m->in.namings.end() || outIt == m->post->namings.end()) continue;
            if (in->second.indicesOf(ref.name).empty() || !outIt->second.indicesOf(ref.name).empty()) continue;
            bool recorded = false;
            for (ElementType t : kTypes)
                for (std::uint32_t i = 0; i < outIt->second.size(t) && !recorded; ++i) {
                    auto nm = outIt->second.nameOf(t, i);
                    if (!nm) continue;
                    for (const auto& src : nm->sources)
                        if (src == ref.name) recorded = true;
                }
            if (recorded) return std::string("consumed_element");
        }
        if (removedByOwnerEdit(ref, body, q)) return std::string("removed_by_owner_edit");
        return std::nullopt;
    };

    // Owner of a reference = the feature that created the name (ref.name.feature). ElementRef::feature on an explicit
    // reference is the body's writer at generation time, which need not be the creator.
    // Reference detachment: a feature that was fine before the edit and now fails because a reference no longer resolves.
    // The lost references are the consumer's explicit references that do not resolve in its input state. A loss is
    // legit_loss when one of them is forced by the edit (see legitLoss). Otherwise it is an F27 failure when a lost
    // reference is hole-created and its hole is still alive and fine: that element should have survived.
    if (&before != &after) {
        auto holeAlive = [&](const naming::ElementRef& ref) {
            auto hp = positionOf(after, ref.name.feature);
            return hp && !after.isSuppressed(ref.name.feature) && postFull->statuses[*hp].state == FeatureStatus::State::Ok &&
                   after.at(*hp)->kind() == "hole";
        };
        for (std::size_t q = 0; q < after.size(); ++q) {
            const auto& fa = after.at(q);
            auto p = positionOf(before, fa->id());
            if (!p || preFull->statuses[*p].state != FeatureStatus::State::Ok || after.isSuppressed(fa->id()) || before.isSuppressed(fa->id())) continue;
            bool holeRef = false; // F27 population: holds a reference to a hole-created element whose hole is alive
            for (const auto& ref : fa->explicitRefs())
                if (ref.name.op.compare(0, 5, "hole.") == 0 && holeAlive(ref)) holeRef = true;
            if (holeRef) ++out.counts.holeRefFeatures;
            const auto& st = postFull->statuses[q];
            if (st.state != FeatureStatus::State::Failed || st.error.message.find("could not be resolved") == std::string::npos) continue;
            ++out.counts.detachedFeatures;

            std::optional<std::string> legit;
            bool lostHole = false;
            const auto mIn = modelOf(after, afterModels, q);
            for (const auto& ref : fa->explicitRefs())
                for (BodyId b : fa->readsBodies()) {
                    auto nm = mIn->in.namings.find(b);
                    if (nm == mIn->in.namings.end() || resolve(ref, nm->second)) continue;
                    if (ref.name.op.compare(0, 5, "hole.") == 0 && holeAlive(ref)) lostHole = true;
                    if (!legit) legit = legitLoss(ref, b, q);
                }
            if (legit) {
                ++out.counts.legitLoss;
                if (lostHole) ++out.counts.legitLossHole;
                ++out.counts.legitLossByCause[*legit];
            } else if (lostHole) {
                ++out.counts.holeRefDetached;
                out.notes.push_back("F27: step " + std::to_string(stepIndex) + ": F" + std::to_string(fa->id().value()) + " (" +
                                    std::string(fa->kind()) + ") lost its hole reference: " + st.error.message);
            }
        }
    }

    // 0. Kernel invariant independent of naming: a join must not lose volume, a cut / hole must not gain it, a move must keep it.
    for (std::size_t q = 0; q < after.size(); ++q) {
        const auto& fa = after.at(q);
        if (postFull->statuses[q].state != FeatureStatus::State::Ok || after.isSuppressed(fa->id())) continue;
        if (&before != &after && prefixIdentical(before, after, q)) continue; // already checked by an earlier step
        const FeatureGenerator* g = reg.find(fa->kind());
        const int rel = g ? g->volumeRelation(*fa) : 0;
        if (rel == 0 || fa->writesBodies().size() != 1) continue;
        const BodyId b = fa->writesBodies().front();
        auto m = modelOf(after, afterModels, q);
        if (!m->post) continue;
        auto vin = m->in.bodies.find(b);
        auto vout = m->post->bodies.find(b);
        if (vin == m->in.bodies.end() || vout == m->post->bodies.end()) continue;
        auto pin = kernel::massProperties(vin->second);
        auto pout = kernel::massProperties(vout->second);
        if (!pin || !pout) continue;
        const double tol = 1e-6 * std::max(1.0, pin->volume) + 1e-6;
        const bool bad = (rel == 1 && pout->volume < pin->volume - tol) || (rel == -1 && pout->volume > pin->volume + tol) ||
                         (rel == 2 && std::abs(pout->volume - pin->volume) > tol);
        ++out.counts.volumeChecks;
        if (bad)
            fail("volume_invariant", "F" + std::to_string(fa->id().value()) + " (" + std::string(fa->kind()) + ") changed the volume of body " +
                                         std::to_string(b.value()) + " from " + std::to_string(pin->volume) + " to " +
                                         std::to_string(pout->volume) + " mm3, against its " +
                                         (rel == 1 ? "join (must not shrink)" : rel == -1 ? "cut (must not grow)" : "move (must be preserved)") + " semantics",
                 std::string(fa->kind()));
    }

    // 1. Capture every name the surviving features assign, with the geometry it names.
    std::vector<Captured> caps;
    for (std::size_t p = 0; p < before.size(); ++p) {
        if (preFull->statuses[p].state != FeatureStatus::State::Ok) continue;
        const auto& f = before.at(p);
        auto m = modelOf(before, beforeModels, p);
        if (!m->post) continue;
        const EvalResult& st = *m->post;
        for (BodyId b : f->writesBodies()) {
            auto shapeIt = st.bodies.find(b);
            auto nameIt = st.namings.find(b);
            if (shapeIt == st.bodies.end() || nameIt == st.namings.end()) continue;
            const kernel::Shape& shape = shapeIt->second;
            for (ElementType t : kTypes) {
                const auto count = shape.subShapeCount(naming::toSubShapeKind(t));
                if (nameIt->second.size(t) != count) {
                    fail("naming_size_mismatch", "F" + std::to_string(f->id().value()) + " body " +
                                                     std::to_string(b.value()) + ": " +
                                                     std::to_string(nameIt->second.size(t)) + " name slots for " +
                                                     std::to_string(count) + " sub-shapes");
                    continue;
                }
                for (std::uint32_t i = 0; i < count; ++i) {
                    auto name = nameIt->second.nameOf(t, i);
                    if (!name) continue;
                    caps.push_back({{f->id(), t, *name}, b, readSig(shape, t, i), p});
                }
            }
        }
    }
    out.counts.refsCaptured += caps.size();

    // 2. Pre-state self-check: the predictions must agree with the geometry the kernel produced before the edit.
    for (const auto& c : caps) {
        const auto& fb = *before.at(c.pos);
        const FeatureGenerator* g = reg.find(fb.kind());
        if (!g) continue;
        auto m = modelOf(before, beforeModels, c.pos);
        const PredictContext pc{m->in, resolve};
        auto pred = predictFor(*g, fb, c.ref.name, pc);
        if (!pred || !usable(*pred)) continue;
        ++out.counts.preStateChecks;
        if (!anyMatch(*pred, c.sig))
            fail("pre_state_inconsistent",
                 nameText(c.ref) + " lies at " + describe(c.sig) + " before the edit; prediction: " + candidatesText(*pred),
                 std::string(fb.kind()));
    }

    // 3. Post-edit resolution.
    for (const auto& c : caps) {
        const std::string ownerKind(before.at(c.pos)->kind());
        KindCounters& kc = out.counts.perKind[ownerKind];
        auto q = positionOf(after, c.ref.feature);
        if (!q) {
            ++out.counts.skippedOwnerGone;
            continue;
        }
        if (after.isSuppressed(c.ref.feature)) {
            ++out.counts.skippedOwnerInactive;
            continue;
        }
        const bool affected = !prefixIdentical(before, after, *q);
        auto m = modelOf(after, afterModels, *q);
        if (!m->post) continue;
        const EvalResult& post = *m->post;
        if (post.statuses[*q].state != FeatureStatus::State::Ok) {
            ++out.counts.skippedOwnerInactive;
            continue;
        }
        auto nameIt = post.namings.find(c.body);
        auto shapeIt = post.bodies.find(c.body);
        if (nameIt == post.namings.end() || shapeIt == post.bodies.end()) {
            ++out.counts.skippedOwnerGone;
            continue;
        }
        ++out.counts.refsChecked;

        auto res = resolve(c.ref, nameIt->second);
        if (!res) {
            const auto& e = res.error();
            if (e.code == ErrorCode::NotFound || e.code == ErrorCode::Ambiguous) {
                if (e.message.empty()) {
                    fail("missing_diagnostic", nameText(c.ref) + " was " + errorCodeName(e.code) + " without a message", ownerKind);
                } else {
                    ++out.counts.diagnosed;
                    ++kc.diagnosed;
                    if (e.code == ErrorCode::Ambiguous) ++out.counts.ambiguous;
                    else ++out.counts.notFound;
                    if (!affected) ++out.counts.stabilityLosses;
                }
            } else {
                fail("resolver_error", nameText(c.ref) + " returned " + errorCodeName(e.code) + ": " + e.message, ownerKind);
            }
            continue;
        }
        const naming::Resolution& r = *res;
        if (r.quality == naming::ResolveQuality::Exact) {
            ++out.counts.resolvedExact;
            ++kc.exact;
        } else {
            ++out.counts.resolvedHistory;
            ++kc.history;
        }
        if (r.quality != naming::ResolveQuality::Exact && r.warnings.empty())
            fail("missing_warning", nameText(c.ref) + " resolved without a warning", ownerKind);

        const auto subCount = shapeIt->second.subShapeCount(naming::toSubShapeKind(c.ref.type));
        if (r.index >= subCount) {
            fail("index_out_of_range", nameText(c.ref) + " resolved to index " + std::to_string(r.index) + " of " +
                                           std::to_string(subCount), ownerKind);
            continue;
        }
        const GeomSig got = readSig(shapeIt->second, c.ref.type, r.index);

        const auto& fa = *after.at(*q);
        const FeatureGenerator* g = reg.find(fa.kind());
        std::optional<std::vector<Expected>> pred;
        if (g) {
            const PredictContext pc{m->in, resolve};
            pred = predictFor(*g, fa, c.ref.name, pc);
        }
        const bool havePred = pred && usable(*pred);
        bool verified = false;
        if (havePred) {
            verified = true;
            ++out.counts.predictedChecks;
            ++kc.predicted;
            if (!anyMatch(*pred, got))
                fail("wrong_pick_predicted", nameText(c.ref) + " resolved (" + qualityName(r.quality) + ") to index " +
                                                 std::to_string(r.index) + " at " + describe(got) +
                                                 "; expected one of " + candidatesText(*pred), ownerKind);
        }
        if (!affected) {
            if (!havePred) {
                verified = true;
                ++out.counts.unaffectedChecks;
                ++kc.unaffected;
            }
            if (!sameGeometry(got, c.sig))
                fail("wrong_pick_unaffected", nameText(c.ref) + " was " + describe(c.sig) +
                                                  " before an edit that did not touch its feature prefix; resolved (" +
                                                  qualityName(r.quality) + ") to index " +
                                                  std::to_string(r.index) + " at " + describe(got), ownerKind);
        }
        if (affected && g && c.ref.name.feature == fa.id()) {
            // Role stability only makes sense while the sibling set is unchanged: same number of elements carrying this
            // (feature, op, type) before and after (a different count means the topology itself changed).
            std::size_t pre = 0, postCount = 0;
            for (const auto& o : caps)
                if (o.ref.feature == c.ref.feature && o.body == c.body && o.ref.type == c.ref.type && o.ref.name.feature == c.ref.name.feature &&
                    o.ref.name.op == c.ref.name.op)
                    ++pre;
            for (std::uint32_t i = 0; i < nameIt->second.size(c.ref.type); ++i) {
                auto nm = nameIt->second.nameOf(c.ref.type, i);
                if (nm && nm->feature == c.ref.name.feature && nm->op == c.ref.name.op) ++postCount;
            }
            if (pre == postCount) {
                if (auto msg = g->roleChanged(*before.at(c.pos), fa, c.ref.name, c.sig, got)) {
                    ++out.counts.roleChecks;
                    fail("role_changed", nameText(c.ref) + " " + *msg + " (resolved " + qualityName(r.quality) + " to index " +
                                             std::to_string(r.index) + " at " + describe(got) + ", was " + describe(c.sig) + ")",
                         ownerKind);
                }
                if (g->advisoryRoleChanged(*before.at(c.pos), fa, c.ref.name, c.sig, got)) ++out.counts.advisoryRoleChanges;
            }
        }
        if (!verified) {
            ++out.counts.unverifiedMoved;
            ++kc.unverified;
            ++out.counts.unverifiedBy[ownerKind + " " + (c.ref.name.feature == fa.id() ? c.ref.name.op : std::string("<inherited>")) + " " +
                                      kTypeLetter[static_cast<int>(c.ref.type)]];
        }
    }
}

} // namespace

KindCounters& KindCounters::operator+=(const KindCounters& o) {
    exact += o.exact;
    history += o.history;
    diagnosed += o.diagnosed;
    predicted += o.predicted;
    unaffected += o.unaffected;
    unverified += o.unverified;
    failures += o.failures;
    return *this;
}

Counters& Counters::operator+=(const Counters& o) {
    stepsChecked += o.stepsChecked;
    refsCaptured += o.refsCaptured;
    refsChecked += o.refsChecked;
    resolvedExact += o.resolvedExact;
    resolvedHistory += o.resolvedHistory;
    ambiguous += o.ambiguous;
    notFound += o.notFound;
    diagnosed += o.diagnosed;
    predictedChecks += o.predictedChecks;
    unaffectedChecks += o.unaffectedChecks;
    unverifiedMoved += o.unverifiedMoved;
    preStateChecks += o.preStateChecks;
    volumeChecks += o.volumeChecks;
    roleChecks += o.roleChecks;
    advisoryRoleChanges += o.advisoryRoleChanges;
    detachedFeatures += o.detachedFeatures;
    holeRefFeatures += o.holeRefFeatures;
    holeRefDetached += o.holeRefDetached;
    legitLoss += o.legitLoss;
    legitLossHole += o.legitLossHole;
    stabilityLosses += o.stabilityLosses;
    skippedOwnerGone += o.skippedOwnerGone;
    skippedOwnerInactive += o.skippedOwnerInactive;
    internalFeatureErrors += o.internalFeatureErrors;
    for (const auto& [k, v] : o.perKind) perKind[k] += v;
    for (const auto& [k, v] : o.unverifiedBy) unverifiedBy[k] += v;
    for (const auto& [k, v] : o.legitLossByCause) legitLossByCause[k] += v;
    return *this;
}

ScenarioOutcome runScenario(const Scenario& scenario, const GeneratorRegistry& registry, const ResolveFn& resolve) {
    ScenarioOutcome out;
    Evaluator ev;
    Timeline tl;
    for (const auto& f : scenario.base) tl.append(f);

    // Baseline: an unedited timeline must resolve every name back to the same geometry.
    checkStep(ev, tl, tl, -1, registry, resolve, out);

    for (std::size_t k = 0; k < scenario.steps.size(); ++k) {
        Timeline after = tl;
        if (auto r = applyStep(after, scenario.steps[k]); !r) continue; // inapplicable step: a no-op
        ++out.counts.stepsChecked;
        checkStep(ev, tl, after, static_cast<int>(k), registry, resolve, out);
        tl = std::move(after);
    }
    return out;
}


namespace {

using nlohmann::json;

json countersToJson(const Counters& c) {
    json j;
    j["steps"] = c.stepsChecked; j["det"] = c.detachedFeatures; j["hrf"] = c.holeRefFeatures; j["hrd"] = c.holeRefDetached; j["lgl"] = c.legitLoss; j["lglh"] = c.legitLossHole; j["cap"] = c.refsCaptured; j["chk"] = c.refsChecked; j["ex"] = c.resolvedExact;
    j["hist"] = c.resolvedHistory; j["amb"] = c.ambiguous; j["nf"] = c.notFound; j["diag"] = c.diagnosed;
    j["pred"] = c.predictedChecks; j["unaff"] = c.unaffectedChecks; j["unv"] = c.unverifiedMoved;
    j["pre"] = c.preStateChecks; j["vol"] = c.volumeChecks; j["role"] = c.roleChecks; j["adv"] = c.advisoryRoleChanges; j["loss"] = c.stabilityLosses;
    j["gone"] = c.skippedOwnerGone; j["inact"] = c.skippedOwnerInactive; j["int"] = c.internalFeatureErrors;
    json k = json::object();
    for (const auto& [name, v] : c.perKind)
        k[name] = {v.exact, v.history, v.diagnosed, v.predicted, v.unaffected, v.unverified, v.failures};
    j["kinds"] = std::move(k);
    j["unvby"] = c.unverifiedBy;
    j["lglc"] = c.legitLossByCause;
    return j;
}

Counters countersFromJson(const json& j) {
    Counters c;
    c.stepsChecked = j["steps"]; c.detachedFeatures = j["det"]; c.holeRefFeatures = j["hrf"]; c.holeRefDetached = j["hrd"]; c.legitLoss = j["lgl"]; c.legitLossHole = j["lglh"]; c.refsCaptured = j["cap"]; c.refsChecked = j["chk"]; c.resolvedExact = j["ex"];
    c.resolvedHistory = j["hist"]; c.ambiguous = j["amb"]; c.notFound = j["nf"]; c.diagnosed = j["diag"];
    c.predictedChecks = j["pred"]; c.unaffectedChecks = j["unaff"]; c.unverifiedMoved = j["unv"];
    c.preStateChecks = j["pre"]; c.volumeChecks = j["vol"]; c.roleChecks = j["role"]; c.advisoryRoleChanges = j["adv"]; c.stabilityLosses = j["loss"];
    c.skippedOwnerGone = j["gone"]; c.skippedOwnerInactive = j["inact"]; c.internalFeatureErrors = j["int"];
    for (const auto& [name, v] : j["kinds"].items()) {
        KindCounters k;
        k.exact = v[0]; k.history = v[1]; k.diagnosed = v[2]; k.predicted = v[3]; k.unaffected = v[4]; k.unverified = v[5]; k.failures = v[6];
        c.perKind[name] = k;
    }
    for (const auto& [name, v] : j["unvby"].items()) c.unverifiedBy[name] = v;
    for (const auto& [name, v] : j["lglc"].items()) c.legitLossByCause[name] = v;
    return c;
}

std::string outcomeToJson(const ScenarioOutcome& o) {
    json j;
    j["counts"] = countersToJson(o.counts);
    json f = json::array();
    for (const auto& x : o.failures) f.push_back({x.category, x.step, x.detail});
    j["failures"] = std::move(f);
    j["internal"] = o.internalErrors;
    j["notes"] = o.notes;
    return j.dump();
}

ScenarioOutcome outcomeFromJson(const std::string& text) {
    ScenarioOutcome o;
    json j = json::parse(text);
    o.counts = countersFromJson(j["counts"]);
    for (const auto& x : j["failures"]) o.failures.push_back({x[0], x[1], x[2]});
    for (const auto& m : j["internal"]) o.internalErrors.push_back(m);
    for (const auto& m : j["notes"]) o.notes.push_back(m);
    return o;
}

} // namespace

namespace {

struct Forked {
    bool ran = false;       ///< fork succeeded
    bool signaled = false;
    int sig = 0;
    int status = 0;
    std::string text;
};

/// Runs `childWork` in a forked child; its return value is sent back through a pipe. The child may write a one-byte marker
/// to the descriptor it is given before returning.
Forked forkRun(const std::function<std::string(int)>& childWork) {
    Forked r;
    int fds[2];
    if (pipe(fds) != 0) return r;
    std::fflush(nullptr);
    const pid_t pid = fork();
    if (pid < 0) {
        close(fds[0]);
        close(fds[1]);
        return r;
    }
    if (pid == 0) { // child: leave without running the parent's destructors
        close(fds[0]);
        alarm(600);
        const std::string text = childWork(fds[1]);
        std::size_t done = 0;
        while (done < text.size()) {
            const ssize_t n = write(fds[1], text.data() + done, text.size() - done);
            if (n <= 0) break;
            done += static_cast<std::size_t>(n);
        }
        close(fds[1]);
        _exit(0);
    }
    close(fds[1]);
    r.ran = true;
    char buf[65536];
    for (;;) {
        const ssize_t n = read(fds[0], buf, sizeof buf);
        if (n <= 0) break;
        r.text.append(buf, static_cast<std::size_t>(n));
    }
    close(fds[0]);
    waitpid(pid, &r.status, 0);
    r.signaled = WIFSIGNALED(r.status);
    r.sig = r.signaled ? WTERMSIG(r.status) : 0;
    return r;
}

ScenarioOutcome crashOutcome(const Forked& f, const char* where) {
    ScenarioOutcome o;
    std::string detail = f.signaled ? "the scenario killed its worker process with signal " + std::to_string(f.sig) +
                                          (f.sig == SIGSEGV ? " (SIGSEGV)" : f.sig == SIGALRM ? " (timeout)" : "")
                                    : "worker process exited abnormally (status " + std::to_string(f.status) + ")";
    o.failures.push_back({f.signaled && f.sig == SIGALRM ? "kernel_hang" : "kernel_crash", -1, detail + " " + where});
    return o;
}

} // namespace

ScenarioOutcome runScenarioIsolated(const Scenario& scenario, const GeneratorRegistry& registry, const ResolveFn& resolve) {
    const Forked f = forkRun([&](int) { return outcomeToJson(runScenario(scenario, registry, resolve)); });
    if (!f.ran) return runScenario(scenario, registry, resolve);
    if (!f.signaled && WIFEXITED(f.status) && WEXITSTATUS(f.status) == 0 && !f.text.empty()) return outcomeFromJson(f.text);
    return crashOutcome(f, "while running it");
}

/// Generates AND runs scenario `seed` in a worker (generation evaluates features too, so it can hit the same kernel crashes).
/// `generated` reports whether the worker got past generation.
static ScenarioOutcome runSeedIsolated(std::uint64_t seed, const GeneratorRegistry& registry, const ScenarioConfig& config,
                                       const ResolveFn& resolve, bool& generated) {
    generated = true;
    const Forked f = forkRun([&](int fd) {
        const Scenario sc = generateScenario(seed, registry, config);
        const char marker = 'G';
        if (write(fd, &marker, 1) != 1) return std::string();
        return outcomeToJson(runScenario(sc, registry, resolve));
    });
    if (!f.ran) return runScenario(generateScenario(seed, registry, config), registry, resolve);
    if (!f.text.empty() && f.text[0] == 'G') {
        if (!f.signaled && WIFEXITED(f.status) && WEXITSTATUS(f.status) == 0 && f.text.size() > 1) return outcomeFromJson(f.text.substr(1));
        return crashOutcome(f, "while running it");
    }
    generated = false;
    return crashOutcome(f, "while generating it");
}

std::string explainScenario(const Scenario& scenario, const GeneratorRegistry& registry, const ResolveFn& resolve) {
    Evaluator ev;
    Timeline tl;
    for (const auto& f : scenario.base) tl.append(f);
    std::string out = describe(scenario);
    auto dump = [&](const std::string& title) {
        out += "== " + title + "\n";
        auto full = ev.evaluate(tl);
        if (!full) { out += "  evaluation error: " + full.error().message + "\n"; return; }
        for (std::size_t p = 0; p < tl.size(); ++p) {
            const auto& f = tl.at(p);
            const auto& st = full->statuses[p];
            out += "  [" + std::to_string(p) + "] " + std::string(f->kind()) + " F" + std::to_string(f->id().value()) + " " +
                   (st.state == FeatureStatus::State::Ok ? "ok" : "NOT-OK " + st.error.message) + "\n";
            if (st.state != FeatureStatus::State::Ok) continue;
            auto in = evaluatePrefix(ev, tl, p);
            auto post = evaluatePrefix(ev, tl, p + 1);
            if (!in || !post) continue;
            const ModelState inState = ModelState::from(*in);
            if (f->kind() == "revolve") { // volume of the tool alone: a boolean must not use an inside-out tool
                auto rp = static_cast<const features::RevolveFeature&>(*f).params();
                if (rp.operation != features::common::BodyOp::NewBody) {
                    rp.operation = features::common::BodyOp::NewBody;
                    Timeline probe;
                    for (std::size_t k = 0; k < p; ++k) probe.append(tl.at(k));
                    probe.append(std::make_shared<features::RevolveFeature>(FeatureId{9999}, rp));
                    if (auto r = ev.evaluate(probe))
                        if (auto sh = r->bodies.find(rp.body); sh != r->bodies.end())
                            if (auto mp = kernel::massProperties(sh->second))
                                out += "      tool-alone volume " + std::to_string(mp->volume) + " mm3\n";
                }
            }
            for (BodyId b : f->writesBodies()) {
                auto sh = post->bodies.find(b);
                if (sh == post->bodies.end()) continue;
                if (auto mp = kernel::massProperties(sh->second))
                    out += "      body " + std::to_string(b.value()) + " volume " + std::to_string(mp->volume) + " mm3\n";
            }
            const FeatureGenerator* g = registry.find(f->kind());
            for (BodyId b : f->writesBodies()) {
                auto sh = post->bodies.find(b);
                auto nm = post->namings.find(b);
                if (sh == post->bodies.end() || nm == post->namings.end()) continue;
                for (ElementType t : kTypes)
                    for (std::uint32_t i = 0; i < sh->second.subShapeCount(naming::toSubShapeKind(t)); ++i) {
                        auto name = nm->second.nameOf(t, i);
                        if (!name) continue;
                        const GeomSig sig = readSig(sh->second, t, i);
                        std::string pred = "no-prediction";
                        if (g) {
                            const PredictContext pc{inState, resolve};
                            if (auto p2 = predictFor(*g, *f, *name, pc)) {
                                if (!usable(*p2)) pred = "vacuous";
                                else pred = anyMatch(*p2, sig) ? "match" : "MISMATCH " + candidatesText(*p2);
                            }
                        }
                        out += "      b" + std::to_string(b.value()) + " " + name->toString() + " @" + std::to_string(i) + " " +
                               describe(sig) + " -> " + pred + "\n";
                    }
            }
        }
    };
    dump("base");
    for (std::size_t k = 0; k < scenario.steps.size(); ++k) {
        if (auto r = applyStep(tl, scenario.steps[k]); !r) continue;
        dump("after step #" + std::to_string(k));
    }
    return out;
}

FuzzSummary runFuzz(const FuzzConfig& config, const GeneratorRegistry& registry, const ResolveFn& resolve) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - start).count(); };

    FuzzSummary summary;
    std::set<std::string> internal;
    const std::size_t total = config.replaySingle ? 1 : config.scenarios;
    for (std::size_t i = 0; i < total; ++i) {
        if (config.timeBudgetSeconds > 0 && elapsed() >= config.timeBudgetSeconds) {
            summary.timedOut = true;
            break;
        }
        const std::uint64_t seed = config.replaySingle ? config.replaySeed : deriveSeed(config.seed, i);
        if (config.trace) std::fprintf(stderr, "scenario %zu seed %llu\n", i, static_cast<unsigned long long>(seed));
        bool generated = true;
        ScenarioOutcome out;
        Scenario sc;
        if (config.isolate) {
            out = runSeedIsolated(seed, registry, config.scenario, resolve, generated);
            if (generated) sc = generateScenario(seed, registry, config.scenario); // safe: the worker got through it
        } else {
            sc = generateScenario(seed, registry, config.scenario);
            out = runScenario(sc, registry, resolve);
        }
        ++summary.scenariosRun;
        summary.counts += out.counts;
        for (const auto& f : out.failures) ++summary.failureCategories[f.category];
        for (const auto& m : out.internalErrors)
            if (internal.size() < 20) internal.insert(m);
        for (const auto& m : out.notes)
            if (summary.notes.size() < 20) summary.notes.push_back("seed " + std::to_string(seed) + ": " + m);

        if (out.failures.empty()) continue;
        ++summary.failingScenarios;
        const std::string category = out.failures.front().category;
        if (!generated) { // nothing to shrink: the kernel died while the scenario was being generated
            FailedScenario fs;
            fs.scenarioSeed = seed;
            fs.index = i;
            fs.category = category;
            fs.detail = out.failures.front().detail;
            fs.reproducer = "scenario seed=" + std::to_string(seed) + " (crashed during generation; no timeline to print)\n";
            summary.failures.push_back(std::move(fs));
            continue;
        }
        const Scenario shrunk = shrinkScenario(
            sc,
            [&](const Scenario& c) {
                const auto o = config.isolate ? runScenarioIsolated(c, registry, resolve) : runScenario(c, registry, resolve);
                return std::any_of(o.failures.begin(), o.failures.end(),
                                   [&](const Failure& f) { return f.category == category; });
            },
            config.shrinkRuns);

        FailedScenario fs;
        fs.scenarioSeed = seed;
        fs.index = i;
        fs.category = category;
        fs.detail = out.failures.front().detail;
        fs.originalSteps = sc.steps.size();
        fs.shrunkSteps = shrunk.steps.size();
        fs.reproducer = describe(shrunk);
        summary.failures.push_back(std::move(fs));
    }
    summary.internalErrors.assign(internal.begin(), internal.end());
    summary.elapsedSeconds = elapsed();
    return summary;
}

std::string toJson(const FuzzConfig& config, const FuzzSummary& s) {
    using nlohmann::json;
    json j;
    j["master_seed"] = config.seed;
    j["replay"] = config.replaySingle;
    j["scenarios_requested"] = config.replaySingle ? 1 : config.scenarios;
    j["scenarios_run"] = s.scenariosRun;
    j["max_features"] = config.scenario.maxFeatures;
    j["max_steps"] = config.scenario.maxSteps;
    j["time_budget_s"] = config.timeBudgetSeconds;
    j["elapsed_s"] = s.elapsedSeconds;
    j["timed_out"] = s.timedOut;
    j["passed"] = s.failingScenarios == 0;
    j["failing_scenarios"] = s.failingScenarios;
    j["failure_categories"] = s.failureCategories;

    const Counters& c = s.counts;
    const double res = static_cast<double>(std::max<std::size_t>(1, c.resolutions()));
    j["metrics"] = {
        {"resolutions", c.resolutions()},
        {"verified_pct", 100.0 * static_cast<double>(c.verified()) / res},
        {"unverified_pct", 100.0 * static_cast<double>(c.unverifiedMoved) / res},
        {"exact", c.resolvedExact},
        {"history_derived", c.resolvedHistory},
        {"ambiguous", c.ambiguous},
        {"not_found", c.notFound},
        {"oracle_failures", s.failureCategories.empty() ? 0u : [&] { std::size_t n = 0; for (const auto& kv : s.failureCategories) n += kv.second; return n; }()},
    };
    j["counters"] = {
        {"steps_checked", c.stepsChecked},
        {"refs_captured", c.refsCaptured},
        {"refs_checked", c.refsChecked},
        {"resolved_exact", c.resolvedExact},
        {"resolved_history_derived", c.resolvedHistory},
        {"diagnosed_not_found_or_ambiguous", c.diagnosed},
        {"predicted_checks", c.predictedChecks},
        {"unaffected_checks", c.unaffectedChecks},
        {"unverified_moved", c.unverifiedMoved},
        {"pre_state_checks", c.preStateChecks},
        {"volume_checks", c.volumeChecks},
        {"role_checks", c.roleChecks},
        {"advisory_role_changes_new_names", c.advisoryRoleChanges},
        {"features_detached_by_edit", c.detachedFeatures},
        {"f27_hole_ref_features", c.holeRefFeatures},
        {"f27_hole_ref_detached", c.holeRefDetached},
        {"legit_loss", c.legitLoss},
        {"legit_loss_hole_ref", c.legitLossHole},
        {"legit_loss_by_cause", c.legitLossByCause},
        {"stability_losses_warning", c.stabilityLosses},
        {"skipped_owner_gone", c.skippedOwnerGone},
        {"skipped_owner_inactive", c.skippedOwnerInactive},
        {"internal_feature_errors", c.internalFeatureErrors},
    };
    json kinds = json::object();
    for (const auto& [k, v] : c.perKind)
        kinds[k] = {{"resolutions", v.exact + v.history}, {"exact", v.exact}, {"history_derived", v.history},
                    {"diagnosed", v.diagnosed}, {"predicted", v.predicted}, {"unaffected_only", v.unaffected},
                    {"unverified", v.unverified}, {"failures", v.failures}};
    j["per_feature"] = std::move(kinds);
    j["unverified_by"] = c.unverifiedBy;

    json fails = json::array();
    for (const auto& f : s.failures) {
        fails.push_back({
            {"scenario_seed", f.scenarioSeed},
            {"index", f.index},
            {"category", f.category},
            {"detail", f.detail},
            {"steps_original", f.originalSteps},
            {"steps_minimal", f.shrunkSteps},
            {"reproducer", f.reproducer},
            {"replay_command", "ironwood-fuzz-naming --replay-seed " + std::to_string(f.scenarioSeed)},
        });
    }
    j["failures"] = std::move(fails);
    j["internal_errors"] = s.internalErrors;
    j["notes"] = s.notes;
    j["limitations"] = json::array({
        "Edges carry a supporting line only when straight; arcs/circles are checked by centroid reach.",
        "Cone reference radius is not part of the signature (not intrinsic); cones compare by axis line.",
        "Pieces of a split face on one surface are told apart only by the pre/post edit comparison, not by prediction.",
        "Elements a blend/boolean generates without any history (op '.new') are checked only where a predictor exists.",
    });
    return j.dump(2);
}

} // namespace iw::fuzz
