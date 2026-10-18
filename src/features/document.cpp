// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/document.hpp"

#include <algorithm>
#include <cctype>
#include <functional>
#include <memory>
#include <string_view>
#include <utility>

#include <nlohmann/json.hpp>

#include "core/expr.hpp"
#include "features/extrude_feature.hpp"
#include "features/feature_common.hpp"

namespace iw::features {

using recompute::Feature;
using recompute::FeatureStatus;
using json = nlohmann::json;

namespace {

/// JSON keys whose string values are expressions (or arrays of them) in the feature and sketch serialisations. A
/// rename rewrites only these, so names, kinds and references are never touched.
bool isExpressionKey(std::string_view key) {
    static constexpr std::string_view kKeys[] = {
        "distance", "distance2", "taper", "angle", "angle2", "radius", "u", "v", "diameter", "depth", "drill_angle",
        "cb_depth", "cb_diameter", "cs_angle", "cs_diameter", "expression", "translate", "points", "a", "b"};
    return std::find(std::begin(kKeys), std::end(kKeys), key) != std::end(kKeys);
}

/// Rewrites every expression string under `node` with `fn`. `changed` is set when any string differs.
Result<void> rewriteExpressions(json& node, bool underExpressionKey, bool& changed,
                                const std::function<Result<std::string>(const std::string&)>& fn) {
    if (node.is_string()) {
        if (!underExpressionKey) return {};
        const std::string text = node.get<std::string>();
        auto next = fn(text);
        if (!next) return std::unexpected(next.error());
        if (*next != text) {
            node = *next;
            changed = true;
        }
        return {};
    }
    if (node.is_array()) {
        for (json& v : node)
            if (auto r = rewriteExpressions(v, underExpressionKey, changed, fn); !r) return r;
    } else if (node.is_object()) {
        for (auto it = node.begin(); it != node.end(); ++it) {
            const bool expr = underExpressionKey || isExpressionKey(it.key());
            if (auto r = rewriteExpressions(it.value(), expr, changed, fn); !r) return r;
        }
    }
    return {};
}

/// Default display label for a feature kind ("extrude" -> "Extrude").
std::string labelOf(std::string_view kind) {
    static const std::pair<std::string_view, std::string_view> kLabels[] = {
        {"extrude", "Extrude"}, {"revolve", "Revolve"}, {"fillet", "Fillet"}, {"chamfer", "Chamfer"}, {"hole", "Hole"},
        {"move", "Move"}, {"sketch", "Sketch"}, {"plane", "Plane"}, {"axis", "Axis"}, {"point", "Point"}};
    for (const auto& [k, label] : kLabels)
        if (k == kind) return std::string(label);
    std::string out(kind);
    if (!out.empty()) out[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(out[0])));
    return out;
}

/// The n of a name of the form "<label> <n>" (n decimal, no sign), or 0.
std::uint64_t numberOfDefaultName(const std::string& name, const std::string& label) {
    const std::string prefix = label + " ";
    if (name.size() <= prefix.size() || name.compare(0, prefix.size(), prefix) != 0) return 0;
    std::uint64_t n = 0;
    for (std::size_t i = prefix.size(); i < name.size(); ++i) {
        const char c = name[i];
        if (c < '0' || c > '9' || n > (std::uint64_t{1} << 50)) return 0;
        n = n * 10 + static_cast<std::uint64_t>(c - '0');
    }
    return n;
}

} // namespace

/// Undoable commands. They are nested in Ops so they can reach the private state of Document (Ops is a friend).
/// Each command captures what it needs on execute(), so redo re-captures a correct "before" state.
struct Document::Ops {
    class AddFeature final : public Command {
    public:
        AddFeature(Document& d, std::shared_ptr<const Feature> f, std::size_t pos) : d_(d), f_(std::move(f)), pos_(pos) {}
        Result<void> execute() override {
            auto& tl = d_.data_.timeline;
            if (!f_ || !f_->id().valid()) return makeError(ErrorCode::InvalidArgument, "feature has no valid id");
            if (tl.indexOf(f_->id())) return makeError(ErrorCode::InvalidArgument, "feature id already on the timeline");
            if (pos_ > tl.size()) return makeError(ErrorCode::InvalidArgument, "feature position out of range");
            tl.insert(pos_, f_);
            if (f_->id().value() >= d_.data_.nextId) d_.data_.nextId = f_->id().value() + 1;
            return {};
        }
        void undo() override { (void)d_.data_.timeline.remove(f_->id()); }
        std::string text() const override { return "Add Feature"; }

    private:
        Document& d_;
        std::shared_ptr<const Feature> f_;
        std::size_t pos_;
    };

    class EditFeature final : public Command {
    public:
        EditFeature(Document& d, std::shared_ptr<const Feature> f) : d_(d), f_(std::move(f)) {}
        Result<void> execute() override {
            auto& tl = d_.data_.timeline;
            auto idx = tl.indexOf(f_->id());
            if (!idx) return makeError(ErrorCode::NotFound, "feature not on the timeline");
            old_ = tl.at(*idx);
            return tl.replace(f_);
        }
        void undo() override { (void)d_.data_.timeline.replace(old_); }
        std::string text() const override { return "Edit Feature"; }

    private:
        Document& d_;
        std::shared_ptr<const Feature> f_;
        std::shared_ptr<const Feature> old_;
    };

    class SetSuppressed final : public Command {
    public:
        SetSuppressed(Document& d, FeatureId id, bool value) : d_(d), id_(id), value_(value) {}
        Result<void> execute() override {
            if (!d_.data_.timeline.indexOf(id_)) return makeError(ErrorCode::NotFound, "feature not on the timeline");
            prev_ = d_.data_.timeline.isSuppressed(id_);
            d_.data_.timeline.setSuppressed(id_, value_);
            return {};
        }
        void undo() override { d_.data_.timeline.setSuppressed(id_, prev_); }
        std::string text() const override { return value_ ? "Suppress Feature" : "Unsuppress Feature"; }

    private:
        Document& d_;
        FeatureId id_;
        bool value_;
        bool prev_ = false;
    };

    class DeleteFeature final : public Command {
    public:
        DeleteFeature(Document& d, FeatureId id) : d_(d), id_(id) {}
        Result<void> execute() override {
            auto& tl = d_.data_.timeline;
            auto idx = tl.indexOf(id_);
            if (!idx) return makeError(ErrorCode::NotFound, "feature not on the timeline");
            feature_ = tl.at(*idx);
            index_ = *idx;
            wasSuppressed_ = tl.isSuppressed(id_);
            tl.setSuppressed(id_, false);
            return tl.remove(id_);
        }
        void undo() override {
            auto& tl = d_.data_.timeline;
            tl.insert(index_, feature_);
            tl.setSuppressed(id_, wasSuppressed_);
        }
        std::string text() const override { return "Delete Feature"; }

    private:
        Document& d_;
        FeatureId id_;
        std::shared_ptr<const Feature> feature_;
        std::size_t index_ = 0;
        bool wasSuppressed_ = false;
    };

    class SetMarker final : public Command {
    public:
        SetMarker(Document& d, std::optional<std::size_t> value) : d_(d), value_(value) {}
        Result<void> execute() override {
            auto& tl = d_.data_.timeline;
            if (value_ && *value_ > tl.size()) return makeError(ErrorCode::InvalidArgument, "rollback marker out of range");
            prev_ = tl.rollbackMarker();
            if (value_) tl.setRollbackMarker(*value_);
            else tl.clearRollbackMarker();
            return {};
        }
        void undo() override {
            auto& tl = d_.data_.timeline;
            if (prev_ >= tl.size()) tl.clearRollbackMarker();
            else tl.setRollbackMarker(prev_);
        }
        std::string text() const override { return "Move Rollback Marker"; }

    private:
        Document& d_;
        std::optional<std::size_t> value_;
        std::size_t prev_ = 0;
    };

    class AddParameter final : public Command {
    public:
        AddParameter(Document& d, std::string name, std::string expr, std::string unit, std::string comment)
            : d_(d), name_(std::move(name)), expr_(std::move(expr)), unit_(std::move(unit)), comment_(std::move(comment)) {}
        Result<void> execute() override { return d_.data_.parameters.add(name_, expr_, unit_, comment_); }
        void undo() override { (void)d_.data_.parameters.remove(name_); }
        std::string text() const override { return "Add Parameter"; }

    private:
        Document& d_;
        std::string name_, expr_, unit_, comment_;
    };

    class SetParameter final : public Command {
    public:
        SetParameter(Document& d, std::string name, std::string expr) : d_(d), name_(std::move(name)), expr_(std::move(expr)) {}
        Result<void> execute() override {
            const Parameter* p = d_.data_.parameters.find(name_);
            if (!p) return makeError(ErrorCode::NotFound, "unknown parameter '" + name_ + "'");
            prev_ = p->expression.source();
            return d_.data_.parameters.setExpression(name_, expr_);
        }
        void undo() override { (void)d_.data_.parameters.setExpression(name_, prev_); }
        std::string text() const override { return "Edit Parameter"; }

    private:
        Document& d_;
        std::string name_, expr_, prev_;
    };

    /// One undoable change to the parameter table and/or a set of features (replaced by id). Used by every parameter
    /// and feature-rewrite command, so an edit that touches both is a single undo step. The "before" state is captured
    /// on execute(), so redo captures a correct one again.
    class ApplyChange final : public Command {
    public:
        ApplyChange(Document& d, std::optional<ParameterTable> params, std::vector<std::shared_ptr<const Feature>> features,
                    std::string text)
            : d_(d), params_(std::move(params)), features_(std::move(features)), text_(std::move(text)) {}
        Result<void> execute() override {
            auto& tl = d_.data_.timeline;
            for (const auto& f : features_)
                if (!tl.indexOf(f->id())) return makeError(ErrorCode::NotFound, "feature not on the timeline");
            beforeFeatures_.clear();
            for (const auto& f : features_) beforeFeatures_.push_back(tl.at(*tl.indexOf(f->id())));
            if (params_) {
                beforeParams_ = d_.data_.parameters;
                d_.data_.parameters = *params_;
            }
            for (const auto& f : features_)
                if (auto r = tl.replace(f); !r) return r;
            return {};
        }
        void undo() override {
            auto& tl = d_.data_.timeline;
            for (const auto& f : beforeFeatures_) (void)tl.replace(f);
            if (params_) d_.data_.parameters = beforeParams_;
        }
        std::string text() const override { return text_; }

    private:
        Document& d_;
        std::optional<ParameterTable> params_;
        std::vector<std::shared_ptr<const Feature>> features_;
        std::string text_;
        std::vector<std::shared_ptr<const Feature>> beforeFeatures_;
        ParameterTable beforeParams_;
    };
};

Document::Document() {
    registerFeatures(registry_);
}

Result<std::shared_ptr<const Feature>> Document::withName(const Feature& f, std::string name) const {
    auto made = registry_.create(std::string(f.kind()), f.id(), f.paramsJson());
    if (!made) return std::unexpected(made.error());
    (*made)->setName(std::move(name));
    return std::shared_ptr<const Feature>(std::move(*made));
}

Result<std::shared_ptr<const Feature>> Document::renamedReferences(const Feature& f, const std::string& from,
                                                                   const std::string& to) const {
    json j = json::parse(f.paramsJson(), nullptr, false);
    if (j.is_discarded() || !j.is_object()) return makeError(ErrorCode::Internal, "feature params are not a JSON object");
    bool changed = false;
    auto rewrite = [&](const std::string& text) -> Result<std::string> {
        const auto refs = common::referencesOf(text);
        if (std::find(refs.begin(), refs.end(), from) == refs.end()) return text;
        return renameIdentifier(text, from, to);
    };
    if (auto r = rewriteExpressions(j, false, changed, rewrite); !r) return std::unexpected(r.error());
    if (!changed) return std::shared_ptr<const Feature>{};
    auto made = registry_.create(std::string(f.kind()), f.id(), j.dump());
    if (!made) return std::unexpected(made.error());
    (*made)->setName(f.name());
    return std::shared_ptr<const Feature>(std::move(*made));
}

std::shared_ptr<const Feature> Document::featureById(FeatureId id) const {
    auto idx = data_.timeline.indexOf(id);
    return idx ? data_.timeline.at(*idx) : nullptr;
}

std::vector<std::string> Document::featureUsers(const std::string& name) const {
    std::vector<std::string> out;
    for (std::size_t k = 0; k < data_.timeline.size(); ++k) {
        const auto& f = data_.timeline.at(k);
        const auto refs = f->parameterReferences();
        if (std::find(refs.begin(), refs.end(), name) == refs.end()) continue;
        out.push_back(f->name().empty() ? std::string(f->kind()) + " " + std::to_string(f->id().value()) : f->name());
    }
    return out;
}

void Document::completeNames() {
    auto& tl = data_.timeline;
    // Raise every counter past the numbers already in use, so a loaded file never hands out a duplicate name.
    for (std::size_t k = 0; k < tl.size(); ++k) {
        const auto& f = tl.at(k);
        const std::string label = labelOf(f->kind());
        const std::uint64_t n = numberOfDefaultName(f->name(), label);
        auto& counter = data_.nameCounters[std::string(f->kind())];
        counter = std::max(counter, n);
    }
    for (std::size_t k = 0; k < tl.size(); ++k) {
        const auto f = tl.at(k);
        if (!f->name().empty()) continue;
        auto& counter = data_.nameCounters[std::string(f->kind())];
        auto named = withName(*f, labelOf(f->kind()) + " " + std::to_string(counter + 1));
        if (!named) continue; // unreachable for registered kinds; the feature keeps its (empty) name
        ++counter;
        (void)tl.replace(*named);
    }
}

Document::~Document() = default;

void Document::setName(std::string name) {
    data_.name = std::move(name);
    notify();
}

FeatureId Document::allocateFeatureId() { return FeatureId{data_.nextId++}; }
BodyId Document::allocateBodyId() { return BodyId{data_.nextId++}; }

Result<void> Document::addFeature(std::shared_ptr<const Feature> feature, std::optional<std::size_t> position) {
    if (!feature) return makeError(ErrorCode::InvalidArgument, "null feature");
    const std::size_t pos = position.value_or(data_.timeline.size());
    // An unnamed feature gets the next default name of its kind. The counter advances only if the add succeeds.
    const std::string kind(feature->kind());
    const bool needsName = feature->name().empty();
    if (needsName) {
        auto named = withName(*feature, labelOf(kind) + " " + std::to_string(data_.nameCounters[kind] + 1));
        if (!named) return std::unexpected(named.error());
        feature = std::move(*named);
    }
    auto r = undo_.push(std::make_unique<Ops::AddFeature>(*this, std::move(feature), pos));
    if (r) {
        if (needsName) ++data_.nameCounters[kind];
        notify();
    }
    return r;
}

Result<void> Document::editFeature(std::shared_ptr<const Feature> feature) {
    if (!feature) return makeError(ErrorCode::InvalidArgument, "null feature");
    auto r = undo_.push(std::make_unique<Ops::EditFeature>(*this, std::move(feature)));
    if (r) notify();
    return r;
}

Result<void> Document::setSuppressed(FeatureId id, bool suppressed) {
    auto r = undo_.push(std::make_unique<Ops::SetSuppressed>(*this, id, suppressed));
    if (r) notify();
    return r;
}

Result<void> Document::deleteFeature(FeatureId id) {
    auto r = undo_.push(std::make_unique<Ops::DeleteFeature>(*this, id));
    if (r) notify();
    return r;
}

Result<void> Document::setRollbackMarker(std::optional<std::size_t> activeCount) {
    auto r = undo_.push(std::make_unique<Ops::SetMarker>(*this, activeCount));
    if (r) notify();
    return r;
}

Result<void> Document::addParameter(std::string name, std::string expression, std::string unit, std::string comment) {
    auto r = undo_.push(std::make_unique<Ops::AddParameter>(*this, std::move(name), std::move(expression), std::move(unit), std::move(comment)));
    if (r) notify();
    return r;
}

Result<void> Document::setParameterExpression(const std::string& name, std::string expression) {
    auto r = undo_.push(std::make_unique<Ops::SetParameter>(*this, name, std::move(expression)));
    if (r) notify();
    return r;
}

Result<void> Document::renameFeature(FeatureId id, std::string name) {
    if (name.find_first_not_of(" \t") == std::string::npos) return makeError(ErrorCode::InvalidArgument, "feature name is empty");
    auto f = featureById(id);
    if (!f) return makeError(ErrorCode::NotFound, "feature not on the timeline");
    auto named = withName(*f, std::move(name));
    if (!named) return std::unexpected(named.error());
    auto r = undo_.push(std::make_unique<Ops::ApplyChange>(*this, std::nullopt, std::vector{*named}, "Rename Feature"));
    if (r) notify();
    return r;
}

Result<void> Document::setParameterUnit(const std::string& name, std::string unit) {
    ParameterTable next = data_.parameters;
    if (auto r = next.setUnit(name, std::move(unit)); !r) return std::unexpected(r.error());
    auto r = undo_.push(std::make_unique<Ops::ApplyChange>(*this, std::move(next), std::vector<std::shared_ptr<const Feature>>{}, "Edit Parameter Unit"));
    if (r) notify();
    return r;
}

Result<void> Document::setParameterComment(const std::string& name, std::string comment) {
    ParameterTable next = data_.parameters;
    if (auto r = next.setComment(name, std::move(comment)); !r) return std::unexpected(r.error());
    auto r = undo_.push(std::make_unique<Ops::ApplyChange>(*this, std::move(next), std::vector<std::shared_ptr<const Feature>>{}, "Edit Parameter Comment"));
    if (r) notify();
    return r;
}

Result<void> Document::setParameterFavourite(const std::string& name, bool favourite) {
    ParameterTable next = data_.parameters;
    if (auto r = next.setFavourite(name, favourite); !r) return std::unexpected(r.error());
    auto r = undo_.push(std::make_unique<Ops::ApplyChange>(*this, std::move(next), std::vector<std::shared_ptr<const Feature>>{}, favourite ? "Favourite Parameter" : "Unfavourite Parameter"));
    if (r) notify();
    return r;
}

Result<void> Document::renameParameter(const std::string& from, const std::string& to) {
    if (from == to) {
        if (!data_.parameters.find(from)) return makeError(ErrorCode::NotFound, "unknown parameter '" + from + "'");
        return {};
    }
    ParameterTable next = data_.parameters;
    if (auto r = next.rename(from, to); !r) return std::unexpected(r.error());
    // Feature parameters and sketch dimensions that reference `from` are rewritten in the same step.
    std::vector<std::shared_ptr<const Feature>> changed;
    for (std::size_t k = 0; k < data_.timeline.size(); ++k) {
        const auto& f = data_.timeline.at(k);
        const auto refs = f->parameterReferences();
        if (std::find(refs.begin(), refs.end(), from) == refs.end()) continue;
        auto nf = renamedReferences(*f, from, to);
        if (!nf) return std::unexpected(nf.error());
        if (*nf) changed.push_back(std::move(*nf));
    }
    auto r = undo_.push(std::make_unique<Ops::ApplyChange>(*this, std::move(next), std::move(changed), "Rename Parameter"));
    if (r) notify();
    return r;
}

Result<void> Document::removeParameter(const std::string& name) {
    if (!data_.parameters.find(name)) return makeError(ErrorCode::NotFound, "unknown parameter '" + name + "'");
    std::vector<std::string> users = data_.parameters.dependents(name);
    for (std::string& u : users) u = "parameter " + u;
    for (std::string& u : featureUsers(name)) users.push_back(u);
    if (!users.empty()) {
        std::string list;
        for (const std::string& u : users) list += (list.empty() ? "" : ", ") + u;
        return makeError(ErrorCode::InvalidArgument, "parameter '" + name + "' is used by: " + list);
    }
    ParameterTable next = data_.parameters;
    if (auto r = next.remove(name); !r) return std::unexpected(r.error());
    auto r = undo_.push(std::make_unique<Ops::ApplyChange>(*this, std::move(next), std::vector<std::shared_ptr<const Feature>>{}, "Delete Parameter"));
    if (r) notify();
    return r;
}

Result<void> Document::replaceAllParameters(ParameterTable table) {
    auto r = undo_.push(std::make_unique<Ops::ApplyChange>(*this, std::move(table), std::vector<std::shared_ptr<const Feature>>{}, "Edit Parameters"));
    if (r) notify();
    return r;
}

Result<void> Document::editConstructionPlane(FeatureId id, ConstructionPlaneParams params) {
    auto old = featureById(id);
    if (!old) return makeError(ErrorCode::NotFound, "feature not on the timeline");
    if (!dynamic_cast<const ConstructionPlaneFeature*>(old.get())) return makeError(ErrorCode::InvalidArgument, "feature is not a plane");
    auto nf = std::make_shared<ConstructionPlaneFeature>(id, std::move(params));
    nf->setName(old->name());
    auto r = undo_.push(std::make_unique<Ops::ApplyChange>(*this, std::nullopt, std::vector<std::shared_ptr<const Feature>>{nf}, "Edit Plane"));
    if (r) notify();
    return r;
}

Result<void> Document::editConstructionAxis(FeatureId id, ConstructionAxisParams params) {
    auto old = featureById(id);
    if (!old) return makeError(ErrorCode::NotFound, "feature not on the timeline");
    if (!dynamic_cast<const ConstructionAxisFeature*>(old.get())) return makeError(ErrorCode::InvalidArgument, "feature is not an axis");
    auto nf = std::make_shared<ConstructionAxisFeature>(id, std::move(params));
    nf->setName(old->name());
    auto r = undo_.push(std::make_unique<Ops::ApplyChange>(*this, std::nullopt, std::vector<std::shared_ptr<const Feature>>{nf}, "Edit Axis"));
    if (r) notify();
    return r;
}

Result<void> Document::editConstructionPoint(FeatureId id, ConstructionPointParams params) {
    auto old = featureById(id);
    if (!old) return makeError(ErrorCode::NotFound, "feature not on the timeline");
    if (!dynamic_cast<const ConstructionPointFeature*>(old.get())) return makeError(ErrorCode::InvalidArgument, "feature is not a point");
    auto nf = std::make_shared<ConstructionPointFeature>(id, std::move(params));
    nf->setName(old->name());
    auto r = undo_.push(std::make_unique<Ops::ApplyChange>(*this, std::nullopt, std::vector<std::shared_ptr<const Feature>>{nf}, "Edit Point"));
    if (r) notify();
    return r;
}

Result<sketch::Sketch> Document::sketchForEdit(FeatureId id) const {
    auto f = featureById(id);
    if (!f) return makeError(ErrorCode::NotFound, "feature not on the timeline");
    const auto* sf = dynamic_cast<const SketchFeature*>(f.get());
    if (!sf) return makeError(ErrorCode::InvalidArgument, "feature is not a sketch");
    return sf->params().sketch;
}

Result<void> Document::replaceSketch(FeatureId id, sketch::Sketch sk) {
    auto f = featureById(id);
    if (!f) return makeError(ErrorCode::NotFound, "feature not on the timeline");
    const auto* sf = dynamic_cast<const SketchFeature*>(f.get());
    if (!sf) return makeError(ErrorCode::InvalidArgument, "feature is not a sketch");
    SketchFeatureParams p = sf->params();
    p.sketch = std::move(sk);
    auto nf = std::make_shared<SketchFeature>(id, std::move(p));
    nf->setName(f->name());
    auto r = undo_.push(std::make_unique<Ops::ApplyChange>(*this, std::nullopt, std::vector<std::shared_ptr<const Feature>>{nf}, "Edit Sketch"));
    if (r) notify();
    return r;
}

Result<void> Document::undo() {
    auto r = undo_.undo();
    if (r) notify();
    return r;
}

Result<void> Document::redo() {
    auto r = undo_.redo();
    if (r) notify();
    return r;
}

Result<DocumentEvaluation> Document::evaluate(CancelToken cancel) {
    const recompute::Timeline& tl = data_.timeline;
    const std::size_t marker = tl.rollbackMarker();
    DocumentEvaluation out;

    // Pass 1: build the timeline the evaluator sees. A sketch feature is solved against the state before it; features
    // that reference a sketch get an embedded copy of the solved sketch. Positions are processed in order, so a
    // reference can only see an earlier sketch.
    recompute::Timeline resolved;
    for (std::size_t k = 0; k < tl.size(); ++k) {
        const std::shared_ptr<const Feature>& f = tl.at(k);
        const bool suppressed = tl.isSuppressed(f->id());
        const bool active = k < marker && !suppressed;

        if (active) {
            if (const auto* sf = dynamic_cast<const SketchFeature*>(f.get())) {
                auto before = evaluator_.evaluate(resolved, &data_.parameters, cancel);
                if (!before) return std::unexpected(before.error());
                std::map<FeatureId, recompute::ConstructionGeometry> cons;
                for (const auto& s : before->statuses)
                    if (s.state == FeatureStatus::State::Ok && s.construction) cons.emplace(s.id, *s.construction);
                recompute::EvalContext ctx{before->bodies, before->namings, cancel, &data_.parameters, &cons};
                auto sol = sf->solve(ctx);
                if (!sol) {
                    if (sol.error().code == ErrorCode::Cancelled) return std::unexpected(sol.error());
                    // A failed solve is reported by the feature's own status; dependents then see no profile.
                } else {
                    out.sketches.insert_or_assign(f->id(), std::move(*sol));
                }
            }
        }

        std::shared_ptr<const Feature> entry = f;
        if (active) {
            if (auto ref = sketchReferenceOf(*f)) {
                auto it = out.sketches.find(*ref);
                if (it != out.sketches.end())
                    if (auto emb = embedSketchReference(*f, it->second.sketch)) entry = std::move(emb);
            }
        }
        resolved.append(std::move(entry));
        if (suppressed) resolved.setSuppressed(f->id(), true);
    }
    if (marker < tl.size()) resolved.setRollbackMarker(marker);

    // Pass 2: evaluate the resolved timeline (prefix reuse through the evaluator's cache).
    auto res = evaluator_.evaluate(resolved, &data_.parameters, cancel);
    if (!res) return std::unexpected(res.error());
    out.statuses = std::move(res->statuses);
    out.bodies = std::move(res->bodies);
    out.namings = std::move(res->namings);
    out.computedCount = res->computedCount;
    out.reusedCount = res->reusedCount;
    for (const FeatureStatus& s : out.statuses)
        if (s.state == FeatureStatus::State::Ok && s.construction) out.construction.emplace(s.id, *s.construction);

    // Features after the rollback marker are not evaluated; still report one status per feature.
    for (std::size_t k = marker; k < tl.size(); ++k) {
        FeatureStatus s;
        s.id = tl.at(k)->id();
        const bool suppressed = tl.isSuppressed(s.id);
        s.state = suppressed ? FeatureStatus::State::Suppressed : FeatureStatus::State::Skipped;
        if (!suppressed)
            s.diagnostics.push_back({recompute::Diagnostic::Severity::Info, "not evaluated: after the rollback marker", std::nullopt});
        out.statuses.push_back(std::move(s));
    }
    return out;
}

Result<void> Document::save(const std::filesystem::path& path) {
    auto r = io::saveDocument(path, data_);
    if (r) undo_.setClean();
    return r;
}

Result<void> Document::load(const std::filesystem::path& path) {
    auto loaded = io::loadDocument(path, registry_);
    if (!loaded) return std::unexpected(loaded.error());
    data_ = std::move(*loaded);
    undo_.clear();
    undo_.setClean();
    evaluator_.clearCache();
    completeNames(); // files before format v2 have no feature names
    notify();
    return {};
}

Document::ListenerId Document::addListener(Listener listener) {
    const ListenerId id = nextListener_++;
    listeners_.emplace(id, std::move(listener));
    return id;
}

void Document::removeListener(ListenerId id) { listeners_.erase(id); }

void Document::notify() {
    // Copy first: a listener may add or remove listeners.
    auto copy = listeners_;
    for (auto& [id, fn] : copy)
        if (fn) fn();
}

} // namespace iw::features
