// SPDX-License-Identifier: GPL-3.0-or-later
#include "features/document.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include "features/extrude_feature.hpp"

namespace iw::features {

using recompute::Feature;
using recompute::FeatureStatus;

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
};

Document::Document() {
    registerFeatures(registry_);
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
    auto r = undo_.push(std::make_unique<Ops::AddFeature>(*this, std::move(feature), pos));
    if (r) notify();
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
