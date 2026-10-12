// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "core/ids.hpp"
#include "recompute/feature.hpp"

namespace iw::recompute {

/// Ordered list of features with a rollback marker and per-feature suppression. Not thread-safe; the
/// evaluator takes a snapshot (copy; features are shared immutable pointers) before going async.
class Timeline {
public:
    std::size_t size() const noexcept { return features_.size(); }
    const std::shared_ptr<const Feature>& at(std::size_t i) const { return features_.at(i); }

    /// Inserts at `position` (<= size()). If the marker was at the end it stays at the end.
    void insert(std::size_t position, std::shared_ptr<const Feature> feature);
    void append(std::shared_ptr<const Feature> feature) { insert(features_.size(), std::move(feature)); }
    /// Replaces the feature with the same id (an edit); NotFound if absent.
    Result<void> replace(std::shared_ptr<const Feature> feature);
    /// Removes by id; NotFound if absent.
    Result<void> remove(FeatureId id);
    /// Index of `id`, or nullopt.
    std::optional<std::size_t> indexOf(FeatureId id) const;

    /// Rollback marker: number of leading features that are "active" (0..size()). Features at or after it
    /// are not evaluated. Default: end.
    std::size_t rollbackMarker() const noexcept { return marker_ ? std::min(*marker_, features_.size()) : features_.size(); }
    void setRollbackMarker(std::size_t activeCount) { marker_ = activeCount; }
    void clearRollbackMarker() { marker_.reset(); }

    void setSuppressed(FeatureId id, bool suppressed);
    bool isSuppressed(FeatureId id) const { return suppressed_.contains(id); }

private:
    std::vector<std::shared_ptr<const Feature>> features_;
    std::optional<std::size_t> marker_;
    std::set<FeatureId> suppressed_;
};

} // namespace iw::recompute
