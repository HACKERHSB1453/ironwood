// SPDX-License-Identifier: GPL-3.0-or-later
#include "recompute/timeline.hpp"

#include <algorithm>

namespace iw::recompute {

void Timeline::insert(std::size_t position, std::shared_ptr<const Feature> feature) {
    position = std::min(position, features_.size());
    const bool atEnd = !marker_ || *marker_ >= features_.size();
    features_.insert(features_.begin() + static_cast<std::ptrdiff_t>(position), std::move(feature));
    if (atEnd) marker_.reset();
    else if (position < *marker_) ++*marker_;
}

std::optional<std::size_t> Timeline::indexOf(FeatureId id) const {
    for (std::size_t i = 0; i < features_.size(); ++i)
        if (features_[i]->id() == id) return i;
    return std::nullopt;
}

Result<void> Timeline::replace(std::shared_ptr<const Feature> feature) {
    auto i = indexOf(feature->id());
    if (!i) return makeError(ErrorCode::NotFound, "feature not in timeline");
    features_[*i] = std::move(feature);
    return {};
}

Result<void> Timeline::remove(FeatureId id) {
    auto i = indexOf(id);
    if (!i) return makeError(ErrorCode::NotFound, "feature not in timeline");
    features_.erase(features_.begin() + static_cast<std::ptrdiff_t>(*i));
    suppressed_.erase(id);
    if (marker_ && *i < *marker_) --*marker_;
    return {};
}

void Timeline::setSuppressed(FeatureId id, bool s) {
    if (s) suppressed_.insert(id);
    else suppressed_.erase(id);
}

} // namespace iw::recompute
