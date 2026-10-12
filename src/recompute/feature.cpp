// SPDX-License-Identifier: GPL-3.0-or-later
#include "recompute/feature.hpp"

namespace iw::recompute {

void FeatureRegistry::add(std::string kind, Factory factory) { factories_[std::move(kind)] = std::move(factory); }

Result<std::shared_ptr<Feature>> FeatureRegistry::create(std::string_view kind, FeatureId id,
                                                         std::string_view paramsJson) const {
    auto it = factories_.find(kind);
    if (it == factories_.end()) return makeError(ErrorCode::NotFound, "unknown feature kind: " + std::string(kind));
    return it->second(id, paramsJson);
}

} // namespace iw::recompute
