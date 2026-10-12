// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// JSON helpers for the feature parameter parsers. PRIVATE to iw_features (nlohmann is not a public dependency).

#include <optional>
#include <string>

#include <nlohmann/json.hpp>

#include "core/result.hpp"
#include "naming/element_name.hpp"

namespace iw::features::common {

Result<std::array<double, 3>> readVec(const nlohmann::json& obj, const char* key);
Result<std::string> readString(const nlohmann::json& obj, const char* key, std::optional<std::string> fallback = std::nullopt);
/// Element reference as {"feature": id, "type": "F"|"E"|"V", "name": ElementName::toString()}.
nlohmann::json refToJson(const naming::ElementRef& ref);
Result<naming::ElementRef> refFromJson(const nlohmann::json& obj, const char* key);
/// Parses an expression; ParseError naming `key` when it does not parse.
Result<void> checkExpression(const std::string& text, const char* key);

} // namespace iw::features::common
