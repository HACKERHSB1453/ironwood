// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <numbers>
#include <optional>
#include <string_view>

namespace iw {

/// Display/input length units. Internally ALL lengths are millimetres (`double`); angles are radians.
/// Conversion happens only at the UI/IO boundary. Thread-safe (pure functions).
enum class LengthUnit { Millimeter, Centimeter, Meter, Inch, Foot };

/// Millimetres per one `u`.
constexpr double mmPerUnit(LengthUnit u) noexcept {
    switch (u) {
    case LengthUnit::Millimeter: return 1.0;
    case LengthUnit::Centimeter: return 10.0;
    case LengthUnit::Meter: return 1000.0;
    case LengthUnit::Inch: return 25.4;
    case LengthUnit::Foot: return 304.8;
    }
    return 1.0;
}

/// Converts a value expressed in `u` to internal millimetres.
constexpr double toMillimeters(double value, LengthUnit u) noexcept { return value * mmPerUnit(u); }
/// Converts internal millimetres to `u`.
constexpr double fromMillimeters(double mm, LengthUnit u) noexcept { return mm / mmPerUnit(u); }
/// Converts between two display units.
constexpr double convertLength(double value, LengthUnit from, LengthUnit to) noexcept {
    return fromMillimeters(toMillimeters(value, from), to);
}

constexpr double degreesToRadians(double deg) noexcept { return deg * std::numbers::pi / 180.0; }
constexpr double radiansToDegrees(double rad) noexcept { return rad * 180.0 / std::numbers::pi; }

/// Short symbol: "mm", "cm", "m", "in", "ft".
std::string_view unitSymbol(LengthUnit u) noexcept;
/// Parses a symbol ("mm", "cm", "m", "in", "ft", also "\""); nullopt if unknown. Case-sensitive.
std::optional<LengthUnit> parseLengthUnit(std::string_view symbol) noexcept;

} // namespace iw
