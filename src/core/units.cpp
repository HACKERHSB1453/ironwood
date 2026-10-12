// SPDX-License-Identifier: GPL-3.0-or-later
#include "core/units.hpp"

#include "core/result.hpp"

namespace iw {

std::string_view unitSymbol(LengthUnit u) noexcept {
    switch (u) {
    case LengthUnit::Millimeter: return "mm";
    case LengthUnit::Centimeter: return "cm";
    case LengthUnit::Meter: return "m";
    case LengthUnit::Inch: return "in";
    case LengthUnit::Foot: return "ft";
    }
    return "mm";
}

std::optional<LengthUnit> parseLengthUnit(std::string_view s) noexcept {
    if (s == "mm") return LengthUnit::Millimeter;
    if (s == "cm") return LengthUnit::Centimeter;
    if (s == "m") return LengthUnit::Meter;
    if (s == "in" || s == "\"") return LengthUnit::Inch;
    if (s == "ft" || s == "'") return LengthUnit::Foot;
    return std::nullopt;
}

const char* errorCodeName(ErrorCode code) noexcept {
    switch (code) {
    case ErrorCode::NotImplemented: return "not_implemented";
    case ErrorCode::InvalidArgument: return "invalid_argument";
    case ErrorCode::NotFound: return "not_found";
    case ErrorCode::IoError: return "io_error";
    case ErrorCode::ParseError: return "parse_error";
    case ErrorCode::Cancelled: return "cancelled";
    case ErrorCode::KernelFailure: return "kernel_failure";
    case ErrorCode::Ambiguous: return "ambiguous";
    case ErrorCode::Internal: return "internal";
    }
    return "internal";
}

} // namespace iw
