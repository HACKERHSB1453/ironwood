// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <expected>
#include <string>
#include <string_view>
#include <utility>

namespace iw {

/// Machine-readable failure category. Stable across versions (may be serialised in diagnostics).
enum class ErrorCode {
    NotImplemented, ///< Stub / not yet written. Never returned by finished functionality.
    InvalidArgument,///< Caller passed an unusable value (null shape, empty path, bad range...).
    NotFound,       ///< Requested entity (file, element, feature) does not exist.
    IoError,        ///< Filesystem / stream failure.
    ParseError,     ///< Input data malformed (file format, serialised name...).
    Cancelled,      ///< Operation observed a cancelled CancelToken. Not a fault.
    KernelFailure,  ///< Geometry kernel reported failure (boolean failed, bad topology...).
    Ambiguous,      ///< Reference resolves to more than one candidate; caller must disambiguate.
    Internal,       ///< Bug or unexpected exception caught at an API boundary.
};

/// Returns a stable lower-case name for `code` (e.g. "not_implemented").
const char* errorCodeName(ErrorCode code) noexcept;

/// A failure description. Value type; thread-safe to copy. Errors are for *failures*; recoverable
/// oddities (e.g. a heuristically rebound reference) are reported as warnings, not Errors.
struct Error {
    ErrorCode code = ErrorCode::Internal;
    std::string message;

    friend bool operator==(const Error&, const Error&) = default;
};

/// Result of a fallible operation: a value or an `Error`. API functions never throw for expected
/// failures; they return an unexpected `Error`. Exceptions escaping an API boundary are a bug.
template <class T> using Result = std::expected<T, Error>;

/// Builds an `std::unexpected<Error>` for `return makeError(...)`.
inline std::unexpected<Error> makeError(ErrorCode code, std::string message) {
    return std::unexpected<Error>(Error{code, std::move(message)});
}

/// Shorthand for the stub convention: `return notImplemented("kernel::makeBox");`
inline std::unexpected<Error> notImplemented(std::string_view what) {
    return makeError(ErrorCode::NotImplemented, std::string(what) + " is not implemented");
}

} // namespace iw
