// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <compare>
#include <cstdint>
#include <functional>

namespace iw {

/// Strongly typed 64-bit identifier. Distinct `Tag`s do not convert into each other.
/// Value 0 is reserved as "invalid / unset". IDs are allocated by the owning document (never reused
/// within a document) and are persisted in the native file format. Trivially copyable, thread-safe.
template <class Tag> class StrongId {
public:
    using ValueType = std::uint64_t;

    constexpr StrongId() noexcept = default;
    constexpr explicit StrongId(ValueType v) noexcept : value_(v) {}

    constexpr ValueType value() const noexcept { return value_; }
    constexpr bool valid() const noexcept { return value_ != 0; }
    constexpr explicit operator bool() const noexcept { return valid(); }

    friend constexpr auto operator<=>(StrongId, StrongId) noexcept = default;

private:
    ValueType value_ = 0;
};

struct FeatureIdTag;
struct BodyIdTag;
struct ComponentIdTag;

/// Identifies a timeline feature. Stable for the feature's lifetime, persisted.
using FeatureId = StrongId<FeatureIdTag>;
/// Identifies a body (solid/surface container). Stable across recomputes, persisted.
using BodyId = StrongId<BodyIdTag>;
/// Identifies a component (assembly node). Stable, persisted.
using ComponentId = StrongId<ComponentIdTag>;

} // namespace iw

template <class Tag> struct std::hash<iw::StrongId<Tag>> {
    std::size_t operator()(iw::StrongId<Tag> id) const noexcept {
        return std::hash<typename iw::StrongId<Tag>::ValueType>{}(id.value());
    }
};
