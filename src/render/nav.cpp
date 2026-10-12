// SPDX-License-Identifier: GPL-3.0-or-later
#include "render/nav.hpp"

namespace iw::render {

namespace {
using enum MouseButton;
constexpr NavMapping kFusion{{Middle, ModShift}, {Middle, ModNone}, {None, ModNone}, true, "Fusion"};
constexpr NavMapping kInventor{{Middle, ModShift}, {Middle, ModNone}, {None, ModNone}, true, "Inventor"};
constexpr NavMapping kSolidWorks{{Middle, ModNone}, {Middle, ModCtrl}, {Middle, ModShift}, true, "SolidWorks"};
constexpr NavMapping kAlias{{Left, ModAlt}, {Middle, ModAlt}, {Right, ModAlt}, true, "Alias"};
constexpr NavMapping kTinkercad{{Right, ModNone}, {Middle, ModNone}, {None, ModNone}, true, "Tinkercad"};
} // namespace

const NavMapping& navMappingFor(NavPreset preset) noexcept {
    switch (preset) {
    case NavPreset::Fusion: return kFusion;
    case NavPreset::Inventor: return kInventor;
    case NavPreset::SolidWorks: return kSolidWorks;
    case NavPreset::Alias: return kAlias;
    case NavPreset::Tinkercad: return kTinkercad;
    }
    return kFusion;
}

} // namespace iw::render
