// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <string_view>

namespace iw::render {

/// Navigation presets modelled on the input conventions of other CAD/DCC tools (selectable in preferences).
/// ALL mappings below are from memory of public documentation and are [VERIFY]: confirm against the
/// running applications / current docs before treating them as final.
enum class NavPreset { Fusion, Inventor, SolidWorks, Alias, Tinkercad };

enum class MouseButton : std::uint8_t { None, Left, Middle, Right };

/// Keyboard modifier bit set (Qt-free).
enum Modifier : std::uint8_t { ModNone = 0, ModShift = 1, ModCtrl = 2, ModAlt = 4 };

/// A button + modifier chord that triggers a navigation gesture.
struct Chord {
    MouseButton button = MouseButton::None;
    std::uint8_t modifiers = ModNone;
    friend constexpr bool operator==(Chord, Chord) = default;
};

/// Gesture bindings of a preset. Mouse wheel always zooms; `wheelZoomsTowardCursor` says toward cursor vs. centre.
struct NavMapping {
    Chord orbit, pan, zoom;       ///< Drag chords; zoom drag may be unset (button None).
    bool wheelZoomsTowardCursor = true;
    std::string_view name;
};

/// Returns the (static) mapping of `preset`:
///  * Fusion     [VERIFY]: MMB drag = pan; Shift + MMB drag = orbit; wheel = zoom toward cursor.
///  * Inventor   [VERIFY]: MMB = pan; Shift + MMB = orbit; wheel = zoom (F3/F4 hotkey modes not modelled).
///  * SolidWorks [VERIFY]: MMB = orbit; Ctrl + MMB = pan; Shift + MMB = zoom; wheel = zoom.
///  * Alias      [VERIFY]: Alt + LMB = orbit (tumble); Alt + MMB = pan; Alt + RMB = zoom; wheel = zoom.
///  * Tinkercad  [VERIFY]: RMB = orbit; MMB = pan; wheel = zoom.
const NavMapping& navMappingFor(NavPreset preset) noexcept;

} // namespace iw::render
