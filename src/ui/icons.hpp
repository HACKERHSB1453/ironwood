// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QIcon>
#include <QImage>

#include <string_view>

class QPalette;

namespace iw::ui {

/// Which UI colour scheme an icon is drawn for. Dark: light strokes on a dark UI. Light: dark strokes on a light UI.
enum class IconTheme { Light, Dark };

/// Theme matching a palette (dark when the window colour is dark).
IconTheme iconThemeFor(const QPalette& palette);

/// True when resources/icons has an icon of this name (e.g. "extrude" -> :/icons/extrude.svg).
bool iconExists(std::string_view name);

/// Renders an icon at size x size pixels. Unknown names render the "missing" fallback glyph, so the result is never
/// null. Strokes (currentColor) take the theme colour; accent colours are fixed per icon group.
QImage renderIcon(std::string_view name, int size, IconTheme theme);

/// QIcon for a toolbar/action: one pixmap at `size`; Qt derives the greyed Disabled state from it.
QIcon icon(std::string_view name, int size, IconTheme theme);

} // namespace iw::ui
