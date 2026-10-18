# SPDX-License-Identifier: GPL-3.0-or-later
# F25 icon set, ui::icon loader and ribbon icon layout (GUI). Offscreen only.
if(IRONWOOD_BUILD_GUI)
    iw_add_module_test(ui_icons LIBS iw_ui)
endif()
