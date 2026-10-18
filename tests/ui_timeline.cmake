# SPDX-License-Identifier: GPL-3.0-or-later
# B2 timeline UI (document binding, timeline strip, browser, modelling dialogs). Offscreen only.
if(IRONWOOD_BUILD_GUI)
    find_package(Qt6 6.5 REQUIRED COMPONENTS Test)
    iw_add_module_test(ui_timeline LIBS iw_ui Qt6::Test)
endif()
