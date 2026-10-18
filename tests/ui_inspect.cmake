# SPDX-License-Identifier: GPL-3.0-or-later
# B6 Inspect widgets (measure dialog, properties panel, section dialog, command registration). Offscreen only.
if(IRONWOOD_BUILD_GUI)
    find_package(Qt6 6.5 REQUIRED COMPONENTS Test)
    iw_add_module_test(ui_inspect LIBS iw_ui Qt6::Test)
endif()
