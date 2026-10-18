# SPDX-License-Identifier: GPL-3.0-or-later
# W2b1 UI integration (parameters on the document, feature names, construction edits, inspect, overlays, icons).
# Offscreen only.
if(IRONWOOD_BUILD_GUI)
    find_package(Qt6 6.5 REQUIRED COMPONENTS Test)
    iw_add_module_test(ui_integration LIBS iw_ui Qt6::Test)
endif()
