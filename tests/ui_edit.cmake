# SPDX-License-Identifier: GPL-3.0-or-later
# UI2: Edit Sketch from the timeline and the extrude distance arrow. Offscreen only.
if(IRONWOOD_BUILD_GUI)
    find_package(Qt6 6.5 REQUIRED COMPONENTS Test)
    iw_add_module_test(ui_edit LIBS iw_ui Qt6::Test)
endif()
