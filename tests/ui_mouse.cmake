# SPDX-License-Identifier: GPL-3.0-or-later
# UX1 real mouse input on the viewport (sketch flat view, pick-dialog selection by hover and click). Offscreen only.
if(IRONWOOD_BUILD_GUI)
    find_package(Qt6 6.5 REQUIRED COMPONENTS Test)
    iw_add_module_test(ui_mouse LIBS iw_ui Qt6::Test)
endif()
