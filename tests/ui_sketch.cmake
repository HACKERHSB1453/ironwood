# SPDX-License-Identifier: GPL-3.0-or-later
# B3a part 1: sketch session (Qt-free) and sketch mode in the shell. Offscreen only (QTest on the viewport).
if(IRONWOOD_BUILD_GUI)
    find_package(Qt6 6.5 REQUIRED COMPONENTS Test)
    iw_add_module_test(ui_sketch LIBS iw_ui Qt6::Test)
endif()
