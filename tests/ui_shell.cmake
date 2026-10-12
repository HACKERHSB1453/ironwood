# SPDX-License-Identifier: GPL-3.0-or-later
# B4 command framework & shell (GUI). Offscreen only.
if(IRONWOOD_BUILD_GUI)
    iw_add_module_test(ui_shell LIBS iw_ui)
endif()
