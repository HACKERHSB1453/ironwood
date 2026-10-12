# SPDX-License-Identifier: GPL-3.0-or-later
# Headless CLI: must never link Qt.
add_executable(ironwood-cli main.cpp)
iw_target_defaults(ironwood-cli)
target_link_libraries(ironwood-cli PRIVATE iw_core iw_kernel iw_io iw_recompute iw_tess iw_naming)
set_target_properties(ironwood-cli PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
