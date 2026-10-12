# SPDX-License-Identifier: GPL-3.0-or-later
# Performance baseline harness. Run in a release build: ironwood-bench > baseline.json
add_executable(ironwood-bench bench.cpp)
iw_target_defaults(ironwood-bench)
target_link_libraries(ironwood-bench PRIVATE iw_core iw_kernel iw_naming iw_tess iw_recompute iw_features)
set_target_properties(ironwood-bench PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
