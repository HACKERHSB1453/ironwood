# SPDX-License-Identifier: GPL-3.0-or-later
add_executable(ironwood main.cpp)
iw_target_defaults(ironwood)
target_link_libraries(ironwood PRIVATE iw_ui iw_render Qt6::Widgets)
target_compile_definitions(ironwood PRIVATE IRONWOOD_VERSION="${PROJECT_VERSION}")
set_target_properties(ironwood PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
