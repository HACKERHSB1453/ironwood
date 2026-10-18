# SPDX-License-Identifier: GPL-3.0-or-later
# B5: headless Python bindings. Module `ironwood` (file ironwood.<abi>.so) over iw::features. Skipped quietly when
# Python development files or pybind11 are missing, so the rest of the build never depends on them.
find_package(Python3 COMPONENTS Interpreter Development.Module)
find_package(pybind11 3 CONFIG)
if(NOT Python3_FOUND OR NOT pybind11_FOUND)
    message(STATUS "ironwood python module: skipped (needs Python3 development files and pybind11 >= 3)")
    return()
endif()

pybind11_add_module(ironwood_py MODULE ironwood_module.cpp)
set_target_properties(ironwood_py PROPERTIES OUTPUT_NAME ironwood)
iw_target_defaults(ironwood_py)
target_link_libraries(ironwood_py PRIVATE iw::features)
