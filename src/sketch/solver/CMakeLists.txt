# SPDX-License-Identifier: GPL-3.0-or-later
# iw_sketch_solver: constraint solver adapter over the vendored FreeCAD planegcs (LGPL-2.0+, see
# third_party/planegcs/README.md). The vendored sources build in their own target so the Ironwood
# warning policy (-Wall -Wextra -Werror via iw_target_defaults) never applies to third-party code.
set(IW_PLANEGCS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/planegcs")

find_package(Eigen3 REQUIRED NO_MODULE)  # system Eigen is 5.0.1; planegcs builds against it (see PATCHES.md)
get_target_property(IW_EIGEN_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

add_library(iw_planegcs STATIC
    "${IW_PLANEGCS_DIR}/GCS.cpp"
    "${IW_PLANEGCS_DIR}/Geo.cpp"
    "${IW_PLANEGCS_DIR}/Constraints.cpp"
    "${IW_PLANEGCS_DIR}/SubSystem.cpp"
    "${IW_PLANEGCS_DIR}/qp_eq.cpp")
target_compile_options(iw_planegcs PRIVATE -w)   # vendored: no warnings, no -Werror
set_target_properties(iw_planegcs PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Eigen and Boost headers are system headers for every consumer (warnings silenced).
target_include_directories(iw_planegcs SYSTEM PUBLIC
    "${IW_PLANEGCS_DIR}"
    "${IW_PLANEGCS_DIR}/compat"
    ${IW_EIGEN_INCLUDE_DIRS})

# bridge.cpp maps sketch::Sketch <-> solver input (Qt-free). It is part of this library because it needs the solver.
iw_add_module(sketch_solver
    SOURCES sketch_solver.cpp bridge.cpp
    PUBLIC_LIBS iw_sketch iw_core
    PRIVATE_LIBS iw_planegcs)
