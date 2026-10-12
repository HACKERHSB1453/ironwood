# SPDX-License-Identifier: GPL-3.0-or-later
# sketch (A2): entities, constraint/dimension records, planar-graph profile detection, region -> face.
# Depends on core + kernel only. OCCT is private (region -> face in profile_face.cpp).
# The constraint solver lives in solver/ (module iw_sketch_solver, built separately). iw_sketch must NOT depend on it.
iw_add_module(sketch
    SOURCES sketch.cpp profile.cpp profile_face.cpp
    PUBLIC_LIBS iw_core iw_kernel
    PRIVATE_LIBS iw_kernel_occt)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/solver/CMakeLists.txt)
    add_subdirectory(solver)
endif()
