# SPDX-License-Identifier: GPL-3.0-or-later
cmake_minimum_required(VERSION 3.28)

project(Ironwood VERSION 0.0.1 DESCRIPTION "Open-source Linux-native parametric CAD" LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(IRONWOOD_BUILD_GUI "Build the Qt GUI (render, ui, ironwood)" ON)
option(IRONWOOD_BUILD_TESTS "Build unit tests" ON)
option(IRONWOOD_TRACY "Enable Tracy profiler (fetched via FetchContent; needs network)" OFF)

include(IronwoodDefaults)   # warnings, sanitizers, ccache, iw_add_module()

find_package(OpenCASCADE CONFIG REQUIRED)
include(IronwoodTBB)        # TBB::tbb: system oneTBB, or built from source under IRONWOOD_SANITIZE=thread
if(IRONWOOD_BUILD_GUI)
    set(QT_NO_PRIVATE_MODULE_WARNING ON)  # QRhi is only available via GuiPrivate
    find_package(Qt6 6.7 REQUIRED COMPONENTS Core Gui Widgets GuiPrivate)
endif()
if(IRONWOOD_BUILD_TESTS)
    find_package(GTest CONFIG REQUIRED)
    enable_testing()
endif()
if(IRONWOOD_TRACY)
    include(IronwoodTracy)
endif()

# Generated version header lives in the build tree: <core/version.hpp>.
configure_file(src/core/version.hpp.in "${CMAKE_BINARY_DIR}/generated/core/version.hpp" @ONLY)

# ---- modules (dependency direction per docs/ARCHITECTURE.md section 3) ------------------------
# core      : std + TBB (private only). No Qt, no OCCT.
# kernel    : core + OCCT
# naming    : core + kernel
# tess      : core + kernel
# recompute : core + kernel + naming
# io        : core + kernel + naming + recompute
# render    : core + tess + Qt Gui/Widgets  (only Qt-Gui user, with ui)
# ui        : everything + render
add_subdirectory(src/core)
add_subdirectory(src/kernel)
add_subdirectory(src/naming)
add_subdirectory(src/tess)
add_subdirectory(src/recompute)
add_subdirectory(src/io)
if(IRONWOOD_BUILD_GUI)
    add_subdirectory(src/render)
    add_subdirectory(src/ui)
endif()

add_subdirectory(apps/ironwood-cli)
add_subdirectory(bench)
if(IRONWOOD_BUILD_GUI)
    add_subdirectory(apps/ironwood)
endif()

if(IRONWOOD_BUILD_TESTS)
    add_subdirectory(tests)
endif()
