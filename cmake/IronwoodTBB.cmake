# SPDX-License-Identifier: GPL-3.0-or-later
# Provides the TBB::tbb target.
#
# Normal builds: system oneTBB via find_package.
# IRONWOOD_SANITIZE=thread: oneTBB is fetched and built from source with -fsanitize=thread,
# so ThreadSanitizer instruments TBB's own synchronisation (task handoffs, arena/queue
# atomics). A system libtbb.so is uninstrumented and produces false "data race" reports
# on every handoff.

set(IRONWOOD_TBB_VERSION "2023.1.0" CACHE STRING "oneTBB tag fetched when IRONWOOD_SANITIZE=thread")

if(NOT IRONWOOD_SANITIZE MATCHES "thread")
    find_package(TBB CONFIG REQUIRED)
    return()
endif()

include(FetchContent)

message(STATUS "TSAN build: fetching oneTBB v${IRONWOOD_TBB_VERSION} and building it with -fsanitize=thread")

# Sanitizer flags for the TBB subproject only: save and restore so project-wide flags stay as they were.
set(_iw_saved_cxx_flags "${CMAKE_CXX_FLAGS}")
set(_iw_saved_c_flags "${CMAKE_C_FLAGS}")
set(_iw_saved_exe_link "${CMAKE_EXE_LINKER_FLAGS}")
set(_iw_saved_shared_link "${CMAKE_SHARED_LINKER_FLAGS}")
set(_iw_saved_shared_libs "${BUILD_SHARED_LIBS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread -fno-omit-frame-pointer")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=thread -fno-omit-frame-pointer")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=thread")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=thread")

set(TBB_TEST OFF CACHE BOOL "" FORCE)
set(TBB_EXAMPLES OFF CACHE BOOL "" FORCE)
set(TBB_STRICT OFF CACHE BOOL "" FORCE)      # do not let TBB's own -Werror break the sanitizer build
set(TBB_INSTALL OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS ON)

FetchContent_Declare(oneTBB
    GIT_REPOSITORY https://github.com/uxlfoundation/oneTBB.git
    GIT_TAG        v${IRONWOOD_TBB_VERSION}
    GIT_SHALLOW    TRUE
    SYSTEM)
FetchContent_MakeAvailable(oneTBB)

set(CMAKE_CXX_FLAGS "${_iw_saved_cxx_flags}")
set(CMAKE_C_FLAGS "${_iw_saved_c_flags}")
set(CMAKE_EXE_LINKER_FLAGS "${_iw_saved_exe_link}")
set(CMAKE_SHARED_LINKER_FLAGS "${_iw_saved_shared_link}")
set(BUILD_SHARED_LIBS "${_iw_saved_shared_libs}")

if(NOT TARGET TBB::tbb)
    message(FATAL_ERROR "oneTBB was fetched but did not provide TBB::tbb")
endif()
