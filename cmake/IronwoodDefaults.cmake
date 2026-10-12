# SPDX-License-Identifier: GPL-3.0-or-later
# Shared build helpers: warnings, sanitizers, optional ccache, module helper.

option(IRONWOOD_WERROR "Treat compiler warnings as errors" ON)
set(IRONWOOD_SANITIZE "" CACHE STRING "Sanitizers to enable, e.g. 'address;undefined'")

# ccache: optional, auto-enabled when found.
find_program(IRONWOOD_CCACHE ccache)
if(IRONWOOD_CCACHE AND NOT CMAKE_CXX_COMPILER_LAUNCHER)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${IRONWOOD_CCACHE}")
    message(STATUS "ccache enabled: ${IRONWOOD_CCACHE}")
endif()

# Applies project warning/sanitizer flags to a target we own (never to third-party code).
function(iw_target_defaults target)
    target_compile_options(${target} PRIVATE -Wall -Wextra)
    if(IRONWOOD_WERROR)
        target_compile_options(${target} PRIVATE -Werror)
    endif()
    if(IRONWOOD_SANITIZE)
        string(REPLACE ";" "," _san "${IRONWOOD_SANITIZE}")
        target_compile_options(${target} PUBLIC -fsanitize=${_san} -fno-omit-frame-pointer)
        target_link_options(${target} PUBLIC -fsanitize=${_san})
    endif()
    target_include_directories(${target} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
        "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>")
    if(IRONWOOD_TRACY)
        target_compile_definitions(${target} PUBLIC IRONWOOD_TRACY_ENABLED=1)
    endif()
endfunction()

# iw_add_module(<name> SOURCES ... [PUBLIC_LIBS ...] [PRIVATE_LIBS ...])
# Creates static library iw_<name> with alias iw::<name>.
function(iw_add_module name)
    cmake_parse_arguments(M "" "" "SOURCES;PUBLIC_LIBS;PRIVATE_LIBS" ${ARGN})
    add_library(iw_${name} STATIC ${M_SOURCES})
    add_library(iw::${name} ALIAS iw_${name})
    iw_target_defaults(iw_${name})
    if(M_PUBLIC_LIBS)
        target_link_libraries(iw_${name} PUBLIC ${M_PUBLIC_LIBS})
    endif()
    if(M_PRIVATE_LIBS)
        target_link_libraries(iw_${name} PRIVATE ${M_PRIVATE_LIBS})
    endif()
endfunction()
