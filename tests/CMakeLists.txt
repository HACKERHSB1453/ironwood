# SPDX-License-Identifier: GPL-3.0-or-later
include(GoogleTest)

# iw_add_module_test(<module> LIBS ...) builds test_<module> from test_<module>.cpp and registers it with CTest.
function(iw_add_module_test name)
    cmake_parse_arguments(T "" "" "LIBS" ${ARGN})
    add_executable(test_${name} test_${name}.cpp)
    iw_target_defaults(test_${name})
    target_link_libraries(test_${name} PRIVATE ${T_LIBS} GTest::gtest_main)
    gtest_discover_tests(test_${name} PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endfunction()

iw_add_module_test(core LIBS iw_core)
iw_add_module_test(kernel LIBS iw_kernel)
iw_add_module_test(naming LIBS iw_naming)
iw_add_module_test(tess LIBS iw_tess)
iw_add_module_test(recompute LIBS iw_recompute iw_io)
iw_add_module_test(io LIBS iw_io)
iw_add_module_test(determinism LIBS iw_tess iw_recompute iw_naming iw_kernel)
if(IRONWOOD_BUILD_GUI)
    iw_add_module_test(render LIBS iw_render)
    iw_add_module_test(ui LIBS iw_ui)
endif()

# P1a: each task drops tests/<name>.cmake containing its own iw_add_module_test(...) line(s).
# This avoids parallel edits to this file.
file(GLOB _iw_test_fragments CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cmake)
foreach(_f IN LISTS _iw_test_fragments)
    include(${_f})
endforeach()
