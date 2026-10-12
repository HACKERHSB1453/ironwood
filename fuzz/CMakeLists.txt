# SPDX-License-Identifier: GPL-3.0-or-later
# Naming fuzzer (P1a gate, A6): library iw_fuzz_naming (generators, scenarios, oracle, shrinker, JSON report) and
# the CLI ironwood-fuzz-naming. The ctest smoke run lives in tests/test_fuzz_smoke.cpp (tests/fuzz_smoke.cmake).
iw_add_module(fuzz_naming
    SOURCES src/rng.cpp src/generator.cpp src/extrude_generator.cpp src/scenario.cpp src/runner.cpp
    PUBLIC_LIBS iw_core iw_kernel iw_naming iw_recompute iw_features)
target_include_directories(iw_fuzz_naming PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
# Vendored nlohmann/json (MIT, third_party/nlohmann) for the JSON report only. SYSTEM silences its headers.
target_include_directories(iw_fuzz_naming SYSTEM PRIVATE "${CMAKE_SOURCE_DIR}/third_party")

add_executable(ironwood-fuzz-naming src/main.cpp)
iw_target_defaults(ironwood-fuzz-naming)
target_link_libraries(ironwood-fuzz-naming PRIVATE iw_fuzz_naming)
set_target_properties(ironwood-fuzz-naming PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
