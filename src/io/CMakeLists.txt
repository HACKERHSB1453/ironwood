# SPDX-License-Identifier: GPL-3.0-or-later
iw_add_module(io
    SOURCES document_io.cpp
    PUBLIC_LIBS iw_core iw_kernel iw_naming iw_recompute)
# Vendored nlohmann/json (MIT, third_party/nlohmann). Private to this module; SYSTEM silences its headers.
target_include_directories(iw_io SYSTEM PRIVATE "${CMAKE_SOURCE_DIR}/third_party")
