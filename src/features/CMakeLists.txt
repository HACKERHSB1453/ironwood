# SPDX-License-Identifier: GPL-3.0-or-later
# features (B1, P1a): real modelling features on top of recompute. Feature classes live in namespace iw::features.
# Dependency direction: features -> sketch (+ solver), io, recompute, naming, kernel, core. OCCT stays private (kernel facade only).
iw_add_module(features
    SOURCES extrude_feature.cpp sketch_json.cpp feature_common.cpp revolve_feature.cpp edge_features.cpp hole_feature.cpp move_feature.cpp construction_features.cpp sketch_feature.cpp document.cpp
    PUBLIC_LIBS iw_core iw_kernel iw_naming iw_recompute iw_sketch iw_sketch_solver iw_io)
# Vendored nlohmann/json (MIT, third_party/nlohmann). Private to this module; SYSTEM silences its headers.
target_include_directories(iw_features SYSTEM PRIVATE "${CMAKE_SOURCE_DIR}/third_party")
