# SPDX-License-Identifier: GPL-3.0-or-later
# B1 (features): feature tests, plus the recompute test now links iw_features because ExtrudeFeature moved there.
iw_add_module_test(features LIBS iw_features iw_io)
target_link_libraries(test_recompute PRIVATE iw_features)
# test_determinism builds timelines of ExtrudeFeature, which now lives in iw_features.
target_link_libraries(test_determinism PRIVATE iw_features)
