# SPDX-License-Identifier: GPL-3.0-or-later
# core: std + oneTBB (private to .cpp files; headers stay TBB-free). No Qt, no OCCT.
iw_add_module(core
    SOURCES units.cpp undo.cpp job.cpp expr.cpp params.cpp
    PRIVATE_LIBS TBB::tbb)
if(IRONWOOD_TRACY)
    target_link_libraries(iw_core PUBLIC Tracy::TracyClient)
endif()
