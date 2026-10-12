# SPDX-License-Identifier: GPL-3.0-or-later
# Optional Tracy profiler (BSD-3). Needs network on first configure; default OFF.
include(FetchContent)
FetchContent_Declare(tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG v0.11.1
    GIT_SHALLOW TRUE)
set(TRACY_ENABLE ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(tracy)
# Consumers link Tracy::TracyClient via iw_core (see src/core/CMakeLists.txt).
