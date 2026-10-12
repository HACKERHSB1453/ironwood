// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

/// Profiling zone macro. Expands to Tracy's ZoneScoped when built with -DIRONWOOD_TRACY=ON, else to nothing.
/// Usage: `IW_ZONE("tessellate");` at the top of a scope.
#ifdef IRONWOOD_TRACY_ENABLED
#include <tracy/Tracy.hpp>
#define IW_ZONE(name) ZoneScopedN(name)
#else
#define IW_ZONE(name) ((void)0)
#endif
