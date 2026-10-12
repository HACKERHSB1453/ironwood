// SPDX-License-Identifier: LGPL-2.0-or-later
// Ironwood compatibility shim for FreeCAD's Base/Console.h (see ../../PATCHES.md).
// planegcs only uses Base::Console().log/warning for solver diagnostics and Base::TimeElapsed for
// timing. Logging is dropped (the solver must not write to stdout/stderr from library code), and
// TimeElapsed is a steady_clock wrapper with the same static diffTimeF() signature.
#ifndef IRONWOOD_PLANEGCS_BASE_CONSOLE_H
#define IRONWOOD_PLANEGCS_BASE_CONSOLE_H

#include <chrono>

namespace Base
{

struct SilentConsole
{
    template<typename... Args>
    void log(const char*, Args&&...) const
    {}
    template<typename... Args>
    void warning(const char*, Args&&...) const
    {}
    template<typename... Args>
    void message(const char*, Args&&...) const
    {}
    template<typename... Args>
    void error(const char*, Args&&...) const
    {}
};

inline SilentConsole Console()
{
    return {};
}

class TimeElapsed
{
public:
    TimeElapsed()
        : start_(std::chrono::steady_clock::now())
    {}

    static double diffTimeF(const TimeElapsed& from, const TimeElapsed& to)
    {
        return std::chrono::duration<double>(to.start_ - from.start_).count();
    }

private:
    std::chrono::steady_clock::time_point start_;
};

}  // namespace Base

#endif  // IRONWOOD_PLANEGCS_BASE_CONSOLE_H
