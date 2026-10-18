// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// B6: Inspect commands (Measure, Properties, Section Analysis) for the B4 command registry.
// Integration (default_commands / ribbon) is a later task: it calls registerCommands() once.

#include <functional>
#include <optional>
#include <vector>

#include "core/result.hpp"
#include "kernel/measure.hpp"
#include "kernel/shape.hpp"
#include "ui/command.hpp"

namespace iw::ui::inspect {

/// What the inspect commands reach. Every member is optional; a missing provider makes the command report
/// an empty selection / no body instead of failing. Owned by the caller; must outlive the registry's commands.
struct InspectContext {
    /// Current selection (pick order). Used by Measure.
    std::function<std::vector<kernel::measure::Entity>()> selection;
    /// The body Properties shows (usually the active or single selected body). Used by Properties.
    std::function<std::optional<kernel::Shape>()> activeBody;
    /// Receives the section RenderRequest on every change. The viewport hook lives outside this module.
    std::function<void(const kernel::measure::section::RenderRequest&)> sectionSink;
};

/// Registers "measure" (shortcut I, Fusion default), "properties" and "section_analysis" in `registry`.
/// Each command docks its widget through CommandContext::showPanel. `ctx` is captured by pointer.
/// Errors: InvalidArgument when an id is already registered.
Result<void> registerCommands(CommandRegistry& registry, InspectContext& ctx);

} // namespace iw::ui::inspect
