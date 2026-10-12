// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "core/result.hpp"
#include "ui/command.hpp"

namespace iw::ui {

/// Registers the shell's commands. Working today: Import, Undo, Redo, Change Parameters, S-box, Repeat Last, Create
/// Sketch (plane chooser, sketch mode). Placeholders (shown, disabled, "coming soon") for the P1a modelling commands
/// that B1/B3/B6 will implement: Extrude, Revolve, Hole, Press Pull, Fillet, Chamfer, Move/Copy, Delete, Offset Plane, Axis, Point,
/// Measure, Joint, New Component, Select. Group order follows registration order (CREATE, MODIFY, ASSEMBLE,
/// CONSTRUCT, INSPECT, INSERT, SELECT).
/// Errors: propagates CommandRegistry::add errors (none for the built-in set).
Result<void> registerDefaultCommands(CommandRegistry& registry);

} // namespace iw::ui
