// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QString>
#include <QStringView>

#include <string>
#include <vector>

#include "ui/command.hpp"

namespace iw::ui {

/// Subsequence match of `query` in `text`, case-insensitive. Returns 0 when the query does not match. Otherwise a
/// positive score: characters matching at a word start, at the beginning of the text, or directly after the previous
/// match score higher; gaps cost a little. Spaces in the query are ignored.
int fuzzyScore(QStringView query, QStringView text);

/// Commands for the S-box, best first. An empty query lists the recent commands (most recent first), then the rest
/// in registry order. A non-empty query keeps only matching commands, scored on label and id, with a small bonus for
/// recently used commands and for commands that are currently implemented. The "sbox" command itself is never listed.
std::vector<const Command*> rankCommands(const CommandRegistry& registry, const QString& query,
                                         const std::vector<std::string>& recent);

} // namespace iw::ui
