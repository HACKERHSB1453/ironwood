// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <functional>
#include <string>

#include "core/params.hpp"
#include "core/result.hpp"
#include "core/undo.hpp"

namespace iw::ui {

/// The editable document as the shell sees it: the parameter table and the undo history of every edit.
/// Not copyable (the undo stack owns its commands and holds pointers into `params`).
struct DocumentState {
    ParameterTable params;
    UndoStack undo;
};

/// Applies `mutate` to a copy of the parameter table; when that succeeds the whole change is recorded as one
/// undoable edit labelled `text`. A rejected mutation leaves the document and the undo stack untouched.
Result<void> editParameters(DocumentState& doc, std::string text,
                            const std::function<Result<void>(ParameterTable&)>& mutate);

} // namespace iw::ui
