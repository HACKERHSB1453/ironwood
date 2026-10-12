// SPDX-License-Identifier: GPL-3.0-or-later
#include "core/undo.hpp"

namespace iw {

Result<void> UndoStack::push(std::unique_ptr<Command> cmd) {
    if (!cmd) return makeError(ErrorCode::InvalidArgument, "null command");
    if (auto r = cmd->execute(); !r) return r;

    // Redo tail is discarded; if the clean state lived in it, it becomes unreachable.
    if (clean_ > static_cast<long>(index_)) clean_ = -1;
    cmds_.resize(index_);

    if (index_ > 0 && cmd->mergeId() >= 0) {
        Command& top = *cmds_[index_ - 1];
        if (top.mergeId() == cmd->mergeId() && top.mergeWith(*cmd)) {
            if (clean_ == static_cast<long>(index_)) clean_ = -1; // saved state was before the merge
            return {};
        }
    }
    cmds_.push_back(std::move(cmd));
    ++index_;
    return {};
}

Result<void> UndoStack::undo() {
    if (!canUndo()) return makeError(ErrorCode::NotFound, "nothing to undo");
    cmds_[index_ - 1]->undo();
    --index_;
    return {};
}

Result<void> UndoStack::redo() {
    if (!canRedo()) return makeError(ErrorCode::NotFound, "nothing to redo");
    if (auto r = cmds_[index_]->execute(); !r) return r;
    ++index_;
    return {};
}

std::string UndoStack::undoText() const { return canUndo() ? cmds_[index_ - 1]->text() : std::string{}; }
std::string UndoStack::redoText() const { return canRedo() ? cmds_[index_]->text() : std::string{}; }

void UndoStack::clear() {
    cmds_.clear();
    index_ = 0;
    clean_ = 0;
}

} // namespace iw
