// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "core/result.hpp"

namespace iw {

/// A reversible document edit. Implementations hold whatever state they need to undo themselves.
/// All methods are called on the owning (UI/document) thread only.
class Command {
public:
    virtual ~Command() = default;

    /// Performs (or re-performs, for redo) the edit. On failure it must leave the document unchanged and
    /// return an Error; the stack then does not record the command.
    virtual Result<void> execute() = 0;
    /// Reverts exactly what `execute()` did. Must not fail for a command that executed successfully.
    virtual void undo() = 0;
    /// Human-readable label for the Edit menu ("Move Feature", ...). Already translated.
    virtual std::string text() const = 0;

    /// Commands with the same non-negative id may be merged (e.g. successive drags of one dimension).
    /// Default -1 = never merge.
    virtual int mergeId() const { return -1; }
    /// Called on the stack-top command with a newer, ALREADY EXECUTED command of the same mergeId.
    /// Return true to absorb `next` (the stack then drops `next`); false to keep both.
    virtual bool mergeWith(const Command& next) {
        (void)next;
        return false;
    }
};

/// Linear undo/redo history with merge and a clean (saved) marker. Not thread-safe. Owns commands.
class UndoStack {
public:
    /// Executes `cmd`; on success appends it (discarding the redo tail), merging if possible.
    /// On failure returns the Error and the stack is unchanged.
    Result<void> push(std::unique_ptr<Command> cmd);

    bool canUndo() const noexcept { return index_ > 0; }
    bool canRedo() const noexcept { return index_ < cmds_.size(); }
    /// Undoes the most recent command; no-op (NotFound error) when nothing to undo.
    Result<void> undo();
    /// Re-executes the next command; returns its execute() error if it fails (index unchanged).
    Result<void> redo();

    /// Text of the command undo()/redo() would act on; empty if none.
    std::string undoText() const;
    std::string redoText() const;

    /// Number of recorded commands, and current position (0..size()).
    std::size_t size() const noexcept { return cmds_.size(); }
    std::size_t index() const noexcept { return index_; }

    /// Marks the current position as the saved state.
    void setClean() noexcept { clean_ = static_cast<long>(index_); }
    /// True if the current position equals the saved state.
    bool isClean() const noexcept { return clean_ == static_cast<long>(index_); }

    void clear();

private:
    std::vector<std::unique_ptr<Command>> cmds_;
    std::size_t index_ = 0;
    long clean_ = 0; ///< position of the saved state; -1 = unreachable
};

} // namespace iw
