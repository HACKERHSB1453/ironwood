// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/doc_state.hpp"

#include <memory>
#include <utility>

namespace iw::ui {
namespace {

/// Undoable whole-table edit: stores the table before and after, so every parameter edit (rename, expression,
/// delete, ...) is undone exactly, including the rewritten references of a rename.
class ParameterTableEdit final : public iw::Command {
public:
    ParameterTableEdit(ParameterTable* table, std::string text, ParameterTable before, ParameterTable after)
        : table_(table), text_(std::move(text)), before_(std::move(before)), after_(std::move(after)) {}

    Result<void> execute() override {
        *table_ = after_;
        return {};
    }
    void undo() override { *table_ = before_; }
    std::string text() const override { return text_; }

private:
    ParameterTable* table_;
    std::string text_;
    ParameterTable before_;
    ParameterTable after_;
};

} // namespace

Result<void> editParameters(DocumentState& doc, std::string text,
                            const std::function<Result<void>(ParameterTable&)>& mutate) {
    ParameterTable after = doc.params;
    if (auto r = mutate(after); !r) return std::unexpected(r.error());
    auto cmd = std::make_unique<ParameterTableEdit>(&doc.params, std::move(text), doc.params, std::move(after));
    return doc.undo.push(std::move(cmd));
}

} // namespace iw::ui
