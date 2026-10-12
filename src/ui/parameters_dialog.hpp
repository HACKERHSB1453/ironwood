// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QWidget>

#include <string>

#include "core/result.hpp"
#include "ui/doc_state.hpp"

class QLabel;
class QTableWidget;

namespace iw::ui {

/// Modify > Change Parameters: the user parameter table (name, unit, expression, value, comment, favourite).
/// Every edit is an undoable change to the document's parameter table. A rejected edit leaves the table untouched and
/// shows its message inline (cycle, unknown unit, parse error, duplicate name...). The value column shows the evaluated
/// value, or the evaluation error for that parameter.
class ParametersDialog : public QWidget {
    Q_OBJECT
public:
    enum Column { Name = 0, Unit, Expression, Value, Comment, Favourite, ColumnCount };

    explicit ParametersDialog(DocumentState& doc, QWidget* parent = nullptr);

    QTableWidget* table() const noexcept { return table_; }
    /// Inline error of the last rejected action; empty after a successful one.
    QString errorText() const;

    /// Adds a parameter. Errors are also shown inline (see errorText()).
    Result<void> addParameter(const std::string& name, const std::string& expression, const std::string& unit = {},
                              const std::string& comment = {});
    /// Removes a parameter (refused while others reference it).
    Result<void> deleteParameter(const std::string& name);
    /// Edits one cell as the user would: column Name/Unit/Expression/Comment take text, Favourite takes a check state.
    /// Row order follows the parameter table.
    Result<void> editCell(int row, int column, const QString& text);
    /// Re-reads the table (values and text) from the document.
    void refresh();

private:
    void onItemChanged(int row, int column);
    void showResult(const Result<void>& r);
    std::string freshName() const;

    DocumentState& doc_;
    QTableWidget* table_ = nullptr;
    QLabel* error_ = nullptr;
};

} // namespace iw::ui
