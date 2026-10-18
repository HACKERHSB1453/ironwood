// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QWidget>

#include <cstdint>
#include <string>

#include "core/result.hpp"

class QLabel;
class QTableWidget;

namespace iw::features {
class Document;
}

namespace iw::ui {

/// Modify > Parameters: the document parameter table (name, unit, expression, value, comment, favourite).
/// Every edit is one undoable command of features::Document, on the same undo stack as the features: Undo in the
/// shell reverts a parameter edit exactly like a feature edit. A rejected edit leaves the document untouched and shows
/// its message inline (cycle, unknown unit, parse error, duplicate name...). The value column shows the evaluated
/// value, or the evaluation error for that parameter. The table follows the document (undo, redo, other edits).
class ParametersDialog : public QWidget {
    Q_OBJECT
public:
    enum Column { Name = 0, Unit, Expression, Value, Comment, Favourite, ColumnCount };

    explicit ParametersDialog(features::Document& doc, QWidget* parent = nullptr);
    ~ParametersDialog() override;

    /// Stops following the document (it is about to be replaced or destroyed). The table stays empty and read-only.
    void detach();

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

    features::Document* doc_ = nullptr;
    std::uint64_t listener_ = 0;   ///< Document listener id (0 = none).
    bool editing_ = false;         ///< An edit is in progress: the document listener does not rebuild the table.
    QTableWidget* table_ = nullptr;
    QLabel* error_ = nullptr;
};

} // namespace iw::ui
