// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QWidget>

#include <cstddef>
#include <string>
#include <vector>

#include "ui/command.hpp"

class QLineEdit;
class QListWidget;

namespace iw::ui {

/// S-box: a searchable command palette (press S). Lists registry commands ranked by fuzzy match with recent commands
/// first when the query is empty. Enter runs the highlighted command, Ctrl+Enter pins it to the quick-access toolbar,
/// Esc closes. Unimplemented commands are listed but cannot be chosen.
class CommandPalette : public QWidget {
    Q_OBJECT
public:
    static constexpr std::size_t kMaxRecent = 12;

    explicit CommandPalette(const CommandRegistry& registry, QWidget* parent = nullptr);

    /// Shows the palette with an empty query and focuses the search field.
    void open();
    void setQuery(const QString& query);
    QString query() const;
    /// Ids of the listed rows, best first.
    std::vector<std::string> results() const;

    void setRecent(std::vector<std::string> ids);
    const std::vector<std::string>& recent() const noexcept { return recent_; }

    void setCurrentRow(int row);
    int currentRow() const;
    /// Emits commandChosen for the highlighted row when it is implemented; returns whether it did.
    bool chooseCurrent();
    /// Emits pinRequested for the highlighted row; returns whether it did.
    bool pinCurrent();

signals:
    void commandChosen(const QString& id);
    void pinRequested(const QString& id);

protected:
    void keyPressEvent(QKeyEvent* e) override;

private:
    void rebuild();
    std::string currentId() const;

    const CommandRegistry& registry_;
    std::vector<std::string> recent_;
    QLineEdit* edit_ = nullptr;
    QListWidget* list_ = nullptr;
};

} // namespace iw::ui
