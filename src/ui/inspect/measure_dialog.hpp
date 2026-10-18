// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// B6: Measure dialog (Fusion-style). Two selection slots, a results list with units, precision and copy.
// The selection comes from an injected provider so the widget works offscreen in tests.

#include <QString>
#include <QStringList>
#include <QWidget>

#include <functional>
#include <optional>
#include <vector>

#include "kernel/measure.hpp"

class QLabel;
class QListWidget;
class QPushButton;
class QSpinBox;

namespace iw::ui::inspect {

/// Returns the currently selected entities, in pick order. May be empty.
using SelectionProvider = std::function<std::vector<kernel::measure::Entity>()>;

/// Human-readable entity name, e.g. "Face 3", "Edge 1 (circle)", "Body".
QString entityLabel(const kernel::measure::Entity& e);

/// Display form of a unit: "mm^2" -> "mm²", "mm^3" -> "mm³", "kg*mm^2" -> "kg·mm²".
QString prettyUnit(const std::string& unit);

class MeasureDialog : public QWidget {
    Q_OBJECT
public:
    explicit MeasureDialog(SelectionProvider provider = {}, QWidget* parent = nullptr);

    /// Slots are 1 and 2 (as on the buttons); any other slot is ignored. Puts `e` in and recomputes.
    void setSelection(int slot, const kernel::measure::Entity& e);
    void clearSelection(int slot);
    const std::optional<kernel::measure::Entity>& selection(int slot) const;
    /// Fills `slot` (1 or 2) from the provider item at the same position (slot 1 = first pick). False when the
    /// provider has no item for that slot.
    bool pickFromProvider(int slot);

    void setPrecision(int decimals);
    int precision() const noexcept;

    /// Rows currently shown (empty when there is no selection or the measurement failed).
    const std::vector<kernel::measure::MeasureRow>& rows() const noexcept { return rows_; }
    /// Formatted lines "label: value unit", as shown and as copied.
    QStringList resultLines() const;
    /// Newline-joined resultLines().
    QString copyText() const;
    /// Puts copyText() on the system clipboard. Returns false when there is nothing to copy.
    bool copyToClipboard();

    QString statusText() const;

signals:
    void resultsChanged();

private:
    void recompute();
    void refresh();

    SelectionProvider provider_;
    std::optional<kernel::measure::Entity> sel_[2];
    std::vector<kernel::measure::MeasureRow> rows_;
    QString status_;

    QLabel* slotLabels_[2] = {nullptr, nullptr};
    QPushButton* pickButtons_[2] = {nullptr, nullptr};
    QPushButton* copyButton_ = nullptr;
    QSpinBox* precision_ = nullptr;
    QListWidget* results_ = nullptr;
    QLabel* status_label_ = nullptr;
};

} // namespace iw::ui::inspect
