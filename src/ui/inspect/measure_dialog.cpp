// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/inspect/measure_dialog.hpp"

#include <QApplication>
#include <QClipboard>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include <algorithm>
#include <cstddef>
#include <string>

namespace iw::ui::inspect {

namespace km = kernel::measure;

namespace {
constexpr int kDefaultPrecision = 3;
constexpr int kMaxPrecision = 9;
} // namespace

QString entityLabel(const km::Entity& e) {
    switch (e.kind) {
    case km::EntityKind::Vertex: return QString("Vertex %1").arg(e.index + 1);
    case km::EntityKind::Face: return QString("Face %1").arg(e.index + 1);
    case km::EntityKind::Edge: {
        const bool circle = km::circleOf(e).has_value();
        return QString("Edge %1%2").arg(e.index + 1).arg(circle ? " (circle)" : "");
    }
    case km::EntityKind::Body: return QStringLiteral("Body");
    }
    return QStringLiteral("?");
}

QString prettyUnit(const std::string& unit) {
    QString u = QString::fromStdString(unit);
    u.replace("^2", QString::fromUtf8("²"));
    u.replace("^3", QString::fromUtf8("³"));
    u.replace('*', QString::fromUtf8("·"));
    return u;
}

MeasureDialog::MeasureDialog(SelectionProvider provider, QWidget* parent)
    : QWidget(parent), provider_(std::move(provider)) {
    setWindowTitle(QStringLiteral("Measure"));
    auto* root = new QVBoxLayout(this);

    auto* selBox = new QGroupBox(QStringLiteral("Selection"), this);
    auto* grid = new QVBoxLayout(selBox);
    for (int slot = 0; slot < 2; ++slot) {
        auto* row = new QHBoxLayout();
        pickButtons_[slot] = new QPushButton(QStringLiteral("Select %1").arg(slot + 1), selBox);
        slotLabels_[slot] = new QLabel(QStringLiteral("-"), selBox);
        slotLabels_[slot]->setObjectName(QStringLiteral("slotLabel%1").arg(slot + 1));
        row->addWidget(pickButtons_[slot]);
        row->addWidget(slotLabels_[slot], 1);
        grid->addLayout(row);
        connect(pickButtons_[slot], &QPushButton::clicked, this, [this, slot] { pickFromProvider(slot + 1); });
    }
    root->addWidget(selBox);

    auto* optRow = new QHBoxLayout();
    optRow->addWidget(new QLabel(QStringLiteral("Precision"), this));
    precision_ = new QSpinBox(this);
    precision_->setRange(0, kMaxPrecision);
    precision_->setValue(kDefaultPrecision);
    precision_->setObjectName(QStringLiteral("precision"));
    optRow->addWidget(precision_);
    optRow->addStretch(1);
    copyButton_ = new QPushButton(QStringLiteral("Copy"), this);
    copyButton_->setObjectName(QStringLiteral("copy"));
    optRow->addWidget(copyButton_);
    root->addLayout(optRow);
    connect(precision_, &QSpinBox::valueChanged, this, [this] { refresh(); });
    connect(copyButton_, &QPushButton::clicked, this, [this] { copyToClipboard(); });

    results_ = new QListWidget(this);
    results_->setObjectName(QStringLiteral("results"));
    root->addWidget(results_, 1);

    status_label_ = new QLabel(this);
    status_label_->setObjectName(QStringLiteral("status"));
    status_label_->setWordWrap(true);
    root->addWidget(status_label_);

    refresh();
}

namespace {
/// Public slots are 1 and 2 (as shown on the buttons); storage is 0-based. -1 when out of range.
int slotIndex(int slot) { return (slot == 1 || slot == 2) ? slot - 1 : -1; }
} // namespace

void MeasureDialog::setSelection(int slot, const km::Entity& e) {
    const int i = slotIndex(slot);
    if (i < 0) return;
    sel_[i] = e;
    recompute();
}

void MeasureDialog::clearSelection(int slot) {
    const int i = slotIndex(slot);
    if (i < 0) return;
    sel_[i].reset();
    recompute();
}

const std::optional<km::Entity>& MeasureDialog::selection(int slot) const {
    static const std::optional<km::Entity> none;
    const int i = slotIndex(slot);
    if (i < 0) return none;
    return sel_[i];
}

bool MeasureDialog::pickFromProvider(int slot) {
    const int i = slotIndex(slot);
    if (i < 0 || !provider_) return false;
    const auto items = provider_();
    // Slot k takes the k-th item of the current selection (pick order), so Select 1 / Select 2 mirror Fusion's inputs.
    if (items.size() <= static_cast<std::size_t>(i)) return false;
    setSelection(slot, items[static_cast<std::size_t>(i)]);
    return true;
}

void MeasureDialog::setPrecision(int decimals) {
    precision_->setValue(std::clamp(decimals, 0, kMaxPrecision));
}

int MeasureDialog::precision() const noexcept { return precision_->value(); }

QStringList MeasureDialog::resultLines() const {
    QStringList out;
    for (const auto& r : rows_) {
        const QString value = QString::number(r.value, 'f', precision());
        out << QStringLiteral("%1: %2 %3").arg(QString::fromStdString(r.label), value, prettyUnit(r.unit));
    }
    return out;
}

QString MeasureDialog::copyText() const { return resultLines().join(QLatin1Char('\n')); }

bool MeasureDialog::copyToClipboard() {
    const QString text = copyText();
    if (text.isEmpty()) return false;
    if (QClipboard* cb = QApplication::clipboard()) cb->setText(text);
    return true;
}

QString MeasureDialog::statusText() const { return status_; }

void MeasureDialog::recompute() {
    rows_.clear();
    status_.clear();
    if (sel_[0] && sel_[1]) {
        auto r = km::between(*sel_[0], *sel_[1]);
        if (r) rows_ = std::move(*r);
        else status_ = QString::fromStdString(r.error().message);
    } else if (sel_[0] || sel_[1]) {
        const km::Entity& only = sel_[0] ? *sel_[0] : *sel_[1];
        auto r = km::describe(only);
        if (r) rows_ = std::move(*r);
        else status_ = QString::fromStdString(r.error().message);
    }
    refresh();
    emit resultsChanged();
}

void MeasureDialog::refresh() {
    for (int slot = 0; slot < 2; ++slot)
        slotLabels_[slot]->setText(sel_[slot] ? entityLabel(*sel_[slot]) : QStringLiteral("-"));
    results_->clear();
    results_->addItems(resultLines());
    copyButton_->setEnabled(!rows_.empty());
    status_label_->setText(status_);
}

} // namespace iw::ui::inspect
