// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/parameters_dialog.hpp"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QVBoxLayout>

#include "core/expr.hpp"

namespace iw::ui {
namespace {
constexpr const char* kErrorStyle = "color: #c0392b;";
} // namespace

ParametersDialog::ParametersDialog(DocumentState& doc, QWidget* parent) : QWidget(parent), doc_(doc) {
    setObjectName("parametersDialog");
    setWindowFlag(Qt::Window);   // a floating dialog even when parented to the main window
    setWindowTitle(tr("Parameters"));
    setAttribute(Qt::WA_DeleteOnClose);
    resize(760, 420);

    auto* root = new QVBoxLayout(this);
    table_ = new QTableWidget(0, ColumnCount, this);
    table_->setObjectName("parametersTable");
    table_->setHorizontalHeaderLabels({tr("Name"), tr("Unit"), tr("Expression"), tr("Value"), tr("Comment"),
                                       tr("Favourite")});
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    root->addWidget(table_);

    error_ = new QLabel(this);
    error_->setObjectName("parametersError");
    error_->setStyleSheet(QString::fromLatin1(kErrorStyle));
    error_->setWordWrap(true);
    root->addWidget(error_);

    auto* buttons = new QHBoxLayout;
    auto* add = new QPushButton(tr("Add"), this);
    add->setObjectName("parametersAdd");
    auto* del = new QPushButton(tr("Delete"), this);
    del->setObjectName("parametersDelete");
    buttons->addWidget(add);
    buttons->addWidget(del);
    buttons->addStretch(1);
    root->addLayout(buttons);
    connect(add, &QPushButton::clicked, this, [this] { showResult(addParameter(freshName(), "10 mm")); });
    connect(del, &QPushButton::clicked, this, [this] {
        const int row = table_->currentRow();
        if (row < 0 || row >= static_cast<int>(doc_.params.size())) {
            showResult(makeError(ErrorCode::InvalidArgument, tr("Select a parameter to delete").toStdString()));
            return;
        }
        showResult(deleteParameter(doc_.params.parameters()[static_cast<std::size_t>(row)].name));
    });

    connect(table_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
        onItemChanged(item->row(), item->column());
    });
    refresh();
}

QString ParametersDialog::errorText() const { return error_->text(); }

void ParametersDialog::showResult(const Result<void>& r) {
    error_->setText(r ? QString() : QString::fromStdString(r.error().message));
    refresh();
}

std::string ParametersDialog::freshName() const {
    for (int i = 1;; ++i) {
        std::string name = "d" + std::to_string(i);
        if (!doc_.params.find(name)) return name;
    }
}

Result<void> ParametersDialog::addParameter(const std::string& name, const std::string& expression,
                                            const std::string& unit, const std::string& comment) {
    auto r = editParameters(doc_, "Add parameter " + name, [&](ParameterTable& t) {
        return t.add(name, expression, unit, comment);
    });
    showResult(r);
    return r;
}

Result<void> ParametersDialog::deleteParameter(const std::string& name) {
    auto r = editParameters(doc_, "Delete parameter " + name, [&](ParameterTable& t) { return t.remove(name); });
    showResult(r);
    return r;
}

Result<void> ParametersDialog::editCell(int row, int column, const QString& text) {
    if (row < 0 || row >= static_cast<int>(doc_.params.size()))
        return makeError(ErrorCode::NotFound, "no parameter in row " + std::to_string(row));
    const std::string name = doc_.params.parameters()[static_cast<std::size_t>(row)].name;
    const std::string value = text.trimmed().toStdString();

    Result<void> r;
    switch (column) {
    case Name:
        r = editParameters(doc_, "Rename parameter", [&](ParameterTable& t) { return t.rename(name, value); });
        break;
    case Unit:
        r = editParameters(doc_, "Change unit", [&](ParameterTable& t) { return t.setUnit(name, value); });
        break;
    case Expression:
        r = editParameters(doc_, "Edit parameter " + name, [&](ParameterTable& t) {
            return t.setExpression(name, value);
        });
        break;
    case Comment:
        r = editParameters(doc_, "Edit comment", [&](ParameterTable& t) { return t.setComment(name, value); });
        break;
    case Favourite: {
        const bool on = value == "1" || value == "true";
        r = editParameters(doc_, "Favourite", [&](ParameterTable& t) { return t.setFavourite(name, on); });
        break;
    }
    default:
        r = makeError(ErrorCode::InvalidArgument, "the value is computed; edit the expression instead");
        break;
    }
    showResult(r);
    return r;
}

void ParametersDialog::onItemChanged(int row, int column) {
    QTableWidgetItem* item = table_->item(row, column);
    if (!item) return;
    // Errors are shown inline by editCell; the table is re-read either way.
    if (column == Favourite) {
        // The check state, not the text, carries the favourite flag.
        (void)editCell(row, column, item->checkState() == Qt::Checked ? QStringLiteral("1") : QStringLiteral("0"));
        return;
    }
    (void)editCell(row, column, item->text());
}

void ParametersDialog::refresh() {
    const QSignalBlocker block(table_);
    table_->setRowCount(0);
    const auto& params = doc_.params.parameters();
    table_->setRowCount(static_cast<int>(params.size()));
    for (int row = 0; row < static_cast<int>(params.size()); ++row) {
        const Parameter& p = params[static_cast<std::size_t>(row)];
        table_->setItem(row, Name, new QTableWidgetItem(QString::fromStdString(p.name)));
        table_->setItem(row, Unit, new QTableWidgetItem(QString::fromStdString(p.unit)));
        table_->setItem(row, Expression, new QTableWidgetItem(QString::fromStdString(p.expression.source())));

        auto* value = new QTableWidgetItem;
        value->setFlags(value->flags() & ~Qt::ItemIsEditable);
        if (auto q = doc_.params.evaluate(p.name)) {
            value->setText(QString::fromStdString(formatQuantity(*q)));
        } else {
            value->setText(QStringLiteral("error: ") + QString::fromStdString(q.error().message));
            value->setForeground(Qt::red);
        }
        table_->setItem(row, Value, value);

        table_->setItem(row, Comment, new QTableWidgetItem(QString::fromStdString(p.comment)));

        auto* fav = new QTableWidgetItem;
        fav->setFlags((fav->flags() | Qt::ItemIsUserCheckable) & ~Qt::ItemIsEditable);
        fav->setCheckState(p.favourite ? Qt::Checked : Qt::Unchecked);
        table_->setItem(row, Favourite, fav);
    }
}

} // namespace iw::ui
