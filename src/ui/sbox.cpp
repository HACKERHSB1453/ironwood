// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/sbox.hpp"

#include "ui/icons.hpp"

#include <QApplication>
#include <QKeyEvent>
#include <QLineEdit>
#include <QListWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <vector>

#include "ui/fuzzy.hpp"

namespace iw::ui {
namespace {
constexpr int kIdRole = Qt::UserRole + 1;
}

CommandPalette::CommandPalette(const CommandRegistry& registry, QWidget* parent)
    : QWidget(parent, Qt::Widget), registry_(registry) {
    setObjectName("sboxPalette");
    setAutoFillBackground(true);
    auto* layout = new QVBoxLayout(this);
    edit_ = new QLineEdit(this);
    edit_->setObjectName("sboxSearch");
    edit_->setPlaceholderText(tr("Search commands"));
    list_ = new QListWidget(this);
    list_->setObjectName("sboxList");
    layout->addWidget(edit_);
    layout->addWidget(list_);
    connect(edit_, &QLineEdit::textChanged, this, [this](const QString&) { rebuild(); });
    connect(edit_, &QLineEdit::returnPressed, this, [this] {
        if (QApplication::keyboardModifiers() & Qt::ControlModifier) pinCurrent();
        else chooseCurrent();
    });
    connect(list_, &QListWidget::itemActivated, this, [this](QListWidgetItem*) { chooseCurrent(); });
    setVisible(false);
    rebuild();
}

void CommandPalette::open() {
    edit_->clear();
    rebuild();
    show();
    raise();
    edit_->setFocus();
}

void CommandPalette::setQuery(const QString& query) { edit_->setText(query); }

QString CommandPalette::query() const { return edit_->text(); }

void CommandPalette::setRecent(std::vector<std::string> ids) {
    if (ids.size() > kMaxRecent) ids.resize(kMaxRecent);
    recent_ = std::move(ids);
    rebuild();
}

void CommandPalette::rebuild() {
    list_->clear();
    for (const Command* c : rankCommands(registry_, edit_->text(), recent_)) {
        QString text = c->label;
        const QKeySequence seq = registry_.shortcutOf(c->id);
        if (!seq.isEmpty()) text += QStringLiteral("    %1").arg(seq.toString(QKeySequence::NativeText));
        if (!c->implemented()) text += QStringLiteral("    ") + tr("coming soon");
        auto* item = new QListWidgetItem(text, list_);
        if (!c->icon.empty()) item->setIcon(icon(c->icon, 16, iconThemeFor(palette())));
        item->setData(kIdRole, QString::fromStdString(c->id));
        if (!c->implemented()) item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
    }
    if (list_->count() > 0) {
        // Highlight the first enabled row, else the first row (placeholders can still be pinned).
        int pick = 0;
        for (int i = 0; i < list_->count(); ++i) {
            if (list_->item(i)->flags() & Qt::ItemIsEnabled) {
                pick = i;
                break;
            }
        }
        list_->setCurrentRow(pick);
    }
}

std::vector<std::string> CommandPalette::results() const {
    std::vector<std::string> out;
    for (int i = 0; i < list_->count(); ++i)
        out.push_back(list_->item(i)->data(kIdRole).toString().toStdString());
    return out;
}

void CommandPalette::setCurrentRow(int row) { list_->setCurrentRow(row); }

int CommandPalette::currentRow() const { return list_->currentRow(); }

std::string CommandPalette::currentId() const {
    QListWidgetItem* item = list_->currentItem();
    return item ? item->data(kIdRole).toString().toStdString() : std::string();
}

bool CommandPalette::chooseCurrent() {
    QListWidgetItem* item = list_->currentItem();
    if (!item || !(item->flags() & Qt::ItemIsEnabled)) return false;
    const QString id = item->data(kIdRole).toString();
    // Recent list: most recent first, without duplicates.
    const std::string key = id.toStdString();
    std::erase(recent_, key);
    recent_.insert(recent_.begin(), key);
    if (recent_.size() > kMaxRecent) recent_.resize(kMaxRecent);
    hide();
    emit commandChosen(id);
    return true;
}

bool CommandPalette::pinCurrent() {
    const std::string id = currentId();
    if (id.empty()) return false;
    emit pinRequested(QString::fromStdString(id));
    return true;
}

void CommandPalette::keyPressEvent(QKeyEvent* e) {
    switch (e->key()) {
    case Qt::Key_Escape: hide(); e->accept(); return;
    case Qt::Key_Down: setCurrentRow(std::min(currentRow() + 1, list_->count() - 1)); e->accept(); return;
    case Qt::Key_Up: setCurrentRow(std::max(currentRow() - 1, 0)); e->accept(); return;
    default: break;
    }
    QWidget::keyPressEvent(e);
}

} // namespace iw::ui
