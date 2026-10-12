// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/command_dialog.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFont>
#include <QAbstractItemView>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace iw::ui {

// ---- values ---------------------------------------------------------------------------------------------------

Result<Quantity> evaluateValueText(std::string_view text, Dimension expected, const ParameterTable* params) {
    std::string_view trimmed = text;
    while (!trimmed.empty() && trimmed.front() == ' ') trimmed.remove_prefix(1);
    while (!trimmed.empty() && trimmed.back() == ' ') trimmed.remove_suffix(1);
    if (trimmed.empty()) return makeError(ErrorCode::InvalidArgument, "enter a value");

    auto expr = Expression::parse(trimmed);
    if (!expr) return std::unexpected(expr.error());

    const ParameterLookup lookup = [params](std::string_view name) -> Result<Quantity> {
        if (!params) return makeError(ErrorCode::NotFound, "unknown parameter: " + std::string(name));
        return params->evaluate(name);
    };
    auto value = expr->evaluate(lookup);
    if (!value) return std::unexpected(value.error());

    Quantity q = *value;
    if (q.dim.unitless() && !expected.unitless()) {
        if (expected == Dimension::Angle) q = Quantity::radians(q.value * std::numbers::pi / 180.0);
        else q = Quantity{q.value, expected};   // bare length number = mm
    }
    if (q.dim != expected) {
        return makeError(ErrorCode::InvalidArgument,
                         "expected " + dimensionName(expected) + ", got " + dimensionName(q.dim));
    }
    return q;
}

// ---- DialogModel ----------------------------------------------------------------------------------------------

DialogModel::DialogModel(std::vector<InputSpec> specs, const ParameterTable* params)
    : specs_(std::move(specs)), params_(params) {
    for (const InputSpec& s : specs_) {
        texts_[s.name] = s.defaultText;
        if (s.kind == InputKind::Selection) selections_[s.name];
    }
}

const InputSpec* DialogModel::spec(std::string_view name) const {
    for (const InputSpec& s : specs_)
        if (s.name == name) return &s;
    return nullptr;
}

void DialogModel::setText(std::string_view name, std::string text) {
    if (spec(name)) texts_[std::string(name)] = std::move(text);
}

std::string DialogModel::text(std::string_view name) const {
    auto it = texts_.find(name);
    return it == texts_.end() ? std::string() : it->second;
}

void DialogModel::toggleSelection(std::string_view name, const std::string& item) {
    auto it = selections_.find(name);
    if (it == selections_.end()) return;
    auto& items = it->second;
    if (auto pos = std::find(items.begin(), items.end(), item); pos != items.end()) items.erase(pos);
    else items.push_back(item);
}

std::vector<std::string> DialogModel::selection(std::string_view name) const {
    auto it = selections_.find(name);
    return it == selections_.end() ? std::vector<std::string>{} : it->second;
}

void DialogModel::setSelection(std::string_view name, std::vector<std::string> items) {
    if (auto it = selections_.find(name); it != selections_.end()) it->second = std::move(items);
}

Result<Quantity> DialogModel::valueOf(std::string_view name) const {
    const InputSpec* s = spec(name);
    if (!s || s->kind != InputKind::Value)
        return makeError(ErrorCode::InvalidArgument, "not a value field: " + std::string(name));
    return evaluateValueText(text(name), s->dim, params_);
}

std::optional<Error> DialogModel::errorFor(std::string_view name) const {
    const InputSpec* s = spec(name);
    if (!s || s->kind != InputKind::Value) return std::nullopt;
    auto v = valueOf(name);
    if (v) return std::nullopt;
    return v.error();
}

bool DialogModel::canAccept() const {
    for (const InputSpec& s : specs_)
        if (s.kind == InputKind::Value && errorFor(s.name)) return false;
    return true;
}

Result<DialogModel::Snapshot> DialogModel::snapshot() const {
    Snapshot snap;
    for (const InputSpec& s : specs_) {
        switch (s.kind) {
        case InputKind::Value: {
            auto v = valueOf(s.name);
            if (!v) return std::unexpected(v.error());
            snap.values[s.name] = *v;
            break;
        }
        case InputKind::Dropdown:
        case InputKind::Checkbox:
            snap.choices[s.name] = text(s.name);
            break;
        case InputKind::Selection:
            snap.selections[s.name] = selection(s.name);
            break;
        }
    }
    return snap;
}

// ---- LivePreview ----------------------------------------------------------------------------------------------

LivePreview::LivePreview(JobSystem& jobs, PreviewFn fn, std::function<void(const PreviewResult&)> onResult)
    : fn_(std::move(fn)), onResult_(std::move(onResult)), latest_(jobs) {}

void LivePreview::request(DialogModel::Snapshot snap) {
    const std::uint64_t seq = ++seq_;
    if (!fn_) return;
    latest_.submit(Priority::High, [seq, snap = std::move(snap), fn = fn_](CancelToken token) -> Result<PreviewResult> {
        if (token.cancelled()) return makeError(ErrorCode::Cancelled, "superseded");
        auto r = fn(snap, token);
        if (token.cancelled()) return makeError(ErrorCode::Cancelled, "superseded");
        if (!r) return PreviewResult{seq, r.error().message, true};
        return PreviewResult{seq, *r, false};
    });
}

bool LivePreview::poll() {
    JobHandle<PreviewResult> h = latest_.latest();
    if (!h.valid() || !h.ready()) return false;
    const Result<PreviewResult>& r = h.wait(); // ready: returns immediately
    if (!r || r->seq <= delivered_) return false;
    delivered_ = r->seq;
    if (onResult_) onResult_(*r);
    return true;
}

// ---- CommandPanel ---------------------------------------------------------------------------------------------

CommandPanel::CommandPanel(CommandDialogSpec spec, const ParameterTable* params, JobSystem& jobs, QWidget* parent)
    : QWidget(parent),
      spec_(std::move(spec)),
      model_(spec_.inputs, params),
      preview_(jobs, spec_.preview, [this](const PreviewResult& r) {
          if (previewLabel_) previewLabel_->setText(QString::fromStdString(r.text));
      }) {
    setAttribute(Qt::WA_DeleteOnClose, false);
    buildForm();
    pollTimer_ = new QTimer(this);
    pollTimer_->setInterval(16);
    connect(pollTimer_, &QTimer::timeout, this, [this] { pollPreview(); });
    pollTimer_->start();
    refreshValidation();
    requestPreview();
}

void CommandPanel::buildForm() {
    auto* root = new QVBoxLayout(this);
    auto* title = new QLabel(spec_.title, this);
    title->setObjectName("panelTitle");
    QFont f = title->font();
    f.setBold(true);
    title->setFont(f);
    root->addWidget(title);

    auto* form = new QFormLayout;
    for (const InputSpec& s : spec_.inputs) {
        auto* field = new QWidget(this);
        auto* col = new QVBoxLayout(field);
        col->setContentsMargins(0, 0, 0, 0);
        switch (s.kind) {
        case InputKind::Value: {
            auto* edit = new QLineEdit(QString::fromStdString(s.defaultText), field);
            edit->setObjectName(QString::fromStdString("input_" + s.name));
            valueEdits_[s.name] = edit;
            auto* err = new QLabel(field);
            err->setStyleSheet(QStringLiteral("color: #c0392b;"));
            fieldErrors_[s.name] = err;
            connect(edit, &QLineEdit::textChanged, this, [this, name = s.name](const QString& t) {
                model_.setText(name, t.toStdString());
                onFieldChanged(name);
            });
            col->addWidget(edit);
            col->addWidget(err);
            break;
        }
        case InputKind::Dropdown: {
            auto* box = new QComboBox(field);
            for (const QString& o : s.options) box->addItem(o);
            box->setCurrentText(QString::fromStdString(s.defaultText));
            dropdowns_[s.name] = box;
            connect(box, &QComboBox::currentTextChanged, this, [this, name = s.name](const QString& t) {
                model_.setText(name, t.toStdString());
                onFieldChanged(name);
            });
            col->addWidget(box);
            break;
        }
        case InputKind::Checkbox: {
            auto* check = new QCheckBox(s.label, field);
            check->setChecked(s.defaultText == "1");
            checks_[s.name] = check;
            connect(check, &QCheckBox::toggled, this, [this, name = s.name](bool on) {
                model_.setText(name, on ? "1" : "0");
                onFieldChanged(name);
            });
            col->addWidget(check);
            break;
        }
        case InputKind::Selection: {
            auto* filter = new QLineEdit(field);
            filter->setPlaceholderText(tr("Filter %1").arg(s.label));
            auto* list = new QListWidget(field);
            list->setSelectionMode(QAbstractItemView::MultiSelection);
            for (const QString& o : s.options) list->addItem(o);
            selectionLists_[s.name] = list;
            connect(filter, &QLineEdit::textChanged, this, [list](const QString& f) {
                for (int i = 0; i < list->count(); ++i)
                    list->item(i)->setHidden(!f.isEmpty() && !list->item(i)->text().contains(f, Qt::CaseInsensitive));
            });
            connect(list, &QListWidget::itemSelectionChanged, this, [this, name = s.name, list] {
                std::vector<std::string> chosen;
                for (QListWidgetItem* it : list->selectedItems()) chosen.push_back(it->text().toStdString());
                model_.setSelection(name, std::move(chosen));
                onFieldChanged(name);
            });
            col->addWidget(filter);
            col->addWidget(list);
            break;
        }
        }
        if (s.kind == InputKind::Checkbox) form->addRow(field);
        else form->addRow(s.label, field);
    }
    root->addLayout(form);

    previewLabel_ = new QLabel(this);
    previewLabel_->setObjectName("panelPreview");
    root->addWidget(previewLabel_);
    errorLabel_ = new QLabel(this);
    errorLabel_->setStyleSheet(QStringLiteral("color: #c0392b;"));
    root->addWidget(errorLabel_);
    root->addStretch(1);

    auto* buttons = new QHBoxLayout;
    auto* cancel = new QPushButton(tr("Cancel"), this);
    okButton_ = new QPushButton(tr("OK"), this);
    okButton_->setDefault(true);
    buttons->addStretch(1);
    buttons->addWidget(cancel);
    buttons->addWidget(okButton_);
    root->addLayout(buttons);
    connect(okButton_, &QPushButton::clicked, this, [this] { accept(); });
    connect(cancel, &QPushButton::clicked, this, [this] { reject(); });
}

void CommandPanel::setText(std::string_view name, const QString& text) {
    if (auto it = valueEdits_.find(name); it != valueEdits_.end()) {
        it->second->setText(text); // emits textChanged -> model + preview
        return;
    }
    model_.setText(name, text.toStdString());
    onFieldChanged(std::string(name));
}

void CommandPanel::onFieldChanged(const std::string& name) {
    (void)name;
    refreshValidation();
    requestPreview();
}

void CommandPanel::refreshValidation() {
    for (const InputSpec& s : spec_.inputs) {
        if (s.kind != InputKind::Value) continue;
        auto err = model_.errorFor(s.name);
        fieldErrors_[s.name]->setText(err ? QString::fromStdString(err->message) : QString());
    }
    okButton_->setEnabled(model_.canAccept());
}

void CommandPanel::requestPreview() {
    if (!spec_.preview) return;
    if (!model_.canAccept()) {
        previewLabel_->clear();
        return;
    }
    auto snap = model_.snapshot();
    if (snap) preview_.request(std::move(*snap));
}

void CommandPanel::pollPreview() { preview_.poll(); }

QString CommandPanel::previewText() const { return previewLabel_ ? previewLabel_->text() : QString(); }
QString CommandPanel::errorText() const { return errorLabel_ ? errorLabel_->text() : QString(); }
bool CommandPanel::okEnabled() const { return okButton_ && okButton_->isEnabled(); }

void CommandPanel::accept() {
    auto snap = model_.snapshot();
    if (!snap) {
        errorLabel_->setText(QString::fromStdString(snap.error().message));
        return;
    }
    if (spec_.accept) {
        if (auto r = spec_.accept(*snap); !r) {
            errorLabel_->setText(QString::fromStdString(r.error().message));
            return;
        }
    }
    errorLabel_->clear();
    emit finished(true);
}

void CommandPanel::reject() { emit finished(false); }

} // namespace iw::ui
