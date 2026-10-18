// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/inspect/section_dialog.hpp"

#include <algorithm>
#include <array>
#include <iterator>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace iw::ui::inspect {

namespace km = kernel::measure;

namespace {

constexpr double kRange = 1.0e6; // mm

struct CapPreset {
    const char* name;
    std::array<float, 4> rgba;
};

const CapPreset kCaps[] = {
    {"Orange", {1.0f, 0.55f, 0.10f, 1.0f}},
    {"Red", {0.90f, 0.20f, 0.20f, 1.0f}},
    {"Blue", {0.20f, 0.45f, 0.90f, 1.0f}},
    {"Grey", {0.60f, 0.60f, 0.60f, 1.0f}},
};

QDoubleSpinBox* makeSpin(QWidget* parent, const char* name, const char* suffix) {
    auto* s = new QDoubleSpinBox(parent);
    s->setObjectName(QLatin1String(name));
    s->setRange(-kRange, kRange);
    s->setDecimals(3);
    s->setSuffix(QLatin1String(suffix));
    return s;
}

} // namespace

SectionDialog::SectionDialog(QWidget* parent) : QWidget(parent) {
    setWindowTitle(QStringLiteral("Section Analysis"));
    auto* root = new QVBoxLayout(this);

    enabled_ = new QCheckBox(QStringLiteral("Show section"), this);
    enabled_->setObjectName(QStringLiteral("enabled"));
    root->addWidget(enabled_);

    auto* form = new QFormLayout();
    axis_ = new QComboBox(this);
    axis_->setObjectName(QStringLiteral("normal"));
    axis_->addItems({QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")});
    axis_->setCurrentIndex(2);
    form->addRow(QStringLiteral("Plane normal"), axis_);

    ox_ = makeSpin(this, "originX", " mm");
    oy_ = makeSpin(this, "originY", " mm");
    oz_ = makeSpin(this, "originZ", " mm");
    form->addRow(QStringLiteral("Origin X"), ox_);
    form->addRow(QStringLiteral("Origin Y"), oy_);
    form->addRow(QStringLiteral("Origin Z"), oz_);

    offset_ = makeSpin(this, "offset", " mm");
    form->addRow(QStringLiteral("Offset"), offset_);

    cap_ = new QComboBox(this);
    cap_->setObjectName(QStringLiteral("cap"));
    for (const auto& c : kCaps) cap_->addItem(QLatin1String(c.name));
    form->addRow(QStringLiteral("Cap colour"), cap_);
    root->addLayout(form);

    auto* row = new QHBoxLayout();
    flip_ = new QPushButton(QStringLiteral("Flip"), this);
    flip_->setObjectName(QStringLiteral("flip"));
    row->addWidget(flip_);
    row->addStretch(1);
    root->addLayout(row);
    root->addStretch(1);

    (void)model_.setPlane({0, 0, 0}, {0, 0, 1});

    connect(enabled_, &QCheckBox::toggled, this, [this] { syncFromWidgets(); });
    connect(axis_, &QComboBox::currentIndexChanged, this, [this] { syncFromWidgets(); });
    connect(ox_, &QDoubleSpinBox::valueChanged, this, [this] { syncFromWidgets(); });
    connect(oy_, &QDoubleSpinBox::valueChanged, this, [this] { syncFromWidgets(); });
    connect(oz_, &QDoubleSpinBox::valueChanged, this, [this] { syncFromWidgets(); });
    connect(offset_, &QDoubleSpinBox::valueChanged, this, [this] { syncFromWidgets(); });
    connect(cap_, &QComboBox::currentIndexChanged, this, [this] { syncFromWidgets(); });
    connect(flip_, &QPushButton::clicked, this, [this] { flip(); });

    syncFromWidgets();
}

void SectionDialog::setRequestSink(RequestSink sink) {
    sink_ = std::move(sink);
    publish();
}

void SectionDialog::setEnabled(bool on) { enabled_->setChecked(on); }

void SectionDialog::setNormalAxis(int axis) { axis_->setCurrentIndex(std::clamp(axis, 0, 2)); }

void SectionDialog::setOrigin(const iw::Vec3& origin) {
    ox_->setValue(origin.x);
    oy_->setValue(origin.y);
    oz_->setValue(origin.z);
}

void SectionDialog::setOffset(double mm) { offset_->setValue(mm); }

void SectionDialog::flip() {
    model_.flip();
    publish();
}

void SectionDialog::syncFromWidgets() {
    if (updating_) return;
    updating_ = true;
    const iw::Vec3 normal = axis_->currentIndex() == 0   ? iw::Vec3{1, 0, 0}
                            : axis_->currentIndex() == 1 ? iw::Vec3{0, 1, 0}
                                                         : iw::Vec3{0, 0, 1};
    // Unit axis vectors are never degenerate, so setPlane cannot fail here.
    (void)model_.setPlane({ox_->value(), oy_->value(), oz_->value()}, normal);
    model_.setOffset(offset_->value());
    model_.setEnabled(enabled_->isChecked());
    const int cap = std::clamp(cap_->currentIndex(), 0, static_cast<int>(std::size(kCaps)) - 1);
    model_.setCapColour(kCaps[cap].rgba);
    updating_ = false;
    publish();
}

void SectionDialog::publish() {
    const km::section::RenderRequest r = model_.request();
    emit requestChanged(r);
    if (sink_) sink_(r);
}

} // namespace iw::ui::inspect
