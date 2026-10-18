// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/inspect/properties_panel.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QVBoxLayout>

#include "ui/inspect/measure_dialog.hpp"

namespace iw::ui::inspect {

namespace km = kernel::measure;

const std::vector<Material>& materials() {
    // Typical densities in kg/m^3 (handbook ballpark). Mass figures are estimates, not certified values.
    static const std::vector<Material> table = {
        {QStringLiteral("Steel (mild)"), 7850.0},
        {QStringLiteral("Stainless 304"), 8000.0},
        {QStringLiteral("Aluminium 6061"), 2700.0},
        {QStringLiteral("Titanium Ti-6Al-4V"), 4430.0},
        {QStringLiteral("Brass"), 8500.0},
        {QStringLiteral("Copper"), 8960.0},
        {QStringLiteral("ABS"), 1040.0},
        {QStringLiteral("PLA"), 1240.0},
        {QStringLiteral("Nylon PA6"), 1140.0},
        {QStringLiteral("Water"), 1000.0},
        {QStringLiteral("Custom"), 0.0},
    };
    return table;
}

namespace {

QString vec(const iw::Vec3& v, int decimals) {
    return QStringLiteral("X %1  Y %2  Z %3 mm")
        .arg(v.x, 0, 'f', decimals)
        .arg(v.y, 0, 'f', decimals)
        .arg(v.z, 0, 'f', decimals);
}

} // namespace

PropertiesPanel::PropertiesPanel(QWidget* parent) : QWidget(parent) {
    setWindowTitle(QStringLiteral("Properties"));
    auto* root = new QVBoxLayout(this);
    nameLabel_ = new QLabel(QStringLiteral("No body"), this);
    nameLabel_->setObjectName(QStringLiteral("name"));
    root->addWidget(nameLabel_);

    auto* matRow = new QFormLayout();
    material_ = new QComboBox(this);
    material_->setObjectName(QStringLiteral("material"));
    for (const auto& m : materials()) material_->addItem(m.name);
    customDensity_ = new QDoubleSpinBox(this);
    customDensity_->setObjectName(QStringLiteral("customDensity"));
    customDensity_->setRange(0.0, 30000.0);
    customDensity_->setDecimals(1);
    customDensity_->setSuffix(QStringLiteral(" kg/m³"));
    customDensity_->setValue(1000.0);
    customDensity_->setEnabled(false);
    matRow->addRow(QStringLiteral("Material"), material_);
    matRow->addRow(QStringLiteral("Custom density"), customDensity_);
    root->addLayout(matRow);

    auto* form = new QFormLayout();
    auto mk = [this](const char* name) {
        auto* l = new QLabel(QStringLiteral("-"), this);
        l->setObjectName(QLatin1String(name));
        l->setTextInteractionFlags(Qt::TextSelectableByMouse);
        return l;
    };
    volume_ = mk("volume");
    area_ = mk("area");
    mass_ = mk("mass");
    density_ = mk("density");
    centre_ = mk("centre");
    bboxMin_ = mk("bboxMin");
    bboxMax_ = mk("bboxMax");
    bboxSize_ = mk("bboxSize");
    moments_ = mk("moments");
    form->addRow(QStringLiteral("Volume"), volume_);
    form->addRow(QStringLiteral("Surface area"), area_);
    form->addRow(QStringLiteral("Density"), density_);
    form->addRow(QStringLiteral("Mass"), mass_);
    form->addRow(QStringLiteral("Centre of mass"), centre_);
    form->addRow(QStringLiteral("Principal moments"), moments_);
    form->addRow(QStringLiteral("Bounding box min"), bboxMin_);
    form->addRow(QStringLiteral("Bounding box max"), bboxMax_);
    form->addRow(QStringLiteral("Bounding box size"), bboxSize_);
    root->addLayout(form);

    status_label_ = new QLabel(this);
    status_label_->setObjectName(QStringLiteral("status"));
    status_label_->setWordWrap(true);
    root->addWidget(status_label_);
    root->addStretch(1);

    connect(material_, &QComboBox::currentIndexChanged, this, [this](int) {
        const bool custom = material_->currentText() == QStringLiteral("Custom");
        customDensity_->setEnabled(custom);
        recompute();
    });
    connect(customDensity_, &QDoubleSpinBox::valueChanged, this, [this](double) { recompute(); });
    recompute();
}

void PropertiesPanel::setBody(const kernel::Shape& body, const QString& name) {
    body_ = body;
    name_ = name.isEmpty() ? QStringLiteral("Body") : name;
    recompute();
}

bool PropertiesPanel::setMaterial(const QString& name) {
    const int idx = material_->findText(name);
    if (idx < 0) return false;
    material_->setCurrentIndex(idx);
    return true;
}

void PropertiesPanel::setCustomDensity(double kgPerM3) {
    setMaterial(QStringLiteral("Custom"));
    customDensity_->setValue(kgPerM3);
    recompute();
}

double PropertiesPanel::density() const {
    const QString name = material_->currentText();
    if (name == QStringLiteral("Custom")) return customDensity_->value();
    for (const auto& m : materials())
        if (m.name == name) return m.densityKgPerM3;
    return 0.0;
}

QString PropertiesPanel::materialName() const { return material_->currentText(); }

QString PropertiesPanel::statusText() const { return status_; }

void PropertiesPanel::recompute() {
    props_.reset();
    status_.clear();
    if (body_.isNull()) {
        status_ = QStringLiteral("No body selected.");
    } else {
        auto r = km::bodyProperties(body_, density());
        if (r) props_ = *r;
        else status_ = QString::fromStdString(r.error().message);
    }
    showProperties();
}

void PropertiesPanel::showProperties() {
    nameLabel_->setText(body_.isNull() ? QStringLiteral("No body") : name_);
    if (!props_) {
        for (QLabel* l : {volume_, area_, mass_, density_, centre_, bboxMin_, bboxMax_, bboxSize_, moments_})
            l->setText(QStringLiteral("-"));
        status_label_->setText(status_);
        return;
    }
    const km::BodyProperties& p = *props_;
    volume_->setText(QStringLiteral("%1 mm³").arg(p.volume, 0, 'f', 3));
    area_->setText(QStringLiteral("%1 mm²").arg(p.surfaceArea, 0, 'f', 3));
    density_->setText(p.density > 0 ? QStringLiteral("%1 kg/m³ (%2)").arg(p.density, 0, 'f', 1).arg(materialName())
                                    : QStringLiteral("not set"));
    mass_->setText(p.density > 0 ? QStringLiteral("%1 kg").arg(p.mass, 0, 'g', 6) : QStringLiteral("-"));
    centre_->setText(vec(p.centreOfMass, 3));
    bboxMin_->setText(vec(p.boundingBox.min, 3));
    bboxMax_->setText(vec(p.boundingBox.max, 3));
    bboxSize_->setText(vec(p.boundingBox.max - p.boundingBox.min, 3));
    moments_->setText(QStringLiteral("%1 / %2 / %3 kg·mm²")
                          .arg(p.principalMoments.x, 0, 'g', 6)
                          .arg(p.principalMoments.y, 0, 'g', 6)
                          .arg(p.principalMoments.z, 0, 'g', 6));
    status_label_->setText(status_);
}

} // namespace iw::ui::inspect
