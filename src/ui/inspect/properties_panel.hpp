// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// B6: Properties panel for one solid body: volume, area, mass (from material density), centre of mass,
// bounding box and principal moments.

#include <QString>
#include <QWidget>

#include <optional>
#include <vector>

#include "kernel/measure.hpp"
#include "kernel/shape.hpp"

class QComboBox;
class QDoubleSpinBox;
class QLabel;

namespace iw::ui::inspect {

/// A material and its typical density (kg/m^3). Approximate handbook values for mass estimates only.
struct Material {
    QString name;
    double densityKgPerM3 = 0;
};

/// Built-in material table. The last entry, "Custom", takes the density from the spin box.
const std::vector<Material>& materials();

class PropertiesPanel : public QWidget {
    Q_OBJECT
public:
    explicit PropertiesPanel(QWidget* parent = nullptr);

    /// Shows `body`. A null body clears the panel and reports "No body" in the status line.
    void setBody(const kernel::Shape& body, const QString& name = {});
    /// Selects a material by name (no-op if unknown).
    bool setMaterial(const QString& name);
    /// Sets the custom density (kg/m^3) and selects "Custom".
    void setCustomDensity(double kgPerM3);

    const std::optional<kernel::measure::BodyProperties>& properties() const noexcept { return props_; }
    double density() const;
    QString materialName() const;
    QString statusText() const;

private:
    void recompute();
    void showProperties();

    kernel::Shape body_;
    QString name_;
    std::optional<kernel::measure::BodyProperties> props_;
    QString status_;

    QLabel* nameLabel_ = nullptr;
    QComboBox* material_ = nullptr;
    QDoubleSpinBox* customDensity_ = nullptr;
    QLabel* volume_ = nullptr;
    QLabel* area_ = nullptr;
    QLabel* mass_ = nullptr;
    QLabel* density_ = nullptr;
    QLabel* centre_ = nullptr;
    QLabel* bboxMin_ = nullptr;
    QLabel* bboxMax_ = nullptr;
    QLabel* bboxSize_ = nullptr;
    QLabel* moments_ = nullptr;
    QLabel* status_label_ = nullptr;
};

} // namespace iw::ui::inspect
