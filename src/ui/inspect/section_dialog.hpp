// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// B6: Section analysis dialog. Edits a Qt-free section::Model and publishes a RenderRequest on every change
// (to a sink function and to the requestChanged signal). The viewport hook itself is out of scope here.

#include <QWidget>

#include <functional>

#include "kernel/measure.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QPushButton;

namespace iw::ui::inspect {

class SectionDialog : public QWidget {
    Q_OBJECT
public:
    using RequestSink = std::function<void(const kernel::measure::section::RenderRequest&)>;

    explicit SectionDialog(QWidget* parent = nullptr);

    const kernel::measure::section::Model& model() const noexcept { return model_; }
    kernel::measure::section::RenderRequest request() const noexcept { return model_.request(); }

    /// Called with the current request after every change (and once when set).
    void setRequestSink(RequestSink sink);

    void setEnabled(bool on);       ///< Turns the section on/off (checkbox).
    void setNormalAxis(int axis);   ///< 0 = X, 1 = Y, 2 = Z (the normal combo).
    void setOrigin(const iw::Vec3& origin);
    void setOffset(double mm);
    void flip();                    ///< Toggles the kept side (flip button).

signals:
    void requestChanged(const iw::kernel::measure::section::RenderRequest& request);

private:
    void syncFromWidgets();
    void publish();

    kernel::measure::section::Model model_;
    RequestSink sink_;
    QCheckBox* enabled_ = nullptr;
    QComboBox* axis_ = nullptr;
    QDoubleSpinBox* ox_ = nullptr;
    QDoubleSpinBox* oy_ = nullptr;
    QDoubleSpinBox* oz_ = nullptr;
    QDoubleSpinBox* offset_ = nullptr;
    QPushButton* flip_ = nullptr;
    QComboBox* cap_ = nullptr;
    bool updating_ = false;
};

} // namespace iw::ui::inspect
