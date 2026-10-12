// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QWidget>

#include <utility>
#include <vector>

#include "ui/sketch/session.hpp"
#include "ui/sketch/sketch_overlay.hpp"

class QAbstractButton;
class QLabel;
class QDoubleSpinBox;

namespace iw::ui {

/// Sketch palette (docked while in sketch mode): tool buttons, display toggles, a status line and Finish Sketch.
/// Pure view: it emits requests and never changes the sketch itself.
class SketchPalette : public QWidget {
    Q_OBJECT
public:
    explicit SketchPalette(QWidget* parent = nullptr);

    /// Shows the tool as checked without emitting toolRequested.
    void setTool(SketchTool t);
    /// Shows the toggles without emitting optionsChanged.
    void setOptions(const SketchViewOptions& o);
    void setSnapEnabled(bool on);
    bool snapEnabled() const;
    void setOffsetDistance(double mm);
    double offsetDistance() const;
    void setStatus(const QString& text);
    void setPlaneName(const QString& name);

signals:
    void toolRequested(SketchTool t);
    void optionsChanged(const SketchViewOptions& o);
    void finishRequested();
    void undoRequested();
    void redoRequested();
    void snapToggled(bool on);
    void offsetDistanceChanged(double mm);

private:
    QAbstractButton* toolButton(SketchTool t) const;
    std::vector<std::pair<SketchTool, QAbstractButton*>> tools_;
    QAbstractButton* profiles_ = nullptr;
    QAbstractButton* points_ = nullptr;
    QAbstractButton* constraints_ = nullptr;
    QAbstractButton* snap_ = nullptr;
    QDoubleSpinBox* offset_ = nullptr;
    QLabel* title_ = nullptr;
    QLabel* status_ = nullptr;
};

} // namespace iw::ui
