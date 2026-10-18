// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QWidget>

#include <utility>
#include <vector>

#include "ui/sketch/session.hpp"
#include "ui/sketch/sketch_overlay.hpp"

class QAbstractButton;
class QLabel;

namespace iw::ui {

/// Sketch palette (docked while in sketch mode): tool buttons, the constraint toolbar, display toggles, DOF and solver
/// status, and Finish Sketch.
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
    /// DOF readout, the fully-constrained state and the solver's message (shown red on conflicts).
    void setConstraintStatus(int dof, ConstraintState state, const QString& message);
    QString dofText() const;
    void setStatus(const QString& text);
    void setPlaneName(const QString& name);

signals:
    void toolRequested(SketchTool t);
    void optionsChanged(const SketchViewOptions& o);
    void finishRequested();
    void undoRequested();
    void redoRequested();
    void snapToggled(bool on);
    /// Constraint toolbar: apply `kind` to the current selection.
    void constraintRequested(sketch::ConstraintKind kind);
    /// Constraint toolbar: Fix / Unfix toggle on the current selection.
    void fixToggleRequested();

private:
    QAbstractButton* toolButton(SketchTool t) const;
    std::vector<std::pair<SketchTool, QAbstractButton*>> tools_;
    QAbstractButton* profiles_ = nullptr;
    QAbstractButton* points_ = nullptr;
    QAbstractButton* constraints_ = nullptr;
    QAbstractButton* snap_ = nullptr;
    QLabel* dof_ = nullptr;
    QLabel* title_ = nullptr;
    QLabel* status_ = nullptr;
};

} // namespace iw::ui
