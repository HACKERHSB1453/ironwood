// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/sketch/sketch_palette.hpp"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QCoreApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QVBoxLayout>

namespace iw::ui {
namespace {

QString T(const char* text) { return QCoreApplication::translate("SketchPalette", text); }

} // namespace

SketchPalette::SketchPalette(QWidget* parent) : QWidget(parent) {
    auto* root = new QVBoxLayout(this);

    title_ = new QLabel(this);
    title_->setObjectName("sketchTitle");
    root->addWidget(title_);

    // Tools: exclusive checkable buttons. Shortcuts L, R, C, P and X are handled by the sketch mode.
    auto* toolRow = new QGridLayout;
    int toolCount = 0;
    auto* group = new QButtonGroup(this);
    group->setExclusive(true);
    const struct {
        SketchTool tool;
        const char* name;
        const char* objectName;
    } kTools[] = {
        {SketchTool::Select, "Select", "tool_select"},
        {SketchTool::Line, "Line (L)", "tool_line"},
        {SketchTool::Rectangle, "Rectangle (R)", "tool_rect"},
        {SketchTool::CenterRectangle, "Centre Rect", "tool_center_rect"},
        {SketchTool::Circle, "Circle (C)", "tool_circle"},
        {SketchTool::Point, "Point (P)", "tool_point"},
        {SketchTool::Arc, "Arc (A)", "tool_arc"},
        {SketchTool::Trim, "Trim (T)", "tool_trim"},
        {SketchTool::Offset, "Offset (O)", "tool_offset"},
        {SketchTool::Dimension, "Dimension (D)", "tool_dimension"},
    };
    for (const auto& t : kTools) {
        auto* b = new QToolButton(this);
        b->setText(T(t.name));
        b->setObjectName(QString::fromLatin1(t.objectName));
        b->setCheckable(true);
        group->addButton(b);
        toolRow->addWidget(b, toolCount / 4, toolCount % 4);
        ++toolCount;
        tools_.emplace_back(t.tool, b);
        connect(b, &QToolButton::clicked, this, [this, tool = t.tool] { emit toolRequested(tool); });
    }
    root->addLayout(toolRow);

    auto* editRow = new QHBoxLayout;
    auto* undo = new QPushButton(T("Undo"), this);
    undo->setObjectName("sketchUndo");
    auto* redo = new QPushButton(T("Redo"), this);
    redo->setObjectName("sketchRedo");
    connect(undo, &QPushButton::clicked, this, [this] { emit undoRequested(); });
    connect(redo, &QPushButton::clicked, this, [this] { emit redoRequested(); });
    editRow->addWidget(undo);
    editRow->addWidget(redo);
    root->addLayout(editRow);

    // Constraint toolbar: select entities, then click.
    auto* conGrid = new QGridLayout;
    const struct {
        sketch::ConstraintKind kind;
        const char* text;
        const char* objectName;
    } kCons[] = {
        {sketch::ConstraintKind::Horizontal, "H", "con_horizontal"},
        {sketch::ConstraintKind::Vertical, "V", "con_vertical"},
        {sketch::ConstraintKind::Coincident, "Coincident", "con_coincident"},
        {sketch::ConstraintKind::Tangent, "Tangent", "con_tangent"},
        {sketch::ConstraintKind::Equal, "Equal", "con_equal"},
        {sketch::ConstraintKind::Parallel, "Parallel", "con_parallel"},
        {sketch::ConstraintKind::Perpendicular, "Perpendicular", "con_perpendicular"},
        {sketch::ConstraintKind::Fix, "Fix / Unfix", "con_fix"},
        {sketch::ConstraintKind::Midpoint, "Midpoint", "con_midpoint"},
        {sketch::ConstraintKind::Concentric, "Concentric", "con_concentric"},
        {sketch::ConstraintKind::Collinear, "Collinear", "con_collinear"},
        {sketch::ConstraintKind::Symmetric, "Symmetry", "con_symmetric"},
        {sketch::ConstraintKind::PointOnObject, "Point on object", "con_pointonobject"},
    };
    int conCount = 0;
    for (const auto& c : kCons) {
        auto* b = new QToolButton(this);
        b->setText(T(c.text));
        b->setObjectName(QString::fromLatin1(c.objectName));
        conGrid->addWidget(b, conCount / 3, conCount % 3);
        ++conCount;
        if (c.kind == sketch::ConstraintKind::Fix) {
            connect(b, &QToolButton::clicked, this, [this] { emit fixToggleRequested(); });
        } else {
            connect(b, &QToolButton::clicked, this, [this, k = c.kind] { emit constraintRequested(k); });
        }
    }
    root->addLayout(conGrid);

    // Display toggles.
    auto emitOptions = [this] {
        SketchViewOptions o;
        o.profiles = profiles_->isChecked();
        o.points = points_->isChecked();
        o.constraints = constraints_->isChecked();
        emit optionsChanged(o);
    };
    profiles_ = new QCheckBox(T("Show profiles"), this);
    profiles_->setObjectName("toggleProfiles");
    points_ = new QCheckBox(T("Show points"), this);
    points_->setObjectName("togglePoints");
    constraints_ = new QCheckBox(T("Show constraints"), this);
    constraints_->setObjectName("toggleConstraints");
    for (QAbstractButton* b : {profiles_, points_, constraints_}) {
        b->setChecked(true);
        root->addWidget(b);
        connect(b, &QAbstractButton::toggled, this, [emitOptions] { emitOptions(); });
    }

    // Inference: snapping and auto-constraints. On by default.
    snap_ = new QCheckBox(T("Snap and auto-constraints"), this);
    snap_->setObjectName("toggleSnap");
    root->addWidget(snap_);
    connect(snap_, &QCheckBox::toggled, this, [this](bool on) { emit snapToggled(on); });

    dof_ = new QLabel(this);
    dof_->setObjectName("sketchDof");
    dof_->setWordWrap(true);
    root->addWidget(dof_);

    status_ = new QLabel(this);
    status_->setObjectName("sketchStatus");
    status_->setWordWrap(true);
    root->addWidget(status_);

    auto* finish = new QPushButton(T("Finish Sketch"), this);
    finish->setObjectName("finishSketch");
    connect(finish, &QPushButton::clicked, this, [this] { emit finishRequested(); });
    root->addWidget(finish);
    root->addStretch(1);

    setTool(SketchTool::Select);
}

QAbstractButton* SketchPalette::toolButton(SketchTool t) const {
    for (const auto& [tool, b] : tools_)
        if (tool == t) return b;
    return nullptr;
}

void SketchPalette::setTool(SketchTool t) {
    if (QAbstractButton* b = toolButton(t)) b->setChecked(true);
}

void SketchPalette::setOptions(const SketchViewOptions& o) {
    for (QAbstractButton* b : {profiles_, points_, constraints_}) b->blockSignals(true);
    profiles_->setChecked(o.profiles);
    points_->setChecked(o.points);
    constraints_->setChecked(o.constraints);
    for (QAbstractButton* b : {profiles_, points_, constraints_}) b->blockSignals(false);
}

void SketchPalette::setSnapEnabled(bool on) {
    snap_->blockSignals(true);
    snap_->setChecked(on);
    snap_->blockSignals(false);
}

bool SketchPalette::snapEnabled() const { return snap_->isChecked(); }

void SketchPalette::setConstraintStatus(int dof, ConstraintState state, const QString& message) {
    QString text = QCoreApplication::translate("SketchPalette", "DOF: %1").arg(dof);
    QString color;
    switch (state) {
    case ConstraintState::Fully:
        text += QCoreApplication::translate("SketchPalette", " - fully constrained");
        break;
    case ConstraintState::Conflicting:
        text += QCoreApplication::translate("SketchPalette", " - conflicting constraints");
        color = QStringLiteral("color: #eb4034;");
        break;
    case ConstraintState::Under:
        text += QCoreApplication::translate("SketchPalette", " - under-constrained");
        break;
    }
    if (!message.isEmpty() && state == ConstraintState::Conflicting) text += QStringLiteral("\n") + message;
    dof_->setStyleSheet(color);
    dof_->setText(text);
}

QString SketchPalette::dofText() const { return dof_->text(); }

void SketchPalette::setStatus(const QString& text) { status_->setText(text); }

void SketchPalette::setPlaneName(const QString& name) {
    title_->setText(T("Sketch on ") + name);
}

} // namespace iw::ui
