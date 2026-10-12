// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/sketch/sketch_palette.hpp"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QCheckBox>
#include <QDoubleSpinBox>
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
    auto* toolRow = new QHBoxLayout;
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
    };
    for (const auto& t : kTools) {
        auto* b = new QToolButton(this);
        b->setText(T(t.name));
        b->setObjectName(QString::fromLatin1(t.objectName));
        b->setCheckable(true);
        group->addButton(b);
        toolRow->addWidget(b);
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

    // Inference: snapping and auto-constraints. Off by default (unassisted drawing is literal).
    snap_ = new QCheckBox(T("Snap and auto-constraints"), this);
    snap_->setObjectName("toggleSnap");
    root->addWidget(snap_);
    connect(snap_, &QCheckBox::toggled, this, [this](bool on) { emit snapToggled(on); });

    auto* offsetRow = new QHBoxLayout;
    offsetRow->addWidget(new QLabel(T("Offset (mm)"), this));
    offset_ = new QDoubleSpinBox(this);
    offset_->setObjectName("offsetDistance");
    offset_->setRange(-1000.0, 1000.0);
    offset_->setDecimals(3);
    offset_->setValue(2.0);
    connect(offset_, &QDoubleSpinBox::valueChanged, this, [this](double v) { emit offsetDistanceChanged(v); });
    offsetRow->addWidget(offset_);
    root->addLayout(offsetRow);

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

void SketchPalette::setOffsetDistance(double mm) {
    offset_->blockSignals(true);
    offset_->setValue(mm);
    offset_->blockSignals(false);
}

double SketchPalette::offsetDistance() const { return offset_->value(); }

void SketchPalette::setStatus(const QString& text) { status_->setText(text); }

void SketchPalette::setPlaneName(const QString& name) {
    title_->setText(T("Sketch on ") + name);
}

} // namespace iw::ui
