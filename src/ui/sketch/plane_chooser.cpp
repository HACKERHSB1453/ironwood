// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/sketch/plane_chooser.hpp"

#include <QCoreApplication>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace iw::ui {

sketch::SketchPlane originPlane(PlaneChoice c) {
    // Validated constructors only fail for zero or non-finite vectors, which these constants are not.
    switch (c) {
    case PlaneChoice::XY: return sketch::SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0}).value();
    case PlaneChoice::XZ: return sketch::SketchPlane::make({0, 0, 0}, {0, -1, 0}, {1, 0, 0}).value();
    case PlaneChoice::YZ: return sketch::SketchPlane::make({0, 0, 0}, {1, 0, 0}, {0, 1, 0}).value();
    }
    return sketch::SketchPlane::make({0, 0, 0}, {0, 0, 1}, {1, 0, 0}).value();
}

QString planeName(PlaneChoice c) {
    switch (c) {
    case PlaneChoice::XY: return QStringLiteral("XY");
    case PlaneChoice::XZ: return QStringLiteral("XZ");
    case PlaneChoice::YZ: return QStringLiteral("YZ");
    }
    return QStringLiteral("XY");
}

SketchPlaneChooser::SketchPlaneChooser(QWidget* parent) : QWidget(parent) {
    setObjectName("sketchPlaneChooser");
    auto* root = new QVBoxLayout(this);
    auto* hint = new QLabel(QCoreApplication::translate("SketchPlaneChooser", "Choose the plane to sketch on:"), this);
    hint->setWordWrap(true);
    root->addWidget(hint);

    const struct {
        PlaneChoice choice;
        const char* objectName;
    } kButtons[] = {
        {PlaneChoice::XY, "planeXY"},
        {PlaneChoice::XZ, "planeXZ"},
        {PlaneChoice::YZ, "planeYZ"},
    };
    for (const auto& b : kButtons) {
        auto* btn = new QPushButton(planeName(b.choice) + QStringLiteral(" plane"), this);
        btn->setObjectName(QString::fromLatin1(b.objectName));
        root->addWidget(btn);
        connect(btn, &QPushButton::clicked, this, [this, c = b.choice] { emit planeChosen(static_cast<int>(c)); });
    }
    auto* cancel = new QPushButton(QCoreApplication::translate("SketchPlaneChooser", "Cancel"), this);
    cancel->setObjectName("planeCancel");
    root->addWidget(cancel);
    connect(cancel, &QPushButton::clicked, this, [this] { emit cancelled(); });
    root->addStretch(1);
}

} // namespace iw::ui
