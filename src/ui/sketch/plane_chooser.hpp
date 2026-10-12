// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QWidget>

#include "sketch/sketch.hpp"

namespace iw::ui {

/// The origin planes a sketch can be created on. Face picking comes later (B3a part 2).
enum class PlaneChoice : int { XY = 0, XZ = 1, YZ = 2 };

/// World origin plane for a choice: XY normal +Z, XZ normal -Y (sketch up = +Z), YZ normal +X (sketch up = +Z).
sketch::SketchPlane originPlane(PlaneChoice c);
QString planeName(PlaneChoice c);

/// Small right-docked chooser shown by "Create Sketch": three origin planes and Cancel.
class SketchPlaneChooser : public QWidget {
    Q_OBJECT
public:
    explicit SketchPlaneChooser(QWidget* parent = nullptr);

signals:
    /// Emitted with a PlaneChoice value (int, for the signal/slot machinery).
    void planeChosen(int choice);
    void cancelled();
};

} // namespace iw::ui
