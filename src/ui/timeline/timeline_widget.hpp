// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Fusion-style timeline strip: one icon and short label per feature in order, a draggable rollback marker, failed
// features in red, warnings in yellow (message on hover), double-click to edit, right-click for the feature menu.
// The widget only paints and reports input; the shell owns the document and applies every change.

#include <QPoint>
#include <QWidget>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "core/ids.hpp"
#include "recompute/evaluator.hpp"
#include "ui/timeline/feature_info.hpp"

namespace iw::ui::timeline {

/// One item of the strip, as the shell derived it from the document and the last evaluation.
struct TimelineEntry {
    FeatureId id;
    std::string kind;
    QString label;
    IconFamily family = IconFamily::Solid;
    std::string icon;         ///< Icon name from the icon set ("timeline_extrude"); empty or unknown: family glyph.
    recompute::FeatureStatus::State state = recompute::FeatureStatus::State::Ok;
    bool warning = false;     ///< Ok, but with a Warning diagnostic (yellow).
    bool suppressed = false;
    QString message;          ///< Hover text: the error, or the warnings; empty when none.
};

class TimelineWidget : public QWidget {
    Q_OBJECT
public:
    explicit TimelineWidget(QWidget* parent = nullptr);

    /// Replaces the items. `activeCount` is the rollback marker (number of active features; == size() at the end).
    void setEntries(std::vector<TimelineEntry> entries, std::size_t activeCount);
    const std::vector<TimelineEntry>& entries() const noexcept { return entries_; }
    std::size_t activeCount() const noexcept { return activeCount_; }
    /// Feature under the pointer, if any.
    std::optional<FeatureId> hovered() const noexcept { return hovered_; }
    /// Screen rectangle of item `index` (widget coordinates), for tests and for anchoring menus.
    QRect itemRect(std::size_t index) const;
    /// Rollback marker handle rectangle (widget coordinates).
    QRect markerRect() const;
    /// Marker position (0..size) for a widget x coordinate: the nearest boundary between items.
    std::size_t markerAt(int x) const;

signals:
    /// The marker is being dragged: `activeCount` is the provisional position (the shell re-evaluates a preview).
    void rollbackPreview(std::size_t activeCount);
    /// The drag ended, or a click moved the marker: commit `activeCount` as one undoable change.
    void rollbackCommitted(std::size_t activeCount);
    void editRequested(FeatureId id);
    void contextRequested(FeatureId id, QPoint globalPos);
    /// Pointer entered an item (invalid optional when it left all items).
    void hoverChanged(std::optional<FeatureId> id);

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void mouseDoubleClickEvent(QMouseEvent* e) override;
    void leaveEvent(QEvent* e) override;
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

private:
    std::optional<std::size_t> itemAt(QPoint p) const;
    std::vector<TimelineEntry> entries_;
    std::size_t activeCount_ = 0;
    std::optional<FeatureId> hovered_;
    bool dragging_ = false;
    std::size_t dragValue_ = 0;
    std::size_t committedAtPress_ = 0;
};

} // namespace iw::ui::timeline
