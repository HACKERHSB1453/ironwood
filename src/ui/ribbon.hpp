// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QWidget>

#include <string>
#include <vector>

#include "ui/command.hpp"

class QComboBox;
class QHBoxLayout;
class QTabBar;
class QStackedWidget;

namespace iw::ui {

class CommandActions;

/// Fusion-style command ribbon: workspace dropdown (DESIGN active, the others listed but disabled), a quick-access
/// row (Undo, Redo, pinned commands), and tabs SOLID / SURFACE / MESH / SHEET METAL / PLASTIC / UTILITIES / MANAGE.
/// Each tab shows its groups; every group has a dropdown listing all of its commands. Commands are drawn from the
/// registry through CommandActions, so unimplemented ones are visible but disabled with a "coming soon" tooltip.
class RibbonBar : public QWidget {
    Q_OBJECT
public:
    RibbonBar(const CommandRegistry& registry, CommandActions& actions, QWidget* parent = nullptr);

    static const std::vector<std::string>& tabNames();
    static const std::vector<std::string>& workspaceNames();

    QComboBox* workspaceBox() const noexcept { return workspace_; }
    QTabBar* tabBar() const noexcept { return tabs_; }
    std::string currentTab() const;
    void setCurrentTab(const std::string& tab);
    /// Group titles shown on a tab, in order.
    std::vector<std::string> groupTitles(const std::string& tab) const;
    /// Ids of the commands in the quick-access row (Undo, Redo, then pinned commands).
    std::vector<std::string> quickAccessIds() const;

    /// Replaces the pinned commands shown after Undo/Redo in the quick-access row.
    void setPinned(const std::vector<std::string>& ids);

signals:
    void workspaceChanged(const QString& workspace);

private:
    void buildTab(const std::string& tab, QWidget* page);
    void buildQuickAccess();

    const CommandRegistry& registry_;
    CommandActions& actions_;
    QComboBox* workspace_ = nullptr;
    QTabBar* tabs_ = nullptr;
    QStackedWidget* pages_ = nullptr;
    QHBoxLayout* quickLayout_ = nullptr;
    std::vector<std::string> quickIds_;
    std::vector<std::string> pinned_;
};

} // namespace iw::ui
