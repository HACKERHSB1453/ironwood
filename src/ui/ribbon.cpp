// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/ribbon.hpp"

#include <QAction>
#include <QComboBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QStackedWidget>
#include <QStandardItemModel>
#include <QTabBar>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>

#include "ui/command_actions.hpp"

namespace iw::ui {
namespace {
constexpr const char* kWorkspace = "DESIGN";
constexpr const char* kQuickAccessTab = "";
} // namespace

const std::vector<std::string>& RibbonBar::tabNames() {
    static const std::vector<std::string> kTabs = {"SOLID",  "SURFACE", "MESH",      "SHEET METAL",
                                                   "PLASTIC", "UTILITIES", "MANAGE"};
    return kTabs;
}

const std::vector<std::string>& RibbonBar::workspaceNames() {
    static const std::vector<std::string> kWorkspaces = {"DESIGN", "RENDER", "SIMULATION", "MANUFACTURE", "DRAWING"};
    return kWorkspaces;
}

RibbonBar::RibbonBar(const CommandRegistry& registry, CommandActions& actions, QWidget* parent)
    : QWidget(parent), registry_(registry), actions_(actions) {
    setObjectName("ribbon");
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(4, 2, 4, 2);

    auto* top = new QHBoxLayout;
    workspace_ = new QComboBox(this);
    workspace_->setObjectName("workspaceBox");
    auto* model = new QStandardItemModel(workspace_);
    for (const std::string& w : workspaceNames()) {
        auto* item = new QStandardItem(QString::fromStdString(w));
        if (w != kWorkspace) {
            item->setEnabled(false);
            item->setToolTip(tr("coming soon"));
        }
        model->appendRow(item);
    }
    workspace_->setModel(model);
    workspace_->setCurrentIndex(0);
    top->addWidget(workspace_);

    auto* quickFrame = new QWidget(this);
    quickFrame->setObjectName("quickAccess");
    quickLayout_ = new QHBoxLayout(quickFrame);
    quickLayout_->setContentsMargins(0, 0, 0, 0);
    top->addWidget(quickFrame, 1);
    root->addLayout(top);
    buildQuickAccess();

    tabs_ = new QTabBar(this);
    tabs_->setObjectName("ribbonTabs");
    pages_ = new QStackedWidget(this);
    pages_->setObjectName("ribbonPages");
    for (const std::string& t : tabNames()) {
        tabs_->addTab(QString::fromStdString(t));
        auto* page = new QWidget(pages_);
        page->setObjectName(QString::fromStdString("tab_" + t));
        buildTab(t, page);
        pages_->addWidget(page);
    }
    connect(tabs_, &QTabBar::currentChanged, pages_, &QStackedWidget::setCurrentIndex);
    connect(workspace_, &QComboBox::activated, this, [this](int i) {
        if (i >= 0 && i < static_cast<int>(workspaceNames().size()) && workspaceNames()[i] == kWorkspace)
            emit workspaceChanged(QString::fromStdString(kWorkspace));
        else
            workspace_->setCurrentIndex(0);
    });
    root->addWidget(tabs_);
    root->addWidget(pages_);
    tabs_->setCurrentIndex(0);
}

void RibbonBar::buildQuickAccess() {
    while (QLayoutItem* item = quickLayout_->takeAt(0)) {
        delete item->widget();
        delete item;
    }
    quickIds_.clear();
    auto add = [this](const std::string& id) {
        QAction* a = actions_.action(id);
        if (!a) return;
        auto* b = new QToolButton(quickLayout_->parentWidget());
        b->setDefaultAction(a);
        b->setToolButtonStyle(Qt::ToolButtonIconOnly);
        b->setObjectName(QString::fromStdString("quick_" + id));
        quickLayout_->addWidget(b);
        quickIds_.push_back(id);
    };
    for (const Command* c : registry_.inTab(kWorkspace, kQuickAccessTab)) add(c->id);
    for (const std::string& id : pinned_) {
        if (std::find(quickIds_.begin(), quickIds_.end(), id) == quickIds_.end()) add(id);
    }
    quickLayout_->addStretch(1);
}

void RibbonBar::buildTab(const std::string& tab, QWidget* page) {
    auto* row = new QHBoxLayout(page);
    row->setContentsMargins(2, 2, 2, 2);
    row->setAlignment(Qt::AlignLeft);

    const auto groups = registry_.groups(kWorkspace, tab);
    if (groups.empty()) {
        auto* note = new QLabel(tr("%1 - coming soon").arg(QString::fromStdString(tab)), page);
        note->setEnabled(false);
        row->addWidget(note);
        row->addStretch(1);
        return;
    }
    for (const std::string& g : groups) {
        auto* panel = new QFrame(page);
        panel->setObjectName(QString::fromStdString("group_" + g));
        panel->setFrameShape(QFrame::StyledPanel);
        auto* col = new QVBoxLayout(panel);
        col->setContentsMargins(4, 2, 4, 2);

        auto* buttons = new QHBoxLayout;
        const auto cmds = registry_.inGroup(kWorkspace, tab, g);
        for (const Command* c : cmds) {
            auto* b = new QToolButton(panel);
            b->setDefaultAction(actions_.action(c->id));
            b->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
            b->setObjectName(QString::fromStdString("btn_" + c->id));
            buttons->addWidget(b);
        }
        auto* title = new QHBoxLayout;
        auto* label = new QLabel(QString::fromStdString(g), panel);
        label->setObjectName(QString::fromStdString("title_" + g));
        title->addWidget(label);
        auto* more = new QToolButton(panel);
        more->setText(QStringLiteral("▾")); // small down triangle: dropdown arrow
        more->setToolTip(tr("More %1 commands").arg(QString::fromStdString(g)));
        more->setPopupMode(QToolButton::InstantPopup);
        auto* menu = new QMenu(more);
        for (const Command* c : cmds) menu->addAction(actions_.action(c->id));
        more->setMenu(menu);
        title->addWidget(more);
        col->addLayout(title);
        col->addLayout(buttons);

        row->addWidget(panel);
    }
    row->addStretch(1);
}

std::string RibbonBar::currentTab() const {
    const int i = tabs_->currentIndex();
    return i >= 0 ? tabNames()[static_cast<std::size_t>(i)] : std::string();
}

void RibbonBar::setCurrentTab(const std::string& tab) {
    const auto& names = tabNames();
    auto it = std::find(names.begin(), names.end(), tab);
    if (it != names.end()) tabs_->setCurrentIndex(static_cast<int>(it - names.begin()));
}

std::vector<std::string> RibbonBar::groupTitles(const std::string& tab) const {
    return registry_.groups(kWorkspace, tab);
}

std::vector<std::string> RibbonBar::quickAccessIds() const { return quickIds_; }

void RibbonBar::setPinned(const std::vector<std::string>& ids) {
    pinned_ = ids;
    buildQuickAccess();
}

} // namespace iw::ui
