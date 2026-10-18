// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/ribbon.hpp"

#include <QAction>
#include <QComboBox>
#include <QEvent>
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
#include "ui/icons.hpp"

namespace iw::ui {
namespace {
constexpr const char* kWorkspace = "DESIGN";
constexpr const char* kQuickAccessTab = "";
constexpr int kRibbonIconPx = 32; // large icon above the label (Fusion-style)
constexpr int kQuickIconPx = 24;  // quick access row is a single compact line
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
    theme_ = iconThemeFor(palette());
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(4, 2, 4, 2);
    root->setSpacing(2);

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
    quickLayout_->setSpacing(2);
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
    applyIcons();
}

void RibbonBar::changeEvent(QEvent* event) {
    QWidget::changeEvent(event);
    if (event->type() == QEvent::PaletteChange) {
        const IconTheme now = iconThemeFor(palette());
        if (now != theme_) {
            theme_ = now;
            applyIcons();
        }
    }
}

void RibbonBar::applyIcons() {
    for (const Command& c : registry_.all()) {
        if (c.icon.empty()) continue;
        if (QAction* a = actions_.action(c.id)) a->setIcon(icon(c.icon, kRibbonIconPx, theme_));
    }
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
        b->setIconSize(QSize(kQuickIconPx, kQuickIconPx));
        b->setAutoRaise(true);
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
    row->setSpacing(4);
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
        col->setContentsMargins(3, 2, 3, 1);
        col->setSpacing(1);

        // Fusion layout: large icons with short labels on top, the group name and its dropdown arrow underneath.
        auto* buttons = new QHBoxLayout;
        buttons->setSpacing(2);
        const auto cmds = registry_.inGroup(kWorkspace, tab, g);
        for (const Command* c : cmds) {
            auto* b = new QToolButton(panel);
            QAction* action = actions_.action(c->id);
            b->setDefaultAction(action);
            if (action && !c->shortLabel.isEmpty()) {
                // The ribbon shows the short label ("Parameters"); the action keeps the full one for menus and S-box.
                // Connected after setDefaultAction, so this runs after the button copied the action's text.
                const QString shortText = c->shortLabel;
                b->setText(shortText);
                connect(action, &QAction::changed, b, [b, shortText] { b->setText(shortText); });
            }
            b->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
            b->setIconSize(QSize(kRibbonIconPx, kRibbonIconPx));
            b->setAutoRaise(true);
            b->setObjectName(QString::fromStdString("btn_" + c->id));
            buttons->addWidget(b);
        }
        auto* title = new QHBoxLayout;
        title->setSpacing(2);
        auto* label = new QLabel(QString::fromStdString(g), panel);
        label->setObjectName(QString::fromStdString("title_" + g));
        label->setAlignment(Qt::AlignCenter);
        title->addStretch(1);
        title->addWidget(label);
        auto* more = new QToolButton(panel);
        more->setText(QStringLiteral("▾")); // small down triangle: dropdown arrow
        more->setToolTip(tr("More %1 commands").arg(QString::fromStdString(g)));
        more->setPopupMode(QToolButton::InstantPopup);
        more->setAutoRaise(true);
        more->setObjectName(QString::fromStdString("more_" + g));
        auto* menu = new QMenu(more);
        for (const Command* c : cmds) menu->addAction(actions_.action(c->id));
        more->setMenu(menu);
        title->addWidget(more);
        title->addStretch(1);
        col->addLayout(buttons);
        col->addLayout(title);

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
