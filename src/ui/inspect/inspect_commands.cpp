// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/inspect/inspect_commands.hpp"

#include <QKeySequence>
#include <QString>

#include "ui/inspect/measure_dialog.hpp"
#include "ui/inspect/properties_panel.hpp"
#include "ui/inspect/section_dialog.hpp"

namespace iw::ui::inspect {

namespace {

std::vector<kernel::measure::Entity> currentSelection(const InspectContext* ctx) {
    if (!ctx || !ctx->selection) return {};
    return ctx->selection();
}

std::optional<kernel::Shape> currentBody(const InspectContext* ctx) {
    if (!ctx || !ctx->activeBody) return std::nullopt;
    return ctx->activeBody();
}

Command makeCommand(std::string id, QString label, QString tooltip, QKeySequence shortcut, std::string icon) {
    Command c;
    c.id = std::move(id);
    c.label = std::move(label);
    c.tooltip = std::move(tooltip);
    c.shortcut = std::move(shortcut);
    c.icon = std::move(icon);
    c.tab = "SOLID";
    c.group = "INSPECT";
    c.repeatable = false;
    return c;
}

} // namespace

Result<void> registerCommands(CommandRegistry& registry, InspectContext& ctx) {
    InspectContext* self = &ctx;

    Command measure = makeCommand("measure", QStringLiteral("Measure"),
                                  QStringLiteral("Distance, angle, length, area, radius and mass properties"),
                                  QKeySequence(QStringLiteral("I")), "measure");
    measure.start = [self](CommandContext& cc) {
        if (!cc.showPanel) return;
        auto* dlg = new MeasureDialog([self] { return currentSelection(self); });
        dlg->setAttribute(Qt::WA_DeleteOnClose);
        cc.showPanel(dlg);
    };
    if (auto r = registry.add(std::move(measure)); !r) return std::unexpected(r.error());

    Command props = makeCommand("properties", QStringLiteral("Properties"),
                                QStringLiteral("Volume, area, mass, centre of mass and bounding box of a body"),
                                QKeySequence(), "properties");
    props.enabled = [self](const CommandContext&) { return currentBody(self).has_value(); };
    props.start = [self](CommandContext& cc) {
        if (!cc.showPanel) return;
        auto* panel = new PropertiesPanel();
        panel->setAttribute(Qt::WA_DeleteOnClose);
        if (auto body = currentBody(self)) panel->setBody(*body);
        cc.showPanel(panel);
    };
    if (auto r = registry.add(std::move(props)); !r) return std::unexpected(r.error());

    Command section = makeCommand("section_analysis", QStringLiteral("Section Analysis"),
                                  QStringLiteral("Cut the model with a plane and show the cross-section"),
                                  QKeySequence(), "section");
    section.start = [self](CommandContext& cc) {
        if (!cc.showPanel) return;
        auto* dlg = new SectionDialog();
        dlg->setAttribute(Qt::WA_DeleteOnClose);
        // Forward through the context so a sink installed after registration is still used.
        dlg->setRequestSink([self](const kernel::measure::section::RenderRequest& r) {
            if (self && self->sectionSink) self->sectionSink(r);
        });
        cc.showPanel(dlg);
    };
    if (auto r = registry.add(std::move(section)); !r) return std::unexpected(r.error());

    return {};
}

} // namespace iw::ui::inspect
