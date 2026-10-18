// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/timeline/feature_info.hpp"

#include <map>
#include <set>

namespace iw::ui::timeline {

std::string kindTitle(std::string_view kind) {
    if (kind == "sketch") return "Sketch";
    if (kind == "extrude") return "Extrude";
    if (kind == "revolve") return "Revolve";
    if (kind == "fillet") return "Fillet";
    if (kind == "chamfer") return "Chamfer";
    if (kind == "hole") return "Hole";
    if (kind == "move") return "Move";
    if (kind == "plane") return "Plane";
    if (kind == "axis") return "Axis";
    if (kind == "point") return "Point";
    return std::string(kind);
}

IconFamily iconFamilyOf(std::string_view kind) {
    if (kind == "sketch") return IconFamily::Sketch;
    if (kind == "plane" || kind == "axis" || kind == "point") return IconFamily::Construction;
    if (kind == "fillet" || kind == "chamfer" || kind == "hole" || kind == "move") return IconFamily::Edit;
    return IconFamily::Solid;
}

std::vector<std::string> displayNames(const recompute::Timeline& tl) {
    std::map<std::string, int> counts;
    std::vector<std::string> out;
    out.reserve(tl.size());
    for (std::size_t i = 0; i < tl.size(); ++i) {
        const recompute::Feature& f = *tl.at(i);
        const std::string title = kindTitle(f.kind());
        if (!f.name().empty()) {
            out.push_back(f.name());
            ++counts[title];
            continue;
        }
        out.push_back(title + " " + std::to_string(++counts[title]));
    }
    return out;
}

std::string timelineIconName(std::string_view kind) {
    static const std::set<std::string_view, std::less<>> kKinds = {
        "sketch", "extrude", "revolve", "fillet", "chamfer", "hole", "move", "plane", "axis", "point"};
    return kKinds.contains(kind) ? "timeline_" + std::string(kind) : std::string("missing");
}

std::optional<BodyId> producedBody(const recompute::Feature& f) {
    const auto bodies = f.writesBodies();
    if (bodies.empty()) return std::nullopt;
    return bodies.front();
}

std::optional<std::size_t> indexOfFeature(const recompute::Timeline& tl, FeatureId id) { return tl.indexOf(id); }

} // namespace iw::ui::timeline
