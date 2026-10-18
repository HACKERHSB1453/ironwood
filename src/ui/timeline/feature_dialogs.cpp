// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/timeline/feature_dialogs.hpp"

#include <QListWidget>
#include <QPointF>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <map>
#include <numbers>
#include <tuple>
#include <utility>

#include "features/construction_features.hpp"
#include "features/edge_features.hpp"
#include "features/extrude_feature.hpp"
#include "features/feature_common.hpp"
#include "features/hole_feature.hpp"
#include "features/move_feature.hpp"
#include "features/revolve_feature.hpp"
#include "features/sketch_feature.hpp"
#include "kernel/ops.hpp"
#include "sketch/profile.hpp"
#include "tess/tessellate.hpp"
#include "render/viewport.hpp"
#include "ui/command_dialog.hpp"
#include "ui/manipulators.hpp"
#include "ui/sketch/view_math.hpp"
#include "ui/timeline/feature_info.hpp"

namespace iw::ui {
namespace {

using recompute::Feature;
using recompute::FeatureStatus;
using Snap = DialogModel::Snapshot;
/// Expression text of a field: raw text on the GUI thread, a formatted number for previews.
using TextFn = std::function<std::string(std::string_view)>;

// ---- choice tables built from the shown evaluation -----------------------------------------------------------

struct BodyChoice {
    QString label;
    BodyId body;
};
struct EdgeChoice {
    QString label;
    BodyId body;
    naming::ElementRef ref;
    std::uint32_t index = 0;
};
struct FaceChoice {
    QString label;
    BodyId body;
    naming::ElementRef ref;
    std::uint32_t index = 0;
};
struct ProfileChoice {
    QString label;       ///< What the dropdown shows: "Profile N", N counting the profiles of its sketch by centroid.
    std::string key;     ///< What the dropdown keeps (its item data): the sketch id and the internal profile name.
    FeatureId sketch;
    std::string profile; ///< Internal profile name (the sketch's own, stable across rebuilds).
    Vec3 anchor{};   ///< Centroid of the profile in world space (where the distance arrow starts).
    Vec3 normal{};   ///< Plane normal of the sketch (the one-side extrude direction).
};

/// Dropdown key of a profile: stable across evaluations (sketch id plus the profile's internal name).
std::string profileKeyOf(FeatureId sketch, const std::string& profile) {
    return std::to_string(sketch.value()) + "/" + profile;
}
struct AxisChoice {
    QString label;
    features::RevolveAxis axis;
    Vec3 origin{};        ///< A point of the axis in world space (valid when `located`).
    Vec3 direction{};     ///< Direction of the axis in world space (not necessarily unit).
    bool located = false; ///< The axis geometry is known (its feature or sketch line is evaluated).
};
struct PlaneChoice {
    QString label;
    FeatureId plane;
};

/// Light orange tint of a finished sketch profile (the Extrude / Revolve dialogs show every profile this way).
render::OverlayFace profileOverlay(const sketch::SketchPlane& plane, const sketch::SketchProfile& profile) {
    render::OverlayFace overlay;
    overlay.rgba = {1.0f, 0.62f, 0.18f, 0.35f};
    auto face = sketch::profileFace(plane, profile);
    if (!face) return overlay;
    auto mesh = tess::tessellate(*face, tess::TessParams{});
    if (!mesh) return overlay;
    for (std::uint32_t v : mesh->indices) {
        const std::size_t i = 3 * static_cast<std::size_t>(v);
        overlay.triangles.push_back(Vec3{mesh->positions[i], mesh->positions[i + 1], mesh->positions[i + 2]});
    }
    return overlay;
}

/// The triangles of a tessellated preview body, as one translucent overlay.
std::vector<render::OverlayFace> meshOverlay(const tess::MeshData& mesh, std::array<float, 4> rgba) {
    render::OverlayFace overlay;
    overlay.rgba = rgba;
    for (std::uint32_t v : mesh.indices) {
        const std::size_t i = 3 * static_cast<std::size_t>(v);
        overlay.triangles.push_back(Vec3{mesh.positions[i], mesh.positions[i + 1], mesh.positions[i + 2]});
    }
    return {std::move(overlay)};
}

/// Everything a dialog needs, captured by value (shared between the panel's callbacks and preview jobs).
struct DialogCtx {
    recompute::Timeline base;      ///< Timeline when the dialog opened.
    ParameterTable params;         ///< Parameter table when the dialog opened.
    std::shared_ptr<recompute::Evaluator> evaluator = std::make_shared<recompute::Evaluator>();
    FeatureId id;                  ///< Id of the feature the dialog creates (or edits).
    bool editing = false;
    BodyId newBody;                ///< Body a NewBody operation creates (the edited feature's body when editing).
    std::vector<BodyChoice> bodies;
    std::vector<EdgeChoice> edges;
    std::vector<EdgeChoice> vertices;   ///< Vertex choices (ref.type == Vertex); same shape as edges.
    std::vector<FaceChoice> faces;
    std::vector<ProfileChoice> profiles;
    std::vector<render::OverlayFace> profileOverlays;   ///< Same order as `profiles`.
    std::vector<AxisChoice> axes;
    std::vector<PlaneChoice> planes;
};

using MakeFn = std::function<Result<std::shared_ptr<const Feature>>(const Snap&, const TextFn&, const DialogCtx&)>;

enum class PickMode { None, Edges, Face, FaceUV };

/// What one command dialog is made of.
struct KindSpec {
    QString title;
    std::vector<InputSpec> inputs;
    MakeFn make;
    PickMode pick = PickMode::None;
    std::vector<QString> preselect;   ///< Selection items to select when the dialog opens (editing).
};

// ---- small constructors for InputSpec ------------------------------------------------------------------------

InputSpec valueField(const char* name, const char* label, Dimension dim, std::string text) {
    InputSpec s;
    s.name = name;
    s.label = QString::fromUtf8(label);
    s.kind = InputKind::Value;
    s.dim = dim;
    s.defaultText = std::move(text);
    return s;
}

InputSpec dropdownField(const char* name, const char* label, std::vector<QString> options, std::string def) {
    InputSpec s;
    s.name = name;
    s.label = QString::fromUtf8(label);
    s.kind = InputKind::Dropdown;
    s.options = std::move(options);
    s.defaultText = std::move(def);
    return s;
}

/// Profile dropdown: the keys are the profiles' keys, the texts their "Profile N" labels.
InputSpec profileField(const std::vector<ProfileChoice>& profiles, std::string def) {
    InputSpec s;
    s.name = "profile";
    s.label = QStringLiteral("Profile");
    s.kind = InputKind::Dropdown;
    s.defaultText = std::move(def);
    for (const ProfileChoice& p : profiles) {
        s.options.push_back(QString::fromStdString(p.key));
        s.optionLabels.push_back(p.label);
    }
    return s;
}

InputSpec checkField(const char* name, const char* label, bool on) {
    InputSpec s;
    s.name = name;
    s.label = QString::fromUtf8(label);
    s.kind = InputKind::Checkbox;
    s.defaultText = on ? "1" : "0";
    return s;
}

InputSpec selectionField(const char* name, const char* label, std::vector<QString> options) {
    InputSpec s;
    s.name = name;
    s.label = QString::fromUtf8(label);
    s.kind = InputKind::Selection;
    s.options = std::move(options);
    return s;
}

/// Mm / degrees text of a value field for previews (bare numbers are read in the field's dimension).
std::string formatted(const Snap& s, std::string_view name) {
    auto it = s.values.find(name);
    if (it == s.values.end()) return "0 mm";
    const Quantity q = it->second;
    char buf[64];
    if (q.dim == Dimension::Angle) std::snprintf(buf, sizeof buf, "%.17g deg", q.value * 180.0 / std::numbers::pi);
    else std::snprintf(buf, sizeof buf, "%.17g mm", q.value);
    return buf;
}

std::string choiceOf(const Snap& s, const char* name) {
    auto it = s.choices.find(name);
    return it == s.choices.end() ? std::string() : it->second;
}

std::vector<std::string> selectionOf(const Snap& s, const char* name) {
    auto it = s.selections.find(name);
    return it == s.selections.end() ? std::vector<std::string>{} : it->second;
}

Result<const ProfileChoice*> profileNamed(const DialogCtx& c, const std::string& key) {
    for (const ProfileChoice& p : c.profiles)
        if (p.key == key) return &p;
    return makeError(ErrorCode::InvalidArgument, "select a sketch profile");
}

Result<BodyId> bodyNamed(const DialogCtx& c, const std::string& label) {
    for (const BodyChoice& b : c.bodies)
        if (b.label.toStdString() == label) return b.body;
    return makeError(ErrorCode::InvalidArgument, "select a target body");
}

Result<std::vector<const EdgeChoice*>> edgesChosen(const DialogCtx& c, const Snap& s) {
    std::vector<const EdgeChoice*> out;
    for (const std::string& label : selectionOf(s, "edges")) {
        const EdgeChoice* hit = nullptr;
        for (const EdgeChoice& e : c.edges)
            if (e.label.toStdString() == label) hit = &e;
        if (!hit) return makeError(ErrorCode::InvalidArgument, "unknown edge selection: " + label);
        out.push_back(hit);
    }
    if (out.empty()) return makeError(ErrorCode::InvalidArgument, "select at least one edge");
    for (const EdgeChoice* e : out)
        if (e->body != out.front()->body) return makeError(ErrorCode::InvalidArgument, "the selected edges belong to different bodies");
    return out;
}

/// Extrude / Revolve operation and target body shared by both.
Result<std::pair<features::common::BodyOp, BodyId>> operationOf(const DialogCtx& c, const Snap& s) {
    const std::string op = choiceOf(s, "operation");
    features::common::BodyOp bop = features::common::BodyOp::NewBody;
    if (op == "Join") bop = features::common::BodyOp::Join;
    else if (op == "Cut") bop = features::common::BodyOp::Cut;
    else if (op == "Intersect") bop = features::common::BodyOp::Intersect;
    if (bop == features::common::BodyOp::NewBody) return std::pair{bop, c.newBody};
    auto body = bodyNamed(c, choiceOf(s, "target"));
    if (!body) return std::unexpected(body.error());
    return std::pair{bop, *body};
}

std::shared_ptr<const Feature> asConst(std::shared_ptr<Feature> f) { return f; }

// ---- the dialogs ---------------------------------------------------------------------------------------------

KindSpec extrudeSpec(const DialogCtx& c, const Feature* existing) {
    KindSpec k;
    k.title = c.editing ? QStringLiteral("Edit Extrude") : QStringLiteral("Extrude");
    const auto* ex = dynamic_cast<const features::ExtrudeFeature*>(existing);
    std::vector<QString> targets;
    for (const BodyChoice& b : c.bodies) targets.push_back(b.label);
    std::string profileDef = c.profiles.empty() ? std::string() : c.profiles.front().key;
    std::string extentDef = "One side", opDef = "New body", targetDef = targets.empty() ? std::string() : targets.front().toStdString();
    std::string distance = "10 mm", distance2 = "0 mm";
    if (ex) {
        const auto& p = ex->params();
        if (const auto* ref = std::get_if<features::SketchReferenceSource>(&p.source)) {
            for (const ProfileChoice& pc : c.profiles)
                if (pc.sketch == ref->sketch && pc.profile == ref->profile) profileDef = pc.key;
        }
        switch (p.extent) {
        case features::ExtrudeParams::Extent::OneSide: extentDef = "One side"; break;
        case features::ExtrudeParams::Extent::TwoSides: extentDef = "Two sides"; break;
        case features::ExtrudeParams::Extent::Symmetric: extentDef = "Symmetric"; break;
        }
        switch (p.operation) {
        case features::ExtrudeParams::Operation::NewBody: opDef = "New body"; break;
        case features::ExtrudeParams::Operation::Join: opDef = "Join"; break;
        case features::ExtrudeParams::Operation::Cut: opDef = "Cut"; break;
        case features::ExtrudeParams::Operation::Intersect: opDef = "Intersect"; break;
        }
        distance = p.distance;
        distance2 = p.distance2;
        for (const BodyChoice& b : c.bodies)
            if (b.body == p.body) targetDef = b.label.toStdString();
    }
    k.inputs = {
        profileField(c.profiles, profileDef),
        dropdownField("extent", "Extent", {"One side", "Two sides", "Symmetric"}, extentDef),
        valueField("distance", "Distance", Dimension::Length, distance),
        valueField("distance2", "Distance 2", Dimension::Length, distance2),
        dropdownField("operation", "Operation", {"New body", "Join", "Cut", "Intersect"}, opDef),
        dropdownField("target", "Target body", targets, targetDef),
    };
    k.make = [](const Snap& s, const TextFn& t, const DialogCtx& c) -> Result<std::shared_ptr<const Feature>> {
        auto prof = profileNamed(c, choiceOf(s, "profile"));
        if (!prof) return std::unexpected(prof.error());
        features::ExtrudeParams p;
        p.source = features::SketchReferenceSource{(*prof)->sketch, (*prof)->profile};
        const std::string ext = choiceOf(s, "extent");
        p.extent = ext == "Two sides" ? features::ExtrudeParams::Extent::TwoSides
                 : ext == "Symmetric" ? features::ExtrudeParams::Extent::Symmetric
                                      : features::ExtrudeParams::Extent::OneSide;
        p.distance = t("distance");
        p.distance2 = t("distance2");
        auto op = operationOf(c, s);
        if (!op) return std::unexpected(op.error());
        switch (op->first) {
        case features::common::BodyOp::NewBody: p.operation = features::ExtrudeParams::Operation::NewBody; break;
        case features::common::BodyOp::Join: p.operation = features::ExtrudeParams::Operation::Join; break;
        case features::common::BodyOp::Cut: p.operation = features::ExtrudeParams::Operation::Cut; break;
        case features::common::BodyOp::Intersect: p.operation = features::ExtrudeParams::Operation::Intersect; break;
        }
        p.body = op->second;
        return asConst(std::make_shared<features::ExtrudeFeature>(c.id, p));
    };
    return k;
}

/// Edge-based dialogs: Fillet and Chamfer.
KindSpec edgeSpec(const DialogCtx& c, const Feature* existing, bool chamfer) {
    KindSpec k;
    k.pick = PickMode::Edges;
    std::vector<QString> edgeLabels;
    for (const EdgeChoice& e : c.edges) edgeLabels.push_back(e.label);
    k.title = chamfer ? (c.editing ? QStringLiteral("Edit Chamfer") : QStringLiteral("Chamfer"))
                      : (c.editing ? QStringLiteral("Edit Fillet") : QStringLiteral("Fillet"));
    const bool chainDef = true;
    if (chamfer) {
        std::string dist = "1 mm";
        bool chain = chainDef;
        const auto* ch = dynamic_cast<const features::ChamferFeature*>(existing);
        if (ch) {
            dist = ch->params().distance;
            chain = ch->params().tangentChain;
        }
        k.inputs = {selectionField("edges", "Edges", edgeLabels), valueField("distance", "Distance", Dimension::Length, dist),
                    checkField("chain", "Tangent chain", chain)};
        k.make = [chamfer](const Snap& s, const TextFn& t, const DialogCtx& c) -> Result<std::shared_ptr<const Feature>> {
            (void)chamfer;
            auto edges = edgesChosen(c, s);
            if (!edges) return std::unexpected(edges.error());
            features::ChamferParams p;
            p.body = (*edges)[0]->body;
            for (const EdgeChoice* e : *edges) p.edges.push_back(e->ref);
            p.mode = features::ChamferParams::Mode::Equal;
            p.distance = t("distance");
            p.tangentChain = choiceOf(s, "chain") == "1";
            return asConst(std::make_shared<features::ChamferFeature>(c.id, p));
        };
    } else {
        std::string radius = "1 mm";
        bool chain = chainDef;
        if (const auto* fi = dynamic_cast<const features::FilletFeature*>(existing)) {
            radius = fi->params().radius;
            chain = fi->params().tangentChain;
        }
        k.inputs = {selectionField("edges", "Edges", edgeLabels), valueField("radius", "Radius", Dimension::Length, radius),
                    checkField("chain", "Tangent chain", chain)};
        k.make = [](const Snap& s, const TextFn& t, const DialogCtx& c) -> Result<std::shared_ptr<const Feature>> {
            auto edges = edgesChosen(c, s);
            if (!edges) return std::unexpected(edges.error());
            features::FilletParams p;
            p.body = (*edges)[0]->body;
            for (const EdgeChoice* e : *edges) p.edges.push_back(e->ref);
            p.radius = t("radius");
            p.tangentChain = choiceOf(s, "chain") == "1";
            return asConst(std::make_shared<features::FilletFeature>(c.id, p));
        };
    }
    // Preselection of editing: the edge references the feature already holds.
    if (existing) {
        const std::vector<naming::ElementRef>* refs = nullptr;
        if (const auto* fi = dynamic_cast<const features::FilletFeature*>(existing)) refs = &fi->params().edges;
        if (const auto* ch = dynamic_cast<const features::ChamferFeature*>(existing)) refs = &ch->params().edges;
        if (refs)
            for (const naming::ElementRef& r : *refs)
                for (const EdgeChoice& e : c.edges)
                    if (e.ref == r) k.preselect.push_back(e.label);
    }
    return k;
}

KindSpec holeSpec(const DialogCtx& c, const Feature* existing) {
    KindSpec k;
    k.title = c.editing ? QStringLiteral("Edit Hole") : QStringLiteral("Hole");
    k.pick = PickMode::FaceUV;
    std::vector<QString> faceLabels;
    for (const FaceChoice& f : c.faces) faceLabels.push_back(f.label);
    features::HoleParams d;
    if (const auto* h = dynamic_cast<const features::HoleFeature*>(existing)) d = h->params();
    const char* kindDef = d.kind == kernel::HoleKind::Counterbore ? "Counterbore"
                        : d.kind == kernel::HoleKind::Countersink ? "Countersink" : "Simple";
    k.inputs = {
        selectionField("face", "Face", faceLabels),
        dropdownField("kind", "Type", {"Simple", "Counterbore", "Countersink"}, kindDef),
        valueField("diameter", "Diameter", Dimension::Length, d.diameter),
        valueField("depth", "Depth", Dimension::Length, d.depth),
        checkField("through", "Through all", d.through),
        valueField("u", "Position X", Dimension::Length, d.u),
        valueField("v", "Position Y", Dimension::Length, d.v),
        valueField("drill", "Drill angle", Dimension::Angle, d.drillAngle),
        valueField("cbd", "Counterbore dia", Dimension::Length, d.counterboreDiameter),
        valueField("cbdepth", "Counterbore depth", Dimension::Length, d.counterboreDepth),
        valueField("csd", "Countersink dia", Dimension::Length, d.countersinkDiameter),
        valueField("csangle", "Countersink angle", Dimension::Angle, d.countersinkAngle),
    };
    if (existing)
        for (const FaceChoice& f : c.faces)
            if (f.ref == d.face) k.preselect.push_back(f.label);
    k.make = [](const Snap& s, const TextFn& t, const DialogCtx& c) -> Result<std::shared_ptr<const Feature>> {
        const auto sel = selectionOf(s, "face");
        if (sel.empty()) return makeError(ErrorCode::InvalidArgument, "select a planar face");
        const FaceChoice* face = nullptr;
        for (const FaceChoice& f : c.faces)
            if (f.label.toStdString() == sel.front()) face = &f;
        if (!face) return makeError(ErrorCode::InvalidArgument, "unknown face selection");
        features::HoleParams p;
        p.body = face->body;
        p.face = face->ref;
        p.u = t("u");
        p.v = t("v");
        const std::string kind = choiceOf(s, "kind");
        p.kind = kind == "Counterbore" ? kernel::HoleKind::Counterbore
               : kind == "Countersink" ? kernel::HoleKind::Countersink : kernel::HoleKind::Simple;
        p.diameter = t("diameter");
        p.depth = t("depth");
        p.through = choiceOf(s, "through") == "1";
        p.drillAngle = t("drill");
        p.counterboreDiameter = t("cbd");
        p.counterboreDepth = t("cbdepth");
        p.countersinkDiameter = t("csd");
        p.countersinkAngle = t("csangle");
        return asConst(std::make_shared<features::HoleFeature>(c.id, p));
    };
    return k;
}

KindSpec revolveSpec(const DialogCtx& c, const Feature* existing) {
    KindSpec k;
    k.title = c.editing ? QStringLiteral("Edit Revolve") : QStringLiteral("Revolve");
    std::vector<QString> axes, targets;
    for (const AxisChoice& a : c.axes) axes.push_back(a.label);
    for (const BodyChoice& b : c.bodies) targets.push_back(b.label);
    std::string profileDef = c.profiles.empty() ? std::string() : c.profiles.front().key;
    std::string axisDef = axes.empty() ? std::string() : axes.front().toStdString();
    std::string extentDef = "Full", angle = "360 deg", angle2 = "0 deg", opDef = "New body";
    std::string targetDef = targets.empty() ? std::string() : targets.front().toStdString();
    if (const auto* rv = dynamic_cast<const features::RevolveFeature*>(existing)) {
        const auto& p = rv->params();
        if (const auto* ref = std::get_if<features::SketchReferenceSource>(&p.source))
            for (const ProfileChoice& pc : c.profiles)
                if (pc.sketch == ref->sketch && pc.profile == ref->profile) profileDef = pc.key;
        for (const AxisChoice& a : c.axes) {
            const bool same = a.axis.kind == p.axis.kind &&
                              (p.axis.kind == features::RevolveAxis::Kind::Construction ? a.axis.construction == p.axis.construction
                                                                             : a.axis.line == p.axis.line);
            if (same) axisDef = a.label.toStdString();
        }
        switch (p.extent) {
        case features::RevolveParams::Extent::Full: extentDef = "Full"; break;
        case features::RevolveParams::Extent::OneSide: extentDef = "One side"; break;
        case features::RevolveParams::Extent::TwoSides: extentDef = "Two sides"; break;
        case features::RevolveParams::Extent::Symmetric: extentDef = "Symmetric"; break;
        }
        angle = p.angle;
        angle2 = p.angle2;
        switch (p.operation) {
        case features::common::BodyOp::NewBody: opDef = "New body"; break;
        case features::common::BodyOp::Join: opDef = "Join"; break;
        case features::common::BodyOp::Cut: opDef = "Cut"; break;
        case features::common::BodyOp::Intersect: opDef = "Intersect"; break;
        }
        for (const BodyChoice& b : c.bodies)
            if (b.body == p.body) targetDef = b.label.toStdString();
    }
    k.inputs = {
        profileField(c.profiles, profileDef),
        dropdownField("axis", "Axis", axes, axisDef),
        dropdownField("extent", "Extent", {"Full", "One side", "Two sides", "Symmetric"}, extentDef),
        valueField("angle", "Angle", Dimension::Angle, angle),
        valueField("angle2", "Angle 2", Dimension::Angle, angle2),
        dropdownField("operation", "Operation", {"New body", "Join", "Cut", "Intersect"}, opDef),
        dropdownField("target", "Target body", targets, targetDef),
    };
    k.make = [](const Snap& s, const TextFn& t, const DialogCtx& c) -> Result<std::shared_ptr<const Feature>> {
        auto prof = profileNamed(c, choiceOf(s, "profile"));
        if (!prof) return std::unexpected(prof.error());
        const std::string axisLabel = choiceOf(s, "axis");
        const AxisChoice* axis = nullptr;
        for (const AxisChoice& a : c.axes)
            if (a.label.toStdString() == axisLabel) axis = &a;
        if (!axis) return makeError(ErrorCode::InvalidArgument, "select a revolve axis");
        features::RevolveParams p;
        p.source = features::SketchReferenceSource{(*prof)->sketch, (*prof)->profile};
        p.axis = axis->axis;
        const std::string ext = choiceOf(s, "extent");
        p.extent = ext == "One side" ? features::RevolveParams::Extent::OneSide
                 : ext == "Two sides" ? features::RevolveParams::Extent::TwoSides
                 : ext == "Symmetric" ? features::RevolveParams::Extent::Symmetric
                                      : features::RevolveParams::Extent::Full;
        p.angle = t("angle");
        p.angle2 = t("angle2");
        auto op = operationOf(c, s);
        if (!op) return std::unexpected(op.error());
        p.operation = op->first;
        p.body = op->second;
        return asConst(std::make_shared<features::RevolveFeature>(c.id, p));
    };
    return k;
}

KindSpec moveSpec(const DialogCtx& c, const Feature* existing) {
    KindSpec k;
    k.title = c.editing ? QStringLiteral("Edit Move") : QStringLiteral("Move/Copy");
    std::vector<QString> targets;
    for (const BodyChoice& b : c.bodies) targets.push_back(b.label);
    std::string bodyDef = targets.empty() ? std::string() : targets.front().toStdString();
    std::array<std::string, 3> tr{"0 mm", "0 mm", "0 mm"};
    std::string angle = "0 deg";
    if (const auto* mv = dynamic_cast<const features::MoveFeature*>(existing)) {
        for (const BodyChoice& b : c.bodies)
            if (b.body == mv->params().body) bodyDef = b.label.toStdString();
        tr = mv->params().translate;
        angle = mv->params().angle;
    }
    k.inputs = {
        dropdownField("body", "Body", targets, bodyDef),
        valueField("tx", "Move X", Dimension::Length, tr[0]),
        valueField("ty", "Move Y", Dimension::Length, tr[1]),
        valueField("tz", "Move Z", Dimension::Length, tr[2]),
        valueField("angle", "Rotate about Z", Dimension::Angle, angle),
    };
    k.make = [](const Snap& s, const TextFn& t, const DialogCtx& c) -> Result<std::shared_ptr<const Feature>> {
        auto body = bodyNamed(c, choiceOf(s, "body"));
        if (!body) return std::unexpected(body.error());
        features::MoveParams p;
        p.body = *body;
        p.copy = false;
        p.translate = {t("tx"), t("ty"), t("tz")};
        p.angle = t("angle");
        return asConst(std::make_shared<features::MoveFeature>(c.id, p));
    };
    return k;
}

KindSpec planeSpec(const DialogCtx& c, const Feature* existing) {
    KindSpec k;
    k.title = c.editing ? QStringLiteral("Edit Offset Plane") : QStringLiteral("Offset Plane");
    k.pick = PickMode::Face;
    std::vector<QString> faceLabels, planeLabels{QStringLiteral("(none)")};
    for (const FaceChoice& f : c.faces) faceLabels.push_back(f.label);
    for (const PlaneChoice& p : c.planes) planeLabels.push_back(p.label);
    std::string distance = "10 mm";
    QString planeDef = QStringLiteral("(none)");
    // Editing: prefilled from the feature (an offset plane; start() refuses the other kinds).
    if (const auto* pf = dynamic_cast<const features::ConstructionPlaneFeature*>(existing)) {
        const features::ConstructionPlaneParams& p = pf->params();
        distance = p.distance;
        if (p.base.isFace) {
            for (const FaceChoice& f : c.faces)
                if (f.body == p.base.body && f.ref == p.base.face) k.preselect = {f.label};
        } else {
            for (const PlaneChoice& pc : c.planes)
                if (pc.plane == p.base.construction) planeDef = pc.label;
        }
    }
    k.inputs = {
        selectionField("face", "Face", faceLabels),
        dropdownField("plane", "Plane (if no face)", planeLabels, planeDef.toStdString()),
        valueField("distance", "Distance", Dimension::Length, distance),
    };
    k.make = [](const Snap& s, const TextFn& t, const DialogCtx& c) -> Result<std::shared_ptr<const Feature>> {
        features::ConstructionPlaneParams p;
        p.kind = features::ConstructionPlaneParams::Kind::Offset;
        const auto faceSel = selectionOf(s, "face");
        if (!faceSel.empty()) {
            const FaceChoice* face = nullptr;
            for (const FaceChoice& f : c.faces)
                if (f.label.toStdString() == faceSel.front()) face = &f;
            if (!face) return makeError(ErrorCode::InvalidArgument, "unknown face selection");
            p.base.isFace = true;
            p.base.body = face->body;
            p.base.face = face->ref;
        } else {
            const std::string planeLabel = choiceOf(s, "plane");
            const PlaneChoice* plane = nullptr;
            for (const PlaneChoice& pc : c.planes)
                if (pc.label.toStdString() == planeLabel) plane = &pc;
            if (!plane) return makeError(ErrorCode::InvalidArgument, "select a face or a construction plane");
            p.base.isFace = false;
            p.base.construction = plane->plane;
        }
        p.distance = t("distance");
        return asConst(std::make_shared<features::ConstructionPlaneFeature>(c.id, p));
    };
    return k;
}

KindSpec axisSpec(const DialogCtx& c, const Feature* existing) {
    KindSpec k;
    k.title = c.editing ? QStringLiteral("Edit Axis") : QStringLiteral("Axis Through");
    k.pick = PickMode::Edges;
    std::vector<QString> edgeLabels;
    for (const EdgeChoice& e : c.edges) edgeLabels.push_back(e.label);
    std::string kindDef = "Two points";
    std::array<std::string, 3> a{"0 mm", "0 mm", "0 mm"}, b{"0 mm", "0 mm", "10 mm"};
    // Editing: prefilled from the feature (two points or a linear edge; start() refuses cylinder axes).
    if (const auto* af = dynamic_cast<const features::ConstructionAxisFeature*>(existing)) {
        const features::ConstructionAxisParams& p = af->params();
        a = p.a;
        b = p.b;
        if (p.kind == features::ConstructionAxisParams::Kind::Edge) {
            kindDef = "Edge";
            for (const EdgeChoice& e : c.edges)
                if (e.body == p.body && e.ref == p.ref) k.preselect = {e.label};
        }
    }
    k.inputs = {
        dropdownField("kind", "Defined by", {QStringLiteral("Two points"), QStringLiteral("Edge")}, kindDef),
        valueField("ax", "Point 1 X", Dimension::Length, a[0]),
        valueField("ay", "Point 1 Y", Dimension::Length, a[1]),
        valueField("az", "Point 1 Z", Dimension::Length, a[2]),
        valueField("bx", "Point 2 X", Dimension::Length, b[0]),
        valueField("by", "Point 2 Y", Dimension::Length, b[1]),
        valueField("bz", "Point 2 Z", Dimension::Length, b[2]),
        selectionField("edge", "Edge", edgeLabels),
    };
    k.make = [](const Snap& s, const TextFn& t, const DialogCtx& c) -> Result<std::shared_ptr<const Feature>> {
        features::ConstructionAxisParams p;
        if (choiceOf(s, "kind") == "Edge") {
            p.kind = features::ConstructionAxisParams::Kind::Edge;
            const auto sel = selectionOf(s, "edge");
            const EdgeChoice* edge = nullptr;
            for (const EdgeChoice& e : c.edges)
                if (!sel.empty() && e.label.toStdString() == sel.front()) edge = &e;
            if (!edge) return makeError(ErrorCode::InvalidArgument, "select an edge");
            p.body = edge->body;
            p.ref = edge->ref;
        } else {
            p.kind = features::ConstructionAxisParams::Kind::TwoPoints;
            p.a = {t("ax"), t("ay"), t("az")};
            p.b = {t("bx"), t("by"), t("bz")};
        }
        return asConst(std::make_shared<features::ConstructionAxisFeature>(c.id, p));
    };
    return k;
}

KindSpec pointSpec(const DialogCtx& c, const Feature* existing) {
    KindSpec k;
    k.title = c.editing ? QStringLiteral("Edit Point") : QStringLiteral("Point");
    k.pick = PickMode::Edges;
    // One list: vertices first, then edges (a circular edge gives its centre). The kind follows the chosen entry.
    std::vector<QString> options;
    for (const EdgeChoice& v : c.vertices) options.push_back(v.label);
    for (const EdgeChoice& e : c.edges) options.push_back(e.label);
    k.inputs = {selectionField("ref", "Vertex or circular edge", options)};
    if (const auto* pf = dynamic_cast<const features::ConstructionPointFeature*>(existing)) {
        const features::ConstructionPointParams& p = pf->params();
        const auto& list = p.kind == features::ConstructionPointParams::Kind::Vertex ? c.vertices : c.edges;
        for (const EdgeChoice& e : list)
            if (e.body == p.body && e.ref == p.ref) k.preselect = {e.label};
    }
    k.make = [](const Snap& s, const TextFn&, const DialogCtx& c) -> Result<std::shared_ptr<const Feature>> {
        const auto sel = selectionOf(s, "ref");
        features::ConstructionPointParams p;
        const EdgeChoice* choice = nullptr;
        for (const EdgeChoice& v : c.vertices)
            if (!sel.empty() && v.label.toStdString() == sel.front()) {
                choice = &v;
                p.kind = features::ConstructionPointParams::Kind::Vertex;
            }
        for (const EdgeChoice& e : c.edges)
            if (!choice && !sel.empty() && e.label.toStdString() == sel.front()) {
                choice = &e;
                p.kind = features::ConstructionPointParams::Kind::CircleCentre;
            }
        if (!choice) return makeError(ErrorCode::InvalidArgument, "select a vertex or a circular edge");
        p.body = choice->body;
        p.ref = choice->ref;
        return asConst(std::make_shared<features::ConstructionPointFeature>(c.id, p));
    };
    return k;
}

} // namespace

// ---- FeatureDialogs ------------------------------------------------------------------------------------------

struct FeatureDialogs::Impl {
    FeatureHost& host;
    CommandPanel* panel = nullptr;     ///< The panel of the open dialog (the shell owns it).
    PickMode pick = PickMode::None;
    std::shared_ptr<const DialogCtx> ctx;
    MakeFn make;
    bool profilePicks = false;         ///< Profile overlays are shown: a click on one chooses that profile.
    bool arrowOn = false;              ///< Extrude is open: the distance arrow follows the profile and the field.
    bool ringOn = false;               ///< Revolve is open: the angle ring follows the profile, axis and the field.
    std::vector<QMetaObject::Connection> manipConns;
    /// Bumped whenever a dialog opens or closes: late preview / hover callbacks of an older dialog are ignored.
    std::uint64_t session = 0;
    std::vector<render::OverlayFace> profileFaces;   ///< Profile overlays (index = position in ctx->profiles).
    int hoveredProfile = -1;                         ///< Profile listed under the mouse in the Profile dropdown.
    std::vector<render::OverlayFace> previewFaces;   ///< The live preview body, drawn after the profiles.
    explicit Impl(FeatureHost& h) : host(h) {}

    /// Pushes the profile overlays (the hovered one brighter) and then the preview body to the viewport. Profile
    /// overlay indices stay the profile positions, so a click on a profile still picks it.
    void pushOverlays() {
        std::vector<render::OverlayFace> all;
        all.reserve(profileFaces.size() + previewFaces.size());
        for (std::size_t i = 0; i < profileFaces.size(); ++i) {
            render::OverlayFace f = profileFaces[i];
            if (static_cast<int>(i) == hoveredProfile) f.rgba[3] = 0.75f;
            all.push_back(std::move(f));
        }
        all.insert(all.end(), previewFaces.begin(), previewFaces.end());
        host.setOverlayFaces(std::move(all));
    }

    const ProfileChoice* profileOf(const std::string& key) const {
        if (!ctx) return nullptr;
        for (const ProfileChoice& p : ctx->profiles)
            if (p.key == key) return &p;
        return nullptr;
    }
    int profileIndexOf(const std::string& key) const {
        if (!ctx) return -1;
        for (std::size_t i = 0; i < ctx->profiles.size(); ++i)
            if (ctx->profiles[i].key == key) return static_cast<int>(i);
        return -1;
    }
    const AxisChoice* axisOf(const std::string& label) const {
        if (!ctx) return nullptr;
        for (const AxisChoice& a : ctx->axes)
            if (a.label.toStdString() == label) return &a;
        return nullptr;
    }

    /// Hover over a dropdown option: the profile it names is highlighted ("" = none).
    void setHoveredProfile(const std::string& key) {
        const int index = key.empty() ? -1 : profileIndexOf(key);
        if (index == hoveredProfile) return;
        hoveredProfile = index;
        pushOverlays();
    }

    std::optional<QPointF> toPx(const Vec3& p) const {
        render::Viewport* vp = host.viewport();
        if (!vp) return std::nullopt;
        const auto ndc = vp->camera().project(p);
        if (!ndc) return std::nullopt;
        return QPointF((ndc->x + 1.0) * 0.5 * vp->width(), (1.0 - ndc->y) * 0.5 * vp->height());
    }

    /// Places the distance arrow at the chosen profile's centroid, along its normal, at the current distance.
    void updateArrow() {
        ManipulatorOverlay* ov = host.manipulators();
        if (!arrowOn || !panel || !ctx || !ov) return;
        const ProfileChoice* prof = profileOf(panel->model().text("profile"));
        const auto value = panel->model().valueOf("distance");
        if (!prof || !value || value->dim != Dimension::Length) {
            ov->clear();
            return;
        }
        render::Viewport* vp = host.viewport();
        if (!vp) return;
        const render::Camera& cam = vp->camera();
        const double h = vp->height();
        const auto a = toPx(prof->anchor);
        const auto b = toPx(prof->anchor + prof->normal);
        if (!a || !b) {
            ov->clear();
            return;
        }
        const QPointF dir = *b - *a;
        if (std::hypot(dir.x(), dir.y()) < 1e-6) {
            ov->clear();
            return;
        }
        ov->showDistance(*a, dir, value->value, sketchPixelsPerMm(cam, h),
                         QStringLiteral("%1 mm").arg(value->value, 0, 'f', 1));
    }

    /// Revolve: the angle ring is drawn on the axis at the chosen profile's centroid, its start at the profile and its
    /// handle at the angle field (screen space: the ring is the projection of the sweep's circle about the axis).
    void updateRing() {
        ManipulatorOverlay* ov = host.manipulators();
        if (!ringOn || !panel || !ctx || !ov) return;
        const ProfileChoice* prof = profileOf(panel->model().text("profile"));
        const AxisChoice* axis = axisOf(panel->model().text("axis"));
        const auto angle = panel->model().valueOf("angle");
        const double len = axis ? length(axis->direction) : 0.0;
        if (!prof || !axis || !axis->located || len < 1e-12 || !angle || angle->dim != Dimension::Angle) {
            ov->clear();
            return;
        }
        const Vec3 dir = axis->direction * (1.0 / len);
        const Vec3 onAxis = axis->origin + dir * dot(prof->anchor - axis->origin, dir);
        const auto centre = toPx(onAxis);
        const auto start = toPx(prof->anchor);
        if (!centre || !start) {
            ov->clear();
            return;
        }
        const double radius = std::max(40.0, std::hypot(start->x() - centre->x(), start->y() - centre->y()));
        const double valueDeg = angle->value * 180.0 / std::numbers::pi;
        ov->showAngle(*centre, screenAngleDeg(*centre, *start), valueDeg, radius,
                      QStringLiteral("%1 deg").arg(valueDeg, 0, 'f', 1));
    }

    /// Dragging the arrow writes the distance back as a plain number, snapped to 0.1 mm (the expression is replaced).
    /// Dragging the ring writes the angle back the same way, in degrees.
    void attachManipulators(CommandPanel* dialog) {
        ManipulatorOverlay* ov = host.manipulators();
        if (!ov) return;
        if (arrowOn)
            manipConns.push_back(QObject::connect(ov, &ManipulatorOverlay::distanceChanged, dialog, [this, dialog](double mm) {
                if (!arrowOn || panel != dialog) return;
                const double snapped = std::round(mm * 10.0) / 10.0;
                dialog->setText("distance", QStringLiteral("%1 mm").arg(snapped, 0, 'f', 1));
            }));
        if (ringOn)
            manipConns.push_back(QObject::connect(ov, &ManipulatorOverlay::angleChanged, dialog, [this, dialog](double deg) {
                if (!ringOn || panel != dialog) return;
                const double snapped = std::round(deg * 10.0) / 10.0;
                dialog->setText("angle", QStringLiteral("%1 deg").arg(snapped, 0, 'f', 1));
            }));
        if (render::Viewport* vp = host.viewport())
            manipConns.push_back(QObject::connect(vp, &render::Viewport::cameraChanged, dialog, [this] {
                updateArrow();
                updateRing();
            }));
    }

    void detachManipulators() {
        for (const QMetaObject::Connection& c : manipConns) QObject::disconnect(c);
        manipConns.clear();
        arrowOn = false;
        ringOn = false;
        if (ManipulatorOverlay* ov = host.manipulators()) ov->clear();
    }

    /// Raw expression text of a field of the open panel.
    std::string rawText(std::string_view name) const { return panel ? panel->model().text(name) : std::string(); }

    QListWidget* listOf(QWidget* w) const { return w ? w->findChild<QListWidget*>() : nullptr; }

    void selectItem(const QString& label, bool exclusive) {
        QListWidget* list = listOf(panel);
        if (!list) return;
        for (int i = 0; i < list->count(); ++i) {
            QListWidgetItem* it = list->item(i);
            if (it->text() != label) {
                if (exclusive) it->setSelected(false);
                continue;
            }
            if (!exclusive) it->setSelected(!it->isSelected());
            else it->setSelected(true);
        }
    }

    void onPick(const render::PickResult& r) {
        if (!panel || !ctx) return;
        // A click on a profile overlay chooses that profile (the Profile dropdown stays as the fallback).
        if (profilePicks && r.overlayIndex) {
            if (*r.overlayIndex < ctx->profiles.size())
                panel->setText("profile", QString::fromStdString(ctx->profiles[*r.overlayIndex].key));
            return;
        }
        if (!r.bodyIndex) return;
        const auto body = host.bodyAtIndex(*r.bodyIndex);
        if (!body) return;
        if (pick == PickMode::Edges && r.edgeIndex) {
            for (const EdgeChoice& e : ctx->edges)
                if (e.body == *body && e.index == *r.edgeIndex) selectItem(e.label, false);
        } else if ((pick == PickMode::Face || pick == PickMode::FaceUV) && r.faceIndex) {
            for (const FaceChoice& f : ctx->faces)
                if (f.body == *body && f.index == *r.faceIndex) {
                    selectItem(f.label, true);
                    if (pick == PickMode::FaceUV && r.point) setUV(*body, *r.faceIndex, *r.point);
                }
        }
    }

    /// Hover filter of the open dialog: only what the active selection can take is highlighted (faces for Hole,
    /// edges for Fillet / Chamfer, profile overlays for Extrude / Revolve). Clicks use onPick with the same rules.
    bool hoverAccepts(const render::PickResult& r) const {
        if (!panel || !ctx) return false;
        if (profilePicks) return r.overlayIndex && *r.overlayIndex < ctx->profiles.size();
        if (!r.bodyIndex) return false;
        const auto body = host.bodyAtIndex(*r.bodyIndex);
        if (!body) return false;
        if (pick == PickMode::Edges && r.edgeIndex)
            return std::any_of(ctx->edges.begin(), ctx->edges.end(),
                               [&](const EdgeChoice& e) { return e.body == *body && e.index == *r.edgeIndex; });
        if ((pick == PickMode::Face || pick == PickMode::FaceUV) && r.faceIndex)
            return std::any_of(ctx->faces.begin(), ctx->faces.end(),
                               [&](const FaceChoice& f) { return f.body == *body && f.index == *r.faceIndex; });
        return false;
    }

    /// Hole placement: the picked point in the face's plane coordinates.
    void setUV(BodyId body, std::uint32_t face, Vec3 point) {
        const timeline::EvalOutput& shown = host.shownEvaluation();
        auto it = shown.eval.bodies.find(body);
        if (it == shown.eval.bodies.end()) return;
        auto pl = kernel::planeFromFace(it->second, face);
        if (!pl) return;
        const Vec3 yDir = cross(pl->normal, pl->xDir);
        const Vec3 d = point - pl->origin;
        // Lengths are written with their unit: the feature reads the raw text, and a bare number is not a length there.
        panel->setText("u", QStringLiteral("%1 mm").arg(dot(d, pl->xDir), 0, 'f', 3));
        panel->setText("v", QStringLiteral("%1 mm").arg(dot(d, yDir), 0, 'f', 3));
    }
};

FeatureDialogs::FeatureDialogs(FeatureHost& host) : host_(host), impl_(std::make_unique<Impl>(host)) {}
FeatureDialogs::~FeatureDialogs() = default;

Result<void> FeatureDialogs::start(const std::string& kind, std::optional<FeatureId> editing) {
    features::Document* doc = host_.model();
    if (!doc) return makeError(ErrorCode::InvalidArgument, "no document is open");

    const recompute::Timeline& tl = doc->timeline();
    const Feature* existing = nullptr;
    if (editing) {
        auto idx = tl.indexOf(*editing);
        if (!idx) return makeError(ErrorCode::NotFound, "feature is not on the timeline");
        existing = tl.at(*idx).get();
        if (existing->kind() != kind) return makeError(ErrorCode::InvalidArgument, "feature is not a " + kind);
        if (const auto* pf = dynamic_cast<const features::ConstructionPlaneFeature*>(existing);
            pf && pf->params().kind != features::ConstructionPlaneParams::Kind::Offset)
            return makeError(ErrorCode::InvalidArgument, "only offset planes have a dialog");
        if (const auto* af = dynamic_cast<const features::ConstructionAxisFeature*>(existing);
            af && af->params().kind == features::ConstructionAxisParams::Kind::Cylinder)
            return makeError(ErrorCode::InvalidArgument, "cylinder axes have no dialog yet");
    } else if (tl.rollbackMarker() < tl.size()) {
        return makeError(ErrorCode::InvalidArgument, "move the rollback marker to the end before adding a feature");
    }

    // Choice tables from the evaluation the user is looking at.
    auto ctx = std::make_shared<DialogCtx>();
    ctx->base = tl;
    ctx->params = doc->parameters();
    ctx->editing = editing.has_value();
    ctx->id = editing ? *editing : doc->allocateFeatureId();
    ctx->newBody = doc->allocateBodyId();
    if (existing) {
        if (auto b = timeline::producedBody(*existing)) ctx->newBody = *b;
    }

    const timeline::EvalOutput& shown = host_.shownEvaluation();
    const std::vector<std::string> names = timeline::displayNames(host_.shownTimeline());
    auto titleOf = [&](FeatureId id) -> std::string {
        const auto& shownTl = host_.shownTimeline();
        for (std::size_t i = 0; i < shownTl.size(); ++i)
            if (shownTl.at(i)->id() == id) return names[i];
        return "Feature " + std::to_string(id.value());
    };

    for (const auto& [body, shape] : shown.eval.bodies) {
        ctx->bodies.push_back({host_.bodyTitle(body), body});
        auto nmIt = shown.eval.namings.find(body);
        if (nmIt == shown.eval.namings.end()) continue;
        const naming::NamingMap& nm = nmIt->second;
        for (std::uint32_t i = 0; i < shape.subShapeCount(kernel::SubShapeKind::Edge); ++i) {
            auto n = nm.nameOf(naming::ElementType::Edge, i);
            if (!n) continue;
            ctx->edges.push_back({host_.bodyTitle(body) + QStringLiteral(" edge %1").arg(i), body,
                                  naming::ElementRef{n->feature, naming::ElementType::Edge, *n}, i});
        }
        for (std::uint32_t i = 0; i < shape.subShapeCount(kernel::SubShapeKind::Vertex); ++i) {
            auto n = nm.nameOf(naming::ElementType::Vertex, i);
            if (!n) continue;
            ctx->vertices.push_back({host_.bodyTitle(body) + QStringLiteral(" vertex %1").arg(i), body,
                                     naming::ElementRef{n->feature, naming::ElementType::Vertex, *n}, i});
        }
        for (std::uint32_t i = 0; i < shape.subShapeCount(kernel::SubShapeKind::Face); ++i) {
            const kernel::FaceGeometry g = shape.faceGeometry(i);
            if (g.kind != kernel::FaceGeometry::Kind::Plane) continue;
            auto n = nm.nameOf(naming::ElementType::Face, i);
            if (!n) continue;
            ctx->faces.push_back({host_.bodyTitle(body) + QStringLiteral(" face %1").arg(i), body,
                                  naming::ElementRef{n->feature, naming::ElementType::Face, *n}, i});
        }
    }
    for (const auto& [sid, sol] : shown.eval.sketches) {
        // Profiles of one sketch are numbered 1, 2, ... by their world centroid (x, then y, then z): the numbers follow
        // the geometry, not the order the solver happens to list the regions in.
        const sketch::SketchPlane& plane = sol.sketch.plane();
        std::vector<const sketch::SketchProfile*> ordered;
        for (const sketch::SketchProfile& p : sol.profiles) ordered.push_back(&p);
        std::stable_sort(ordered.begin(), ordered.end(), [&plane](const sketch::SketchProfile* a, const sketch::SketchProfile* b) {
            const Vec3 ca = plane.toWorld(a->centroid), cb = plane.toWorld(b->centroid);
            return std::tie(ca.x, ca.y, ca.z) < std::tie(cb.x, cb.y, cb.z);
        });
        std::uint32_t number = 0;
        for (const sketch::SketchProfile* p : ordered) {
            ++number;
            ctx->profiles.push_back({QStringLiteral("Profile %1").arg(number), profileKeyOf(sid, p->name), sid, p->name,
                                     plane.toWorld(p->centroid), plane.normal});
            ctx->profileOverlays.push_back(profileOverlay(plane, *p));
        }
    }
    for (std::size_t i = 0; i < tl.size(); ++i) {
        const Feature& f = *tl.at(i);
        if (editing && f.id() == *editing) continue;   // a construction cannot be based on itself
        if (f.kind() == "axis") {
            features::RevolveAxis a;
            a.kind = features::RevolveAxis::Kind::Construction;
            a.construction = f.id();
            AxisChoice choice{QString::fromStdString(titleOf(f.id())), a};
            if (auto g = shown.eval.construction.find(f.id()); g != shown.eval.construction.end()) {
                choice.origin = g->second.origin;
                choice.direction = g->second.direction;
                choice.located = true;
            }
            ctx->axes.push_back(std::move(choice));
        }
        if (f.kind() == "plane") ctx->planes.push_back({QString::fromStdString(titleOf(f.id())), f.id()});
    }
    for (const auto& [sid, sol] : shown.eval.sketches) {
        // Sketch lines as revolve axes
        for (const auto& [eid, ent] : sol.sketch.entities()) {
            if (const auto* ln = std::get_if<sketch::LineGeom>(&ent.geometry)) {
                features::RevolveAxis a;
                a.kind = features::RevolveAxis::Kind::SketchLine;
                a.line = eid;
                AxisChoice choice{QString::fromStdString(titleOf(sid) + " / line " + std::to_string(eid.value())), a};
                choice.origin = sol.sketch.plane().toWorld(ln->start);
                choice.direction = sol.sketch.plane().toWorld(ln->end) - choice.origin;
                choice.located = true;
                ctx->axes.push_back(std::move(choice));
            }
        }
    }

    // Dialog spec for the kind.
    KindSpec spec;
    if (kind == "extrude") spec = extrudeSpec(*ctx, existing);
    else if (kind == "fillet") spec = edgeSpec(*ctx, existing, false);
    else if (kind == "chamfer") spec = edgeSpec(*ctx, existing, true);
    else if (kind == "hole") spec = holeSpec(*ctx, existing);
    else if (kind == "revolve") spec = revolveSpec(*ctx, existing);
    else if (kind == "move") spec = moveSpec(*ctx, existing);
    else if (kind == "plane") spec = planeSpec(*ctx, existing);
    else if (kind == "axis") spec = axisSpec(*ctx, existing);
    else if (kind == "point") spec = pointSpec(*ctx, existing);
    else return makeError(ErrorCode::NotFound, "no dialog for " + kind);
    if (spec.inputs.empty() || spec.make == nullptr) return makeError(ErrorCode::Internal, "dialog has no fields");

    // Candidate feature from the snapshot. Previews use formatted numbers; OK uses the raw expression text.
    const MakeFn make = spec.make;
    const std::shared_ptr<const DialogCtx> cc = ctx;
    impl_->ctx = cc;
    impl_->make = make;
    impl_->pick = spec.pick;
    const std::uint64_t gen = ++impl_->session;

    // Colour of the preview body: red where the feature removes material, green where it adds or reshapes.
    const bool removes = kind == "hole";
    CommandDialogSpec dlg;
    dlg.title = spec.title;
    dlg.inputs = spec.inputs;
    // The distance arrow (Extrude) and the angle ring (Revolve) follow the profile and their field while open.
    dlg.fieldChanged = [this](const std::string& field) {
        if (impl_->arrowOn && (field == "distance" || field == "profile")) impl_->updateArrow();
        if (impl_->ringOn && (field == "angle" || field == "profile" || field == "axis")) impl_->updateRing();
    };
    dlg.geometryPreview = [make, cc, removes](const Snap& s, CancelToken token) -> Result<PreviewOutput> {
        const TextFn text = [&s](std::string_view n) { return formatted(s, n); };
        auto cand = make(s, text, *cc);
        if (!cand) return std::unexpected(cand.error());
        recompute::Timeline tl = cc->base;
        if (cc->editing) {
            if (auto r = tl.replace(*cand); !r) return std::unexpected(r.error());
        } else {
            tl.append(*cand);
        }
        auto ev = timeline::evaluateTimeline(tl, cc->params, *cc->evaluator, token);
        if (!ev) return std::unexpected(ev.error());
        auto idx = tl.indexOf((*cand)->id());
        if (!idx || *idx >= ev->statuses.size()) return makeError(ErrorCode::Internal, "preview missing");
        const FeatureStatus& st = ev->statuses[*idx];
        if (st.state == FeatureStatus::State::Failed) return std::unexpected(st.error);
        PreviewOutput out;
        out.text = "OK";
        if (auto body = timeline::producedBody(**cand)) {
            if (auto vol = timeline::volumeOfBody(*ev, *body))
                out.text = "Volume " + std::to_string(static_cast<long long>(std::llround(*vol))) + " mm3";
            // The produced body, tessellated on this job and drawn translucent by the dialog.
            if (auto shape = ev->bodies.find(*body); shape != ev->bodies.end()) {
                auto mesh = tess::tessellate(shape->second, tess::TessParams{}, token);
                if (!mesh) return std::unexpected(mesh.error());
                out.geometry.mesh = std::make_shared<const tess::MeshData>(std::move(*mesh));
                out.geometry.rgba = removes || choiceOf(s, "operation") == "Cut"
                                        ? std::array<float, 4>{0.95f, 0.22f, 0.18f, 0.45f}
                                        : std::array<float, 4>{0.20f, 0.85f, 0.30f, 0.45f};
            }
        }
        return out;
    };
    dlg.previewDelivered = [this, gen](const PreviewResult& r) {
        if (gen != impl_->session || !impl_->panel) return;   // a result of a dialog that is no longer open
        impl_->previewFaces.clear();
        if (!r.failed && r.geometry.mesh) impl_->previewFaces = meshOverlay(*r.geometry.mesh, r.geometry.rgba);
        impl_->pushOverlays();
    };
    dlg.optionHovered = [this, gen](const std::string& field, const std::string& option) {
        if (gen != impl_->session || field != "profile") return;
        impl_->setHoveredProfile(option);
    };
    dlg.accept = [this, make, cc](const Snap&) -> Result<void> {
        // Re-snapshot on the GUI thread: the panel's raw text is the source of expressions.
        if (!impl_->panel) return makeError(ErrorCode::Internal, "no open dialog");
        auto snap = impl_->panel->model().snapshot();
        if (!snap) return std::unexpected(snap.error());
        const TextFn text = [this](std::string_view n) { return impl_->rawText(n); };
        auto cand = make(*snap, text, *cc);
        if (!cand) return std::unexpected(cand.error());
        return host_.commitFeature(*cand, cc->editing);
    };

    QString hint;
    if (kind == "fillet") hint = QStringLiteral("Select edges to fillet");
    else if (kind == "chamfer") hint = QStringLiteral("Select edges to chamfer");
    else if (kind == "hole") hint = QStringLiteral("Select a face to place the hole");
    else if (kind == "extrude") hint = QStringLiteral("Select a profile to extrude");
    else if (kind == "revolve") hint = QStringLiteral("Select a profile to revolve");
    dlg.selectionHint = hint;   // before the spec is moved into the panel

    // The panel validates expressions against this parameter snapshot; the previous one is retired with its panel.
    params_ = std::make_shared<ParameterTable>(cc->params);
    auto* panel = new CommandPanel(std::move(dlg), params_.get(), host_.jobs());
    impl_->panel = panel;
    for (const QString& label : spec.preselect) impl_->selectItem(label, spec.pick == PickMode::Face || spec.pick == PickMode::FaceUV);
    // Profile overlays while the Extrude / Revolve dialog is open (a profile is chosen by clicking it).
    impl_->profilePicks = kind == "extrude" || kind == "revolve";
    impl_->profileFaces = impl_->profilePicks ? cc->profileOverlays : std::vector<render::OverlayFace>{};
    impl_->previewFaces.clear();
    impl_->hoveredProfile = -1;
    impl_->pushOverlays();
    host_.setPickHandler([this](const render::PickResult& r) { impl_->onPick(r); });
    impl_->arrowOn = kind == "extrude";
    impl_->ringOn = kind == "revolve";
    if (impl_->arrowOn || impl_->ringOn) impl_->attachManipulators(panel);

    // Pick mode: the first selection input is active at once. Its hint goes to the status bar, the viewport reserves
    // the left button for picks (a click never orbits or pans), and hovering highlights only what the input accepts.
    host_.setPickHint(hint);
    if (render::Viewport* vp = host_.viewport()) {
        vp->setLeftButtonReserved(true);
        vp->setHoverFilter([this, gen](const render::PickResult& r) { return gen == impl_->session && impl_->hoverAccepts(r); });
        // Hovering a profile overlay brightens it (and the dropdown names it), as hovering its dropdown entry does.
        QObject::connect(vp, &render::Viewport::hoverChanged, panel, [this, gen](const render::PickResult& r) {
            if (gen != impl_->session || !impl_->panel || !impl_->profilePicks || !impl_->ctx) return;
            std::string key;
            if (r.overlayIndex && *r.overlayIndex < impl_->ctx->profiles.size()) key = impl_->ctx->profiles[*r.overlayIndex].key;
            impl_->setHoveredProfile(key);
        });
    }
    QObject::connect(panel, &CommandPanel::selectionActivated, panel, [this, gen](const QString& h) {
        if (gen == impl_->session) host_.setPickHint(h);
    });
    QObject::connect(panel, &CommandPanel::selectionChanged, panel, [this, gen] {
        if (gen != impl_->session || !impl_->panel) return;
        // The viewport's orange pick follows the list: it clears when the active selection is empty.
        if (impl_->panel->model().selection(impl_->panel->activeSelection()).empty())
            if (render::Viewport* vp = host_.viewport()) vp->setSelection(render::PickResult{});
    });
    QObject::connect(panel, &CommandPanel::finished, [this](bool) {
        host_.setPickHandler({});
        ++impl_->session;   // late results of this dialog are ignored
        host_.setPickHint({});
        if (render::Viewport* vp = host_.viewport()) {
            vp->setHoverFilter({});
            vp->setLeftButtonReserved(false);
        }
        impl_->profileFaces.clear();
        impl_->previewFaces.clear();
        impl_->hoveredProfile = -1;
        impl_->pushOverlays();
        impl_->detachManipulators();
        impl_->profilePicks = false;
        impl_->panel = nullptr;
    });
    host_.showPanel(panel);
    impl_->updateArrow();
    impl_->updateRing();
    return {};
}

} // namespace iw::ui
