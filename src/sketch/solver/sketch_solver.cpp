// SPDX-License-Identifier: GPL-3.0-or-later
#include "sketch/solver/sketch_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Vendored FreeCAD planegcs (LGPL-2.0-or-later). See third_party/planegcs/README.md and PATCHES.md.
#include "GCS.h"

namespace iw::sketch {
namespace {

// Internal only: thrown while building the model, caught at the public API boundary.
struct InvalidInput {
    std::string message;
};

[[noreturn]] void invalid(std::string message)
{
    throw InvalidInput{std::move(message)};
}

constexpr std::size_t kNoConstraint = std::numeric_limits<std::size_t>::max();

struct Entity {
    enum class Kind : std::uint8_t { Point, Line, Circle, Arc };
    Kind kind = Kind::Point;
    GCS::Point point;               // Point
    GCS::Line* line = nullptr;      // Line
    GCS::Circle* circle = nullptr;  // Circle, and Arc (an Arc is a Circle)
    GCS::Arc* arc = nullptr;        // Arc
};

// A planegcs constant that is refreshed from the sketch on every solve. Either a dimension value
// (`constraint` indexes SolverSketch::constraints; target = value * scale), or, for Fix, the current
// value of the parameter `source`.
struct ValueSlot {
    double* target = nullptr;
    const double* source = nullptr;
    std::size_t constraint = kNoConstraint;
    double scale = 1.0;
};

// One planegcs model. Member order matters: geometry and parameter storage are declared before the
// System, because constraints keep pointers into them, and the System is destroyed first.
struct Model {
    std::vector<std::uint64_t> signature;  // structure only (ids, kinds, references), not values
    std::deque<double> store;              // stable addresses for every parameter and constant
    std::deque<GCS::Line> lines;
    std::deque<GCS::Circle> circles;
    std::deque<GCS::Arc> arcs;
    std::unordered_map<SolverId, Entity> entities;
    std::vector<double*> unknowns;
    std::vector<GCS::Point> pointOrder;  // aligned with SolverSketch::points
    std::vector<double*> circleRadius;   // aligned with SolverSketch::circles
    std::vector<ValueSlot> valueSlots;
    // planegcs constraint indices. Some constraints derive their scale from the geometry when they
    // are constructed (e.g. Parallel, Perpendicular), so they are rescaled once values are set.
    std::vector<int> constraintIds;
    double* dragX = nullptr;
    double* dragY = nullptr;
    bool structureChanged = false;  // set when constraints were dropped: the model no longer matches
    GCS::System sys;

    double* slot(double value)
    {
        store.push_back(value);
        return &store.back();
    }

    const Entity& find(SolverId id) const
    {
        auto it = entities.find(id);
        if (it == entities.end()) {
            invalid("unknown entity id " + std::to_string(id));
        }
        return it->second;
    }

    GCS::Point pointOf(const SolverRef& ref) const
    {
        const Entity& e = find(ref.entity);
        switch (e.kind) {
            case Entity::Kind::Point:
                return e.point;
            case Entity::Kind::Line:
                if (ref.role == SolverRole::Start) {
                    return e.line->p1;
                }
                if (ref.role == SolverRole::End) {
                    return e.line->p2;
                }
                break;
            case Entity::Kind::Circle:
                if (ref.role == SolverRole::Center) {
                    return e.circle->center;
                }
                break;
            case Entity::Kind::Arc:
                if (ref.role == SolverRole::Start) {
                    return e.arc->start;
                }
                if (ref.role == SolverRole::End) {
                    return e.arc->end;
                }
                if (ref.role == SolverRole::Center) {
                    return e.arc->center;
                }
                break;
        }
        invalid("entity " + std::to_string(ref.entity) + " has no such point role");
    }

    GCS::Line& lineOf(SolverId id) const
    {
        const Entity& e = find(id);
        if (e.kind != Entity::Kind::Line) {
            invalid("entity " + std::to_string(id) + " is not a line");
        }
        return *e.line;
    }

    // Circle or arc.
    const Entity& curveOf(SolverId id) const
    {
        const Entity& e = find(id);
        if (e.kind != Entity::Kind::Circle && e.kind != Entity::Kind::Arc) {
            invalid("entity " + std::to_string(id) + " is not a circle or arc");
        }
        return e;
    }

    // Midpoint of `line`: the point lies on the line and on its perpendicular bisector.
    void addMidpoint(GCS::Point p, GCS::Line& line, int tag)
    {
        keep(sys.addConstraintPointOnLine(p, line, tag));
        keep(sys.addConstraintPointOnPerpBisector(p, line.p1, line.p2, tag));
    }

    // Dimension constant of constraint `index`, filled from the sketch on every solve.
    double* valueOf(std::size_t index, double scale)
    {
        double* target = slot(0.0);
        valueSlots.push_back(ValueSlot{target, nullptr, index, scale});
        return target;
    }

    void keep(int planegcsIndex) { constraintIds.push_back(planegcsIndex); }

    void addConstraint(const SolverConstraint& c, std::size_t index, int tag);
    void addFix(const SolverConstraint& c, int tag);
};

void Model::addFix(const SolverConstraint& c, int tag)
{
    // Each fixed coordinate or radius is pinned to its current value, refreshed from the sketch on
    // every solve (the target is a constant, the source is the parameter itself).
    auto pinPoint = [&](GCS::Point p) {
        double* tx = slot(0.0);
        double* ty = slot(0.0);
        keep(sys.addConstraintCoordinateX(p, tx, tag));
        keep(sys.addConstraintCoordinateY(p, ty, tag));
        valueSlots.push_back(ValueSlot{tx, p.x, kNoConstraint, 1.0});
        valueSlots.push_back(ValueSlot{ty, p.y, kNoConstraint, 1.0});
    };
    const Entity& e = find(c.refs[0].entity);
    switch (e.kind) {
        case Entity::Kind::Point:
            pinPoint(e.point);
            break;
        case Entity::Kind::Line:
            pinPoint(e.line->p1);
            pinPoint(e.line->p2);
            break;
        case Entity::Kind::Circle: {
            pinPoint(e.circle->center);
            double* tr = slot(0.0);
            keep(sys.addConstraintCircleRadius(*e.circle, tr, tag));
            valueSlots.push_back(ValueSlot{tr, e.circle->rad, kNoConstraint, 1.0});
            break;
        }
        case Entity::Kind::Arc: {
            pinPoint(e.arc->center);
            pinPoint(e.arc->start);
            pinPoint(e.arc->end);
            double* tr = slot(0.0);
            keep(sys.addConstraintArcRadius(*e.arc, tr, tag));
            valueSlots.push_back(ValueSlot{tr, e.arc->rad, kNoConstraint, 1.0});
            break;
        }
    }
}

void Model::addConstraint(const SolverConstraint& c, std::size_t index, int tag)
{
    using K = SolverConstraintKind;
    const auto& r = c.refs;
    auto need = [&](std::size_t n) {
        if (r.size() != n) {
            invalid("constraint " + std::to_string(c.id) + " expects " + std::to_string(n) + " reference(s)");
        }
    };

    switch (c.kind) {
        case K::Coincident: {
            need(2);
            if (r[0].role == SolverRole::Mid && r[1].role == SolverRole::Mid) {
                invalid("constraint " + std::to_string(c.id) + ": two midpoints cannot be coincident this way");
            }
            if (r[0].role == SolverRole::Mid) {
                addMidpoint(pointOf(r[1]), lineOf(r[0].entity), tag);
            } else if (r[1].role == SolverRole::Mid) {
                addMidpoint(pointOf(r[0]), lineOf(r[1].entity), tag);
            } else {
                GCS::Point a = pointOf(r[0]);
                GCS::Point b = pointOf(r[1]);
                keep(sys.addConstraintP2PCoincident(a, b, tag));
            }
            break;
        }
        case K::Horizontal:
        case K::Vertical: {
            const bool horizontal = c.kind == K::Horizontal;
            if (r.size() == 1) {
                GCS::Line& l = lineOf(r[0].entity);
                if (horizontal) {
                    keep(sys.addConstraintHorizontal(l, tag));
                } else {
                    keep(sys.addConstraintVertical(l, tag));
                }
            } else {
                need(2);
                GCS::Point a = pointOf(r[0]);
                GCS::Point b = pointOf(r[1]);
                if (horizontal) {
                    keep(sys.addConstraintHorizontal(a, b, tag));
                } else {
                    keep(sys.addConstraintVertical(a, b, tag));
                }
            }
            break;
        }
        case K::Parallel:
        case K::Perpendicular: {
            need(2);
            GCS::Line& a = lineOf(r[0].entity);
            GCS::Line& b = lineOf(r[1].entity);
            if (c.kind == K::Parallel) {
                keep(sys.addConstraintParallel(a, b, tag));
            } else {
                keep(sys.addConstraintPerpendicular(a, b, tag));
            }
            break;
        }
        case K::Tangent: {
            need(2);
            const Entity* ea = &find(r[0].entity);
            const Entity* eb = &find(r[1].entity);
            if (ea->kind == Entity::Kind::Line && eb->kind == Entity::Kind::Line) {
                invalid("constraint " + std::to_string(c.id) + ": two lines cannot be tangent");
            }
            if (eb->kind == Entity::Kind::Line) {
                std::swap(ea, eb);  // line first
            }
            if (ea->kind == Entity::Kind::Line) {
                if (eb->kind == Entity::Kind::Arc) {
                    keep(sys.addConstraintTangent(*ea->line, *eb->arc, tag));
                } else if (eb->kind == Entity::Kind::Circle) {
                    keep(sys.addConstraintTangent(*ea->line, *eb->circle, tag));
                } else {
                    invalid("constraint " + std::to_string(c.id) + ": tangency needs curves");
                }
            } else {
                curveOf(r[0].entity);  // rejects points and other non-curves
                curveOf(r[1].entity);
                const bool aArc = ea->kind == Entity::Kind::Arc;
                const bool bArc = eb->kind == Entity::Kind::Arc;
                if (aArc && bArc) {
                    keep(sys.addConstraintTangent(*ea->arc, *eb->arc, tag));
                } else if (aArc) {
                    keep(sys.addConstraintTangent(*eb->circle, *ea->arc, tag));
                } else if (bArc) {
                    keep(sys.addConstraintTangent(*ea->circle, *eb->arc, tag));
                } else {
                    keep(sys.addConstraintTangent(*ea->circle, *eb->circle, tag));
                }
            }
            break;
        }
        case K::Equal: {
            need(2);
            const Entity& ea = find(r[0].entity);
            const Entity& eb = find(r[1].entity);
            if (ea.kind == Entity::Kind::Line && eb.kind == Entity::Kind::Line) {
                keep(sys.addConstraintEqualLength(*ea.line, *eb.line, tag));
                break;
            }
            curveOf(r[0].entity);
            curveOf(r[1].entity);
            const bool aArc = ea.kind == Entity::Kind::Arc;
            const bool bArc = eb.kind == Entity::Kind::Arc;
            if (aArc && bArc) {
                keep(sys.addConstraintEqualRadius(*ea.arc, *eb.arc, tag));
            } else if (aArc) {
                keep(sys.addConstraintEqualRadius(*eb.circle, *ea.arc, tag));
            } else if (bArc) {
                keep(sys.addConstraintEqualRadius(*ea.circle, *eb.arc, tag));
            } else {
                keep(sys.addConstraintEqualRadius(*ea.circle, *eb.circle, tag));
            }
            break;
        }
        case K::Concentric: {
            need(2);
            const Entity& ea = curveOf(r[0].entity);
            const Entity& eb = curveOf(r[1].entity);
            GCS::Point ca = ea.circle->center;
            GCS::Point cb = eb.circle->center;
            keep(sys.addConstraintP2PCoincident(ca, cb, tag));
            break;
        }
        case K::Midpoint: {
            need(2);
            GCS::Point p = pointOf(r[0]);
            addMidpoint(p, lineOf(r[1].entity), tag);
            break;
        }
        case K::Collinear: {
            need(2);
            GCS::Line& a = lineOf(r[0].entity);
            GCS::Line& b = lineOf(r[1].entity);
            keep(sys.addConstraintPointOnLine(a.p1, b, tag));
            keep(sys.addConstraintPointOnLine(a.p2, b, tag));
            break;
        }
        case K::Symmetric: {
            need(3);
            GCS::Point a = pointOf(r[0]);
            GCS::Point b = pointOf(r[1]);
            const Entity& axis = find(r[2].entity);
            if (axis.kind == Entity::Kind::Line) {
                // planegcs adds TWO constraints (perpendicular + midpoint-on-line) and returns only the last index.
                // Both must be registered: updateModel() rescales every registered constraint at the real
                // coordinates, and one left at its construction-time scale (all parameters 0 then) breaks the solve.
                keep(sys.addConstraintPerpendicular(a, b, axis.line->p1, axis.line->p2, tag));
                keep(sys.addConstraintMidpointOnLine(a, b, axis.line->p1, axis.line->p2, tag));
            } else if (axis.kind == Entity::Kind::Point) {
                GCS::Point m = axis.point;
                // Same two-constraint pitfall as the line mirror above.
                keep(sys.addConstraintPointOnPerpBisector(m, a, b, tag));
                keep(sys.addConstraintPointOnLine(m, a, b, tag));
            } else {
                invalid("constraint " + std::to_string(c.id) + ": symmetry axis must be a line or a point");
            }
            break;
        }
        case K::Fix:
            need(1);
            addFix(c, tag);
            break;
        case K::PointOnObject: {
            need(2);
            GCS::Point p = pointOf(r[0]);
            const Entity& e = find(r[1].entity);
            switch (e.kind) {
                case Entity::Kind::Line:
                    keep(sys.addConstraintPointOnLine(p, *e.line, tag));
                    break;
                case Entity::Kind::Circle:
                    keep(sys.addConstraintPointOnCircle(p, *e.circle, tag));
                    break;
                case Entity::Kind::Arc:
                    keep(sys.addConstraintPointOnArc(p, *e.arc, tag));
                    break;
                default:
                    invalid("constraint " + std::to_string(c.id) + ": point-on-object needs a line, circle or arc");
            }
            break;
        }
        case K::Distance: {
            need(2);
            GCS::Point a = pointOf(r[0]);
            GCS::Point b = pointOf(r[1]);
            keep(sys.addConstraintP2PDistance(a, b, valueOf(index, 1.0), tag));
            break;
        }
        case K::DistanceX:
        case K::DistanceY: {
            need(2);
            GCS::Point a = pointOf(r[0]);
            GCS::Point b = pointOf(r[1]);
            if (c.kind == K::DistanceX) {
                keep(sys.addConstraintDifference(a.x, b.x, valueOf(index, 1.0), tag));
            } else {
                keep(sys.addConstraintDifference(a.y, b.y, valueOf(index, 1.0), tag));
            }
            break;
        }
        case K::Radius:
        case K::Diameter: {
            need(1);
            const Entity& e = curveOf(r[0].entity);
            const bool isDiameter = c.kind == K::Diameter;
            if (e.kind == Entity::Kind::Arc) {
                if (isDiameter) {
                    keep(sys.addConstraintArcDiameter(*e.arc, valueOf(index, 1.0), tag));
                } else {
                    keep(sys.addConstraintArcRadius(*e.arc, valueOf(index, 1.0), tag));
                }
            } else if (isDiameter) {
                keep(sys.addConstraintCircleDiameter(*e.circle, valueOf(index, 1.0), tag));
            } else {
                keep(sys.addConstraintCircleRadius(*e.circle, valueOf(index, 1.0), tag));
            }
            break;
        }
        case K::Angle: {
            need(2);
            GCS::Line& a = lineOf(r[0].entity);
            GCS::Line& b = lineOf(r[1].entity);
            keep(sys.addConstraintL2LAngle(a, b, valueOf(index, std::numbers::pi / 180.0), tag));
            break;
        }
    }
}

// Structure of `s` (ids, kinds, references, drag target), without values. Two sketches with equal
// signatures can share one planegcs model.
std::vector<std::uint64_t> structureSignature(const SolverSketch& s, const SolverRef* drag)
{
    std::vector<std::uint64_t> sig;
    sig.push_back(s.points.size());
    for (const SolverPoint& p : s.points) {
        sig.push_back(p.id);
    }
    sig.push_back(s.lines.size());
    for (const SolverLine& l : s.lines) {
        sig.insert(sig.end(), {l.id, l.start, l.end});
    }
    sig.push_back(s.circles.size());
    for (const SolverCircle& c : s.circles) {
        sig.insert(sig.end(), {c.id, c.center});
    }
    sig.push_back(s.arcs.size());
    for (const SolverArc& a : s.arcs) {
        sig.insert(sig.end(), {a.id, a.center, a.start, a.end});
    }
    sig.push_back(s.constraints.size());
    for (const SolverConstraint& c : s.constraints) {
        sig.insert(sig.end(), {c.id, static_cast<std::uint64_t>(c.kind), c.refs.size()});
        for (const SolverRef& r : c.refs) {
            sig.insert(sig.end(), {r.entity, static_cast<std::uint64_t>(r.role)});
        }
    }
    if (drag != nullptr) {
        sig.insert(sig.end(), {1U, drag->entity});
    } else {
        sig.push_back(0U);
    }
    return sig;
}

// Builds the planegcs structure for `input`. Values are filled in by updateModel().
std::unique_ptr<Model> buildModel(const SolverSketch& input, const SolverRef* drag)
{
    auto m = std::make_unique<Model>();
    m->signature = structureSignature(input, drag);

    for (const SolverPoint& p : input.points) {
        Entity e;
        e.kind = Entity::Kind::Point;
        e.point = GCS::Point(m->slot(0.0), m->slot(0.0));
        if (!m->entities.emplace(p.id, e).second) {
            invalid("duplicate entity id " + std::to_string(p.id));
        }
        m->unknowns.push_back(e.point.x);
        m->unknowns.push_back(e.point.y);
        m->pointOrder.push_back(e.point);
    }

    for (const SolverLine& l : input.lines) {
        if (m->find(l.start).kind != Entity::Kind::Point || m->find(l.end).kind != Entity::Kind::Point) {
            invalid("line " + std::to_string(l.id) + " must refer to point ids");
        }
        const GCS::Point a = m->find(l.start).point;
        const GCS::Point b = m->find(l.end).point;
        m->lines.emplace_back();
        GCS::Line& line = m->lines.back();
        line.p1 = a;
        line.p2 = b;
        Entity e;
        e.kind = Entity::Kind::Line;
        e.line = &line;
        if (!m->entities.emplace(l.id, e).second) {
            invalid("duplicate entity id " + std::to_string(l.id));
        }
    }

    for (const SolverCircle& c : input.circles) {
        if (m->find(c.center).kind != Entity::Kind::Point) {
            invalid("circle " + std::to_string(c.id) + " must refer to a point id for its centre");
        }
        m->circles.emplace_back();
        GCS::Circle& circle = m->circles.back();
        circle.center = m->find(c.center).point;
        circle.rad = m->slot(0.0);
        m->unknowns.push_back(circle.rad);
        m->circleRadius.push_back(circle.rad);
        Entity e;
        e.kind = Entity::Kind::Circle;
        e.circle = &circle;
        if (!m->entities.emplace(c.id, e).second) {
            invalid("duplicate entity id " + std::to_string(c.id));
        }
    }

    for (const SolverArc& a : input.arcs) {
        if (m->find(a.center).kind != Entity::Kind::Point || m->find(a.start).kind != Entity::Kind::Point ||
            m->find(a.end).kind != Entity::Kind::Point) {
            invalid("arc " + std::to_string(a.id) + " must refer to point ids");
        }
        m->arcs.emplace_back();
        GCS::Arc& arc = m->arcs.back();
        arc.center = m->find(a.center).point;
        arc.start = m->find(a.start).point;
        arc.end = m->find(a.end).point;
        arc.rad = m->slot(0.0);
        arc.startAngle = m->slot(0.0);
        arc.endAngle = m->slot(0.0);
        m->unknowns.push_back(arc.rad);
        m->unknowns.push_back(arc.startAngle);
        m->unknowns.push_back(arc.endAngle);
        // Structural: start and end lie on the circle at their angles. Tag 0 = high priority, so
        // these never appear in conflict or redundancy reports.
        m->keep(m->sys.addConstraintArcRules(arc, 0));

        Entity e;
        e.kind = Entity::Kind::Arc;
        e.arc = &arc;
        e.circle = &arc;
        if (!m->entities.emplace(a.id, e).second) {
            invalid("duplicate entity id " + std::to_string(a.id));
        }
    }

    for (std::size_t i = 0; i < input.constraints.size(); ++i) {
        const SolverConstraint& c = input.constraints[i];
        for (std::size_t j = 0; j < i; ++j) {
            if (input.constraints[j].id == c.id) {
                invalid("duplicate constraint id " + std::to_string(c.id));
            }
        }
        // Tags are 1-based indices into the constraint list, so diagnostics map back to ids.
        m->addConstraint(c, i, static_cast<int>(i) + 1);
    }

    if (drag != nullptr) {
        if (m->find(drag->entity).kind != Entity::Kind::Point) {
            invalid("only points can be dragged");
        }
        GCS::Point p = m->find(drag->entity).point;
        m->dragX = m->slot(0.0);
        m->dragY = m->slot(0.0);
        // Temporary soft constraints: tag -1 keeps them out of diagnostics.
        m->keep(m->sys.addConstraintCoordinateX(p, m->dragX, -1));
        m->keep(m->sys.addConstraintCoordinateY(p, m->dragY, -1));
    }
    return m;
}

// Writes the current sketch values into the model. Order matters: positions and radii first, then
// arc angles (derived from positions), then constants (Fix pins read the parameters just set).
void updateModel(Model& m, const SolverSketch& s, double dragX, double dragY)
{
    for (std::size_t i = 0; i < s.points.size(); ++i) {
        *m.pointOrder[i].x = s.points[i].x;
        *m.pointOrder[i].y = s.points[i].y;
    }
    for (std::size_t i = 0; i < s.circles.size(); ++i) {
        *m.circleRadius[i] = s.circles[i].radius;
    }
    for (GCS::Arc& arc : m.arcs) {
        const double cx = *arc.center.x;
        const double cy = *arc.center.y;
        const double sx = *arc.start.x;
        const double sy = *arc.start.y;
        const double ex = *arc.end.x;
        const double ey = *arc.end.y;
        const double r0 = std::hypot(sx - cx, sy - cy);
        if (!(r0 > 0.0)) {
            invalid("arc has a zero radius");
        }
        const double startAngle = std::atan2(sy - cy, sx - cx);
        double sweep = std::fmod(std::atan2(ey - cy, ex - cx) - startAngle, 2.0 * std::numbers::pi);
        if (sweep <= 0.0) {
            sweep += 2.0 * std::numbers::pi;
        }
        *arc.rad = r0;
        *arc.startAngle = startAngle;
        *arc.endAngle = startAngle + sweep;
    }
    for (const ValueSlot& v : m.valueSlots) {
        if (v.source != nullptr) {
            *v.target = *v.source;
        } else {
            *v.target = s.constraints[v.constraint].value * v.scale;
        }
    }
    if (m.dragX != nullptr) {
        *m.dragX = dragX;
        *m.dragY = dragY;
    }
    for (int id : m.constraintIds) {
        m.sys.rescaleConstraint(id, 1.0);
    }
}

// Reads the diagnosis planegcs holds for the current model (valid when hasDiagnosis is set).
SolverDiagnostics readDiagnostics(const Model& m, const std::vector<SolverConstraint>& constraints)
{
    SolverDiagnostics d;
    d.dof = m.sys.dofsNumber();
    auto toIds = [&](const GCS::VEC_I& tags) {
        std::vector<SolverId> ids;
        for (int tag : tags) {
            if (tag >= 1 && static_cast<std::size_t>(tag) <= constraints.size()) {
                ids.push_back(constraints[static_cast<std::size_t>(tag) - 1].id);
            }
        }
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
        return ids;
    };
    GCS::VEC_I tags;
    m.sys.getConflicting(tags);
    d.conflicting = toIds(tags);
    m.sys.getRedundant(tags);
    d.redundant = toIds(tags);
    m.sys.getPartiallyRedundant(tags);
    d.partiallyRedundant = toIds(tags);
    return d;
}

// Diagnoses the current model from scratch (never from a cached result).
SolverDiagnostics diagnoseModel(Model& m, const std::vector<SolverConstraint>& constraints)
{
    m.sys.invalidatedDiagnosis();
    m.sys.diagnose(GCS::DogLeg);
    return readDiagnostics(m, constraints);
}

struct Outcome {
    bool solved = false;
    bool droppedRedundant = false;  // diagnostics then describe the model before the drop
    SolverDiagnostics diagnostics;
};

// Solves `m` from its current values. On success the system is left at the solution; on failure it
// is restored to the starting values.
//
// Diagnostics: planegcs diagnoses inside initSolution() whenever the model has no cached diagnosis,
// which is the expensive part. A cached diagnosis stays valid across value-only changes, so repeated
// solves skip it. A successful solve reports the cached DOF and redundancy and no conflicts (a
// consistent system has none). A failed solve re-diagnoses at the starting values, so its conflict
// report is current.
//
// planegcs's DogLeg reports success only for a non-singular system. A constraint implied by the others
// (e.g. a second Horizontal on the same line) makes the Jacobian rank-deficient, so the solve can stop
// without success even though the residuals are zero. Such fully redundant constraints are dropped and
// the rest is solved. Conflicts are never dropped.
Outcome solveModel(Model& m, const std::vector<SolverConstraint>& constraints, bool wantDiagnostics)
{
    // Least-norm steps stay well defined for rank-deficient constraint sets.
    m.sys.dogLegGaussStep = GCS::LeastNormFullPivLU;
    m.sys.autoChooseAlgorithm = false;
    m.sys.qrAlgorithm = GCS::EigenSparseQR;
    m.sys.declareUnknowns(m.unknowns);
    m.sys.initSolution(GCS::DogLeg);

    Outcome out;
    if (m.sys.solve(true, GCS::DogLeg) == GCS::Success) {
        m.sys.applySolution();
        out.solved = true;
        if (wantDiagnostics) {
            out.diagnostics = readDiagnostics(m, constraints);
            out.diagnostics.conflicting.clear();  // a solved system has no conflicts
        }
        return out;
    }

    m.sys.undoSolution();
    out.diagnostics = diagnoseModel(m, constraints);
    if (out.diagnostics.conflicting.empty() && !out.diagnostics.redundant.empty()) {
        // Tags 1..N are user constraints. Tag 0 is structural (arc rules): never dropped.
        GCS::VEC_I tags;
        m.sys.getRedundant(tags);
        for (int tag : tags) {
            if (tag >= 1) {
                m.sys.clearByTag(tag);
            }
        }
        m.structureChanged = true;
        out.droppedRedundant = true;
        m.sys.initSolution(GCS::DogLeg);
        if (m.sys.solve(true, GCS::DogLeg) == GCS::Success) {
            m.sys.applySolution();
            out.solved = true;
        } else {
            m.sys.undoSolution();
        }
    }
    return out;
}

void writeBack(const Model& m, SolverSketch& sketch)
{
    for (std::size_t i = 0; i < sketch.points.size(); ++i) {
        sketch.points[i].x = *m.pointOrder[i].x;
        sketch.points[i].y = *m.pointOrder[i].y;
    }
    for (std::size_t i = 0; i < sketch.circles.size(); ++i) {
        sketch.circles[i].radius = *m.circleRadius[i];
    }
}

}  // namespace

// Cached planegcs model, reused while the sketch structure is unchanged.
struct SketchSolver::Impl {
    std::unique_ptr<Model> model;
};

SketchSolver::SketchSolver(SolverSketch sketch)
    : sketch_(std::move(sketch))
    , impl_(std::make_unique<Impl>())
{}

SketchSolver::~SketchSolver() = default;

SketchSolver::SketchSolver(const SketchSolver& other)
    : sketch_(other.sketch_)
    , impl_(std::make_unique<Impl>())
{}

SketchSolver& SketchSolver::operator=(const SketchSolver& other)
{
    if (this != &other) {
        sketch_ = other.sketch_;
        impl_ = std::make_unique<Impl>();
    }
    return *this;
}

SketchSolver::SketchSolver(SketchSolver&& other) noexcept = default;

SketchSolver& SketchSolver::operator=(SketchSolver&& other) noexcept = default;

namespace {

// Runs the solver for `s` (with an optional drag target), reusing the cached model when possible.
// With `commit`, a successful solve writes the solution into `s`.
Outcome runOn(std::unique_ptr<Model>& cache, SolverSketch& s, const SolverRef* drag, double dragX, double dragY,
              bool wantDiagnostics, bool commit)
{
    if (!cache || cache->signature != structureSignature(s, drag)) {
        cache = buildModel(s, drag);
    }
    Model& m = *cache;
    updateModel(m, s, dragX, dragY);
    Outcome out = solveModel(m, s.constraints, wantDiagnostics);
    if (out.solved && commit) {
        writeBack(m, s);
    }
    if (m.structureChanged) {
        cache.reset();
    }
    return out;
}

}  // namespace

SolverResult SketchSolver::solve()
{
    SolverResult result;
    try {
        Outcome out = runOn(impl_->model, sketch_, nullptr, 0.0, 0.0, true, true);
        result.diagnostics = std::move(out.diagnostics);
        if (out.solved) {
            result.status = SolverStatus::Solved;
        } else {
            result.status = SolverStatus::Failed;
            result.message = "no consistent solution; see conflicting and redundant constraints";
        }
    } catch (const InvalidInput& e) {
        impl_->model.reset();
        result.status = SolverStatus::InvalidInput;
        result.message = e.message;
    }
    return result;
}

SolverResult SketchSolver::dragPoint(SolverRef point, double x, double y)
{
    SolverResult result;
    try {
        // The drag target is a soft constraint (tag -1): the point ends at the target when the other
        // constraints allow it, and otherwise at the nearest consistent position.
        Outcome out = runOn(impl_->model, sketch_, &point, x, y, false, true);
        if (out.solved) {
            result.status = SolverStatus::Solved;
        } else {
            result.status = SolverStatus::Failed;
            result.message = "constraints could not be solved with the dragged point";
        }
    } catch (const InvalidInput& e) {
        impl_->model.reset();
        result.status = SolverStatus::InvalidInput;
        result.message = e.message;
    }
    return result;
}

SolverDiagnostics SketchSolver::diagnose()
{
    try {
        // Works on a scratch copy of the values: the sketch itself is not changed.
        SolverSketch scratch = sketch_;
        std::unique_ptr<Model>& cache = impl_->model;
        if (!cache || cache->signature != structureSignature(scratch, nullptr)) {
            cache = buildModel(scratch, nullptr);
        }
        Model& m = *cache;
        updateModel(m, scratch, 0.0, 0.0);
        const Outcome out = solveModel(m, scratch.constraints, false);
        SolverDiagnostics d = (out.solved && !out.droppedRedundant) ? diagnoseModel(m, scratch.constraints)
                                                                     : out.diagnostics;
        if (m.structureChanged) {
            cache.reset();
        }
        return d;
    } catch (const InvalidInput&) {
        impl_->model.reset();
        return SolverDiagnostics{};
    }
}

int SketchSolver::dof()
{
    return diagnose().dof;
}

}  // namespace iw::sketch
