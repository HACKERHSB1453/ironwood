// SPDX-License-Identifier: GPL-3.0-or-later
#include "sketch/sketch.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numbers>
#include <utility>

namespace iw::sketch {

namespace {

constexpr double kLengthTol = 1e-7; // mm; matches the profile vertex-merge tolerance
constexpr double kTwoPi = 2.0 * std::numbers::pi;

bool finite(double v) noexcept { return std::isfinite(v); }
bool finite(Point2 p) noexcept { return finite(p.x) && finite(p.y); }

Vec3 normalised(const Vec3& v) { return v * (1.0 / length(v)); }

std::unexpected<Error> invalid(const char* what) {
    return makeError(ErrorCode::InvalidArgument, std::string("sketch: ") + what);
}

} // namespace

double distance(Point2 a, Point2 b) noexcept {
    return std::hypot(a.x - b.x, a.y - b.y);
}

// ---- plane ---------------------------------------------------------------------------------------------------

Vec3 SketchPlane::yDir() const { return cross(normal, xDir); }

Vec3 SketchPlane::toWorld(Point2 p) const {
    return origin + xDir * p.x + yDir() * p.y;
}

Point2 SketchPlane::toLocal(const Vec3& p) const {
    const Vec3 d = p - origin;
    return {dot(d, xDir), dot(d, yDir())};
}

Result<SketchPlane> SketchPlane::make(const Vec3& origin, const Vec3& normal, const Vec3& xDir) {
    const Vec3* vecs[] = {&origin, &normal, &xDir};
    for (const Vec3* v : vecs)
        if (!finite(v->x) || !finite(v->y) || !finite(v->z)) return invalid("plane vector is not finite");
    if (length(normal) < 1e-12) return invalid("plane normal is zero");
    const Vec3 n = normalised(normal);
    // Project xDir onto the plane and normalise it.
    const Vec3 xp = xDir - n * dot(xDir, n);
    if (length(xp) < 1e-12) return invalid("plane xDir is parallel to its normal");
    return SketchPlane{origin, n, normalised(xp)};
}

// ---- geometry helpers ----------------------------------------------------------------------------------------

EntityKind SketchEntity::kind() const noexcept {
    return static_cast<EntityKind>(geometry.index());
}

Point2 arcStart(const ArcGeom& a) noexcept {
    return {a.center.x + a.radius * std::cos(a.startAngle), a.center.y + a.radius * std::sin(a.startAngle)};
}

Point2 arcEnd(const ArcGeom& a) noexcept {
    const double t = a.startAngle + a.sweep;
    return {a.center.x + a.radius * std::cos(t), a.center.y + a.radius * std::sin(t)};
}

Point2 arcMid(const ArcGeom& a) noexcept {
    const double t = a.startAngle + 0.5 * a.sweep;
    return {a.center.x + a.radius * std::cos(t), a.center.y + a.radius * std::sin(t)};
}

// ---- Sketch: entities ----------------------------------------------------------------------------------------

bool isReferenceEntity(SketchEntityId id) noexcept {
    return id == kOriginPointId || id == kXAxisId || id == kYAxisId;
}

Sketch::Sketch(SketchPlane plane) : plane_(plane) {
    entities_.emplace(kOriginPointId, SketchEntity{kOriginPointId, true, PointGeom{{0, 0}}});
    entities_.emplace(kXAxisId, SketchEntity{kXAxisId, true,
                                             LineGeom{{-kReferenceAxisHalfLength, 0}, {kReferenceAxisHalfLength, 0}}});
    entities_.emplace(kYAxisId, SketchEntity{kYAxisId, true,
                                             LineGeom{{0, -kReferenceAxisHalfLength}, {0, kReferenceAxisHalfLength}}});
}

Result<SketchEntityId> Sketch::addPoint(Point2 p, bool construction) {
    if (!finite(p)) return invalid("point is not finite");
    const SketchEntityId id{allocateId()};
    entities_.emplace(id, SketchEntity{id, construction, PointGeom{p}});
    return id;
}

Result<SketchEntityId> Sketch::addLine(Point2 start, Point2 end, bool construction) {
    if (!finite(start) || !finite(end)) return invalid("line end is not finite");
    if (distance(start, end) <= kLengthTol) return invalid("line has zero length");
    const SketchEntityId id{allocateId()};
    entities_.emplace(id, SketchEntity{id, construction, LineGeom{start, end}});
    return id;
}

Result<SketchEntityId> Sketch::addArc(Point2 center, double radius, double startAngle, double sweep,
                                      bool construction) {
    if (!finite(center) || !finite(radius) || !finite(startAngle) || !finite(sweep))
        return invalid("arc parameters are not finite");
    if (!(radius > kLengthTol)) return invalid("arc radius must be positive");
    if (!(sweep > 0.0 && sweep < kTwoPi)) return invalid("arc sweep must be in (0, 2*pi)");
    const SketchEntityId id{allocateId()};
    entities_.emplace(id, SketchEntity{id, construction, ArcGeom{center, radius, startAngle, sweep}});
    return id;
}

Result<SketchEntityId> Sketch::addCircle(Point2 center, double radius, bool construction) {
    if (!finite(center) || !finite(radius)) return invalid("circle parameters are not finite");
    if (!(radius > kLengthTol)) return invalid("circle radius must be positive");
    const SketchEntityId id{allocateId()};
    entities_.emplace(id, SketchEntity{id, construction, CircleGeom{center, radius}});
    return id;
}

namespace {

/// The geometry rules shared by creation, replacement and restore.
Result<void> validateGeometry(const EntityGeometry& geometry) {
    if (const auto* p = std::get_if<PointGeom>(&geometry)) {
        if (!finite(p->p)) return invalid("point geometry is not finite");
    } else if (const auto* l = std::get_if<LineGeom>(&geometry)) {
        if (!finite(l->start) || !finite(l->end) || distance(l->start, l->end) <= kLengthTol)
            return invalid("line geometry is degenerate");
    } else if (const auto* a = std::get_if<ArcGeom>(&geometry)) {
        if (!finite(a->center) || !finite(a->radius) || !finite(a->startAngle) || !finite(a->sweep) ||
            !(a->radius > kLengthTol) || !(a->sweep > 0.0 && a->sweep < kTwoPi))
            return invalid("arc geometry is invalid");
    } else if (const auto* c = std::get_if<CircleGeom>(&geometry)) {
        if (!finite(c->center) || !finite(c->radius) || !(c->radius > kLengthTol))
            return invalid("circle geometry is invalid");
    }
    return {};
}

} // namespace

Result<void> Sketch::setGeometry(SketchEntityId id, EntityGeometry geometry) {
    auto it = entities_.find(id);
    if (it == entities_.end()) return makeError(ErrorCode::NotFound, "sketch: no such entity");
    if (geometry.index() != it->second.geometry.index()) return invalid("setGeometry must keep the entity kind");
    if (auto ok = validateGeometry(geometry); !ok) return ok;
    it->second.geometry = std::move(geometry);
    return {};
}

bool Sketch::idInUse(std::uint64_t id) const noexcept {
    return entities_.contains(SketchEntityId{id}) || constraints_.contains(SketchConstraintId{id}) ||
           dimensions_.contains(SketchDimensionId{id});
}

Result<void> Sketch::restoreEntity(SketchEntityId id, EntityGeometry geometry, bool construction) {
    if (!id.valid()) return invalid("restored entity id must be non-zero");
    if (idInUse(id.value())) return invalid("restored entity id is already in use");
    if (auto ok = validateGeometry(geometry); !ok) return ok;
    entities_.emplace(id, SketchEntity{id, construction, std::move(geometry)});
    nextId_ = std::max(nextId_, id.value() + 1);
    return {};
}

Result<void> Sketch::restoreConstraint(SketchConstraintId id, ConstraintKind kind, std::vector<SketchRef> refs) {
    if (!id.valid()) return invalid("restored constraint id must be non-zero");
    if (idInUse(id.value())) return invalid("restored constraint id is already in use");
    for (const SketchRef& r : refs)
        if (auto ok = checkRef(r); !ok) return std::unexpected(ok.error());
    constraints_.emplace(id, SketchConstraint{id, kind, std::move(refs)});
    nextId_ = std::max(nextId_, id.value() + 1);
    return {};
}

Result<void> Sketch::restoreDimension(SketchDimensionId id, DimensionKind kind, std::vector<SketchRef> refs,
                                      std::string expression, bool driven) {
    if (!id.valid()) return invalid("restored dimension id must be non-zero");
    if (idInUse(id.value())) return invalid("restored dimension id is already in use");
    for (const SketchRef& r : refs)
        if (auto ok = checkRef(r); !ok) return std::unexpected(ok.error());
    dimensions_.emplace(id, SketchDimension{id, kind, std::move(refs), std::move(expression), driven});
    nextId_ = std::max(nextId_, id.value() + 1);
    return {};
}

Result<void> Sketch::restoreNextId(std::uint64_t next) {
    std::uint64_t maxLive = 0;
    // Reference geometry lives in the reserved range above every allocated id: it does not count.
    for (const auto& [id, e] : entities_)
        if (!isReferenceEntity(id)) maxLive = std::max(maxLive, id.value());
    for (const auto& [id, c] : constraints_) maxLive = std::max(maxLive, id.value());
    for (const auto& [id, d] : dimensions_) maxLive = std::max(maxLive, id.value());
    if (next <= maxLive) return invalid("next id must be greater than every live id");
    if (next < nextId_) return invalid("next id must not lower the counter (retired ids would be reused)");
    nextId_ = next;
    return {};
}

Result<void> Sketch::setConstruction(SketchEntityId id, bool construction) {
    auto it = entities_.find(id);
    if (it == entities_.end()) return makeError(ErrorCode::NotFound, "sketch: no such entity");
    if (isReferenceEntity(id)) return invalid("reference geometry is always construction");
    it->second.construction = construction;
    return {};
}

Result<void> Sketch::removeEntity(SketchEntityId id) {
    auto it = entities_.find(id);
    if (it == entities_.end()) return makeError(ErrorCode::NotFound, "sketch: no such entity");
    if (isReferenceEntity(id)) return invalid("reference geometry (origin, axes) cannot be removed");
    entities_.erase(it);
    auto refersTo = [id](const SketchRef& r) { return r.entity == id; };
    for (auto c = constraints_.begin(); c != constraints_.end();) {
        const auto& refs = c->second.refs;
        c = std::any_of(refs.begin(), refs.end(), refersTo) ? constraints_.erase(c) : std::next(c);
    }
    for (auto d = dimensions_.begin(); d != dimensions_.end();) {
        const auto& refs = d->second.refs;
        d = std::any_of(refs.begin(), refs.end(), refersTo) ? dimensions_.erase(d) : std::next(d);
    }
    return {};
}

const SketchEntity* Sketch::findEntity(SketchEntityId id) const noexcept {
    auto it = entities_.find(id);
    return it == entities_.end() ? nullptr : &it->second;
}

Result<Point2> Sketch::pointOf(const SketchRef& ref) const {
    if (auto ok = checkRef(ref); !ok) return std::unexpected(ok.error());
    const SketchEntity& e = entities_.at(ref.entity);
    if (const auto* p = std::get_if<PointGeom>(&e.geometry)) return p->p;
    if (const auto* l = std::get_if<LineGeom>(&e.geometry)) {
        if (!ref.role || *ref.role == PointRole::Start) return l->start;
        if (*ref.role == PointRole::End) return l->end;
        return Point2{0.5 * (l->start.x + l->end.x), 0.5 * (l->start.y + l->end.y)};
    }
    if (const auto* a = std::get_if<ArcGeom>(&e.geometry)) {
        if (!ref.role) return a->center;
        switch (*ref.role) {
        case PointRole::Start: return arcStart(*a);
        case PointRole::End: return arcEnd(*a);
        case PointRole::Mid: return arcMid(*a);
        case PointRole::Center: return a->center;
        }
    }
    if (const auto* c = std::get_if<CircleGeom>(&e.geometry)) return c->center;
    return makeError(ErrorCode::Internal, "sketch: unknown entity kind");
}

// ---- Sketch: records -----------------------------------------------------------------------------------------

Result<void> Sketch::checkRef(const SketchRef& ref) const {
    const SketchEntity* e = findEntity(ref.entity);
    if (!e) return makeError(ErrorCode::NotFound, "sketch: reference to a missing entity");
    if (!ref.role) return {};
    const PointRole r = *ref.role;
    bool ok = false;
    switch (e->kind()) {
    case EntityKind::Point: ok = r == PointRole::Start; break;
    case EntityKind::Line: ok = r == PointRole::Start || r == PointRole::End || r == PointRole::Mid; break;
    case EntityKind::Arc: ok = true; break;
    case EntityKind::Circle: ok = r == PointRole::Center; break;
    }
    if (!ok) return invalid("point role is not valid for this entity kind");
    return {};
}

Result<SketchConstraintId> Sketch::addConstraint(ConstraintKind kind, std::vector<SketchRef> refs) {
    for (const SketchRef& r : refs)
        if (auto ok = checkRef(r); !ok) return std::unexpected(ok.error());
    const SketchConstraintId id{allocateId()};
    constraints_.emplace(id, SketchConstraint{id, kind, std::move(refs)});
    return id;
}

Result<SketchDimensionId> Sketch::addDimension(DimensionKind kind, std::vector<SketchRef> refs,
                                               std::string expression, bool driven) {
    for (const SketchRef& r : refs)
        if (auto ok = checkRef(r); !ok) return std::unexpected(ok.error());
    const SketchDimensionId id{allocateId()};
    dimensions_.emplace(id, SketchDimension{id, kind, std::move(refs), std::move(expression), driven});
    return id;
}

Result<void> Sketch::removeConstraint(SketchConstraintId id) {
    if (constraints_.erase(id) == 0) return makeError(ErrorCode::NotFound, "sketch: no such constraint");
    return {};
}

Result<void> Sketch::removeDimension(SketchDimensionId id) {
    if (dimensions_.erase(id) == 0) return makeError(ErrorCode::NotFound, "sketch: no such dimension");
    return {};
}

Result<void> Sketch::setDimensionExpression(SketchDimensionId id, std::string expression) {
    auto it = dimensions_.find(id);
    if (it == dimensions_.end()) return makeError(ErrorCode::NotFound, "sketch: no such dimension");
    it->second.expression = std::move(expression);
    return {};
}

Result<void> Sketch::setDimensionDriven(SketchDimensionId id, bool driven) {
    auto it = dimensions_.find(id);
    if (it == dimensions_.end()) return makeError(ErrorCode::NotFound, "sketch: no such dimension");
    it->second.driven = driven;
    return {};
}

Result<double> Sketch::measureDimension(SketchDimensionId id) const {
    auto it = dimensions_.find(id);
    if (it == dimensions_.end()) return makeError(ErrorCode::NotFound, "sketch: no such dimension");
    const SketchDimension& d = it->second;
    auto bad = [&](const char* what) { return makeError(ErrorCode::InvalidArgument, std::string("dimension ") + std::to_string(id.value()) + ": " + what); };
    switch (d.kind) {
    case DimensionKind::Distance:
    case DimensionKind::DistanceX:
    case DimensionKind::DistanceY: {
        if (d.refs.size() != 2) return bad("needs two points");
        auto a = pointOf(d.refs[0]);
        auto b = pointOf(d.refs[1]);
        if (!a) return std::unexpected(a.error());
        if (!b) return std::unexpected(b.error());
        const double dx = b->x - a->x, dy = b->y - a->y;
        if (d.kind == DimensionKind::DistanceX) return std::abs(dx);
        if (d.kind == DimensionKind::DistanceY) return std::abs(dy);
        return std::hypot(dx, dy);
    }
    case DimensionKind::Radius:
    case DimensionKind::Diameter: {
        if (d.refs.size() != 1) return bad("needs one arc or circle");
        const SketchEntity* e = findEntity(d.refs[0].entity);
        if (!e) return makeError(ErrorCode::NotFound, "dimension references a missing entity");
        double r = 0;
        if (const auto* c = std::get_if<CircleGeom>(&e->geometry)) r = c->radius;
        else if (const auto* a = std::get_if<ArcGeom>(&e->geometry)) r = a->radius;
        else return bad("needs an arc or circle");
        return d.kind == DimensionKind::Radius ? r : 2 * r;
    }
    case DimensionKind::Angle: {
        if (d.refs.size() != 2) return bad("needs two lines");
        Point2 dirs[2];
        for (int i = 0; i < 2; ++i) {
            const SketchEntity* e = findEntity(d.refs[i].entity);
            if (!e) return makeError(ErrorCode::NotFound, "dimension references a missing entity");
            const auto* l = std::get_if<LineGeom>(&e->geometry);
            if (!l) return bad("angle needs two lines");
            dirs[i] = {l->end.x - l->start.x, l->end.y - l->start.y};
        }
        const double n = std::hypot(dirs[0].x, dirs[0].y) * std::hypot(dirs[1].x, dirs[1].y);
        if (!(n > 1e-12)) return bad("angle of a zero-length line");
        const double c = std::clamp((dirs[0].x * dirs[1].x + dirs[0].y * dirs[1].y) / n, -1.0, 1.0);
        return std::acos(c);
    }
    }
    return bad("unknown dimension kind");
}

} // namespace iw::sketch
