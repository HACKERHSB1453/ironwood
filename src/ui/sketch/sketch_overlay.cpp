// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/sketch/sketch_overlay.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPen>
#include <QColor>

#include <QFont>

#include <cmath>
#include <numbers>
#include <optional>
#include <type_traits>
#include <variant>

#include "render/camera.hpp"
#include "render/viewport.hpp"
#include "sketch/profile.hpp"
#include "sketch/sketch.hpp"
#include "ui/sketch/session.hpp"
#include "ui/sketch/view_math.hpp"

namespace iw::ui {
namespace {

using sketch::Point2;

constexpr int kCircleSegments = 96;
constexpr int kArcSegments = 48;

const QColor kCurve{74, 144, 226};          // blue: under-constrained curves (B3b colours them properly)
const QColor kConstruction{138, 148, 166};  // grey dashed
const QColor kSelected{255, 210, 63};       // yellow
const QColor kPreview{156, 195, 255};       // light blue dashed
const QColor kProfile{255, 170, 80, 90};    // translucent light orange
const QColor kFinished{107, 119, 144};      // dimmed curves of sketches that are no longer active

/// Projects sketch points to widget pixels; points behind the eye are dropped.
class Projector {
public:
    Projector(const render::Camera& cam, const sketch::SketchPlane& plane, double w, double h)
        : cam_(cam), plane_(plane), w_(w), h_(h) {}
    std::optional<QPointF> operator()(Point2 p) const {
        const auto s = planeToScreen(cam_, w_, h_, plane_, p);
        if (!s) return std::nullopt;
        return QPointF(s->x, s->y);
    }

private:
    const render::Camera& cam_;
    const sketch::SketchPlane& plane_;
    double w_, h_;
};

/// Points of an arc (or full circle) sampled CCW, as sketch coordinates.
template <class Fn> void sampleArc(Point2 c, double r, double a0, double sweep, int segments, Fn&& sink) {
    for (int k = 0; k <= segments; ++k) {
        const double a = a0 + sweep * k / segments;
        sink(Point2{c.x + r * std::cos(a), c.y + r * std::sin(a)});
    }
}

/// Strokes one curve (line, circle or arc) through the projector; points behind the eye are skipped.
void strokeCurve(QPainter& p, const Projector& proj, const sketch::EntityGeometry& g) {
    std::visit(
        [&](const auto& geom) {
            using T = std::decay_t<decltype(geom)>;
            if constexpr (std::is_same_v<T, sketch::LineGeom>) {
                const auto a = proj(geom.start), b = proj(geom.end);
                if (a && b) p.drawLine(*a, *b);
            } else if constexpr (std::is_same_v<T, sketch::CircleGeom>) {
                QPolygonF poly;
                sampleArc(geom.center, geom.radius, 0, 2 * std::numbers::pi, kCircleSegments, [&](Point2 q) {
                    if (auto s = proj(q)) poly << *s;
                });
                p.drawPolyline(poly);
            } else if constexpr (std::is_same_v<T, sketch::ArcGeom>) {
                QPolygonF poly;
                sampleArc(geom.center, geom.radius, geom.startAngle, geom.sweep, kArcSegments, [&](Point2 q) {
                    if (auto s = proj(q)) poly << *s;
                });
                p.drawPolyline(poly);
            }
        },
        g);
}

void paintProfiles(QPainter& p, const Projector& proj, const SketchSession& session) {
    // One path per profile: outer loop plus its holes, odd-even. Sharing one path across profiles would count a
    // disc's boundary together with its ring's hole and fill the hole again.
    for (const sketch::SketchProfile& prof : session.profiles()) {
        QPainterPath path;
        auto addLoop = [&](const sketch::ProfileLoop& loop) {
            if (loop.edges.empty()) return;
            bool first = true;
            auto to = [&](Point2 q) {
                if (auto sc = proj(q)) {
                    if (first) {
                        path.moveTo(*sc);
                        first = false;
                    } else {
                        path.lineTo(*sc);
                    }
                }
            };
            for (const sketch::ProfileEdge& e : loop.edges) {
                if (!e.isArc) {
                    if (first) to(e.start);
                    to(e.end);
                    continue;
                }
                if (first) to(e.start);
                for (int k = 1; k <= kArcSegments; ++k) {
                    const double a = e.startAngle + e.sweep * k / kArcSegments;
                    to(Point2{e.center.x + e.radius * std::cos(a), e.center.y + e.radius * std::sin(a)});
                }
            }
            path.closeSubpath();
        };
        addLoop(prof.outer);
        for (const sketch::ProfileLoop& h : prof.holes) addLoop(h);
        path.setFillRule(Qt::OddEvenFill);
        p.fillPath(path, kProfile);
    }
}

void paintConstraints(QPainter& p, const Projector& proj, const sketch::Sketch& s) {
    for (const auto& [id, c] : s.constraints()) {
        if (c.kind == sketch::ConstraintKind::Horizontal || c.kind == sketch::ConstraintKind::Vertical) {
            if (c.refs.size() != 1) continue;
            const sketch::SketchEntity* e = s.findEntity(c.refs[0].entity);
            if (!e || e->kind() != sketch::EntityKind::Line) continue;
            const auto& line = std::get<sketch::LineGeom>(e->geometry);
            const auto a = proj(line.start), b = proj(line.end);
            if (!a || !b) continue;
            const QPointF mid((a->x() + b->x()) / 2, (a->y() + b->y()) / 2);
            p.setPen(QPen(kCurve, 1));
            p.drawText(mid + QPointF(-4, -6),
                       c.kind == sketch::ConstraintKind::Horizontal ? QStringLiteral("H") : QStringLiteral("V"));
        } else if (c.kind == sketch::ConstraintKind::Coincident && c.refs.size() == 2) {
            const auto pt = s.pointOf(c.refs[0]);
            if (!pt) continue;
            if (auto sc = proj(*pt)) {
                p.setPen(QPen(kCurve, 1));
                p.setBrush(Qt::NoBrush);
                p.drawEllipse(*sc, 4.0, 4.0);
            }
        }
    }
}

const QColor kSnap{255, 90, 210};          // magenta: snap glyphs and their guides

/// Glyph of a snap at its point plus its dashed guide lines. Shapes: endpoint square, centre circle with cross,
/// midpoint triangle, intersection X, on-curve small circle, H/V tick labels, perpendicular right-angle mark,
/// tangent "T".
void paintSnap(QPainter& p, const Projector& proj, const SnapHit& hit) {
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(kSnap, 1.2, Qt::DashLine));
    for (const auto& [a, b] : hit.guides) {
        const auto sa = proj(a), sb = proj(b);
        if (sa && sb) p.drawLine(*sa, *sb);
    }
    const auto at = proj(hit.point);
    if (!at) return;
    p.setPen(QPen(kSnap, 1.5));
    const double x = at->x(), y = at->y();
    switch (hit.kind) {
    case SnapKind::Endpoint:
        p.drawRect(QRectF(x - 4, y - 4, 8, 8));
        break;
    case SnapKind::Center:
        p.drawEllipse(*at, 5.0, 5.0);
        p.drawLine(QPointF(x - 3, y), QPointF(x + 3, y));
        p.drawLine(QPointF(x, y - 3), QPointF(x, y + 3));
        break;
    case SnapKind::Midpoint:
        p.drawPolygon(QPolygonF{QPointF(x, y - 5), QPointF(x + 5, y + 4), QPointF(x - 5, y + 4)});
        break;
    case SnapKind::Intersection:
        p.drawLine(QPointF(x - 4, y - 4), QPointF(x + 4, y + 4));
        p.drawLine(QPointF(x - 4, y + 4), QPointF(x + 4, y - 4));
        break;
    case SnapKind::OnCurve:
        p.drawEllipse(*at, 2.5, 2.5);
        break;
    case SnapKind::Horizontal:
        p.drawText(QPointF(x + 6, y - 6), QStringLiteral("H"));
        break;
    case SnapKind::Vertical:
        p.drawText(QPointF(x + 6, y - 6), QStringLiteral("V"));
        break;
    case SnapKind::Perpendicular:
        p.drawRect(QRectF(x - 4, y - 4, 4, 4));
        break;
    case SnapKind::Tangent:
        p.drawText(QPointF(x + 6, y - 6), QStringLiteral("T"));
        p.drawEllipse(*at, 4.0, 4.0);
        break;
    }
}

void paintSession(QPainter& p, const render::Camera& cam, double w, double h, const SketchSession& session,
                  bool active, const SketchViewOptions& opts) {
    const sketch::Sketch& s = session.sketch();
    const Projector proj(cam, s.plane(), w, h);

    if (opts.profiles) paintProfiles(p, proj, session);

    // Plane origin with the sketch axes, so the orientation is readable.
    {
        const auto o = proj(Point2{0, 0}), ux = proj(Point2{30.0 / sketchPixelsPerMm(cam, h), 0}),
                   uy = proj(Point2{0, 30.0 / sketchPixelsPerMm(cam, h)});
        if (o && ux && uy && active) {
            p.setPen(QPen(QColor(230, 80, 80), 1.5));
            p.drawLine(*o, *ux);
            p.setPen(QPen(QColor(80, 200, 110), 1.5));
            p.drawLine(*o, *uy);
        }
    }

    const QColor base = active ? kCurve : kFinished;
    for (const auto& [id, e] : s.entities()) {
        const bool selected = active && session.selected() == id;
        QColor col = e.construction ? kConstruction : base;
        if (selected) col = kSelected;
        QPen pen(col, selected ? 2.5 : 2.0);
        if (e.construction) {
            pen.setStyle(Qt::DashLine);
            pen.setWidthF(1.5);
        }
        p.setPen(pen);
        p.setBrush(Qt::NoBrush);
        strokeCurve(p, proj, e.geometry);

        if (opts.points || e.kind() == sketch::EntityKind::Point) {
            p.setPen(QPen(col, 1));
            p.setBrush(col);
            if (const auto* pg = std::get_if<sketch::PointGeom>(&e.geometry)) {
                if (auto sc = proj(pg->p)) p.drawRect(QRectF(sc->x() - 3, sc->y() - 3, 6, 6));
            } else if (opts.points) {
                if (const auto* lg = std::get_if<sketch::LineGeom>(&e.geometry)) {
                    for (const Point2& q : {lg->start, lg->end})
                        if (auto sc = proj(q)) p.drawEllipse(*sc, 2.5, 2.5);
                }
            }
            p.setBrush(Qt::NoBrush);
        }
    }

    if (opts.constraints) {
        p.setPen(QPen(kCurve, 1));
        paintConstraints(p, proj, s);
    }

    if (active) {
        p.setPen(QPen(kPreview, 1.5, Qt::DashLine));
        for (const sketch::EntityGeometry& g : session.preview()) strokeCurve(p, proj, g);
        if (session.snapHint()) paintSnap(p, proj, *session.snapHint());
    }
}

} // namespace

void paintSketches(QPainter& p, const render::Camera& cam, double width, double height,
                   const std::vector<const SketchSession*>& sessions, const SketchSession* active,
                   const SketchViewOptions& opts) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);
    for (const SketchSession* s : sessions) {
        if (!s) continue;
        paintSession(p, cam, width, height, *s, s == active, opts);
    }
    p.restore();
}

SketchOverlay::SketchOverlay(render::Viewport* viewport) : QWidget(viewport), viewport_(viewport) {
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setAttribute(Qt::WA_TranslucentBackground);
    setGeometry(viewport->rect());
    connect(viewport, &render::Viewport::cameraChanged, this, [this] { update(); });
    hide();
}

void SketchOverlay::setContent(std::vector<const SketchSession*> sessions, const SketchSession* active,
                               SketchViewOptions opts) {
    sessions_ = std::move(sessions);
    active_ = active;
    opts_ = opts;
    setGeometry(viewport_->rect());
    setVisible(!sessions_.empty());
    update();
}

void SketchOverlay::paintEvent(QPaintEvent* e) {
    Q_UNUSED(e);
    QPainter p(this);
    paintSketches(p, viewport_->camera(), width(), height(), sessions_, active_, opts_);
}

} // namespace iw::ui
