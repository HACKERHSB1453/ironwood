// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/sketch/sketch_overlay.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPen>
#include <QColor>

#include <QFont>
#include <QFontMetricsF>

#include <algorithm>
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

const QColor kDimension{190, 200, 215};     // dimension lines and text
const QColor kDimensionLight{50, 62, 84};   // dimension lines and text on a light theme
const QColor kDriven{150, 158, 172};        // driven (reference) dimensions
const QColor kConflict{235, 64, 52};        // conflicting / redundant constraints and dimensions

QPolygonF arrowHead(QPointF tip, QPointF dirToTip) {
    const double len = std::hypot(dirToTip.x(), dirToTip.y());
    if (len < 1e-9) return {};
    const QPointF u(dirToTip.x() / len, dirToTip.y() / len), n(-u.y(), u.x());
    const QPointF base = tip - u * 9.0;
    return QPolygonF{tip, base + n * 3.0, base - n * 3.0};
}

void paintGlyphs(QPainter& p, const render::Camera& cam, double w, double h, const SketchSession& session) {
    const auto sel = session.selectedConstraint();
    QFont f = p.font();
    f.setPixelSize(11);
    f.setBold(true);
    p.setFont(f);
    for (const ConstraintGlyph& g : session.glyphs()) {
        const auto at = glyphScreenPos(cam, w, h, session.plane(), g);
        if (!at) continue;
        QColor col = g.conflicting ? kConflict : kCurve;
        if (sel && *sel == g.id) col = kSelected;
        p.setPen(QPen(col, 1.3));
        p.setBrush(Qt::NoBrush);
        if (g.label.empty()) {
            p.drawEllipse(*at, 4.5, 4.5);
        } else {
            const QString text = QString::fromStdString(g.label);
            const QRectF box(at->x() - 8, at->y() - 8, 16 + 4.0 * (text.size() - 1), 16);
            p.setBrush(QColor(20, 24, 32, 150));
            p.drawRoundedRect(box, 3, 3);
            p.drawText(box, Qt::AlignCenter, text);
        }
    }
}

void paintDimensions(QPainter& p, const render::Camera& cam, double w, double h, const SketchSession& session,
                     bool dark) {
    const auto sel = session.selectedDimension();
    QFont f = p.font();
    f.setPixelSize(12);
    f.setBold(false);
    p.setFont(f);
    for (const DimensionView& d : session.dimensionViews()) {
        const auto lay = layoutDimension(cam, w, h, session, d);
        if (!lay) continue;
        QColor col = d.driven ? kDriven : (dark ? kDimension : kDimensionLight);
        if (d.conflicting) col = kConflict;
        if (sel && sel->id == d.id && sel->driven == d.driven) col = kSelected;
        p.setPen(QPen(col, 1.0, d.driven ? Qt::DashLine : Qt::SolidLine));
        for (const QLineF& l : lay->lines) p.drawLine(l);
        p.setPen(QPen(col, 1.0));
        p.setBrush(col);
        for (const QPolygonF& a : lay->arrows) p.drawPolygon(a);
        p.setBrush(Qt::NoBrush);
        p.drawText(lay->textRect, Qt::AlignCenter, lay->text);
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

    const QColor base = active ? sketchCurveColor(session.constraintState(), opts.dark) : kFinished;
    for (const auto& [id, e] : s.entities()) {
        const bool selected = active && std::any_of(session.selection().begin(), session.selection().end(),
                                                    [&](const sketch::SketchRef& r) { return r.entity == id && !r.role; });
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

    if (active) {
        // Selected points and the Dimension tool's picks.
        p.setPen(QPen(kSelected, 2.0));
        p.setBrush(Qt::NoBrush);
        auto ring = [&](const sketch::SketchRef& r) {
            if (!r.role && s.findEntity(r.entity) && s.findEntity(r.entity)->kind() != sketch::EntityKind::Point) return;
            if (const auto q = s.pointOf(r)) {
                if (auto sc = proj(*q)) p.drawEllipse(*sc, 6.0, 6.0);
            }
        };
        for (const sketch::SketchRef& r : session.selection()) ring(r);
        for (const sketch::SketchRef& r : session.dimensionPicks()) ring(r);
        // Whole entities picked for a dimension are highlighted like a selection.
        for (const sketch::SketchRef& r : session.dimensionPicks()) {
            const sketch::SketchEntity* e = s.findEntity(r.entity);
            if (e && !r.role && e->kind() != sketch::EntityKind::Point) {
                p.setPen(QPen(kSelected, 2.5));
                strokeCurve(p, proj, e->geometry);
            }
        }
    }

    if (opts.constraints && active) paintGlyphs(p, cam, w, h, session);
    if (opts.dimensions) paintDimensions(p, cam, w, h, session, opts.dark);

    if (active) {
        p.setPen(QPen(kPreview, 1.5, Qt::DashLine));
        for (const sketch::EntityGeometry& g : session.preview()) strokeCurve(p, proj, g);
        if (session.snapHint()) paintSnap(p, proj, *session.snapHint());
    }
}

} // namespace

QColor sketchCurveColor(ConstraintState state, bool dark) {
    if (state == ConstraintState::Fully) return dark ? QColor(245, 245, 245) : QColor(20, 20, 20);
    return kCurve;
}

std::optional<QPointF> glyphScreenPos(const render::Camera& cam, double width, double height,
                                      const sketch::SketchPlane& plane, const ConstraintGlyph& g) {
    const auto s = planeToScreen(cam, width, height, plane, g.anchor);
    if (!s) return std::nullopt;
    const bool ring = g.label.empty();
    return QPointF(s->x + 14.0 * g.slot + (ring ? 0.0 : 10.0), s->y - (ring ? 0.0 : 12.0));
}

std::optional<DimensionLayout> layoutDimension(const render::Camera& cam, double width, double height,
                                               const SketchSession& session, const DimensionView& dim) {
    using DK = sketch::DimensionKind;
    const sketch::Sketch& sk = session.sketch();
    const sketch::SketchPlane& plane = sk.plane();
    auto scr = [&](Point2 q) -> std::optional<QPointF> {
        const auto s = planeToScreen(cam, width, height, plane, q);
        if (!s) return std::nullopt;
        return QPointF(s->x, s->y);
    };
    DimensionLayout out;
    out.text = QString::fromStdString(dim.text);
    QPointF textAt;
    constexpr double kOff = 28.0;
    auto addDimLine = [&](QPointF a, QPointF b) {
        out.lines.emplace_back(a, b);
        out.arrows.push_back(arrowHead(a, a - b));
        out.arrows.push_back(arrowHead(b, b - a));
    };

    switch (dim.kind) {
    case DK::Distance:
    case DK::DistanceX:
    case DK::DistanceY: {
        if (dim.refs.size() != 2) return std::nullopt;
        const auto pa = sk.pointOf(dim.refs[0]), pb = sk.pointOf(dim.refs[1]);
        if (!pa || !pb) return std::nullopt;
        const auto a = scr(*pa), b = scr(*pb);
        if (!a || !b) return std::nullopt;
        if (dim.kind == DK::Distance) {
            const QPointF d = *b - *a;
            const double len = std::hypot(d.x(), d.y());
            if (len < 1e-6) return std::nullopt;
            const QPointF n(-d.y() / len, d.x() / len);
            const QPointF a2 = *a + n * kOff, b2 = *b + n * kOff;
            out.lines.emplace_back(*a, a2 + n * 4.0);
            out.lines.emplace_back(*b, b2 + n * 4.0);
            addDimLine(a2, b2);
            textAt = (a2 + b2) / 2.0 + n * 10.0;
        } else if (dim.kind == DK::DistanceX) {
            const double y = std::min(a->y(), b->y()) - kOff;
            out.lines.emplace_back(*a, QPointF(a->x(), y - 4.0));
            out.lines.emplace_back(*b, QPointF(b->x(), y - 4.0));
            addDimLine(QPointF(a->x(), y), QPointF(b->x(), y));
            textAt = QPointF((a->x() + b->x()) / 2.0, y - 10.0);
        } else {
            const double x = std::max(a->x(), b->x()) + kOff;
            out.lines.emplace_back(*a, QPointF(x + 4.0, a->y()));
            out.lines.emplace_back(*b, QPointF(x + 4.0, b->y()));
            addDimLine(QPointF(x, a->y()), QPointF(x, b->y()));
            textAt = QPointF(x + 12.0, (a->y() + b->y()) / 2.0);
        }
        break;
    }
    case DK::Radius:
    case DK::Diameter: {
        if (dim.refs.size() != 1) return std::nullopt;
        const sketch::SketchEntity* e = sk.findEntity(dim.refs[0].entity);
        if (!e) return std::nullopt;
        Point2 c{};
        double r = 0;
        if (const auto* circ = std::get_if<sketch::CircleGeom>(&e->geometry)) {
            c = circ->center;
            r = circ->radius;
        } else if (const auto* arc = std::get_if<sketch::ArcGeom>(&e->geometry)) {
            c = arc->center;
            r = arc->radius;
        } else {
            return std::nullopt;
        }
        const double ang = std::numbers::pi / 4;
        const Point2 rim{c.x + r * std::cos(ang), c.y + r * std::sin(ang)};
        const auto sc = scr(c), sr = scr(rim);
        if (!sc || !sr) return std::nullopt;
        const QPointF d = *sr - *sc;
        const double len = std::hypot(d.x(), d.y());
        if (len < 1e-6) return std::nullopt;
        const QPointF u = d / len;
        if (dim.kind == DK::Radius) {
            out.lines.emplace_back(*sc, *sr);
            out.arrows.push_back(arrowHead(*sr, u));
            textAt = *sr + u * 18.0;
        } else {
            const QPointF opp = *sc - d;
            out.lines.emplace_back(opp, *sr);
            out.arrows.push_back(arrowHead(*sr, u));
            out.arrows.push_back(arrowHead(opp, -u));
            textAt = *sr + u * 18.0;
        }
        break;
    }
    case DK::Angle: {
        if (dim.refs.size() != 2) return std::nullopt;
        const sketch::SketchEntity* ea = sk.findEntity(dim.refs[0].entity);
        const sketch::SketchEntity* eb = sk.findEntity(dim.refs[1].entity);
        if (!ea || !eb) return std::nullopt;
        const auto* la = std::get_if<sketch::LineGeom>(&ea->geometry);
        const auto* lb = std::get_if<sketch::LineGeom>(&eb->geometry);
        if (!la || !lb) return std::nullopt;
        const Point2 d1 = la->end - la->start, d2 = lb->end - lb->start;
        const double den = d1.x * d2.y - d1.y * d2.x;
        Point2 x = la->start;
        if (std::abs(den) > 1e-9) {
            const Point2 w0 = lb->start - la->start;
            const double t = (w0.x * d2.y - w0.y * d2.x) / den;
            x = la->start + d1 * t;
        }
        const Point2 ma = (la->start + la->end) * 0.5, mb = (lb->start + lb->end) * 0.5;
        const auto sx = scr(x), sma = scr(ma), smb = scr(mb);
        if (!sx || !sma || !smb) return std::nullopt;
        const QPointF ua = *sma - *sx, ub = *smb - *sx;
        const double a0 = std::atan2(ua.y(), ua.x());
        double da = std::atan2(ub.y(), ub.x()) - a0;
        while (da > std::numbers::pi) da -= 2 * std::numbers::pi;
        while (da < -std::numbers::pi) da += 2 * std::numbers::pi;
        constexpr double kR = 44.0;
        QPointF prev = *sx + QPointF(kR * std::cos(a0), kR * std::sin(a0));
        for (int k = 1; k <= 16; ++k) {
            const double a = a0 + da * k / 16.0;
            const QPointF q = *sx + QPointF(kR * std::cos(a), kR * std::sin(a));
            out.lines.emplace_back(prev, q);
            prev = q;
        }
        const double am = a0 + da / 2;
        textAt = *sx + QPointF((kR + 16.0) * std::cos(am), (kR + 16.0) * std::sin(am));
        break;
    }
    }
    QFont f;
    f.setPixelSize(12);
    const QFontMetricsF fm(f);
    const double tw = fm.horizontalAdvance(out.text) + 8.0, th = fm.height() + 2.0;
    out.textRect = QRectF(textAt.x() - tw / 2, textAt.y() - th / 2, tw, th);
    return out;
}

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
