// SPDX-License-Identifier: GPL-3.0-or-later
#include "render/viewport.hpp"

#include <QElapsedTimer>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPointer>
#include <QTimer>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <rhi/qrhi.h>

#include "render/offscreen.hpp"
#include "render/scene_renderer.hpp"
#include "render/viewcube_widget.hpp"

namespace iw::render {

namespace {
enum class Gesture { None, Orbit, Pan, Zoom, Click };

std::uint8_t modifiersOf(Qt::KeyboardModifiers m) {
    std::uint8_t r = ModNone;
    if (m & Qt::ShiftModifier) r |= ModShift;
    if (m & Qt::ControlModifier) r |= ModCtrl;
    if (m & Qt::AltModifier) r |= ModAlt;
    return r;
}
MouseButton buttonOf(Qt::MouseButton b) {
    switch (b) {
    case Qt::LeftButton: return MouseButton::Left;
    case Qt::MiddleButton: return MouseButton::Middle;
    case Qt::RightButton: return MouseButton::Right;
    default: return MouseButton::None;
    }
}
/// A pick that has not produced a result this long after pick() returns an empty PickResult (no frames rendered,
/// lost device, hidden window). Real GPU picks complete within 1-2 frames.
constexpr int kPickTimeoutMs = 2000;
double easeInOut(double t) { return t < 0.5 ? 4 * t * t * t : 1 - std::pow(-2 * t + 2, 3) / 2; }
} // namespace

struct Viewport::Impl {
    std::vector<SceneBody> scene;
    Box3 bounds;
    Camera camera;
    NavPreset nav = NavPreset::Fusion;
    bool viewCubeVisible = true;
    ViewCubeWidget* cube = nullptr;
    std::unique_ptr<SceneRenderer> renderer;
    QRhi* rhi = nullptr;
    // Headless (QPA offscreen): QRhiWidget gets no paint events there, so picks run on a private QRhi instead.
    // Declared in this order so the renderer is destroyed before the RHI it uses.
    std::unique_ptr<QRhi> headlessRhi;
    std::unique_ptr<SceneRenderer> headlessRenderer;
    PickResult hover, selection;

    /// One pick request. fire() delivers the result exactly once: from the GPU readback, from the watchdog
    /// timeout, or (empty) from teardown.
    struct PickRequest {
        int x = 0, y = 0;
        std::function<void(PickResult)> done;
        bool fired = false;
        void fire(PickResult r) {
            if (fired) return;
            fired = true;
            auto f = std::move(done);
            done = nullptr;
            if (f) f(std::move(r));
        }
    };
    std::shared_ptr<PickRequest> pending;                 ///< Queued, not yet rendered.
    std::vector<std::shared_ptr<PickRequest>> inFlight;   ///< Rendered, readback outstanding.
    void forget(const std::shared_ptr<PickRequest>& r) {
        if (pending == r) pending.reset();
        std::erase(inFlight, r);
    }

    // mouse gesture state
    Gesture gesture = Gesture::None;
    QPointF lastPos, pressPos;
    std::optional<Vec3> pivot;
    std::uint64_t gestureSerial = 0;
    QTimer hoverTimer;
    QPointF hoverPos;
    std::optional<Vec3> hoverPoint;     ///< World point under the cursor from the last hover pick.
    QPointF hoverPointPos;

    // animation
    QTimer animTimer;
    QElapsedTimer animClock;
    CameraState animFrom, animTo;
    int animMs = 0;
    bool animating = false;

    // smooth wheel zoom
    QTimer zoomTimer;
    double zoomRemaining = 0;           ///< log(factor) still to apply
    Vec3 zoomAnchor{};

    bool warned = false;
};

Viewport::Viewport(QWidget* parent) : QRhiWidget(parent), d_(std::make_unique<Impl>()) {
    setSampleCount(4);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
#ifndef NDEBUG
    setDebugLayerEnabled(true);
#endif
    // Backend: Vulkan unless overridden (IRONWOOD_RHI=opengl|null) or running headless (offscreen QPA has no
    // Vulkan/GL surface; Null still exercises the whole code path).
    const QByteArray env = qgetenv("IRONWOOD_RHI").toLower();
    if (env == "opengl") setApi(Api::OpenGL);
    else if (env == "null" || QGuiApplication::platformName() == "offscreen") setApi(Api::Null);
    else setApi(Api::Vulkan);

    d_->camera.setViewportSize(width(), height());
    d_->cube = new ViewCubeWidget(&d_->camera, this);
    connect(d_->cube, &ViewCubeWidget::viewRequested, this,
            [this](double x, double y, double z) { viewFromDirection({x, y, z}); });
    connect(d_->cube, &ViewCubeWidget::homeRequested, this, [this] { viewHome(); });
    connect(d_->cube, &ViewCubeWidget::orbitDragged, this, [this](double dx, double dy) {
        d_->animating = false;
        d_->animTimer.stop();
        d_->camera.orbit(dx, dy);
        update();
        emit cameraChanged();
    });
    connect(this, &Viewport::cameraChanged, d_->cube, qOverload<>(&QWidget::update));

    d_->hoverTimer.setSingleShot(true);
    d_->hoverTimer.setInterval(40);
    connect(&d_->hoverTimer, &QTimer::timeout, this, [this] {
        if (d_->gesture != Gesture::None) return;
        const QPointF p = d_->hoverPos;
        pick(static_cast<int>(p.x()), static_cast<int>(p.y()), [this, p](PickResult r) {
            if (r.point) { d_->hoverPoint = r.point; d_->hoverPointPos = p; }
            else d_->hoverPoint.reset();
            const bool changed = r.bodyIndex != d_->hover.bodyIndex || r.faceIndex != d_->hover.faceIndex ||
                                 r.edgeIndex != d_->hover.edgeIndex;
            d_->hover = r;
            if (d_->renderer) d_->renderer->setHover(r);
            if (changed) { update(); emit hoverChanged(r); }
        });
    });

    d_->animTimer.setInterval(8);
    connect(&d_->animTimer, &QTimer::timeout, this, [this] {
        const double t = std::min(1.0, d_->animClock.elapsed() / static_cast<double>(std::max(d_->animMs, 1)));
        d_->camera.setState(lerp(d_->animFrom, d_->animTo, easeInOut(t)));
        if (t >= 1.0) {
            d_->camera.setState(d_->animTo);
            d_->animating = false;
            d_->animTimer.stop();
        }
        update();
        emit cameraChanged();
    });

    d_->zoomTimer.setInterval(8);
    connect(&d_->zoomTimer, &QTimer::timeout, this, [this] {
        auto& z = d_->zoomRemaining;
        double step = z * 0.30;
        if (std::abs(z) < 2e-3) step = z;
        z -= step;
        d_->camera.zoomAbout(std::exp(step), d_->zoomAnchor);
        if (z == 0.0) d_->zoomTimer.stop();
        update();
        emit cameraChanged();
    });
}

Viewport::~Viewport() { flushPicks(); }

Box3 Viewport::sceneBounds() const {
    Box3 b;
    for (const auto& body : d_->scene) {
        if (!body.visible || !body.mesh || body.mesh->bounds.isEmpty()) continue;
        const auto& mb = body.mesh->bounds;
        for (int i = 0; i < 8; ++i)
            b.extend(body.transform.apply({(i & 1) ? mb.max.x : mb.min.x, (i & 2) ? mb.max.y : mb.min.y,
                                           (i & 4) ? mb.max.z : mb.min.z}));
    }
    return b;
}

void Viewport::setScene(std::vector<SceneBody> bodies) {
    const bool wasEmpty = d_->bounds.isEmpty();
    d_->scene = std::move(bodies);
    d_->bounds = sceneBounds();
    d_->camera.setSceneBounds(d_->bounds);
    d_->hover = d_->selection = PickResult{};
    d_->hoverPoint.reset();
    if (d_->renderer) {
        d_->renderer->setScene(d_->scene);
        d_->renderer->setHover({});
        d_->renderer->setSelection({});
    }
    if (wasEmpty && !d_->bounds.isEmpty()) viewHome(0);
    update();
    emit cameraChanged();
}

void Viewport::setBodyVisible(std::size_t index, bool visible) {
    if (index >= d_->scene.size() || d_->scene[index].visible == visible) return;
    d_->scene[index].visible = visible;
    d_->bounds = sceneBounds();
    d_->camera.setSceneBounds(d_->bounds);
    if (d_->renderer) d_->renderer->setScene(d_->scene);
    update();
    emit cameraChanged();
}

const std::vector<SceneBody>& Viewport::scene() const noexcept { return d_->scene; }
Camera& Viewport::camera() noexcept { return d_->camera; }
const Camera& Viewport::camera() const noexcept { return d_->camera; }
void Viewport::setNavPreset(NavPreset p) { d_->nav = p; }
NavPreset Viewport::navPreset() const noexcept { return d_->nav; }

void Viewport::setViewCubeVisible(bool v) {
    d_->viewCubeVisible = v;
    d_->cube->setVisible(v);
    update();
}
bool Viewport::viewCubeVisible() const noexcept { return d_->viewCubeVisible; }

void Viewport::setUpAxis(UpAxis axis) {
    if (axis == d_->camera.upAxis()) return;
    d_->camera.setUpAxis(axis);
    viewHome(0);
}
UpAxis Viewport::upAxis() const noexcept { return d_->camera.upAxis(); }

void Viewport::setProjectionMode(Projection p) {
    d_->camera.setProjection(p);
    update();
    emit cameraChanged();
}
Projection Viewport::projectionMode() const noexcept { return d_->camera.projection(); }

bool Viewport::isAnimating() const noexcept { return d_->animating; }

void Viewport::animateTo(const CameraState& s, int ms) {
    d_->zoomRemaining = 0;
    d_->zoomTimer.stop();
    if (ms <= 0) {
        d_->animating = false;
        d_->animTimer.stop();
        d_->camera.setState(s);
        update();
        emit cameraChanged();
        return;
    }
    d_->animFrom = d_->camera.state();
    d_->animTo = s;
    d_->animMs = ms;
    d_->animClock.start();
    d_->animating = true;
    d_->animTimer.start();
}

void Viewport::viewHome(int ms) {
    Camera c = d_->camera;
    if (!d_->bounds.isEmpty()) {
        c.setState(c.poseForEyeDirection(c.homeEyeDirection()));
        c.fit(d_->bounds);
    } else {
        CameraState s = c.state();
        s = c.poseForEyeDirection(c.homeEyeDirection());
        c.setState(s);
    }
    animateTo(c.state(), ms);
}

void Viewport::viewFit(int ms) {
    if (d_->bounds.isEmpty()) return;
    Camera c = d_->camera;
    c.fit(d_->bounds);
    animateTo(c.state(), ms);
}

void Viewport::viewFromDirection(const Vec3& eyeDir, int ms) {
    animateTo(d_->camera.poseForEyeDirection(eyeDir), ms);
}

void Viewport::setSelection(const PickResult& r) {
    d_->selection = r;
    if (d_->renderer) d_->renderer->setSelection(r);
    update();
}
PickResult Viewport::selection() const noexcept { return d_->selection; }
PickResult Viewport::hover() const noexcept { return d_->hover; }

bool Viewport::headless() {
    return QGuiApplication::platformName() == QLatin1String("offscreen");
}

void Viewport::serviceHeadlessPick() {
    if (!d_->pending) return;
    if (!d_->headlessRhi) {
        d_->headlessRhi = createHeadlessRhi();
        if (d_->headlessRhi) d_->headlessRenderer = std::make_unique<SceneRenderer>(d_->headlessRhi.get());
    }
    auto req = d_->pending;
    d_->pending.reset();
    QRhi* rhi = d_->headlessRhi.get();
    QRhiCommandBuffer* cb = nullptr;
    if (!rhi || rhi->beginOffscreenFrame(&cb) != QRhi::FrameOpSuccess) {
        req->fire(PickResult{});
        return;
    }
    d_->headlessRenderer->setScene(d_->scene);
    const QSize px(std::max(width(), 1), std::max(height(), 1));
    d_->inFlight.push_back(req);
    const bool ok = d_->headlessRenderer->submitPick(cb, d_->camera, px, req->x, req->y, [req](PickResult r) {
        req->fire(std::move(r));
    });
    rhi->endOffscreenFrame();
    rhi->finish();   // the readback completes inside finish()
    if (!ok) {
        std::erase(d_->inFlight, req);
        req->fire(PickResult{});
    }
    d_->forget(req);
}

void Viewport::pick(int x, int y, std::function<void(PickResult)> done) {
    if (d_->pending) {
        auto old = d_->pending;
        d_->pending.reset();
        QTimer::singleShot(0, this, [old] { old->fire(PickResult{}); });
    }
    auto req = std::make_shared<Impl::PickRequest>();
    req->x = x;
    req->y = y;
    req->done = std::move(done);
    d_->pending = req;
    if (headless()) QTimer::singleShot(0, this, [this] { serviceHeadlessPick(); });
    // Watchdog: frames may never arrive (lost device, hidden window). The caller always gets an answer.
    QPointer<Viewport> self(this);
    QTimer::singleShot(kPickTimeoutMs, this, [self, req] {
        if (self) self->d_->forget(req);
        req->fire(PickResult{});
    });
    update();
}

void Viewport::flushPicks() {
    std::vector<std::shared_ptr<Impl::PickRequest>> all = std::move(d_->inFlight);
    d_->inFlight.clear();
    if (d_->pending) {
        all.push_back(std::move(d_->pending));
        d_->pending.reset();
    }
    for (auto& r : all) r->fire(PickResult{});
}

void Viewport::initialize(QRhiCommandBuffer*) {
    if (d_->renderer && d_->rhi == rhi()) return;
    d_->rhi = rhi();
    d_->renderer = std::make_unique<SceneRenderer>(rhi());
    d_->renderer->setScene(d_->scene);
    d_->renderer->setHover(d_->hover);
    d_->renderer->setSelection(d_->selection);
}

void Viewport::releaseResources() {
    flushPicks();
    d_->headlessRenderer.reset();
    d_->headlessRhi.reset();
    d_->renderer.reset();
    d_->rhi = nullptr;
}

void Viewport::render(QRhiCommandBuffer* cb) {
    if (!d_->renderer) return;
    auto* rt = renderTarget();
    const QSize px = rt->pixelSize();
    d_->renderer->render(cb, rt, d_->camera, px);

    if (d_->pending && d_->renderer->canSubmitPick()) {
        auto req = d_->pending;
        d_->pending.reset();
        const qreal dpr = devicePixelRatioF();
        const int x = std::clamp(static_cast<int>(req->x * dpr), 0, std::max(px.width() - 1, 0));
        const int y = std::clamp(static_cast<int>(req->y * dpr), 0, std::max(px.height() - 1, 0));
        QPointer<Viewport> self(this);
        d_->inFlight.push_back(req);
        const bool ok = d_->renderer->submitPick(cb, d_->camera, px, x, y, [self, req](PickResult r) {
            if (!self) {
                req->fire(std::move(r));
                return;
            }
            QTimer::singleShot(0, self, [self, req, r] {
                self->d_->forget(req);
                req->fire(r);
            });
        });
        if (!ok) {
            std::erase(d_->inFlight, req);
            QTimer::singleShot(0, this, [req] { req->fire(PickResult{}); });
        }
    }
    // Readbacks complete on a later frame; keep frames coming until they are drained.
    if (!d_->inFlight.empty() || d_->pending) QTimer::singleShot(8, this, [this] { update(); });
}

void Viewport::resizeEvent(QResizeEvent* e) {
    QRhiWidget::resizeEvent(e);
    d_->camera.setViewportSize(width(), height());
    d_->cube->move(width() - ViewCubeWidget::kSize - 8, 8);
    d_->cube->raise();
    emit cameraChanged();
}

void Viewport::mousePressEvent(QMouseEvent* e) {
    setFocus();
    d_->animating = false;
    d_->animTimer.stop();
    d_->hoverTimer.stop();
    const auto& m = navMappingFor(d_->nav);
    const Chord c{buttonOf(e->button()), modifiersOf(e->modifiers())};
    Gesture g = Gesture::None;
    if (c.button != MouseButton::None) {
        if (c == m.orbit) g = Gesture::Orbit;
        else if (c == m.pan) g = Gesture::Pan;
        else if (c == m.zoom) g = Gesture::Zoom;
        else if (c.button == MouseButton::Left && c.modifiers == ModNone) g = Gesture::Click;
    }
    d_->gesture = g;
    d_->lastPos = d_->pressPos = e->position();
    d_->pivot.reset();
    if (g == Gesture::Orbit) {
        const std::uint64_t serial = ++d_->gestureSerial;
        const QPointF p = e->position();
        // Pivot = scene centre until the pick under the cursor reports a surface point.
        if (!d_->bounds.isEmpty()) {
            const auto& b = d_->bounds;
            d_->pivot = Vec3{(b.min.x + b.max.x) / 2, (b.min.y + b.max.y) / 2, (b.min.z + b.max.z) / 2};
        }
        pick(static_cast<int>(p.x()), static_cast<int>(p.y()), [this, serial](PickResult r) {
            if (serial == d_->gestureSerial && d_->gesture == Gesture::Orbit && r.point) d_->pivot = r.point;
        });
    }
    if (g == Gesture::None) QRhiWidget::mousePressEvent(e);
    else e->accept();
}

void Viewport::mouseMoveEvent(QMouseEvent* e) {
    const QPointF pos = e->position();
    const double dx = pos.x() - d_->lastPos.x(), dy = pos.y() - d_->lastPos.y();
    switch (d_->gesture) {
    case Gesture::Orbit:
        if (d_->pivot) d_->camera.orbitAbout(dx, dy, *d_->pivot);
        else d_->camera.orbit(dx, dy);
        break;
    case Gesture::Pan: d_->camera.pan(dx, dy); break;
    case Gesture::Zoom: d_->camera.zoomAbout(std::exp(-dy * 0.01), d_->camera.target()); break;
    case Gesture::Click:
    case Gesture::None: break;
    }
    if (d_->gesture == Gesture::Orbit || d_->gesture == Gesture::Pan || d_->gesture == Gesture::Zoom) {
        d_->lastPos = pos;
        update();
        emit cameraChanged();
        return;
    }
    d_->lastPos = pos;
    if (d_->gesture == Gesture::None && !d_->scene.empty()) {
        d_->hoverPos = pos;
        d_->hoverTimer.start();
    }
}

void Viewport::mouseReleaseEvent(QMouseEvent* e) {
    const Gesture g = d_->gesture;
    d_->gesture = Gesture::None;
    ++d_->gestureSerial;
    d_->pivot.reset();
    if (g == Gesture::Click && (e->position() - d_->pressPos).manhattanLength() <= 4) {
        const QPointF p = e->position();
        pick(static_cast<int>(p.x()), static_cast<int>(p.y()), [this](PickResult r) {
            setSelection(r);
            emit picked(r);
        });
    }
}

void Viewport::leaveEvent(QEvent* e) {
    d_->hoverTimer.stop();
    if (d_->hover.bodyIndex) {
        d_->hover = {};
        if (d_->renderer) d_->renderer->setHover({});
        update();
        emit hoverChanged({});
    }
    QRhiWidget::leaveEvent(e);
}

void Viewport::wheelEvent(QWheelEvent* e) {
    const double notches = e->angleDelta().y() / 120.0;
    if (notches == 0.0) { e->ignore(); return; }
    d_->animating = false;
    d_->animTimer.stop();
    const auto& m = navMappingFor(d_->nav);
    const QPointF pos = e->position();
    Vec3 anchor = d_->camera.target();
    if (m.wheelZoomsTowardCursor) {
        const double nx = 2 * pos.x() / std::max(width(), 1) - 1, ny = 1 - 2 * pos.y() / std::max(height(), 1);
        // Prefer the surface point under the cursor (from the hover pick) if it is still where the cursor is.
        bool useHover = false;
        if (d_->hoverPoint && (pos - d_->hoverPointPos).manhattanLength() < 12) {
            if (auto ndc = d_->camera.project(*d_->hoverPoint)) {
                const double sx = (ndc->x * 0.5 + 0.5) * width(), sy = (0.5 - ndc->y * 0.5) * height();
                useHover = std::abs(sx - pos.x()) < 6 && std::abs(sy - pos.y()) < 6;
            }
        }
        if (useHover) anchor = *d_->hoverPoint;
        else {
            const Ray r = d_->camera.rayThrough(nx, ny);
            // point on the target plane under the cursor
            const double den = dot(r.dir, d_->camera.forward());
            const double t = den > 1e-9 ? dot(d_->camera.target() - r.origin, d_->camera.forward()) / den : 0.0;
            anchor = r.origin + r.dir * t;
        }
    }
    // A new anchor mid-glide: flush what is left on the old one so nothing is lost.
    if (d_->zoomRemaining != 0.0 && !(anchor == d_->zoomAnchor)) {
        d_->camera.zoomAbout(std::exp(d_->zoomRemaining), d_->zoomAnchor);
        d_->zoomRemaining = 0.0;
    }
    d_->zoomAnchor = anchor;
    d_->zoomRemaining += notches * 0.15;   // ~16% per wheel notch
    d_->zoomTimer.start();
    e->accept();
}

void Viewport::keyPressEvent(QKeyEvent* e) {
    if (e->key() == Qt::Key_Home) viewHome();
    else if (e->key() == Qt::Key_F && e->modifiers() == Qt::NoModifier) viewFit();
    else QRhiWidget::keyPressEvent(e);
}

} // namespace iw::render
