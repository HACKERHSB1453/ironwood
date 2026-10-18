// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <condition_variable>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>

#include <QGuiApplication>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <optional>
#include <set>

#include "render/camera.hpp"
#include "render/demo_scene.hpp"
#include "render/nav.hpp"
#include "render/offscreen.hpp"
#include "render/scene_renderer.hpp"
#include "render/viewcube_model.hpp"

using namespace iw;
using namespace iw::render;

namespace {

/// Hard per-test deadline for GPU tests. A wait that never returns (no frame, a stuck readback or driver call)
/// fails the run: after kLimit the watchdog prints the test name and terminates the process with exit code 1.
/// The timer runs on its own thread, so it fires even when the test thread is blocked.
class TestWatchdog : public ::testing::EmptyTestEventListener {
public:
    static constexpr std::chrono::seconds kLimit{60};

    TestWatchdog() : thread_([this] { loop(); }) {}
    ~TestWatchdog() override {
        {
            std::lock_guard lock(m_);
            quit_ = true;
            ++gen_;
        }
        cv_.notify_all();
        thread_.join();
    }
    void OnTestStart(const ::testing::TestInfo& info) override {
        {
            std::lock_guard lock(m_);
            name_ = std::string(info.test_suite_name()) + "." + info.name();
            armed_ = true;
            ++gen_;
        }
        cv_.notify_all();
    }
    void OnTestEnd(const ::testing::TestInfo&) override {
        {
            std::lock_guard lock(m_);
            armed_ = false;
            ++gen_;
        }
        cv_.notify_all();
    }

private:
    void loop() {
        std::unique_lock lock(m_);
        while (!quit_) {
            if (!armed_) {
                cv_.wait(lock);
                continue;
            }
            const unsigned long long gen = gen_;
            if (!cv_.wait_for(lock, kLimit, [&] { return quit_ || gen_ != gen; })) {
                std::fprintf(stderr, "TIMEOUT: %s did not finish within %lld s (hard watchdog, test failed)\n",
                             name_.c_str(), static_cast<long long>(kLimit.count()));
                std::fflush(stderr);
                std::_Exit(1);
            }
        }
    }

    std::mutex m_;
    std::condition_variable cv_;
    bool armed_ = false;
    bool quit_ = false;
    unsigned long long gen_ = 0;
    std::string name_;
    std::thread thread_;   // declared last: starts after the members above are initialised
};

/// One QGuiApplication for the whole binary (offscreen platform unless the caller chose another).
class QtEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
        app_ = new QGuiApplication(argc_, argv_);
        ::testing::UnitTest::GetInstance()->listeners().Append(new TestWatchdog);
    }
    void TearDown() override { delete app_; }

private:
    int argc_ = 1;
    char name_[16] = "test_render";
    char* argv_[2] = {name_, nullptr};
    QGuiApplication* app_ = nullptr;
};
const auto* const kEnv = ::testing::AddGlobalTestEnvironment(new QtEnv);

constexpr double kPi = std::numbers::pi;

Camera makeCamera(UpAxis up, Projection proj, int w = 800, int h = 600) {
    Camera c;
    c.setUpAxis(up);
    c.setProjection(proj);
    c.setViewportSize(w, h);
    return c;
}

std::vector<Camera> allCameras() {
    std::vector<Camera> v;
    for (auto up : {UpAxis::Y, UpAxis::Z})
        for (auto pr : {Projection::Perspective, Projection::Orthographic}) {
            Camera c = makeCamera(up, pr);
            c.setState({{10, -20, 30}, 250.0, 0.9, 0.4});
            v.push_back(c);
        }
    return v;
}

} // namespace

// ---------------------------------------------------------------------------------------------- nav
TEST(Nav, FusionDefaultMapping) {
    const auto& m = navMappingFor(NavPreset::Fusion);
    EXPECT_EQ(m.pan, (Chord{MouseButton::Middle, ModNone}));
    EXPECT_EQ(m.orbit, (Chord{MouseButton::Middle, ModShift}));
    EXPECT_TRUE(m.wheelZoomsTowardCursor);
}

TEST(Nav, AllPresetsHaveDistinctOrbitAndPan) {
    for (auto p : {NavPreset::Fusion, NavPreset::Inventor, NavPreset::SolidWorks, NavPreset::Alias,
                   NavPreset::Tinkercad}) {
        const auto& m = navMappingFor(p);
        EXPECT_NE(m.orbit, m.pan) << m.name;
        EXPECT_NE(m.orbit.button, MouseButton::None) << m.name;
    }
}

// -------------------------------------------------------------------------------------------- camera
TEST(Camera, DefaultUpAxisIsY) { EXPECT_EQ(Camera{}.upAxis(), UpAxis::Y); }

TEST(Camera, ViewMatrixIsRigidAndPlacesEyeAtOrigin) {
    for (const Camera& c : allCameras()) {
        const Transform v = c.viewMatrix();
        const Vec3 e = v.apply(c.eye());
        EXPECT_NEAR(length(e), 0.0, 1e-9);
        const Vec3 t = v.apply(c.target());
        EXPECT_NEAR(t.x, 0.0, 1e-9);
        EXPECT_NEAR(t.y, 0.0, 1e-9);
        EXPECT_NEAR(t.z, -c.distance(), 1e-9);
        EXPECT_NEAR(length(c.right()), 1.0, 1e-12);
        EXPECT_NEAR(length(c.upVector()), 1.0, 1e-12);
        EXPECT_NEAR(dot(c.right(), c.upVector()), 0.0, 1e-12);
        EXPECT_NEAR(dot(c.right(), c.forward()), 0.0, 1e-12);
        // handedness: right x up == back
        const Vec3 back = cross(c.right(), c.upVector());
        EXPECT_NEAR(dot(back, c.forward()), -1.0, 1e-12);
        // screen-up never points below the world horizon
        EXPECT_GT(dot(c.upVector(), c.worldUp()), 0.0);
    }
}

TEST(Camera, UpAxisConventionsForStandardViews) {
    Camera y = makeCamera(UpAxis::Y, Projection::Perspective);
    y.setState(y.poseForEyeDirection({0, 0, 1}));              // Front
    EXPECT_NEAR(y.eye().z, y.distance(), 1e-9);
    EXPECT_NEAR(dot(y.upVector(), Vec3{0, 1, 0}), 1.0, 1e-9);
    EXPECT_NEAR(dot(y.right(), Vec3{1, 0, 0}), 1.0, 1e-9);     // Right of the screen = +X
    y.setState(y.poseForEyeDirection({1, 0, 0}));              // Right view
    EXPECT_NEAR(y.eye().x, y.distance(), 1e-9);
    EXPECT_NEAR(dot(y.right(), Vec3{0, 0, -1}), 1.0, 1e-9);

    Camera z = makeCamera(UpAxis::Z, Projection::Perspective);
    EXPECT_EQ(z.frontEyeDirection(), (Vec3{0, -1, 0}));
    z.setState(z.poseForEyeDirection(z.frontEyeDirection()));
    EXPECT_NEAR(z.eye().y, -z.distance(), 1e-9);
    EXPECT_NEAR(dot(z.upVector(), Vec3{0, 0, 1}), 1.0, 1e-9);
    EXPECT_NEAR(dot(z.right(), Vec3{1, 0, 0}), 1.0, 1e-9);
}

TEST(Camera, PoseForEyeDirectionRoundTrips) {
    for (auto up : {UpAxis::Y, UpAxis::Z}) {
        Camera c = makeCamera(up, Projection::Perspective);
        for (const Vec3 d : {Vec3{1, 1, 1}, Vec3{-1, 0.3, 2}, Vec3{0, 1, 0}, Vec3{0, 0, -1}, Vec3{3, -2, -1}}) {
            c.setState(c.poseForEyeDirection(d));
            const Vec3 e = c.eye() - c.target();
            const double l = length(e) * length(d);
            // vertical views are clamped by < 0.06 deg
            EXPECT_NEAR(dot(e, d) / l, 1.0, 1e-5) << "up=" << int(up);
        }
    }
}

TEST(Camera, TopViewKeepsHeading) {
    Camera c = makeCamera(UpAxis::Y, Projection::Perspective);
    c.setState({{}, 100, 1.1, 0.3});
    const CameraState top = c.poseForEyeDirection(c.worldUp());
    EXPECT_NEAR(top.yaw, 1.1, 1e-9);
    EXPECT_NEAR(top.pitch, Camera::kMaxPitch, 1e-12);
}

TEST(Camera, PitchIsClamped) {
    for (Camera c : allCameras()) {
        for (int i = 0; i < 50; ++i) c.orbit(0, 400);   // drag down forever
        EXPECT_LE(c.pitch(), Camera::kMaxPitch + 1e-15);
        EXPECT_GT(c.pitch(), 1.5);
        for (int i = 0; i < 100; ++i) c.orbit(0, -400);
        EXPECT_GE(c.pitch(), -Camera::kMaxPitch - 1e-15);
        // no flip: screen-up stays on the upper hemisphere
        EXPECT_GT(dot(c.upVector(), c.worldUp()), 0.0);
        const auto v = c.viewMatrix();
        for (double m : v.m) EXPECT_TRUE(std::isfinite(m));
    }
}

TEST(Camera, OrbitDirectionsFollowTheMouse) {
    for (Camera c : allCameras()) {
        c.setState({{}, 100, 0.7, 0.2});
        // a point on the surface nearest the viewer; dragging right must carry it to the right on screen
        const Vec3 near = c.target() + (c.eye() - c.target()) * 0.1;
        const double x0 = c.project(near)->x;
        c.orbit(40, 0);
        EXPECT_GT(c.project(near)->x, x0);
        // dragging down carries it down
        c.setState({{}, 100, 0.7, 0.2});
        const double y0 = c.project(near)->y;
        c.orbit(0, 40);
        EXPECT_LT(c.project(near)->y, y0);
    }
}

TEST(Camera, OrbitAboutKeepsPivotOnScreen) {
    for (Camera c : allCameras()) {
        const Vec3 pivot{40, 5, -12};
        const auto before = *c.project(pivot);
        c.orbitAbout(77, -31, pivot);
        c.orbitAbout(-12, 55, pivot);
        const auto after = *c.project(pivot);
        EXPECT_NEAR(after.x, before.x, 1e-9);
        EXPECT_NEAR(after.y, before.y, 1e-9);
        // orientation actually changed
        EXPECT_GT(std::abs(c.yaw() - 0.9) + std::abs(c.pitch() - 0.4), 0.1);
    }
}

TEST(Camera, PanMovesSceneOneToOneAtTargetDepth) {
    for (Camera c : allCameras()) {
        const Vec3 t = c.target();
        const auto n0 = *c.project(t);
        c.pan(30, -20);
        const auto n1 = *c.project(t);
        // dragging right/up moves the scene right/up by exactly the cursor delta (NDC: 2*d/size)
        EXPECT_NEAR(n1.x - n0.x, 2.0 * 30 / 800, 1e-9);
        EXPECT_NEAR(n1.y - n0.y, 2.0 * 20 / 600, 1e-9);
    }
}

TEST(Camera, ZoomToCursorKeepsPointUnderCursorFixed) {
    const double ndcs[][2] = {{0.0, 0.0}, {0.6, -0.3}, {-0.9, 0.8}, {0.25, 0.7}};
    for (Camera c : allCameras()) {
        for (auto& ndc : ndcs) {
            Camera k = c;
            const Ray r = k.rayThrough(ndc[0], ndc[1]);
            // the world point on the target plane under the cursor
            const double t = dot(k.target() - r.origin, k.forward()) / dot(r.dir, k.forward());
            const Vec3 p = r.origin + r.dir * t;
            const auto before = *k.project(p);
            for (double factor : {1.7, 0.4, 3.0}) {
                k.zoom(factor, ndc[0], ndc[1], true);
                const auto after = *k.project(p);
                EXPECT_NEAR(after.x, ndc[0], 1e-6);
                EXPECT_NEAR(after.y, ndc[1], 1e-6);
                EXPECT_NEAR(after.x, before.x, 1e-6);
                EXPECT_NEAR(after.y, before.y, 1e-6);
                // the *same* world point stays under the cursor
                const Ray r2 = k.rayThrough(ndc[0], ndc[1]);
                const Vec3 d = p - r2.origin;
                EXPECT_NEAR(length(cross(d, r2.dir)), 0.0, 1e-6);
            }
        }
    }
}

TEST(Camera, ZoomScalesAndRoundTrips) {
    for (Camera c : allCameras()) {
        const double d = c.distance();
        c.zoom(2.0, 0.3, -0.2, true);
        EXPECT_NEAR(c.distance(), d / 2, 1e-12);
        c.zoom(0.5, 0.3, -0.2, true);
        EXPECT_NEAR(c.distance(), d, 1e-9);
        EXPECT_NEAR(length(c.target() - Vec3{10, -20, 30}), 0.0, 1e-9);
        c.zoom(4.0, 0.9, 0.9, false);   // toward target: target unchanged
        EXPECT_NEAR(length(c.target() - Vec3{10, -20, 30}), 0.0, 1e-9);
        c.zoom(-1.0, 0, 0, true);       // invalid factors are ignored
        c.zoom(0.0, 0, 0, true);
        EXPECT_NEAR(c.distance(), d / 4, 1e-9);
    }
}

TEST(Camera, ZoomAboutArbitraryPointKeepsItFixed) {
    for (Camera c : allCameras()) {
        const Vec3 p{-30, 80, 14};
        const auto b = *c.project(p);
        c.zoomAbout(2.3, p);
        const auto a = *c.project(p);
        EXPECT_NEAR(a.x, b.x, 1e-9);
        EXPECT_NEAR(a.y, b.y, 1e-9);
    }
}

TEST(Camera, FitContainsBounds) {
    Box3 boxes[3];
    boxes[0].extend({-50, -20, -10});  boxes[0].extend({50, 20, 10});
    boxes[1].extend({1000, 1000, 1000});  boxes[1].extend({1010, 1500, 1002});
    boxes[2].extend({0, 0, 0});  boxes[2].extend({3, 3, 3});
    for (auto up : {UpAxis::Y, UpAxis::Z})
        for (auto pr : {Projection::Perspective, Projection::Orthographic})
            for (auto [w, h] : {std::pair{800, 600}, std::pair{300, 900}, std::pair{1920, 400}})
                for (const Box3& b : boxes) {
                    Camera c = makeCamera(up, pr, w, h);
                    c.setState({{1, 2, 3}, 17.0, 2.2, 0.35});
                    c.fit(b);
                    for (int i = 0; i < 8; ++i) {
                        const Vec3 corner{(i & 1) ? b.max.x : b.min.x, (i & 2) ? b.max.y : b.min.y,
                                          (i & 4) ? b.max.z : b.min.z};
                        const auto n = c.project(corner);
                        ASSERT_TRUE(n.has_value());
                        EXPECT_LE(std::abs(n->x), 1.0) << w << "x" << h;
                        EXPECT_LE(std::abs(n->y), 1.0) << w << "x" << h;
                        EXPECT_GE(n->z, -1.0);
                        EXPECT_LE(n->z, 1.0);
                    }
                    // tight: something touches (within the 5% margin) the frustum
                    double m = 0;
                    for (int i = 0; i < 8; ++i) {
                        const Vec3 corner{(i & 1) ? b.max.x : b.min.x, (i & 2) ? b.max.y : b.min.y,
                                          (i & 4) ? b.max.z : b.min.z};
                        const auto n = *c.project(corner);
                        m = std::max({m, std::abs(n.x), std::abs(n.y)});
                    }
                    EXPECT_GT(m, 0.8);
                }
}

TEST(Camera, FitEmptyBoxIsNoOp) {
    Camera c = makeCamera(UpAxis::Y, Projection::Perspective);
    const CameraState s = c.state();
    c.fit({});
    EXPECT_EQ(c.state().distance, s.distance);
    EXPECT_EQ(c.state().target, s.target);
}

TEST(Camera, ProjectionToggleKeepsTargetPlaneScale) {
    Camera p = makeCamera(UpAxis::Y, Projection::Perspective);
    p.setState({{5, 6, 7}, 120.0, 0.8, 0.3});
    Camera o = p;
    o.setProjection(Projection::Orthographic);
    const Vec3 q = p.target() + p.right() * 20.0 + p.upVector() * -15.0;
    const auto a = *p.project(q), b = *o.project(q);
    EXPECT_NEAR(a.x, b.x, 1e-9);
    EXPECT_NEAR(a.y, b.y, 1e-9);
}

TEST(Camera, ProjectAndRayAreInverse) {
    for (Camera c : allCameras()) {
        const Ray r = c.rayThrough(0.37, -0.52);
        for (double t : {50.0, 250.0, 400.0}) {
            const auto n = c.project(r.origin + r.dir * t);
            ASSERT_TRUE(n);
            EXPECT_NEAR(n->x, 0.37, 1e-9);
            EXPECT_NEAR(n->y, -0.52, 1e-9);
        }
    }
}

TEST(Camera, ClipPlanesBracketScene) {
    Box3 b;
    b.extend({-50, -50, -50});
    b.extend({50, 50, 50});
    for (auto pr : {Projection::Perspective, Projection::Orthographic}) {
        Camera c = makeCamera(UpAxis::Y, pr);
        c.fit(b);
        const auto [n, f] = c.clipPlanes();
        EXPECT_LT(n, f);
        if (pr == Projection::Perspective) { EXPECT_GT(n, 0.0); }
        const Transform v = c.viewMatrix();
        for (int i = 0; i < 8; ++i) {
            const Vec3 corner{(i & 1) ? 50. : -50., (i & 2) ? 50. : -50., (i & 4) ? 50. : -50.};
            const double depth = -v.apply(corner).z;
            EXPECT_GT(depth, n);
            EXPECT_LT(depth, f);
        }
    }
}

TEST(Camera, SetTargetKeepEyeKeepsEyeAndOrientation) {
    Camera c = makeCamera(UpAxis::Z, Projection::Perspective);
    c.setState({{0, 0, 0}, 200, 0.5, 0.4});
    const Vec3 eye = c.eye();
    const Vec3 fwd = c.forward();
    c.setTargetKeepEye(c.target() + c.right() * 30.0 + fwd * 55.0);
    EXPECT_NEAR(length(c.eye() - eye), 0.0, 1e-9);
    EXPECT_NEAR(dot(c.forward(), fwd), 1.0, 1e-12);
    EXPECT_NEAR(c.distance(), 255.0, 1e-9);
}

TEST(Camera, LerpEndpointsAndShortestYawArc) {
    const CameraState a{{0, 0, 0}, 100, 3.0, 0.1}, b{{10, 0, 0}, 400, -3.0, 0.5};
    const CameraState s0 = lerp(a, b, 0), s1 = lerp(a, b, 1), sm = lerp(a, b, 0.5);
    EXPECT_NEAR(s0.distance, 100, 1e-12);
    EXPECT_NEAR(s1.distance, 400, 1e-9);
    EXPECT_NEAR(sm.distance, 200, 1e-9);           // geometric
    EXPECT_NEAR(sm.target.x, 5, 1e-12);
    // 3.0 -> -3.0 is a 0.283 rad hop through pi, not a 6 rad sweep
    EXPECT_NEAR(std::abs(std::remainder(sm.yaw - kPi, 2 * kPi)), 0.0, 1e-9);
}

// ----------------------------------------------------------------------------------------- viewcube
TEST(ViewCube, HasFiftyFourCellsAndTwentySixDirections) {
    const auto& cells = viewCubeCells();
    EXPECT_EQ(cells.size(), 54u);
    std::vector<Vec3> dirs;
    for (const auto& c : cells) {
        bool seen = false;
        for (const auto& d : dirs) seen |= (d == c.dir);
        if (!seen) dirs.push_back(c.dir);
        EXPECT_FALSE(c.dir == Vec3{});
    }
    EXPECT_EQ(dirs.size(), 26u);
}

TEST(ViewCube, FrontViewHitTestsFaceEdgeAndCorner) {
    Camera c = makeCamera(UpAxis::Y, Projection::Perspective);
    c.setState(c.poseForEyeDirection({0, 0, 1}));
    const Transform v = c.viewMatrix();
    const auto cells = projectViewCube(v, 100, 100, 60);
    EXPECT_EQ(cells.size(), 9u);                    // only the front face is visible
    const double s = 60 * 0.58;
    auto at = [&](double ux, double uy) { return hitTestViewCube(cells, 100 + ux * s, 100 - uy * s); };
    auto dirOf = [&](int i) { return cells[static_cast<std::size_t>(i)].cell->dir; };
    ASSERT_GE(at(0, 0), 0);
    EXPECT_EQ(dirOf(at(0, 0)), (Vec3{0, 0, 1}));    // face
    EXPECT_EQ(dirOf(at(0.8, 0)), (Vec3{1, 0, 1}));  // right edge strip
    EXPECT_EQ(dirOf(at(0, 0.8)), (Vec3{0, 1, 1}));  // top edge strip
    EXPECT_EQ(dirOf(at(0.8, 0.8)), (Vec3{1, 1, 1}));// corner
    EXPECT_EQ(dirOf(at(-0.8, -0.8)), (Vec3{-1, -1, 1}));
    EXPECT_EQ(at(1.5, 0), -1);                      // outside
}

TEST(ViewCube, HomeViewShowsThreeFacesAndAllKindsAreHittable) {
    Camera c = makeCamera(UpAxis::Z, Projection::Perspective);
    c.setState(c.poseForEyeDirection(c.homeEyeDirection()));
    const auto cells = projectViewCube(c.viewMatrix(), 100, 100, 60);
    EXPECT_EQ(cells.size(), 27u);
    bool kinds[3] = {false, false, false};
    for (std::size_t i = 0; i < cells.size(); ++i) {
        double cx = 0, cy = 0;
        for (auto& p : cells[i].pts) { cx += p[0] / 4; cy += p[1] / 4; }
        const int hit = hitTestViewCube(cells, cx, cy);
        ASSERT_GE(hit, 0);
        kinds[static_cast<int>(cells[static_cast<std::size_t>(hit)].cell->kind)] = true;
    }
    EXPECT_TRUE(kinds[0] && kinds[1] && kinds[2]);
    // the cell facing the camera most directly is the corner (1,1,1)-ish of the home direction
    const auto labels = viewCubeFaceLabels(UpAxis::Z);
    EXPECT_STREQ(labels[4], "TOP");
    EXPECT_STREQ(labels[3], "FRONT");
    EXPECT_STREQ(viewCubeFaceLabels(UpAxis::Y)[2], "TOP");
    EXPECT_STREQ(viewCubeFaceLabels(UpAxis::Y)[4], "FRONT");
}

TEST(ViewCube, ClickingEveryDirectionReachesThatView) {
    for (auto up : {UpAxis::Y, UpAxis::Z}) {
        Camera c = makeCamera(up, Projection::Perspective);
        for (const auto& cell : viewCubeCells()) {
            c.setState(c.poseForEyeDirection(cell.dir));
            const Vec3 e = c.eye() - c.target();
            const double l = length(e) * length(cell.dir);
            EXPECT_GT(dot(e, cell.dir) / l, 0.999999);
        }
    }
}

// ------------------------------------------------------------------------------------------ picking
TEST(Pick, DecodePrefersEdgesWithinRadius) {
    constexpr int W = SceneRenderer::kPickWindow;
    std::vector<float> win(W * W * 4, 0.f);
    auto set = [&](int x, int y, float body, float id, float kind, float depth) {
        float* p = &win[(static_cast<std::size_t>(y) * W + x) * 4];
        p[0] = body; p[1] = id; p[2] = kind; p[3] = depth;
    };
    for (int y = 0; y < W; ++y)
        for (int x = 0; x < W; ++x) set(x, y, 1, 3, 0, 10);                // face 2 of body 0 everywhere
    auto d = decodePickWindow(win.data(), W, 6.0);
    ASSERT_TRUE(d.hit);
    EXPECT_EQ(d.result.faceIndex, 2u);
    EXPECT_FALSE(d.result.edgeIndex);
    set(8 + 4, 8 - 3, 1, 8, 1, 9.5f);                                       // edge 7 at distance 5
    d = decodePickWindow(win.data(), W, 6.0);
    EXPECT_EQ(d.result.edgeIndex, 7u);
    EXPECT_EQ(d.result.faceIndex, 2u);                                      // face under the cursor is reported too
    set(8 + 1, 8, 1, 4, 1, 9.0f);                                           // nearer edge 3
    d = decodePickWindow(win.data(), W, 6.0);
    EXPECT_EQ(d.result.edgeIndex, 3u);
    EXPECT_EQ(d.px, 9);
    d = decodePickWindow(win.data(), W, 0.5);                               // radius too small: face only
    EXPECT_FALSE(d.result.edgeIndex);
    std::vector<float> empty(W * W * 4, 0.f);
    EXPECT_FALSE(decodePickWindow(empty.data(), W, 6.0).hit);
}

// ----------------------------------------------------------------------- headless GPU rendering
namespace {
QRgb px(const QImage& im, int x, int y) { return im.pixel(x, y); }
int luma(QRgb c) { return (qRed(c) * 3 + qGreen(c) * 6 + qBlue(c)) / 10; }

std::vector<SceneBody> oneBox(Vec3 lo = {-50, -50, -50}, Vec3 hi = {50, 50, 50}) {
    SceneBody b;
    b.mesh = makeBoxMesh(lo, hi);
    return {b};
}
} // namespace

namespace {
/// IW_TEST_DUMP_DIR=<dir> saves the rendered frames as PNGs for eyeballing.
void maybeDump(const QImage& im, const char* name) {
    if (const auto dir = qEnvironmentVariable("IW_TEST_DUMP_DIR"); !dir.isEmpty()) im.save(dir + "/" + name + ".png");
}
} // namespace

TEST(Offscreen, RendersShadedCubeWithEdgesAndBackground) {
    Camera c = makeCamera(UpAxis::Y, Projection::Perspective, 480, 360);
    const auto bodies = oneBox();
    c.setState(c.poseForEyeDirection(c.homeEyeDirection()));
    c.fit(bodies[0].mesh->bounds);
    auto frame = renderOffscreen(bodies, c, {480, 360});
    if (!frame) GTEST_SKIP() << "no QRhi backend available";
    const QImage& im = frame->image;
    ASSERT_EQ(im.size(), QSize(480, 360));
    maybeDump(im, "cube_home");

    // background gradient: light grey, lighter at the bottom than the top, identical along a row
    const QRgb top = px(im, 5, 5), bottom = px(im, 5, 354);
    EXPECT_GT(luma(top), 170);
    EXPECT_GT(luma(bottom), luma(top));
    EXPECT_LE(std::abs(luma(px(im, 5, 100)) - luma(px(im, 470, 100))), 2);

    // the cube is in the middle and shaded differently from the background
    const QRgb mid = px(im, 240, 180);
    EXPECT_NE(mid, px(im, 240, 5));
    EXPECT_LT(luma(mid), luma(px(im, 5, 180)) + 60);

    // dark edge lines exist, and no pixel is the clear-colour junk
    int dark = 0, bluish = 0;
    for (int y = 0; y < im.height(); ++y)
        for (int x = 0; x < im.width(); ++x) {
            const QRgb p = px(im, x, y);
            if (luma(p) < 45) ++dark;
            bluish += (qBlue(p) > 100 && luma(p) < 200) ? 1 : 0;
        }
    EXPECT_GT(dark, 200);
    EXPECT_GT(bluish, 1000);
    // the three visible faces have three different shades (key light direction matters)
    std::set<int> shades;
    for (auto [x, y] : {std::pair{190, 200}, std::pair{290, 200}, std::pair{240, 120}}) shades.insert(luma(px(im, x, y)));
    EXPECT_EQ(shades.size(), 3u);
}

TEST(Offscreen, PickReturnsBodyFaceEdgeAndPoint) {
    Camera c = makeCamera(UpAxis::Y, Projection::Perspective, 480, 360);
    const auto bodies = oneBox();
    c.setState(c.poseForEyeDirection({0, 0, 1}));   // Front: looking at the +Z face
    c.fit(bodies[0].mesh->bounds);
    // pixel of the front top edge midpoint and a point well inside the front face
    const auto topEdge = *c.project({0, 50, 50});
    const auto inside = *c.project({10, -10, 50});
    auto toPx = [&](const Vec3& n) { return QPoint(int((n.x * 0.5 + 0.5) * 480), int((0.5 - n.y * 0.5) * 360)); };
    const QPoint pEdge = toPx(topEdge) + QPoint(0, 3);   // 3px inside the face: edge radius must win
    const QPoint pFace = toPx(inside);
    const QPoint pNone(3, 3);
    auto frame = renderOffscreen(bodies, c, {480, 360}, {pFace, pEdge, pNone});
    if (!frame) GTEST_SKIP() << "no QRhi backend available";
    ASSERT_EQ(frame->picks.size(), 3u);

    const PickResult& f = frame->picks[0];
    EXPECT_EQ(f.bodyIndex, 0u);
    EXPECT_EQ(f.faceIndex, 5u);                    // +Z
    EXPECT_FALSE(f.edgeIndex);
    ASSERT_TRUE(f.point);
    EXPECT_NEAR(f.point->z, 50.0, 0.5);
    EXPECT_NEAR(f.point->x, 10.0, 1.0);
    EXPECT_NEAR(f.point->y, -10.0, 1.0);

    const PickResult& e = frame->picks[1];
    EXPECT_EQ(e.bodyIndex, 0u);
    EXPECT_EQ(e.edgeIndex, 11u);                   // corners 6-7: y=hi, z=hi
    EXPECT_EQ(e.faceIndex, 5u);

    EXPECT_FALSE(frame->picks[2].bodyIndex);
    EXPECT_FALSE(frame->picks[2].faceIndex);
    EXPECT_FALSE(frame->picks[2].point);
}

TEST(Offscreen, PickOrientationMatchesImageAndHiddenEdgesAreNotPicked) {
    // Two bodies: A upper-left, B lower-right (asymmetric so a flipped readback is detected).
    SceneBody a, b;
    a.mesh = makeBoxMesh({-100, 40, -20}, {-40, 100, 20});
    b.mesh = makeBoxMesh({40, -100, -20}, {100, -40, 20});
    std::vector<SceneBody> bodies{a, b};
    Camera c = makeCamera(UpAxis::Y, Projection::Orthographic, 400, 400);
    c.setState(c.poseForEyeDirection({0, 0, 1}));
    c.setState({{0, 0, 0}, 300.0, 0, 0});
    c.setSceneBounds([&] { Box3 bb; bb.extend({-100, -100, -20}); bb.extend({100, 100, 20}); return bb; }());
    auto ptA = *c.project({-70, 70, 20});
    auto ptB = *c.project({70, -70, 20});
    auto toPx = [&](const Vec3& n) { return QPoint(int((n.x * 0.5 + 0.5) * 400), int((0.5 - n.y * 0.5) * 400)); };
    auto frame = renderOffscreen(bodies, c, {400, 400}, {toPx(ptA), toPx(ptB), toPx(*c.project({70, 70, 0})),
                                                         toPx(*c.project({-70, -70, 0}))});
    if (!frame) GTEST_SKIP() << "no QRhi backend available";
    EXPECT_EQ(frame->picks[0].bodyIndex, 0u);
    EXPECT_EQ(frame->picks[1].bodyIndex, 1u);
    EXPECT_FALSE(frame->picks[2].bodyIndex);   // empty quadrants
    EXPECT_FALSE(frame->picks[3].bodyIndex);
    // the same pixels in the colour image are non-background where the pick hit
    const QPoint pa = toPx(ptA), pb = toPx(ptB), pe = toPx(*c.project({70, 70, 0}));
    auto dist = [](QRgb a, QRgb b) { return std::abs(qRed(a) - qRed(b)) + std::abs(qGreen(a) - qGreen(b)) + std::abs(qBlue(a) - qBlue(b)); };
    const QRgb bgA = px(frame->image, pe.x(), pa.y());   // empty pixel on the same row as body A
    EXPECT_GT(dist(px(frame->image, pa.x(), pa.y()), bgA), 15);
    EXPECT_GT(dist(px(frame->image, pb.x(), pb.y()), px(frame->image, pa.x(), pb.y())), 15);
    EXPECT_LE(dist(px(frame->image, pe.x(), pe.y()), bgA), 6);
}

TEST(Offscreen, HoverAndSelectionHighlightChangeShading) {
    Camera c = makeCamera(UpAxis::Y, Projection::Perspective, 320, 240);
    const auto bodies = oneBox();
    c.setState(c.poseForEyeDirection({0, 0, 1}));
    c.fit(bodies[0].mesh->bounds);
    PickResult sel;
    sel.bodyIndex = 0;
    sel.faceIndex = 5;
    PickResult hov;
    hov.bodyIndex = 0;
    hov.faceIndex = 5;
    auto plain = renderOffscreen(bodies, c, {320, 240});
    auto selected = renderOffscreen(bodies, c, {320, 240}, {}, 4, {}, sel);
    auto hovered = renderOffscreen(bodies, c, {320, 240}, {}, 4, hov, {});
    if (!plain || !selected || !hovered) GTEST_SKIP() << "no QRhi backend available";
    const QRgb p0 = px(plain->image, 160, 120), p1 = px(selected->image, 160, 120), p2 = px(hovered->image, 160, 120);
    EXPECT_GT(qRed(p1), qRed(p0) + 25);          // orange tint
    EXPECT_LT(qBlue(p1), qBlue(p0));
    EXPECT_GT(qBlue(p2), qRed(p2));              // blue tint
    EXPECT_NE(p2, p0);
}

TEST(Offscreen, MeshWithSharedVerticesAcrossFacesAndMissingNormalsStillPicksCorrectFaces) {
    // One quad pair sharing vertices between two "faces" (no per-face vertex split) and no normals.
    auto m = std::make_shared<tess::MeshData>();
    m->positions = {-50, -50, 0, 50, -50, 0, 50, 50, 0, -50, 50, 0};
    m->indices = {0, 1, 2, 0, 2, 3};
    m->faceRanges = {{0, 3}, {3, 3}};   // two faces, triangles share vertices 0 and 2
    m->bounds.extend({-50, -50, 0});
    m->bounds.extend({50, 50, 0});
    SceneBody b;
    b.mesh = m;
    Camera c = makeCamera(UpAxis::Z, Projection::Orthographic, 200, 200);
    c.setState(c.poseForEyeDirection({0, 0, 1}));
    c.fit(m->bounds);
    // triangle (0,1,2) lower-right half = face 0; triangle (0,2,3) upper-left half = face 1
    const auto lr = *c.project({30, -20, 0}), ul = *c.project({-30, 20, 0});
    auto toPx = [&](const Vec3& n) { return QPoint(int((n.x * 0.5 + 0.5) * 200), int((0.5 - n.y * 0.5) * 200)); };
    auto frame = renderOffscreen({b}, c, {200, 200}, {toPx(lr), toPx(ul)});
    if (!frame) GTEST_SKIP() << "no QRhi backend available";
    EXPECT_EQ(frame->picks[0].faceIndex, 0u);
    EXPECT_EQ(frame->picks[1].faceIndex, 1u);
}

// ------------------------------------------------------------------------------ stale pick readback (GPU)
TEST(Offscreen, PickInFlightAcrossSceneChangeReportsNothing) {
    std::vector<SceneBody> box(1);
    box[0].id = BodyId{1};
    box[0].mesh = makeBoxMesh({-10, -10, -10}, {10, 10, 10});
    Camera c = makeCamera(UpAxis::Y, Projection::Perspective, 200, 200);
    c.setState({{0, 0, 0}, 100, 0.7, 0.3});   // the view centre is on the box

    auto control = pickAcrossSceneSwap(box, std::nullopt, c, {200, 200}, 100, 100);
    if (!control) GTEST_SKIP() << "no QRhi backend available";
    ASSERT_TRUE(control->bodyIndex.has_value());
    EXPECT_EQ(*control->bodyIndex, 0u);

    auto stale = pickAcrossSceneSwap(box, std::vector<SceneBody>{}, c, {200, 200}, 100, 100);
    ASSERT_TRUE(stale.has_value());
    EXPECT_FALSE(stale->bodyIndex.has_value()) << "a readback taken before setScene must not name a body";
}

// ------------------------------------------------------------------- section analysis & translucent overlays
namespace {
Camera viewFrom(const Vec3& eyeDir, const Box3& bounds, int w = 480, int h = 360) {
    Camera c = makeCamera(UpAxis::Y, Projection::Perspective, w, h);
    c.setState(c.poseForEyeDirection(eyeDir));
    c.fit(bounds);
    return c;
}
Box3 unionOf(const std::vector<SceneBody>& bodies) {
    Box3 b;
    for (const auto& body : bodies) {
        b.extend(body.mesh->bounds.min);
        b.extend(body.mesh->bounds.max);
    }
    return b;
}
QPoint toPx(const Camera& c, const Vec3& w) {
    const auto n = *c.project(w);
    return QPoint(int((n.x * 0.5 + 0.5) * 480), int((0.5 - n.y * 0.5) * 360));
}
bool isOrange(QRgb p, int tol = 4) {
    return std::abs(qRed(p) - 255) <= tol && std::abs(qGreen(p) - 140) <= tol && std::abs(qBlue(p) - 26) <= tol;
}
/// Section with the kept side n·p + d >= 0.
SectionRequest sectionAt(Vec3 n, double d) {
    SectionRequest s;
    s.enabled = true;
    s.clipPlane = {n.x, n.y, n.z, d};
    return s;
}
} // namespace

TEST(Section, ClipRemovesTheHalfSpaceAndKeepsTheOtherSide) {
    const auto bodies = oneBox();
    const Camera c = viewFrom({0, 0, 1}, unionOf(bodies));   // front: looking at the +Z face
    const QPoint pRemoved = toPx(c, {-25, 0, 50});           // x < 0: removed by the plane x = 0
    const QPoint pKept = toPx(c, {25, 0, 50});               // x > 0: kept
    const auto plain = renderOffscreen(bodies, c, {480, 360});
    if (!plain) GTEST_SKIP() << "no QRhi backend available";
    const auto clipped = renderOffscreen(bodies, c, {480, 360}, {}, 4, {}, {}, sectionAt({1, 0, 0}, 0));
    ASSERT_TRUE(clipped.has_value());
    const QImage& cut = clipped->image;
    maybeDump(cut, "section_front_halfspace");

    // removed side shows the background (same as the far-left column of the same row)
    EXPECT_LE(std::abs(luma(px(cut, pRemoved.x(), pRemoved.y())) - luma(px(cut, 5, pRemoved.y()))), 2);
    // the unclipped frame shows the box there, so the removal is what changed it
    EXPECT_GT(std::abs(luma(px(plain->image, pRemoved.x(), pRemoved.y())) - luma(px(cut, 5, pRemoved.y()))), 20);
    // kept side is unchanged
    EXPECT_LE(std::abs(luma(px(cut, pKept.x(), pKept.y())) - luma(px(plain->image, pKept.x(), pKept.y()))), 2);
}

TEST(Section, CapPaintsTheCutInCapColour) {
    const auto bodies = oneBox();
    const Camera c = viewFrom({-1, 0, 0}, unionOf(bodies));   // from -X: the removed half is between eye and cut
    auto plain = renderOffscreen(bodies, c, {480, 360});
    if (!plain) GTEST_SKIP() << "no QRhi backend available";
    EXPECT_FALSE(isOrange(px(plain->image, 240, 180))) << "without a section the near face is grey";

    const auto cut = renderOffscreen(bodies, c, {480, 360}, {}, 4, {}, {}, sectionAt({1, 0, 0}, 0));
    ASSERT_TRUE(cut.has_value());
    maybeDump(cut->image, "section_box_cap");
    EXPECT_TRUE(isOrange(px(cut->image, 240, 180))) << "the cut face must show capColour";
    int capPixels = 0;
    for (int y = 0; y < 360; ++y)
        for (int x = 0; x < 480; ++x) capPixels += isOrange(px(cut->image, x, y)) ? 1 : 0;
    EXPECT_GT(capPixels, 5000);   // most of the 100 x 100 mm cut is in view
}

TEST(Section, FlippingThePlaneFlipsTheKeptHalf) {
    const auto bodies = oneBox();
    const Vec3 fromNegX{-1, 0, 0}, fromPosX{1, 0, 0};
    const Camera fromNeg = viewFrom(fromNegX, unionOf(bodies));
    const Camera fromPos = viewFrom(fromPosX, unionOf(bodies));
    const auto a = renderOffscreen(bodies, fromNeg, {480, 360}, {}, 4, {}, {}, sectionAt({1, 0, 0}, 0));
    if (!a) GTEST_SKIP() << "no QRhi backend available";
    const auto flipped = renderOffscreen(bodies, fromNeg, {480, 360}, {}, 4, {}, {}, sectionAt({-1, 0, 0}, 0));
    const auto flippedOtherSide = renderOffscreen(bodies, fromPos, {480, 360}, {}, 4, {}, {}, sectionAt({-1, 0, 0}, 0));
    const auto original = renderOffscreen(bodies, fromPos, {480, 360}, {}, 4, {}, {}, sectionAt({1, 0, 0}, 0));
    ASSERT_TRUE(flipped && flippedOtherSide && original);
    // kept x >= 0: the cap is seen from -X; kept x <= 0: the cap is hidden and the near face at x = -50 is seen
    EXPECT_TRUE(isOrange(px(a->image, 240, 180)));
    EXPECT_FALSE(isOrange(px(flipped->image, 240, 180)));
    // mirror: kept x <= 0 seen from +X shows the cap; kept x >= 0 seen from +X shows the far face at x = 50
    EXPECT_TRUE(isOrange(px(flippedOtherSide->image, 240, 180)));
    EXPECT_FALSE(isOrange(px(original->image, 240, 180)));
}

TEST(Section, PickingIgnoresTheClippedHalf) {
    // A is entirely in the removed half (x < 0), B is behind the cut (x > 0). Viewed from -X the ray through the
    // centre meets A first unless the section clips it.
    std::vector<SceneBody> two(2);
    two[0].mesh = makeBoxMesh({-60, -20, -20}, {-20, 20, 20});
    two[1].mesh = makeBoxMesh({20, -20, -20}, {60, 20, 20});
    const Camera c = viewFrom({-1, 0, 0}, unionOf(two));
    const QPoint centre(240, 180);
    const auto plain = renderOffscreen(two, c, {480, 360}, {centre});
    if (!plain) GTEST_SKIP() << "no QRhi backend available";
    ASSERT_EQ(plain->picks.size(), 1u);
    EXPECT_EQ(plain->picks[0].bodyIndex, 0u);

    const auto clipped = renderOffscreen(two, c, {480, 360}, {centre}, 4, {}, {}, sectionAt({1, 0, 0}, 0));
    ASSERT_TRUE(clipped.has_value());
    ASSERT_EQ(clipped->picks.size(), 1u);
    EXPECT_EQ(clipped->picks[0].bodyIndex, 1u) << "the pick must pass through the clipped half and hit B";

    // only geometry in the removed half: nothing is picked
    const std::vector<SceneBody> onlyA{two[0]};
    const Camera cA = viewFrom({-1, 0, 0}, unionOf(onlyA));
    const auto none = renderOffscreen(onlyA, cA, {480, 360}, {centre}, 4, {}, {}, sectionAt({1, 0, 0}, 0));
    ASSERT_TRUE(none.has_value());
    EXPECT_FALSE(none->picks[0].bodyIndex.has_value());
}

TEST(Overlay, TranslucentOverlayTintsTheBodyAndIsPickedWithPriority) {
    const auto bodies = oneBox();
    const Camera c = viewFrom({0, 0, 1}, unionOf(bodies));
    OverlayFace profile;
    profile.triangles = {{-20, -20, 50.5}, {20, -20, 50.5}, {20, 20, 50.5},
                         {-20, -20, 50.5}, {20, 20, 50.5}, {-20, 20, 50.5}};
    profile.rgba = {1.f, 0.f, 0.f, 0.5f};
    const QPoint inside = toPx(c, {0, 0, 50});
    const QPoint outside = toPx(c, {35, 0, 50});
    const auto plain = renderOffscreen(bodies, c, {480, 360}, {inside, outside});
    if (!plain) GTEST_SKIP() << "no QRhi backend available";
    const auto frame = renderOffscreen(bodies, c, {480, 360}, {inside, outside}, 4, {}, {}, {}, {profile});
    ASSERT_TRUE(frame.has_value());
    maybeDump(frame->image, "profile_overlay");

    const QRgb tinted = px(frame->image, inside.x(), inside.y());
    const QRgb beside = px(frame->image, outside.x(), outside.y());
    EXPECT_GT(qRed(tinted), qRed(beside) + 25) << "overlay is red over the grey face";
    EXPECT_LT(qGreen(tinted), qGreen(beside) - 25) << "translucent: the face still shows through, tinted";
    EXPECT_GT(qGreen(tinted), 0);

    ASSERT_EQ(frame->picks.size(), 2u);
    EXPECT_EQ(frame->picks[0].overlayIndex, 0u) << "the overlay wins over the body behind it";
    EXPECT_FALSE(frame->picks[0].bodyIndex.has_value());
    EXPECT_EQ(frame->picks[1].bodyIndex, 0u);
    EXPECT_FALSE(frame->picks[1].overlayIndex.has_value());
}

TEST(Overlay, OverlayBehindABodyIsHiddenAndNotPicked) {
    const auto bodies = oneBox();
    const Camera c = viewFrom({0, 0, 1}, unionOf(bodies));
    OverlayFace hidden;
    hidden.triangles = {{-20, -20, -60}, {20, -20, -60}, {20, 20, -60}, {-20, -20, -60}, {20, 20, -60}, {-20, 20, -60}};
    hidden.rgba = {1.f, 0.f, 0.f, 0.9f};
    const QPoint centre(240, 180);
    const auto frame = renderOffscreen(bodies, c, {480, 360}, {centre}, 4, {}, {}, {}, {hidden});
    if (!frame) GTEST_SKIP() << "no QRhi backend available";
    const QRgb p = px(frame->image, centre.x(), centre.y());
    EXPECT_LT(qRed(p) - qGreen(p), 30) << "depth test: the box in front hides the overlay";
    ASSERT_EQ(frame->picks.size(), 1u);
    EXPECT_FALSE(frame->picks[0].overlayIndex.has_value());
    EXPECT_EQ(frame->picks[0].bodyIndex, 0u);
}
