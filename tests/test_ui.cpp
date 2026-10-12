// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <condition_variable>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>

#include <QAction>
#include <QApplication>
#include <QMenuBar>
#include <QMouseEvent>
#include <QToolBar>
#include <QElapsedTimer>
#include <QWheelEvent>
#include <QTreeWidget>

#include <cstdlib>
#include <filesystem>
#include <optional>

#include "kernel/exchange.hpp"
#include "kernel/ops.hpp"
#include "render/offscreen.hpp"

#include "render/demo_scene.hpp"
#include "render/viewcube_widget.hpp"
#include "render/viewport.hpp"
#include "ui/main_window.hpp"

using namespace iw;

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
/// One QApplication for the binary; the offscreen platform unless the caller picked one.
class QtEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
        app_ = new QApplication(argc_, argv_);
        ::testing::UnitTest::GetInstance()->listeners().Append(new TestWatchdog);
    }
    void TearDown() override { delete app_; }

private:
    int argc_ = 1;
    char name_[8] = "test_ui";
    char* argv_[2] = {name_, nullptr};
    QApplication* app_ = nullptr;
};
const auto* const kEnv = ::testing::AddGlobalTestEnvironment(new QtEnv);

/// Runs the event loop for `ms` (timers/animations tick).
void wait(int ms) {
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 10);
}

/// Pumps the event loop until `pred` holds or `ms` elapse. Returns the final value of `pred`.
template <class Pred> bool waitFor(Pred pred, int ms) {
    QElapsedTimer t;
    t.start();
    while (!pred() && t.elapsed() < ms) {
        QApplication::processEvents(QEventLoop::AllEvents, 10);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pred();
}

/// Writes a three-part STEP assembly to `path` through kernel::exportStep: group "Assembly" holding "Bracket"
/// (red, centred on the origin), "Plate" (green) and "Bolt" (blue), the last two placed by their local transforms.
/// The bounds are symmetric around the origin, so the view centre lands on the Bracket.
void writeFixtureStep(const std::filesystem::path& path) {
    auto box = [](Vec3 origin) {
        auto r = kernel::makeBox(origin, 20, 20, 20);
        EXPECT_TRUE(r.has_value());
        return r->shape;
    };
    auto translate = [](double x) {
        Transform t;
        t.m[3] = x;
        return t;
    };
    kernel::ImportedAssembly a;
    kernel::AssemblyNode group;
    group.name = "Assembly";
    group.children = {1, 2, 3};
    a.nodes.push_back(group);
    kernel::AssemblyNode bracket;
    bracket.name = "Bracket";
    bracket.color = kernel::Color{0.9f, 0.2f, 0.2f, 1.f};
    bracket.shape = box({-10, -10, -10});
    bracket.parent = 0;
    kernel::AssemblyNode plate;
    plate.name = "Plate";
    plate.color = kernel::Color{0.2f, 0.8f, 0.3f, 1.f};
    plate.shape = box({0, -10, -10});
    plate.local = translate(-35);
    plate.parent = 0;
    kernel::AssemblyNode bolt;
    bolt.name = "Bolt";
    bolt.color = kernel::Color{0.2f, 0.4f, 0.9f, 1.f};
    bolt.shape = box({0, -10, -10});
    bolt.local = translate(15);
    bolt.parent = 0;
    a.nodes.push_back(bracket);
    a.nodes.push_back(plate);
    a.nodes.push_back(bolt);
    a.roots = {0};
    a.lengthUnitInFile = "mm";
    auto written = kernel::exportStep(path, a);
    ASSERT_TRUE(written.has_value()) << written.error().message;
}

std::filesystem::path scratchPath(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("ironwood-ui-test-" + name);
}

void click(QWidget* w, QPoint p) {
    for (auto type : {QEvent::MouseButtonPress, QEvent::MouseButtonRelease}) {
        const bool down = type == QEvent::MouseButtonPress;
        QMouseEvent e(type, QPointF(p), w->mapToGlobal(QPointF(p)), Qt::LeftButton,
                      down ? Qt::LeftButton : Qt::NoButton, Qt::NoModifier);
        QApplication::sendEvent(w, &e);
    }
}
} // namespace

TEST(Ui, DefaultTitle) { EXPECT_FALSE(ui::MainWindow::defaultTitle().isEmpty()); }

TEST(Ui, MainWindowConstructsUnderOffscreenPlatform) {
    ui::MainWindow w;
    EXPECT_EQ(w.windowTitle(), ui::MainWindow::defaultTitle());
    ASSERT_NE(w.viewport(), nullptr);
    EXPECT_TRUE(w.viewport()->viewCubeVisible());
    EXPECT_EQ(w.viewport()->upAxis(), render::UpAxis::Y);          // default up axis is Y
    EXPECT_EQ(w.viewport()->scene().size(), 2u);                   // demo scene is loaded
    EXPECT_FALSE(w.viewport()->sceneBounds().isEmpty());
    EXPECT_NE(w.menuBar(), nullptr);
    EXPECT_NE(w.findChild<QToolBar*>("viewToolbar"), nullptr);
}

TEST(Ui, DemoSceneIsFramedAndCameraStateIsSane) {
    ui::MainWindow w;
    auto* vp = w.viewport();
    const auto b = vp->sceneBounds();
    const auto& cam = vp->camera();
    // the camera target is the scene centre and the whole demo is in front of the eye
    EXPECT_NEAR(cam.target().x, (b.min.x + b.max.x) / 2, 1e-6);
    EXPECT_NEAR(cam.target().y, (b.min.y + b.max.y) / 2, 1e-6);
    EXPECT_GT(cam.distance(), 100.0);
}

TEST(Ui, ViewMenuActionsDriveTheViewport) {
    ui::MainWindow w;
    auto* vp = w.viewport();
    auto find = [&](const QString& text) -> QAction* {
        for (QAction* a : w.findChildren<QAction*>())
            if (a->text().remove('&') == text) return a;
        return nullptr;
    };
    QAction* ortho = find("Orthographic");
    ASSERT_NE(ortho, nullptr);
    ortho->setChecked(true);
    EXPECT_EQ(vp->projectionMode(), render::Projection::Orthographic);
    ortho->setChecked(false);
    EXPECT_EQ(vp->projectionMode(), render::Projection::Perspective);

    QAction* z = find("Z up");
    ASSERT_NE(z, nullptr);
    z->trigger();
    EXPECT_EQ(vp->upAxis(), render::UpAxis::Z);
    QAction* y = find("Y up");
    ASSERT_NE(y, nullptr);
    y->trigger();
    EXPECT_EQ(vp->upAxis(), render::UpAxis::Y);

    QAction* cube = find("ViewCube");
    ASSERT_NE(cube, nullptr);
    cube->setChecked(false);
    EXPECT_FALSE(vp->viewCubeVisible());
}

TEST(Ui, ViewportNavigationGesturesMoveTheCamera) {
    ui::MainWindow w;
    w.show();   // offscreen platform; QRhiWidget runs on the Null backend there
    auto* vp = w.viewport();
    vp->resize(800, 600);
    QApplication::processEvents();
    vp->viewHome(0);
    const render::CameraState s0 = vp->camera().state();

    auto press = [&](Qt::MouseButton b, Qt::KeyboardModifiers m, QPoint p) {
        QMouseEvent e(QEvent::MouseButtonPress, QPointF(p), vp->mapToGlobal(QPointF(p)), b, b, m);
        QApplication::sendEvent(vp, &e);
    };
    auto move = [&](Qt::MouseButton b, Qt::KeyboardModifiers m, QPoint p) {
        QMouseEvent e(QEvent::MouseMove, QPointF(p), vp->mapToGlobal(QPointF(p)), Qt::NoButton, b, m);
        QApplication::sendEvent(vp, &e);
    };
    auto release = [&](Qt::MouseButton b, QPoint p) {
        QMouseEvent e(QEvent::MouseButtonRelease, QPointF(p), vp->mapToGlobal(QPointF(p)), b, Qt::NoButton, Qt::NoModifier);
        QApplication::sendEvent(vp, &e);
    };

    // Fusion: MMB drag = pan
    press(Qt::MiddleButton, Qt::NoModifier, {400, 300});
    move(Qt::MiddleButton, Qt::NoModifier, {450, 330});
    release(Qt::MiddleButton, {450, 330});
    const render::CameraState s1 = vp->camera().state();
    EXPECT_GT(length(s1.target - s0.target), 1e-3);
    EXPECT_DOUBLE_EQ(s1.yaw, s0.yaw);

    // Fusion: Shift+MMB drag = orbit
    press(Qt::MiddleButton, Qt::ShiftModifier, {400, 300});
    move(Qt::MiddleButton, Qt::ShiftModifier, {460, 300});
    release(Qt::MiddleButton, {460, 300});
    EXPECT_NE(vp->camera().yaw(), s1.yaw);

    // wheel zooms (smoothed over a few timer ticks)
    const double d0 = vp->camera().distance();
    QWheelEvent wheel(QPointF(300, 200), QPointF(300, 200), QPoint(), QPoint(0, 120), Qt::NoButton, Qt::NoModifier,
                      Qt::NoScrollPhase, false);
    QApplication::sendEvent(vp, &wheel);
    wait(300);
    EXPECT_LT(vp->camera().distance(), d0 * 0.95);
}

TEST(Ui, AnimatedViewTransitionLandsOnTheRequestedView) {
    ui::MainWindow w;
    w.show();
    auto* vp = w.viewport();
    vp->viewHome(0);
    vp->viewFromDirection({0, 0, 1}, 250);
    EXPECT_TRUE(vp->isAnimating());
    wait(400);
    EXPECT_FALSE(vp->isAnimating());
    const auto eyeDir = vp->camera().eye() - vp->camera().target();
    EXPECT_NEAR(eyeDir.x, 0.0, 1e-6);
    EXPECT_NEAR(eyeDir.y, 0.0, 1e-6);
    EXPECT_GT(eyeDir.z, 1.0);
}

TEST(Ui, ViewCubeWidgetClicksRequestViewsAndHome) {
    render::Camera cam;
    cam.setState(cam.poseForEyeDirection({0, 0, 1}));   // Front
    render::ViewCubeWidget cube(&cam);
    cube.resize(render::ViewCubeWidget::kSize, render::ViewCubeWidget::kSize);
    cube.show();
    int views = 0, homes = 0;
    Vec3 got;
    QObject::connect(&cube, &render::ViewCubeWidget::viewRequested, [&](double x, double y, double z) {
        ++views;
        got = {x, y, z};
    });
    QObject::connect(&cube, &render::ViewCubeWidget::homeRequested, [&] { ++homes; });

    const QPointF centre(render::ViewCubeWidget::kSize / 2.0 + 6, render::ViewCubeWidget::kSize / 2.0 + 4);
    ASSERT_TRUE(cube.directionAt(centre).has_value());
    EXPECT_EQ(*cube.directionAt(centre), (Vec3{0, 0, 1}));
    click(&cube, centre.toPoint());
    EXPECT_EQ(views, 1);
    EXPECT_EQ(got, (Vec3{0, 0, 1}));
    // top-right corner of the front face -> corner view (1,1,1)
    const double s = (render::ViewCubeWidget::kSize / 2.0 - 8) * 0.58;   // cube half-extent in px
    const QPointF corner = centre + QPointF(0.8 * s, -0.8 * s);
    click(&cube, corner.toPoint());
    EXPECT_EQ(views, 2);
    EXPECT_EQ(got, (Vec3{1, 1, 1}));
    // home button
    ASSERT_TRUE(cube.homeButtonAt(QPointF(10, 10)));
    click(&cube, QPoint(10, 10));
    EXPECT_EQ(homes, 1);
    // clicking the empty corner does nothing
    click(&cube, QPoint(render::ViewCubeWidget::kSize - 2, render::ViewCubeWidget::kSize - 2));
    EXPECT_EQ(views, 2);
}

TEST(Ui, DescribePick) {
    render::PickResult r;
    EXPECT_FALSE(ui::MainWindow::describePick(r).isEmpty());
    r.bodyIndex = 1;
    r.faceIndex = 4;
    EXPECT_TRUE(ui::MainWindow::describePick(r).contains("Face 4"));
    r.edgeIndex = 9;
    EXPECT_TRUE(ui::MainWindow::describePick(r).contains("Edge 9"));
}

// ------------------------------------------------------------------------------------ import (STEP fixture)
TEST(UiImport, StepAssemblyImportsIntoTreeAndViewport) {
    const auto step = scratchPath("assembly.step");
    writeFixtureStep(step);
    ui::MainWindow w;
    w.show();
    ASSERT_TRUE(w.openFile(QString::fromStdString(step.string())));
    ASSERT_TRUE(w.importing());
    ASSERT_TRUE(waitFor([&] { return !w.importing(); }, 60000)) << "import did not finish within 60 s";
    ASSERT_FALSE(w.lastImportReport().isEmpty()) << "import failed or was cancelled";
    EXPECT_TRUE(w.lastImportReport().contains("3 bodies"));
    EXPECT_TRUE(w.windowTitle().contains("assembly.step"));

    // Tree: one root group with three named leaves, in assembly order.
    QTreeWidget* tree = w.browser();
    ASSERT_EQ(tree->topLevelItemCount(), 1);
    QTreeWidgetItem* root = tree->topLevelItem(0);
    EXPECT_EQ(root->text(0), "Assembly");
    ASSERT_EQ(root->childCount(), 3);
    EXPECT_EQ(root->child(0)->text(0), "Bracket");
    EXPECT_EQ(root->child(1)->text(0), "Plate");
    EXPECT_EQ(root->child(2)->text(0), "Bolt");

    // Viewport: one body per leaf, STEP colours carried over, assembly transforms applied.
    render::Viewport* vp = w.viewport();
    ASSERT_EQ(vp->scene().size(), 3u);
    EXPECT_NEAR(vp->scene()[0].color[0], 0.9f, 1e-6);
    EXPECT_NEAR(vp->scene()[2].color[2], 0.9f, 1e-6);
    EXPECT_NEAR(vp->scene()[1].transform.m[3], -35.0, 1e-6);
    EXPECT_NEAR(vp->scene()[2].transform.m[3], 15.0, 1e-6);
    EXPECT_NEAR(vp->sceneBounds().min.x, -35.0, 1e-3);
    EXPECT_NEAR(vp->sceneBounds().max.x, 35.0, 1e-3);

    // Visibility checkbox hides and shows the body; the group reflects the partial state.
    root->child(1)->setCheckState(0, Qt::Unchecked);
    EXPECT_FALSE(vp->scene()[1].visible);
    EXPECT_TRUE(vp->scene()[0].visible);
    EXPECT_EQ(root->checkState(0), Qt::PartiallyChecked);
    EXPECT_NEAR(vp->sceneBounds().min.x, -10.0, 1e-3);   // hidden bodies leave the bounds
    root->child(1)->setCheckState(0, Qt::Checked);
    EXPECT_TRUE(vp->scene()[1].visible);

    // Picking the view centre (on the Bracket) selects its tree node, and selecting the node highlights it.
    vp->resize(800, 600);
    w.show();
    QApplication::processEvents();
    vp->viewFit(0);
    // A plain click at the centre runs a GPU pick; the viewport then selects and emits `picked`.
    std::optional<std::uint32_t> pickedBody;
    QObject::connect(vp, &render::Viewport::picked, &w, [&](const render::PickResult& r) { pickedBody = r.bodyIndex; });
    click(vp, QPoint(vp->width() / 2, vp->height() / 2));
    ASSERT_TRUE(waitFor([&] { return pickedBody.has_value(); }, 15000)) << "centre click never picked a body";
    EXPECT_EQ(*pickedBody, 0u);
    EXPECT_EQ(tree->currentItem(), root->child(0));
    EXPECT_EQ(vp->selection().bodyIndex.value_or(99u), 0u);

    tree->setCurrentItem(root->child(2));
    EXPECT_EQ(vp->selection().bodyIndex.value_or(99u), 2u);

    // Screenshot of the imported assembly (headless render of the same scene) when a dump dir is given.
    if (const char* dir = std::getenv("IW_TEST_DUMP_DIR")) {
        const QSize size(800, 600);
        render::Camera cam = vp->camera();
        cam.setViewportSize(size.width(), size.height());
        auto frame = render::renderOffscreen(vp->scene(), cam, size);
        EXPECT_TRUE(frame.has_value());
        if (frame) {
            EXPECT_TRUE(frame->image.save(QString::fromLocal8Bit(dir) + "/ui-import.png"));
        }
    }
    std::error_code ec;
    std::filesystem::remove(step, ec);
}

TEST(UiImport, CancelAndFailureAreReportedWithoutHanging) {
    ui::MainWindow w;
    EXPECT_FALSE(w.openFile("drawing.dwg"));                       // unsupported type: rejected immediately
    EXPECT_FALSE(w.importing());

    ASSERT_TRUE(w.openFile(QString::fromStdString(scratchPath("missing.step").string())));
    ASSERT_TRUE(waitFor([&] { return !w.importing(); }, 30000));
    EXPECT_TRUE(w.lastImportReport().isEmpty());                    // IoError: no report
    EXPECT_EQ(w.viewport()->scene().size(), 2u);                    // demo scene still shown on failure

    const auto step = scratchPath("cancel.step");
    writeFixtureStep(step);
    ASSERT_TRUE(w.openFile(QString::fromStdString(step.string())));
    w.cancelImport();                                              // Esc
    EXPECT_FALSE(w.importing());
    std::error_code ec;
    std::filesystem::remove(step, ec);
}

TEST(UiImport, PickInFlightIsAnsweredWhenTheViewportIsDestroyed) {
    int calls = 0;
    bool empty = false;
    auto* vp = new render::Viewport;
    vp->pick(10, 10, [&](render::PickResult r) {
        ++calls;
        empty = !r.bodyIndex.has_value();
    });
    delete vp;   // teardown flushes queued picks with an empty result: exactly one callback
    EXPECT_EQ(calls, 1);
    EXPECT_TRUE(empty);
}
