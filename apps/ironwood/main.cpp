// SPDX-License-Identifier: GPL-3.0-or-later
#include <QApplication>
#include <QTimer>
#include <cstdio>
#include <cstring>

#include "render/viewport.hpp"
#include "ui/main_window.hpp"

namespace {

/// Hard limit for `--selftest`: a frame or pick that never arrives fails the run instead of hanging it.
constexpr int kSelfTestTimeoutMs = 20000;

void printUsage(std::FILE* out) {
    std::fprintf(out,
                 "Usage: ironwood [options] [file]\n"
                 "\n"
                 "Opens a STEP (.step, .stp) or STL (.stl) file in the modeller.\n"
                 "\n"
                 "Options:\n"
                 "  -h, --help          show this help and exit\n"
                 "  --version           print the version and exit\n"
                 "  --selftest <prefix> smoke test for a machine with a display: renders, writes\n"
                 "                      <prefix>-viewport.png and <prefix>-window.png, picks the viewport centre\n"
                 "                      and exits 0 only if it hits a body (fails after 20 s at most)\n");
}

/// `ironwood --selftest <out-prefix>`: shows the window, lets a few frames render, saves <prefix>-viewport.png
/// (GPU frame) and <prefix>-window.png (whole window incl. ViewCube), runs a GPU pick at the viewport centre and
/// exits 0 only if it hit a body. Every wait is bounded: the viewport's pick watchdog answers an empty result
/// when no frame arrives, and the whole run is cut off after kSelfTestTimeoutMs.
void runSelfTest(iw::ui::MainWindow& window, const QString& prefix) {
    QTimer::singleShot(kSelfTestTimeoutMs, qApp, [] {
        std::fprintf(stderr, "selftest: timed out after %d s\n", kSelfTestTimeoutMs / 1000);
        QCoreApplication::exit(3);
    });
    auto* vp = window.viewport();
    QTimer::singleShot(1500, &window, [&window, vp, prefix] {
        vp->grabFramebuffer().save(prefix + "-viewport.png");
        vp->pick(vp->width() / 2, vp->height() / 2, [&window, vp, prefix](iw::render::PickResult r) {
            std::printf("pick: body=%d face=%d edge=%d\n", r.bodyIndex ? int(*r.bodyIndex) : -1,
                        r.faceIndex ? int(*r.faceIndex) : -1, r.edgeIndex ? int(*r.edgeIndex) : -1);
            vp->viewFromDirection({0, 0, 1}, 0);
            QTimer::singleShot(500, &window, [&window, prefix, hit = r.bodyIndex.has_value()] {
                window.grab().save(prefix + "-window.png");
                QApplication::exit(hit ? 0 : 1);
            });
        });
    });
}

} // namespace

int main(int argc, char** argv) {
    // Argument handling happens before QApplication exists: --help/--version never create a window or a GPU
    // context.
    QString file;
    QString selftest;
    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        if (std::strcmp(arg, "-h") == 0 || std::strcmp(arg, "--help") == 0) {
            printUsage(stdout);
            return 0;
        }
        if (std::strcmp(arg, "--version") == 0) {
            std::printf("ironwood %s\n", IRONWOOD_VERSION);
            return 0;
        }
        if (std::strcmp(arg, "--selftest") == 0) {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "ironwood: --selftest needs an output prefix\n");
                return 2;
            }
            selftest = QString::fromLocal8Bit(argv[++i]);
            continue;
        }
        if (arg[0] == '-' && arg[1] != '\0') {
            std::fprintf(stderr, "ironwood: unknown option '%s'\n\n", arg);
            printUsage(stderr);
            return 2;
        }
        if (!file.isEmpty()) {
            std::fprintf(stderr, "ironwood: only one file can be opened at a time\n");
            return 2;
        }
        file = QString::fromLocal8Bit(arg);
    }

    QApplication app(argc, argv);
    app.setApplicationName("Ironwood");
    app.setOrganizationName("Ironwood");   // QSettings location for shortcuts, pinned commands, recent commands
    app.setApplicationVersion(IRONWOOD_VERSION);

    // The Viewport is a QRhiWidget: Qt creates and owns the QVulkanInstance of the top-level window itself
    // (attaching our own from render/vulkan_support.hpp here makes window teardown use mismatched instances and
    // crash). Debug builds turn the validation layer on through QRhiWidget::setDebugLayerEnabled (Viewport).
    iw::ui::MainWindow window;
    window.show();
    if (!file.isEmpty()) window.openFile(file);
    if (!selftest.isEmpty()) runSelfTest(window, selftest);
    return app.exec();
}
