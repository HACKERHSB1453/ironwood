// SPDX-License-Identifier: GPL-3.0-or-later
#include "render/offscreen.hpp"

#include <QOffscreenSurface>
#include <QSurfaceFormat>
#include <QVulkanInstance>
#include <memory>
#include <rhi/qrhi.h>

#include "render/scene_renderer.hpp"
#include "render/vulkan_support.hpp"

namespace iw::render {

namespace {
std::unique_ptr<QRhi> makeRhi(bool& vulkan) {
    static bool vulkanUnavailable = false;   // e.g. the offscreen QPA plugin has no Vulkan support
    QVulkanInstance* inst = vulkanUnavailable ? nullptr : createVulkanInstance();
    if (!inst) vulkanUnavailable = true;
    if (inst) {
        QRhiVulkanInitParams p;
        p.inst = inst;
        if (auto* rhi = QRhi::create(QRhi::Vulkan, &p, {}, nullptr)) {
            vulkan = true;
            return std::unique_ptr<QRhi>(rhi);
        }
    }
    vulkan = false;
    QSurfaceFormat fmt;   // the embedded shaders carry GLSL 330, so ask for a 3.3 core context
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QRhiGles2InitParams gl;
    gl.format = fmt;
    gl.fallbackSurface = QRhiGles2InitParams::newFallbackSurface(fmt);
    if (auto* rhi = QRhi::create(QRhi::OpenGLES2, &gl, {}, nullptr)) return std::unique_ptr<QRhi>(rhi);
    qWarning("iw::render: no QRhi backend could be created");
    return nullptr;
}
} // namespace

std::unique_ptr<QRhi> createHeadlessRhi(bool* vulkan) {
    bool v = false;
    auto rhi = makeRhi(v);
    if (vulkan) *vulkan = v;
    return rhi;
}

namespace {

std::optional<OffscreenFrame> fail(int where) {
    qWarning("iw::render::renderOffscreen failed at step %d", where);
    return std::nullopt;
}
} // namespace
std::optional<OffscreenFrame> renderOffscreen(const std::vector<SceneBody>& bodies, const Camera& camera, QSize size,
                                              const std::vector<QPoint>& pickPoints, int samples,
                                              const PickResult& hover, const PickResult& selection,
                                              const SectionRequest& section, const std::vector<OverlayFace>& overlays) {
    OffscreenFrame out;
    auto rhi = makeRhi(out.vulkan);
    if (!rhi) return fail(1);
    {
        SceneRenderer renderer(rhi.get());
        renderer.setScene(bodies);
        renderer.setHover(hover);
        renderer.setSelection(selection);
        renderer.setSection(section);
        renderer.setOverlays(overlays);

        std::unique_ptr<QRhiTexture> color(rhi->newTexture(
            QRhiTexture::RGBA8, size, 1, QRhiTexture::RenderTarget | QRhiTexture::UsedAsTransferSource));
        std::unique_ptr<QRhiRenderBuffer> msaa;
        const int sc = rhi->supportedSampleCounts().contains(samples) ? samples : 1;
        std::unique_ptr<QRhiRenderBuffer> depth(rhi->newRenderBuffer(QRhiRenderBuffer::DepthStencil, size, sc));
        if (!color->create() || !depth->create()) return fail(2);
        QRhiColorAttachment att;
        if (sc > 1) {
            msaa.reset(rhi->newRenderBuffer(QRhiRenderBuffer::Color, size, sc));
            if (!msaa->create()) return fail(3);
            att.setRenderBuffer(msaa.get());
            att.setResolveTexture(color.get());
        } else {
            att.setTexture(color.get());
        }
        QRhiTextureRenderTargetDescription desc(att);
        desc.setDepthStencilBuffer(depth.get());
        std::unique_ptr<QRhiTextureRenderTarget> rt(rhi->newTextureRenderTarget(desc));
        std::unique_ptr<QRhiRenderPassDescriptor> rpd(rt->newCompatibleRenderPassDescriptor());
        rt->setRenderPassDescriptor(rpd.get());
        if (!rt->create()) return fail(4);

        QRhiCommandBuffer* cb = nullptr;
        if (rhi->beginOffscreenFrame(&cb) != QRhi::FrameOpSuccess) return fail(5);
        renderer.render(cb, rt.get(), camera, size);
        QRhiReadbackResult rb;
        auto* ru = rhi->nextResourceUpdateBatch();
        ru->readBackTexture(QRhiReadbackDescription(color.get()), &rb);
        cb->resourceUpdate(ru);
        rhi->endOffscreenFrame();
        rhi->finish();
        QImage img(reinterpret_cast<const uchar*>(rb.data.constData()), rb.pixelSize.width(), rb.pixelSize.height(),
                   QImage::Format_RGBA8888);
        out.image = rhi->isYUpInFramebuffer() ? img.flipped(Qt::Vertical) : img.copy();

        for (const QPoint& pt : pickPoints) {
            PickResult result;
            bool got = false;
            if (rhi->beginOffscreenFrame(&cb) != QRhi::FrameOpSuccess) return fail(6);
            const bool queued = renderer.submitPick(cb, camera, size, pt.x(), pt.y(), [&](PickResult r) {
                result = r;
                got = true;
            });
            rhi->endOffscreenFrame();
            rhi->finish();
            if (!queued || !got) return fail(7);
            out.picks.push_back(result);
        }
    }
    return out;
}

std::optional<PickResult> pickAcrossSceneSwap(const std::vector<SceneBody>& bodies,
                                              const std::optional<std::vector<SceneBody>>& replacement, const Camera& camera,
                                              QSize size, int x, int y) {
    bool vulkan = false;
    auto rhi = makeRhi(vulkan);
    if (!rhi) return std::nullopt;
    SceneRenderer renderer(rhi.get());
    renderer.setScene(bodies);
    QRhiCommandBuffer* cb = nullptr;
    if (rhi->beginOffscreenFrame(&cb) != QRhi::FrameOpSuccess) return std::nullopt;
    std::optional<PickResult> got;
    const bool queued = renderer.submitPick(cb, camera, size, x, y, [&](PickResult r) { got = r; });
    if (replacement) renderer.setScene(*replacement);   // the scene changes while the readback is in flight
    rhi->endOffscreenFrame();
    rhi->finish();
    if (!queued || !got) return std::nullopt;
    return got;
}

} // namespace iw::render
