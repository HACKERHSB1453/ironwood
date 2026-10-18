// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Internal to the render module (not part of the public interface): draws a scene of SceneBody onto any QRhi
// render target and implements GPU-ID-buffer picking. Kept independent of QRhiWidget so it is testable with a
// headless QRhi.

#include <QSize>
#include <array>
#include <functional>
#include <memory>
#include <vector>

#include "render/camera.hpp"
#include "render/viewport.hpp"

class QRhi;
class QRhiCommandBuffer;
class QRhiRenderTarget;

namespace iw::render {

class SceneRenderer {
public:
    explicit SceneRenderer(QRhi* rhi);
    ~SceneRenderer();
    SceneRenderer(const SceneRenderer&) = delete;
    SceneRenderer& operator=(const SceneRenderer&) = delete;

    /// Replaces the scene (GPU buffers are rebuilt lazily on the next render).
    void setScene(std::vector<SceneBody> bodies);
    void setHover(const PickResult& r);
    void setSelection(const PickResult& r);
    /// Section clip + cap (applies to bodies, edges and picks). Does not invalidate pending picks.
    void setSection(const SectionRequest& s);
    /// Translucent overlays; index = position in `faces`. Does not invalidate pending picks.
    void setOverlays(std::vector<OverlayFace> faces);

    /// Draws gradient background + shaded faces + edge lines into `rt` (must be inside a frame, outside a pass).
    /// `pixelSize` is the render target size in device pixels; the camera aspect is taken from it.
    void render(QRhiCommandBuffer* cb, QRhiRenderTarget* rt, const Camera& camera, QSize pixelSize);

    /// Renders the ID pass for the window around device pixel (x,y) and queues an asynchronous readback.
    /// `done` is called from inside QRhi when the readback completes (GUI thread). Returns false if all
    /// readback slots are still in flight (caller should retry on a later frame).
    bool submitPick(QRhiCommandBuffer* cb, const Camera& camera, QSize pixelSize, int x, int y,
                    std::function<void(PickResult)> done);
    bool canSubmitPick() const;
    /// Side length (px) of the pick window and radius within which edges beat faces.
    static constexpr int kPickWindow = 17;
    static constexpr double kEdgePickRadius = 6.0;

private:
    struct Impl;
    std::unique_ptr<Impl> d_;
};

/// Outcome of decoding a pick window; `px,py` is the chosen texel (window coordinates, top row first).
struct PickDecode {
    PickResult result;
    bool hit = false;
    int px = 0, py = 0;
    float depth = 0;  ///< Distance along the camera forward axis at the chosen texel.
};
/// Decodes a pick window (RGBA32F, window^2 texels, top row first; centre texel = the picked pixel).
/// Overlays within `edgeRadius` texels beat everything; then edges beat faces. Exposed for tests.
PickDecode decodePickWindow(const float* rgba, int window, double edgeRadius);

} // namespace iw::render
