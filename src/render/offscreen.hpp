// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QImage>
#include <QPoint>
#include <QSize>
#include <memory>
#include <optional>
#include <vector>

#include "render/camera.hpp"
#include "render/viewport.hpp"

class QRhi;

namespace iw::render {

/// A QRhi for headless work: Vulkan when available, else OpenGL 3.3 core. Returns nullptr when neither works.
/// `vulkan` (optional) reports which backend was created. Needs a QGuiApplication. GUI thread only.
std::unique_ptr<QRhi> createHeadlessRhi(bool* vulkan = nullptr);

struct OffscreenFrame {
    QImage image;                    ///< Shaded scene, Format_RGBA8888, top row first.
    std::vector<PickResult> picks;   ///< One result per requested pick point (same order).
    bool vulkan = false;             ///< Which backend ran (Vulkan preferred, OpenGL fallback).
};

/// Renders `bodies` headlessly through the same SceneRenderer the Viewport uses (no window needed), with MSAA
/// `samples`, and runs GPU-ID picks at `pickPoints` (device pixels, y down). Returns nullopt when no QRhi
/// backend can be created (e.g. no GPU/driver) - callers/tests should skip then. Needs a QGuiApplication.
/// Also usable for thumbnails/screenshots. GUI thread only.
std::optional<OffscreenFrame> renderOffscreen(const std::vector<SceneBody>& bodies, const Camera& camera,
                                              QSize size, const std::vector<QPoint>& pickPoints = {},
                                              int samples = 4, const PickResult& hover = {},
                                              const PickResult& selection = {},
                                              const SectionRequest& section = {},
                                              const std::vector<OverlayFace>& overlays = {});

/// Test seam for the stale-readback guard: submits a pick at (x,y) for `bodies`, then swaps the scene to
/// `replacement` (when given) before the GPU readback is taken. Returns what the pick callback received (empty when the scene
/// changed in flight), or nullopt when no QRhi backend can be created.
std::optional<PickResult> pickAcrossSceneSwap(const std::vector<SceneBody>& bodies,
                                              const std::optional<std::vector<SceneBody>>& replacement, const Camera& camera,
                                              QSize size, int x, int y);

} // namespace iw::render
