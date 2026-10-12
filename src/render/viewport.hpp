// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QRhiWidget>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include <QPoint>

#include "core/ids.hpp"
#include "core/math.hpp"
#include "render/camera.hpp"
#include "render/nav.hpp"
#include "tess/tessellate.hpp"

namespace iw::render {

/// One drawable body. The mesh is shared immutable data (tessellated off the UI thread).
struct SceneBody {
    BodyId id;
    std::shared_ptr<const tess::MeshData> mesh;
    Transform transform;
    bool visible = true;
    std::array<float, 3> color{0.68f, 0.72f, 0.78f}; ///< Linear RGB surface colour (default light grey).
};

/// Result of a pick. Indices follow the B-rep sub-shape ordering of the picked body's Shape
/// (== MeshData::faceRanges / edgePolylines indices). `bodyIndex` indexes the vector passed to setScene.
struct PickResult {
    std::optional<std::uint32_t> bodyIndex;
    std::optional<std::uint32_t> faceIndex;
    std::optional<std::uint32_t> edgeIndex; ///< Edges take priority over faces within a screen radius.
    std::optional<Vec3> point;              ///< World-space hit point (mm), when something was hit.
};

/// 3D viewport on QRhi (Vulkan preferred, see createVulkanInstance). UI-thread object; every method must be
/// called from the GUI thread. Rendering never blocks on geometry: callers hand it finished MeshData.
///
/// Interaction (gestures from `navMappingFor(navPreset())`): orbit pivots around the picked point under the cursor
/// when the pick finds one (else the scene centre); wheel zoom is smoothed and keeps the point under the cursor
/// fixed; a plain left click selects (emits `picked`); the ViewCube overlay animates the camera (~250 ms).
/// Keys: Home = home view, F = fit.
class Viewport : public QRhiWidget {
    Q_OBJECT
public:
    explicit Viewport(QWidget* parent = nullptr);
    ~Viewport() override;

    /// Replaces the scene. Meshes are shared, not copied. Triggers a repaint. The first non-empty scene is
    /// framed automatically (home view).
    void setScene(std::vector<SceneBody> bodies);
    /// Shows/hides one body (index into scene()). Hidden bodies are neither drawn nor picked. No-op if unchanged.
    void setBodyVisible(std::size_t index, bool visible);
    const std::vector<SceneBody>& scene() const noexcept;
    /// World bounds of the visible bodies (empty box if none).
    Box3 sceneBounds() const;

    Camera& camera() noexcept;
    const Camera& camera() const noexcept;
    void setNavPreset(NavPreset preset);
    NavPreset navPreset() const noexcept;
    /// Shows/hides the ViewCube overlay (click a face/edge/corner -> animate to that view).
    void setViewCubeVisible(bool visible);
    bool viewCubeVisible() const noexcept;

    /// Up axis (document/camera setting, default Y). Re-frames the home view.
    void setUpAxis(UpAxis axis);
    UpAxis upAxis() const noexcept;
    void setProjectionMode(Projection p);
    Projection projectionMode() const noexcept;

    /// Animated view changes (duration in ms; 0 = immediate).
    void viewHome(int ms = 250);
    void viewFit(int ms = 250);
    /// Look from `eyeDir` (target -> eye, e.g. a ViewCube sign vector), keeping target and distance.
    void viewFromDirection(const Vec3& eyeDir, int ms = 250);
    void animateTo(const CameraState& s, int ms = 250);
    bool isAnimating() const noexcept;

    void setSelection(const PickResult& r);
    PickResult selection() const noexcept;
    PickResult hover() const noexcept;

    /// Asynchronous GPU-ID-buffer pick at widget pixel (x,y). `done` is invoked exactly once, later on the GUI
    /// thread (typically 1-2 frames) with the result. It is called with an empty PickResult when a newer pick
    /// supersedes this one, when the GPU readback fails, when no result arrives within 2 s (no frames rendered,
    /// e.g. QPA offscreen), and on teardown. Never blocks.
    void pick(int x, int y, std::function<void(PickResult)> done);

signals:
    void cameraChanged();
    /// A plain left click: what is under the cursor (empty result = clicked on nothing; selection cleared).
    void picked(const iw::render::PickResult& result);
    void hoverChanged(const iw::render::PickResult& result);

protected:
    void initialize(QRhiCommandBuffer* cb) override;
    void render(QRhiCommandBuffer* cb) override;
    void releaseResources() override;
    void resizeEvent(QResizeEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void keyPressEvent(QKeyEvent* e) override;
    void leaveEvent(QEvent* e) override;

private:
    void flushPicks();
    static bool headless();
    void serviceHeadlessPick();
    struct Impl;
    std::unique_ptr<Impl> d_;
};

} // namespace iw::render
