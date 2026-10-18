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

/// Section (cut) plane. Render-local mirror of kernel `section::RenderRequest` (same fields, same meaning), so the
/// render module stays free of OCCT. The UI converts one to the other field by field.
/// Fragments with `clipPlane` (nx,ny,nz,d) dot p + d < 0 are discarded (kept side n·p + d >= 0, mm). When
/// `enabled` is false nothing is clipped. With the section on, the cut solids get a flat cap in `capColour`.
struct SectionRequest {
    bool enabled = false;
    std::array<double, 4> clipPlane{0, 0, 1, 0};
    std::array<float, 4> capColour{1.0f, 0.55f, 0.1f, 1.0f}; ///< RGBA 0..1
    friend bool operator==(const SectionRequest&, const SectionRequest&) = default;
};

/// A translucent flat overlay drawn after the opaque bodies (depth-tested, not depth-written), e.g. a sketch
/// profile tint. `triangles` holds 3 world-space vertices (mm) per triangle.
struct OverlayFace {
    std::vector<Vec3> triangles;
    std::array<float, 4> rgba{0.2f, 0.5f, 1.0f, 0.35f}; ///< RGBA 0..1, straight alpha.
    friend bool operator==(const OverlayFace&, const OverlayFace&) = default;
};

/// Result of a pick. Indices follow the B-rep sub-shape ordering of the picked body's Shape
/// (== MeshData::faceRanges / edgePolylines indices). `bodyIndex` indexes the vector passed to setScene.
struct PickResult {
    std::optional<std::uint32_t> bodyIndex;
    std::optional<std::uint32_t> faceIndex;
    std::optional<std::uint32_t> edgeIndex; ///< Edges take priority over faces within a screen radius.
    std::optional<Vec3> point;              ///< World-space hit point (mm), when something was hit.
    /// Index into the list given to setOverlayFaces. Set alone (no body/face/edge) when an overlay is hit: an
    /// overlay that is visible at the pick point wins over the bodies and edges behind it.
    std::optional<std::uint32_t> overlayIndex;
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

    /// Section analysis: clips every body (and its edges) to the kept side and draws a cap on the cut. Picking
    /// ignores the clipped half. Triggers a repaint.
    void setSectionRequest(const SectionRequest& r);
    SectionRequest sectionRequest() const noexcept;

    /// Translucent overlays (sketch profile shading). Replaces the whole list; index = position in `faces`.
    /// Drawn after the opaque geometry; pickable through PickResult::overlayIndex.
    void setOverlayFaces(std::vector<OverlayFace> faces);
    const std::vector<OverlayFace>& overlayFaces() const noexcept;

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
