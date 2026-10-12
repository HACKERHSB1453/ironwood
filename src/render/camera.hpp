// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <optional>
#include <utility>

#include "core/math.hpp"

namespace iw::render {

enum class Projection { Perspective, Orthographic };

/// Which world axis points "up" on screen. Document/camera setting; default Y-up.
/// [VERIFY] Fusion's default modelling orientation is Y-up (Z-up selectable); confirm against the running app.
/// Y-up: Front = eye on +Z, Right = eye on +X, Top = eye on +Y.
/// Z-up: Front = eye on -Y, Right = eye on +X, Top = eye on +Z.
enum class UpAxis { Y, Z };

/// A complete orbit-camera pose; used for animated view transitions.
struct CameraState {
    Vec3 target{};
    double distance = 100.0;
    double yaw = 0.6;
    double pitch = 0.5;
};

/// Interpolates between two poses (`t` in [0,1]): target linear, distance geometric, yaw along the shortest arc.
CameraState lerp(const CameraState& a, const CameraState& b, double t);

/// A world-space ray (`dir` is unit length).
struct Ray {
    Vec3 origin, dir;
};

/// Orbit camera (Qt-free, pure math, single-threaded - UI thread). Right-handed world; the up axis is a setting.
/// State: look-at `target`, `distance`, `yaw`/`pitch` (radians), vertical field of view, projection mode.
/// Eye direction (target -> eye) = cos(pitch)cos(yaw) A + cos(pitch)sin(yaw) B + sin(pitch) U, where (A,B,U) is a
/// right-handed frame whose U is the up axis (Z-up: X,Y,Z; Y-up: Z,X,Y).
/// Input deltas are in device-independent pixels of the viewport, Qt screen convention (x right, y DOWN):
/// the scene follows the mouse (drag right -> scene turns/moves right; drag down -> scene turns/moves down).
class Camera {
public:
    void setProjection(Projection p) noexcept { projection_ = p; }
    Projection projection() const noexcept { return projection_; }
    void setViewportSize(int width, int height) noexcept { width_ = width < 1 ? 1 : width; height_ = height < 1 ? 1 : height; }
    int viewportWidth() const noexcept { return width_; }
    int viewportHeight() const noexcept { return height_; }
    double aspect() const noexcept { return static_cast<double>(width_) / height_; }

    /// Changes the up axis. Orientation (yaw/pitch) is reinterpreted, so callers normally re-frame afterwards.
    void setUpAxis(UpAxis a) noexcept { up_ = a; }
    UpAxis upAxis() const noexcept { return up_; }
    /// Bounds of the scene, used for near/far planes. Empty = unknown (planes derived from `distance`).
    void setSceneBounds(const Box3& b) noexcept { sceneBounds_ = b; }
    const Box3& sceneBounds() const noexcept { return sceneBounds_; }

    /// Rotates around `target`.
    void orbit(double dxPixels, double dyPixels);
    /// Rotates around an arbitrary world `pivot`: the pivot keeps its screen position (target moves with the eye).
    void orbitAbout(double dxPixels, double dyPixels, const Vec3& pivot);
    /// Translates `target` in the view plane so the scene follows the cursor 1:1 at the target depth.
    void pan(double dxPixels, double dyPixels);
    /// Dollies/scales by `factor` (>1 = zoom in), keeping the world point under the cursor fixed
    /// (cursor given in NDC, x,y in [-1,1], y up) when `towardCursor`, else toward the target.
    /// The fixed point is the one on the plane through `target` facing the camera.
    void zoom(double factor, double ndcX, double ndcY, bool towardCursor);
    /// Zoom by `factor` as a homothety around the world point `p` (p stays at the same screen position).
    void zoomAbout(double factor, const Vec3& p);
    /// Frames `bounds` entirely in view (tight fit), keeping the current view direction. No-op for an empty box.
    void fit(const Box3& bounds, double margin = 1.05);
    /// Re-targets to `p` without changing the orientation or eye position (pivot = point on the view axis).
    void setTargetKeepEye(const Vec3& p);

    /// World -> view (row-major, column vectors).
    Transform viewMatrix() const;
    /// Column-major 4x4 (m[col*4+row]) OpenGL-convention clip matrix (z in [-1,1], y up) for the given aspect
    /// ratio (width/height); backend clip corrections (Vulkan y down, z in [0,1]) are applied by the renderer.
    std::array<double, 16> projectionMatrix(double aspect) const;
    /// Near/far planes used by `projectionMatrix`.
    std::pair<double, double> clipPlanes() const;
    /// Projects a world point to NDC (x,y in [-1,1], y up; z in [-1,1]); nullopt if behind the eye (perspective).
    std::optional<Vec3> project(const Vec3& world) const;
    /// World-space ray through NDC (x,y).
    Ray rayThrough(double ndcX, double ndcY) const;

    Vec3 eye() const;
    Vec3 forward() const;  ///< Unit vector eye -> target.
    Vec3 right() const;
    Vec3 upVector() const; ///< Screen-up in world space.
    Vec3 worldUp() const;  ///< The world up axis as a unit vector.
    /// Visible half-height at the target depth (mm).
    double halfHeightAtTarget() const;

    /// Pose of this camera looking from direction `eyeDir` (target -> eye, need not be unit) with the same
    /// target/distance. Near-vertical directions keep the current yaw (no heading jump on Top/Bottom).
    CameraState poseForEyeDirection(const Vec3& eyeDir) const;
    /// Unit eye direction for the default isometric "home" view (front-right-top).
    Vec3 homeEyeDirection() const;
    /// Front-face eye direction for this up axis.
    Vec3 frontEyeDirection() const;

    CameraState state() const noexcept { return {target_, distance_, yaw_, pitch_}; }
    void setState(const CameraState& s);

    Vec3 target() const noexcept { return target_; }
    double distance() const noexcept { return distance_; }
    double yaw() const noexcept { return yaw_; }
    double pitch() const noexcept { return pitch_; }
    double fovY() const noexcept { return fovY_; }

    static constexpr double kMaxPitch = 1.5707963267948966 - 1e-3;

private:
    struct Frame { Vec3 right, up, back; };
    Frame frame() const;
    void axes(Vec3& a, Vec3& b, Vec3& u) const;
    static Frame frameFor(UpAxis up, double yaw, double pitch);

    Projection projection_ = Projection::Perspective;
    UpAxis up_ = UpAxis::Y;
    Vec3 target_{};
    double distance_ = 100.0;   ///< mm
    double yaw_ = 0.6, pitch_ = 0.5;
    double fovY_ = 0.7853981633974483; ///< 45 deg
    int width_ = 1, height_ = 1;
    Box3 sceneBounds_{};
};

} // namespace iw::render
