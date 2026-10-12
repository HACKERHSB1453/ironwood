// SPDX-License-Identifier: GPL-3.0-or-later
#include "render/camera.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace iw::render {

namespace {
constexpr double kPi = std::numbers::pi;
constexpr double kOrbitRadPerScreenHeight = kPi; // dragging the full viewport height turns the model 180 deg
constexpr double kMinDistance = 1e-6;

Vec3 normalized(const Vec3& v) {
    const double l = length(v);
    return l > 0 ? v * (1.0 / l) : Vec3{};
}
double wrapPi(double a) {
    a = std::fmod(a + kPi, 2 * kPi);
    if (a < 0) a += 2 * kPi;
    return a - kPi;
}
} // namespace

CameraState lerp(const CameraState& a, const CameraState& b, double t) {
    CameraState r;
    r.target = a.target + (b.target - a.target) * t;
    r.distance = a.distance > 0 && b.distance > 0 ? a.distance * std::pow(b.distance / a.distance, t)
                                                   : a.distance + (b.distance - a.distance) * t;
    r.yaw = a.yaw + wrapPi(b.yaw - a.yaw) * t;
    r.pitch = a.pitch + (b.pitch - a.pitch) * t;
    return r;
}

void Camera::axes(Vec3& a, Vec3& b, Vec3& u) const {
    if (up_ == UpAxis::Z) { a = {1, 0, 0}; b = {0, 1, 0}; u = {0, 0, 1}; }
    else                  { a = {0, 0, 1}; b = {1, 0, 0}; u = {0, 1, 0}; }
}

Camera::Frame Camera::frameFor(UpAxis up, double yaw, double pitch) {
    Vec3 a, b, u;
    if (up == UpAxis::Z) { a = {1, 0, 0}; b = {0, 1, 0}; u = {0, 0, 1}; }
    else                 { a = {0, 0, 1}; b = {1, 0, 0}; u = {0, 1, 0}; }
    const double cp = std::cos(pitch), sp = std::sin(pitch), cy = std::cos(yaw), sy = std::sin(yaw);
    Frame f;
    f.back = a * (cp * cy) + b * (cp * sy) + u * sp;         // target -> eye
    // right = forward x up = (-back) x u, normalised; horizontal by construction
    f.right = normalized(cross(f.back * -1.0, u));
    f.up = cross(f.right, f.back * -1.0);
    return f;
}

Camera::Frame Camera::frame() const { return frameFor(up_, yaw_, pitch_); }

Vec3 Camera::worldUp() const { return up_ == UpAxis::Z ? Vec3{0, 0, 1} : Vec3{0, 1, 0}; }
Vec3 Camera::eye() const { return target_ + frame().back * distance_; }
Vec3 Camera::forward() const { return frame().back * -1.0; }
Vec3 Camera::right() const { return frame().right; }
Vec3 Camera::upVector() const { return frame().up; }
double Camera::halfHeightAtTarget() const { return distance_ * std::tan(fovY_ / 2); }

Vec3 Camera::frontEyeDirection() const { return up_ == UpAxis::Z ? Vec3{0, -1, 0} : Vec3{0, 0, 1}; }
Vec3 Camera::homeEyeDirection() const {
    const Vec3 front = frontEyeDirection();
    const Vec3 side{1, 0, 0};
    return normalized(front + side + worldUp());
}

void Camera::setState(const CameraState& s) {
    target_ = s.target;
    distance_ = std::max(s.distance, kMinDistance);
    yaw_ = wrapPi(s.yaw);
    pitch_ = std::clamp(s.pitch, -kMaxPitch, kMaxPitch);
}

CameraState Camera::poseForEyeDirection(const Vec3& eyeDir) const {
    Vec3 a, b, u;
    axes(a, b, u);
    const Vec3 d = normalized(eyeDir);
    CameraState s = state();
    if (length(d) == 0) return s;
    const double ca = dot(d, a), cb = dot(d, b), cu = dot(d, u);
    s.pitch = std::clamp(std::asin(std::clamp(cu, -1.0, 1.0)), -kMaxPitch, kMaxPitch);
    if (std::hypot(ca, cb) > 1e-6) s.yaw = std::atan2(cb, ca);
    return s;
}

void Camera::orbit(double dx, double dy) { orbitAbout(dx, dy, target_); }

void Camera::orbitAbout(double dx, double dy, const Vec3& pivot) {
    const double k = kOrbitRadPerScreenHeight / height_;
    const Frame f0 = frame();
    yaw_ = wrapPi(yaw_ - dx * k);
    pitch_ = std::clamp(pitch_ + dy * k, -kMaxPitch, kMaxPitch);
    const Frame f1 = frame();
    // rotation taking basis f0 -> f1, applied to (target - pivot)
    const Vec3 v = target_ - pivot;
    const double cr = dot(v, f0.right), cu = dot(v, f0.up), cb = dot(v, f0.back);
    target_ = pivot + f1.right * cr + f1.up * cu + f1.back * cb;
}

void Camera::pan(double dx, double dy) {
    const Frame f = frame();
    const double perPixel = 2.0 * halfHeightAtTarget() / height_;
    target_ = target_ - f.right * (dx * perPixel) + f.up * (dy * perPixel);
}

void Camera::zoomAbout(double factor, const Vec3& p) {
    if (!(factor > 0)) return;
    const double s = 1.0 / factor;
    target_ = p + (target_ - p) * s;
    distance_ = std::max(distance_ * s, kMinDistance);
}

void Camera::zoom(double factor, double ndcX, double ndcY, bool towardCursor) {
    if (!towardCursor) { zoomAbout(factor, target_); return; }
    const Frame f = frame();
    const double hh = halfHeightAtTarget();
    // point on the target plane under the cursor
    const Vec3 p = target_ + f.right * (ndcX * hh * aspect()) + f.up * (ndcY * hh);
    zoomAbout(factor, p);
}

void Camera::setTargetKeepEye(const Vec3& p) {
    const Frame f = frame();
    const Vec3 e = eye();
    const double depth = dot(p - e, f.back * -1.0);
    if (depth <= kMinDistance) return;
    target_ = e + f.back * (-depth);
    distance_ = depth;
}

void Camera::fit(const Box3& b, double margin) {
    if (b.isEmpty()) return;
    const Frame f = frame();
    const Vec3 c{(b.min.x + b.max.x) / 2, (b.min.y + b.max.y) / 2, (b.min.z + b.max.z) / 2};
    const Vec3 fwd = f.back * -1.0;
    const double tanY = std::tan(fovY_ / 2) / margin, tanX = tanY * aspect();
    double d = kMinDistance;
    double radius = 1e-6;
    for (int i = 0; i < 8; ++i) {
        const Vec3 corner{(i & 1) ? b.max.x : b.min.x, (i & 2) ? b.max.y : b.min.y, (i & 4) ? b.max.z : b.min.z};
        const Vec3 r = corner - c;
        const double x = std::abs(dot(r, f.right)), y = std::abs(dot(r, f.up)), z = dot(r, fwd);
        radius = std::max(radius, length(r));
        if (projection_ == Projection::Orthographic) d = std::max({d, x / tanX, y / tanY});
        else d = std::max({d, x / tanX - z, y / tanY - z});
    }
    if (projection_ == Projection::Perspective) d = std::max(d, radius * 1.001); // keep the eye outside the sphere
    target_ = c;
    distance_ = d;
    sceneBounds_ = b;
}

Transform Camera::viewMatrix() const {
    const Frame f = frame();
    const Vec3 e = eye();
    Transform t;
    t.m = {f.right.x, f.right.y, f.right.z, -dot(f.right, e),
           f.up.x,    f.up.y,    f.up.z,    -dot(f.up, e),
           f.back.x,  f.back.y,  f.back.z,  -dot(f.back, e),
           0,         0,         0,         1};
    return t;
}

std::pair<double, double> Camera::clipPlanes() const {
    const Frame f = frame();
    const Vec3 e = eye();
    double lo, hi;
    if (sceneBounds_.isEmpty()) {
        lo = distance_ * 0.01;
        hi = distance_ * 100.0;
        if (projection_ == Projection::Orthographic) lo = -distance_ * 100.0;
    } else {
        const Vec3 c{(sceneBounds_.min.x + sceneBounds_.max.x) / 2, (sceneBounds_.min.y + sceneBounds_.max.y) / 2,
                     (sceneBounds_.min.z + sceneBounds_.max.z) / 2};
        const double r = std::max(0.5 * length(sceneBounds_.max - sceneBounds_.min), 1e-3) * 1.05;
        const double dc = dot(c - e, f.back * -1.0);
        // always include the target as well, so a pan/zoom away from the scene still shows something
        hi = std::max(dc + r, distance_ * 1.5);
        lo = dc - r;
        if (projection_ == Projection::Perspective) lo = std::max(lo, hi * 2e-4);
    }
    return {lo, hi};
}

std::array<double, 16> Camera::projectionMatrix(double aspect) const {
    const auto [n, fa] = clipPlanes();
    std::array<double, 16> m{};
    const double hh = (projection_ == Projection::Perspective) ? std::tan(fovY_ / 2) : halfHeightAtTarget();
    if (projection_ == Projection::Perspective) {
        const double t = 1.0 / hh;
        m[0] = t / aspect;
        m[5] = t;
        m[10] = -(fa + n) / (fa - n);
        m[11] = -1.0;
        m[14] = -2.0 * fa * n / (fa - n);
    } else {
        m[0] = 1.0 / (hh * aspect);
        m[5] = 1.0 / hh;
        m[10] = -2.0 / (fa - n);
        m[14] = -(fa + n) / (fa - n);
        m[15] = 1.0;
    }
    return m;
}

std::optional<Vec3> Camera::project(const Vec3& w) const {
    const Transform v = viewMatrix();
    const Vec3 p = v.apply(w);
    const auto m = projectionMatrix(aspect());
    const double x = m[0] * p.x, y = m[5] * p.y;
    const double z = m[10] * p.z + m[14];
    const double wc = m[11] * p.z + m[15];
    if (projection_ == Projection::Perspective && wc <= 0) return std::nullopt;
    return Vec3{x / wc, y / wc, z / wc};
}

Ray Camera::rayThrough(double ndcX, double ndcY) const {
    const Frame f = frame();
    const Vec3 e = eye();
    const Vec3 fwd = f.back * -1.0;
    const double hh = projection_ == Projection::Perspective ? std::tan(fovY_ / 2) : halfHeightAtTarget();
    const Vec3 lateral = f.right * (ndcX * hh * aspect()) + f.up * (ndcY * hh);
    if (projection_ == Projection::Perspective) return {e, normalized(fwd + lateral)};
    return {e + lateral, fwd};
}

} // namespace iw::render
