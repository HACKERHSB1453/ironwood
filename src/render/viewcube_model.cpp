// SPDX-License-Identifier: GPL-3.0-or-later
#include "render/viewcube_model.hpp"

#include <algorithm>
#include <cmath>

namespace iw::render {

namespace {
constexpr double kBorder = 0.6; // grid split: [-1,-kBorder] [-kBorder,kBorder] [kBorder,1]

std::vector<ViewCubeCell> buildCells() {
    std::vector<ViewCubeCell> out;
    const double cuts[4] = {-1.0, -kBorder, kBorder, 1.0};
    for (int axis = 0; axis < 3; ++axis) {
        for (int sign = -1; sign <= 1; sign += 2) {
            const int u = (axis + 1) % 3, v = (axis + 2) % 3;
            // (u,v,axis) is right-handed, so for sign>0 the CCW order (u,v) is outward-facing.
            for (int j = 0; j < 3; ++j) {
                for (int i = 0; i < 3; ++i) {
                    auto pt = [&](double a, double b) {
                        double c[3];
                        c[axis] = sign;
                        c[u] = a;
                        c[v] = b;
                        return Vec3{c[0], c[1], c[2]};
                    };
                    ViewCubeCell cell;
                    if (sign > 0) cell.corners = {pt(cuts[i], cuts[j]), pt(cuts[i + 1], cuts[j]),
                                                  pt(cuts[i + 1], cuts[j + 1]), pt(cuts[i], cuts[j + 1])};
                    else cell.corners = {pt(cuts[i], cuts[j]), pt(cuts[i], cuts[j + 1]),
                                         pt(cuts[i + 1], cuts[j + 1]), pt(cuts[i + 1], cuts[j])};
                    double d[3];
                    d[axis] = sign;
                    d[u] = i - 1;
                    d[v] = j - 1;
                    cell.dir = {d[0], d[1], d[2]};
                    double n[3] = {0, 0, 0};
                    n[axis] = sign;
                    cell.faceNormal = {n[0], n[1], n[2]};
                    const int nonZero = (d[0] != 0) + (d[1] != 0) + (d[2] != 0);
                    cell.kind = nonZero == 1 ? ViewCubeCell::Kind::Face
                              : nonZero == 2 ? ViewCubeCell::Kind::Edge : ViewCubeCell::Kind::Corner;
                    out.push_back(cell);
                }
            }
        }
    }
    return out;
}

bool inQuad(const std::array<std::array<double, 2>, 4>& q, double x, double y) {
    bool pos = false, neg = false;
    for (int i = 0; i < 4; ++i) {
        const auto& a = q[i];
        const auto& b = q[(i + 1) % 4];
        const double c = (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]);
        if (c > 0) pos = true;
        if (c < 0) neg = true;
    }
    return !(pos && neg);
}
} // namespace

const std::vector<ViewCubeCell>& viewCubeCells() {
    static const std::vector<ViewCubeCell> cells = buildCells();
    return cells;
}

std::vector<ProjectedCell> projectViewCube(const Transform& view, double cx, double cy, double halfSizePx) {
    const double scale = halfSizePx * 0.58;
    auto rot = [&](const Vec3& p) { // rotation part only
        const auto& m = view.m;
        return Vec3{m[0] * p.x + m[1] * p.y + m[2] * p.z, m[4] * p.x + m[5] * p.y + m[6] * p.z,
                    m[8] * p.x + m[9] * p.y + m[10] * p.z};
    };
    std::vector<ProjectedCell> out;
    for (const auto& cell : viewCubeCells()) {
        const Vec3 n = rot(cell.faceNormal);
        if (n.z <= 1e-9) continue; // back-facing (view-space +z points at the viewer)
        ProjectedCell pc;
        pc.cell = &cell;
        for (int i = 0; i < 4; ++i) {
            const Vec3 p = rot(cell.corners[static_cast<std::size_t>(i)]);
            pc.pts[static_cast<std::size_t>(i)] = {cx + p.x * scale, cy - p.y * scale};
            pc.depth += p.z * 0.25;
        }
        out.push_back(pc);
    }
    std::stable_sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.depth < b.depth; });
    return out;
}

int hitTestViewCube(const std::vector<ProjectedCell>& cells, double x, double y) {
    for (int i = static_cast<int>(cells.size()) - 1; i >= 0; --i)
        if (inQuad(cells[static_cast<std::size_t>(i)].pts, x, y)) return i;
    return -1;
}

std::array<const char*, 6> viewCubeFaceLabels(UpAxis up) {
    // +X,-X,+Y,-Y,+Z,-Z
    if (up == UpAxis::Y) return {"RIGHT", "LEFT", "TOP", "BOTTOM", "FRONT", "BACK"};
    return {"RIGHT", "LEFT", "BACK", "FRONT", "TOP", "BOTTOM"};
}

} // namespace iw::render
