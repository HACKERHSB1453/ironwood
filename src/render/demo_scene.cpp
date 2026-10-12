// SPDX-License-Identifier: GPL-3.0-or-later
#include "render/demo_scene.hpp"

#include <array>

namespace iw::render {

std::shared_ptr<const tess::MeshData> makeBoxMesh(const Vec3& lo, const Vec3& hi) {
    auto m = std::make_shared<tess::MeshData>();
    auto corner = [&](int i) {
        return Vec3{(i & 1) ? hi.x : lo.x, (i & 2) ? hi.y : lo.y, (i & 4) ? hi.z : lo.z};
    };
    // face = axis (0..2) and sign; the 4 corners are listed CCW seen from outside.
    struct Face { int axis; int sign; std::array<int, 4> c; };
    const Face faces[6] = {
        {0, -1, {0, 4, 6, 2}}, {0, +1, {1, 3, 7, 5}},   // -X, +X
        {1, -1, {0, 1, 5, 4}}, {1, +1, {2, 6, 7, 3}},   // -Y, +Y
        {2, -1, {0, 2, 3, 1}}, {2, +1, {4, 5, 7, 6}},   // -Z, +Z
    };
    for (const auto& f : faces) {
        const auto base = static_cast<std::uint32_t>(m->positions.size() / 3);
        float n[3] = {0, 0, 0};
        n[f.axis] = static_cast<float>(f.sign);
        for (int k : f.c) {
            const Vec3 p = corner(k);
            m->positions.insert(m->positions.end(), {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
            m->normals.insert(m->normals.end(), {n[0], n[1], n[2]});
        }
        m->faceRanges.push_back({static_cast<std::uint32_t>(m->indices.size()), 6});
        m->indices.insert(m->indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    // 12 edges: pairs of corners differing in one bit
    for (int a = 0; a < 8; ++a) {
        for (int bit = 1; bit <= 4; bit <<= 1) {
            if (a & bit) continue;
            const Vec3 p = corner(a), q = corner(a | bit);
            m->edgePolylines.push_back({{static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z),
                                         static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z)}});
        }
    }
    m->bounds.extend(lo);
    m->bounds.extend(hi);
    return m;
}

} // namespace iw::render
