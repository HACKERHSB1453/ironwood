// SPDX-License-Identifier: GPL-3.0-or-later
#version 440
layout(std140, binding = 0) uniform Buf {
    mat4 mvp;
    mat4 mv;
    vec4 ids;   // x = body id + 1, y = hover face, z = hover edge, w = selected face (-1 = none)
    vec4 ids2;  // x = selected edge (-1 = none), y = +1/-1 clip-space y direction of the backend (background)
    vec4 color; // rgb = body surface colour (mirrors UboData; used by mesh.frag only)
} ubuf;
layout(location = 0) flat in float vEdge;
layout(location = 1) in float vViewZ;
layout(location = 0) out vec4 fragColor;

void main() {
    vec3 col = vec3(0.07, 0.08, 0.10);
    if (ubuf.ids.z >= 0.0 && abs(vEdge - ubuf.ids.z) < 0.5) col = vec3(0.05, 0.40, 1.0);
    if (ubuf.ids2.x >= 0.0 && abs(vEdge - ubuf.ids2.x) < 0.5) col = vec3(1.0, 0.45, 0.0);
    fragColor = vec4(col, 1.0);
}
