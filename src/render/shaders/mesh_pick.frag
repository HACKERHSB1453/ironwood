// SPDX-License-Identifier: GPL-3.0-or-later
#version 440
layout(std140, binding = 0) uniform Buf {
    mat4 mvp;
    mat4 mv;
    vec4 ids;   // x = body id + 1, y = hover face, z = hover edge, w = selected face (-1 = none)
    vec4 ids2;  // x = selected edge (-1 = none), y = +1/-1 clip-space y direction of the backend (background)
    vec4 color; // rgb = body surface colour (mirrors UboData; used by mesh.frag only)
} ubuf;
layout(location = 0) in vec3 vNormal;
layout(location = 1) flat in float vFace;
layout(location = 2) in float vViewZ;
layout(location = 0) out vec4 fragColor;

void main() {
    // r = body id + 1, g = face id + 1, b = 0 (face), a = view depth along the camera forward axis
    fragColor = vec4(ubuf.ids.x, vFace + 1.0, 0.0, vViewZ);
}
