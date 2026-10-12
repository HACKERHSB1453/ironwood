// SPDX-License-Identifier: GPL-3.0-or-later
#version 440
layout(std140, binding = 0) uniform Buf {
    mat4 mvp;
    mat4 mv;
    vec4 ids;   // x = body id + 1, y = hover face, z = hover edge, w = selected face (-1 = none)
    vec4 ids2;  // x = selected edge (-1 = none), y = +1/-1 clip-space y direction of the backend (background)
    vec4 color; // rgb = body surface colour (mirrors UboData; used by mesh.frag only)
} ubuf;
layout(location = 0) out float vT;   // 0 = top of the screen, 1 = bottom

void main() {
    vec2 p = vec2(float((gl_VertexIndex << 1) & 2), float(gl_VertexIndex & 2));
    gl_Position = vec4(p * 2.0 - 1.0, 1.0, 1.0);
    // clip y = +1 is the top on y-up backends (OpenGL) and the bottom on y-down backends (Vulkan)
    vT = 0.5 - 0.5 * gl_Position.y * ubuf.ids2.y;
}
