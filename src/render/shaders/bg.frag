// SPDX-License-Identifier: GPL-3.0-or-later
#version 440
layout(std140, binding = 0) uniform Buf {
    mat4 mvp;
    mat4 mv;
    vec4 ids;   // x = body id + 1, y = hover face, z = hover edge, w = selected face (-1 = none)
    vec4 ids2;  // x = selected edge (-1 = none), y = +1/-1 clip-space y direction of the backend (background)
    vec4 color; // rgb = body surface colour (mirrors UboData; used by mesh.frag only)
} ubuf;
layout(location = 0) in float vT;
layout(location = 0) out vec4 fragColor;

void main() {
    // light-grey vertical gradient, slightly bluish at the top, near white at the bottom
    vec3 top = vec3(0.76, 0.79, 0.84);
    vec3 bottom = vec3(0.95, 0.95, 0.96);
    fragColor = vec4(mix(top, bottom, clamp(vT, 0.0, 1.0)), 1.0);
}
