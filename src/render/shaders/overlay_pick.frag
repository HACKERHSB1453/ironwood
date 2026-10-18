// SPDX-License-Identifier: GPL-3.0-or-later
#version 440
layout(std140, binding = 0) uniform Buf {
    mat4 mvp;
    mat4 mv;
    vec4 ids;   // x = body id + 1 (overlays: overlay id + 1), y = hover face, z = hover edge, w = selected face (-1 = none)
    vec4 ids2;  // x = selected edge (-1 = none), y = +1/-1 clip-space y direction of the backend (background)
    vec4 color; // rgb = body surface colour (overlays / caps: rgba)
    vec4 clip;  // section plane in object space: xyz = normal, w = offset; kept side is dot >= 0
    vec4 sec;   // x = 1 when the section clip is active
} ubuf;
layout(location = 0) in float vViewZ;
layout(location = 0) out vec4 fragColor;

void main() {
    // r = 0, g = overlay id + 1, b = 2 (overlay marker, see decodePickWindow), a = view depth
    fragColor = vec4(0.0, ubuf.ids.x, 2.0, vViewZ);
}
