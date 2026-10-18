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
layout(location = 3) in float vClip;
layout(location = 0) out vec4 fragColor;

void main() {
    if (ubuf.sec.x > 0.5 && vClip < 0.0) discard;   // section: clipped half is gone (also for picking)
    fragColor = vec4(0.0);   // colour writes are masked off; the stencil Invert op does the work
}
