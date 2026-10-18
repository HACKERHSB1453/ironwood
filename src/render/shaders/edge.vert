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
layout(location = 0) in vec3 position;
layout(location = 1) in float edgeId;
layout(location = 0) flat out float vEdge;
layout(location = 1) out float vViewZ;
layout(location = 2) out float vClip;

void main() {
    gl_Position = ubuf.mvp * vec4(position, 1.0);
    vEdge = edgeId;
    vViewZ = -(ubuf.mv * vec4(position, 1.0)).z;
    vClip = dot(ubuf.clip.xyz, position) + ubuf.clip.w;
}
