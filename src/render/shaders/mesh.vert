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
layout(location = 1) in vec3 normal;
layout(location = 2) in float faceId;
layout(location = 0) out vec3 vNormal;
layout(location = 1) flat out float vFace;
layout(location = 2) out float vViewZ;
layout(location = 3) out float vClip;

void main() {
    gl_Position = ubuf.mvp * vec4(position, 1.0);
    vec4 vp = ubuf.mv * vec4(position, 1.0);
    vNormal = mat3(ubuf.mv) * normal;
    vFace = faceId;
    vViewZ = -vp.z;
    vClip = dot(ubuf.clip.xyz, position) + ubuf.clip.w;
}
