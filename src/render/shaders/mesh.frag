// SPDX-License-Identifier: GPL-3.0-or-later
#version 440
layout(std140, binding = 0) uniform Buf {
    mat4 mvp;
    mat4 mv;
    vec4 ids;   // x = body id + 1, y = hover face, z = hover edge, w = selected face (-1 = none)
    vec4 ids2;  // x = selected edge (-1 = none), y = +1/-1 clip-space y direction of the backend, z = body selected
    vec4 color; // rgb = body surface colour
} ubuf;
layout(location = 0) in vec3 vNormal;
layout(location = 1) flat in float vFace;
layout(location = 2) in float vViewZ;
layout(location = 0) out vec4 fragColor;

void main() {
    vec3 n = normalize(vNormal);
    if (n.z < 0.0) n = -n;   // two-sided shading
    float key = max(dot(n, normalize(vec3(-0.35, 0.55, 0.75))), 0.0);
    float fill = max(dot(n, normalize(vec3(0.65, -0.25, 0.35))), 0.0);
    float head = n.z;
    float light = 0.50 + 0.30 * key + 0.10 * fill + 0.10 * head;
    vec3 base = ubuf.color.rgb;
    vec3 col = base * light + vec3(0.10) * pow(max(head, 0.0), 24.0);
    if (ubuf.ids.y >= 0.0 && abs(vFace - ubuf.ids.y) < 0.5) col = mix(col, vec3(0.35, 0.65, 1.0), 0.55);
    if (ubuf.ids.w >= 0.0 && abs(vFace - ubuf.ids.w) < 0.5) col = mix(col, vec3(1.0, 0.60, 0.12), 0.65);
    if (ubuf.ids2.z > 0.5) col = mix(col, vec3(1.0, 0.60, 0.12), 0.35);
    fragColor = vec4(col, 1.0);
}
