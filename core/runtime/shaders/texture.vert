#version 450

// Stereo video-window vertex shader used by the sawOpenXR runtime.

layout(push_constant) uniform WindowTransform {
    mat4 mvp;
} transform;

layout(location = 0) out vec2 out_uv;

void main() {
    const vec3 positions[6] = vec3[](
        vec3(-0.8, -0.45, 0.0), vec3( 0.8, -0.45, 0.0), vec3( 0.8,  0.45, 0.0),
        vec3(-0.8, -0.45, 0.0), vec3( 0.8,  0.45, 0.0), vec3(-0.8,  0.45, 0.0));
    const vec2 uvs[6] = vec2[](
        vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(1.0, 1.0),
        vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(0.0, 1.0));
    gl_Position = transform.mvp * vec4(positions[gl_VertexIndex], 1.0);
    out_uv = uvs[gl_VertexIndex];
}
