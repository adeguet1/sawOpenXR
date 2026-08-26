#version 450

// Stereo video-window fragment shader used by the sawOpenXR runtime.

layout(set = 0, binding = 0) uniform sampler2D test_image;
layout(location = 0) in vec2 in_uv;
layout(location = 0) out vec4 out_color;

void main() {
    out_color = texture(test_image, vec2(in_uv.x, 1.0 - in_uv.y));
}
