#version 120

varying vec2 texpos;
uniform sampler2D tex;
varying vec4 color;

void main(void) 
{
    gl_FragColor = vec4(1.0f, 1.0f, 1.0f, texture2D(tex, texpos).a) * color;
}
