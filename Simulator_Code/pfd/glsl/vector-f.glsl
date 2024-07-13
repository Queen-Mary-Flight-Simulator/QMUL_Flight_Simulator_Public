#version 120

varying vec3 Color;		    // Color value from the vertex shader (smoothed)

void main()
{
    gl_FragColor = vec4(Color, 1.0f);   // Add alpha value of 1.0
}
