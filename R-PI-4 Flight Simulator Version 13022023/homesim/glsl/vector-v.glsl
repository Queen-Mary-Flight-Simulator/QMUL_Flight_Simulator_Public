#version 120

#define width  1920.0f
#define height 1080.0f

attribute vec2 VertexPosition;   // Position in attribute location 0
attribute vec3 VertexColor;      // Color in attribute location 1

varying vec3 Color;	             // Output a color to the fragment shader

mat4 ProjectMatrix = mat4(2.0f/width,        0.0f,    0.0f,   0.0f, // column-row ordering
                                0.0f, 2.0f/height,    0.0f,   0.0f,
                                0.0f,        0.0f,   -1.0f,   0.0f,
                               -1.0f,       -1.0f,    0.0f,   1.0f);
 
uniform mat4 TransformationMatrix;  // translation and rotation

void main()
{
   gl_Position = ProjectMatrix * TransformationMatrix * vec4(VertexPosition, 0.0, 1.0);
   Color = VertexColor;
}
