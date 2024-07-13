#version 120

#define width  1024.0f
#define height 768.0f

attribute vec2 pos;
attribute vec2 texcoord;
attribute vec3 stringcolor;

varying vec2 texpos;
varying vec4 color;

vec4 position = vec4(pos, 0.0f, 1.0f);

mat4 ProjectMatrix = mat4(2.0f/width,        0.0f,    0.0f,   0.0f, // column-row ordering
                                0.0f, 2.0f/height,    0.0f,   0.0f,
                                0.0f,        0.0f,   -1.0f,   0.0f,
                               -1.0f,       -1.0f,    0.0f,   1.0f);
 
uniform mat4 TransformationMatrix;  // translation and rotation

void main(void) 
{
	gl_Position = ProjectMatrix * TransformationMatrix * position;
    texpos = texcoord;
    color = vec4 (stringcolor, 1.0f);
}
