#version 120

#define width  1920.0f
#define height 1080.0f

attribute vec2 pos;
attribute vec2 texture_coord;

varying vec2 texture_coord_from_vshader;

vec4 position = vec4(pos, 0.0f, 1.0f);

mat4 ProjectMatrix = mat4(2.0f/width,        0.0f,    0.0f,   0.0f, // column-row ordering
                                0.0f, 2.0f/height,    0.0f,   0.0f,
                                0.0f,        0.0f,   -1.0f,   0.0f,
                               -1.0f,       -1.0f,    0.0f,   1.0f);
  
uniform mat4 TransformationMatrix;  // translation and rotation

void main() 
{
	gl_Position = ProjectMatrix * TransformationMatrix * position;
	texture_coord_from_vshader = texture_coord;
}
