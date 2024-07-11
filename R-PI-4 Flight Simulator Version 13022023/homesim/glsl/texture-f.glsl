#version 120

varying vec2 texture_coord_from_vshader;

uniform sampler2D texture_sampler;

void main() 
{
	gl_FragColor = texture2D(texture_sampler, texture_coord_from_vshader);
}
