#version 330 core

out vec4 FragColor;
uniform vec3 objectColor;
uniform sampler2D texture_diffuse1;
in vec2 TexCoords;
uniform bool use_textures;

#include "compute_shading.frag"

void main()
{
	if (use_textures == false){
		FragColor = vec4(computeBasicShading() * objectColor, 1.0);
	}else{
		FragColor = vec4(computeBasicShading(), 1.0) * texture(texture_diffuse1, TexCoords);
	}
} 
