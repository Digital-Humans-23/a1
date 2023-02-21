#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D texture_diffuse1;
uniform float tile_size;

void main()
{
	vec2 c = TexCoords*tile_size;
	FragColor = texture(texture_diffuse1, c);
}
