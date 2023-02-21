#version 330 core

out vec4 FragColor;
uniform vec3 objectColor;
uniform sampler2D texture_diffuse1;
in vec2 TexCoords;
uniform bool use_textures;	
uniform vec2 zRange;
uniform vec4 isoLines;
in float zVal;

#include "compute_shading.frag"

vec3 colourBasedOnZVal(float val) {
	float saturation = 0.5;

	float levels[6];
	for (int i=0;i<4;i++)
		levels[1 + i] = isoLines[i];
	levels[0] = zRange.x;
	levels[5] = zRange.y;

	float minF = zRange.x;
	float maxF = zRange.y;
	float isolevelNr = 0;
	for (int i=1;i<6;i++){
		if (val > levels[i-1] && val <= levels[i]){
			minF = levels[i-1];
			maxF = levels[i];
			isolevelNr = i - 1;
		}
	}

//	float intensity = (val - minF) / (maxF - minF);
	float intensity = (sqrt(val) - sqrt(minF)) / (sqrt(maxF) - sqrt(minF));

	if (isolevelNr == 0)
		intensity = pow(intensity, 2);

	intensity = intensity / 5 + isolevelNr / 5;

	intensity = pow(intensity, 0.25);

	vec3 color;
	if (intensity < 0.5){
		color.x = 1 * saturation + (1 - saturation);
		color.y = saturation * saturation;
		color.z = 2 * intensity * saturation + (1 - saturation);
	}else{
		color.x = 2 * (1-intensity) * saturation + (1 - saturation);
		color.y = saturation * saturation;
		color.z = 1 * saturation + (1 - saturation);
	}

	float isoValIntensity = 0;
	for (int i=0;i<4;i++){
		//isoline line thickness
		float dl = 0.02;
		if (i == 0)
			dl = 0.01;
		float isoLineVal = isoLines[i];

		float dist = abs(isoLineVal - val);
		if (dist < dl)
			isoValIntensity = (dl - dist) / dl;
	}

	return color * (1 - isoValIntensity)* (1 - isoValIntensity);
}

void main()
{
	FragColor = vec4(colourBasedOnZVal(zVal), 1.0);
} 
