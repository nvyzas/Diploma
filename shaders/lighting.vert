#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec2 texCoord;
layout (location = 2) in vec3 normal;


uniform	mat4 modelView;
uniform mat4 projection;


// Light and material properties
uniform vec3 lightPosition  = vec3(5.0, 5.0, 5.0);				//vec3(5.0, 5.0, 5.0); 
uniform vec3 diffuseAlbedo  = vec3(1.0, 0.0, 0.0); 				//vec3(0.5, 0.2, 0.7); 		
uniform vec3 specularAlbedo = vec3(0.7, 0.7, 0.7);				//vec3(0.7);				  
uniform float specularPower = 128.0;							//128.0;
uniform vec3 ambient		= vec3(0.05, 0.05, 0.05);			//vec3(0.05, 0.05, 0.05);				

out VS_OUT
{
	vec3 color;
} vs_out;

void main()
{
	// Calculate position in view space
	vec4 P = modelView * vec4(position, 1.0);
	
	// Calculate normal in view space
	vec3 N = mat3(modelView) * normal;
	
	// Calculate light vector in view space
	vec3 L = lightPosition - P.xyz;
	
	// Calculate view vector (simply the negative of the view-space position)
	vec3 V = -P.xyz;
	
	// Normalize all three vectors
	N = normalize(N);
	L = normalize(L);
	V = normalize(V);
	
	// Calculate R by reflecting -L around the plane defined by N
	vec3 R = reflect(-L, N);
	
	// Calculate the diffuse and specular color contributions
	vec3 diffuse = max(dot(N, L), 0.0) * diffuseAlbedo;
	vec3 specular = pow(max(dot(R, V), 0.0), specularPower) * specularAlbedo;
	
	// Send the color output to the fragment shader
	vs_out.color = ambient + diffuse + specular;
	
	// Calculate the clip-space position of each vertex
	gl_Position = projection * P;
}