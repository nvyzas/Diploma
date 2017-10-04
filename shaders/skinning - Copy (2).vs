#version 330 compatibility                                                                    
                                                                                    
layout (location = 0) in vec3 Position;                                             
layout (location = 1) in vec2 TexCoord;                                             
layout (location = 2) in vec3 Normal;                                               
layout (location = 3) in ivec4 BoneIDs;
layout (location = 4) in vec4 Weights;

out vec2 TexCoord0;
out vec3 Normal0;                                                                   
out vec3 WorldPos0;
//out bool visibleFragment;                                                                

const int MAX_BONES = 100;
const mat4 ZeroMat4 = mat4(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);

uniform mat4 gWVP;
uniform mat4 gWorld;
uniform mat4 gBones[MAX_BONES];
uniform bool skinningOn;
uniform bool visible[MAX_BONES];

void main()
{
	mat4 BoneTransform;
	mat4 FinalTransforms[4];
	// find max weight index
	float max = 0;
	int i,j;
	for (i=0; i<4; i++){
		if (Weights[i] >= max){
			max = Weights[i];
			j = i;
		}
	}
	// visible?
	if (visible[BoneIDs[j]]) visibleFragment = true;
	else visibleFragment = false;

	// skinning?
	if (!skinningOn){
		BoneTransform 	   =  FinalTransforms[j];
	}else{		
		BoneTransform 	   =  FinalTransforms[0] * Weights[0];
		BoneTransform     +=  FinalTransforms[1] * Weights[1];
		BoneTransform     +=  FinalTransforms[2] * Weights[2];
		BoneTransform     +=  FinalTransforms[3] * Weights[3];
	}

    vec4 PosL    = BoneTransform * vec4(Position, 1.0);
    gl_Position  = gWVP * PosL;
    TexCoord0    = TexCoord;
    vec4 NormalL = BoneTransform * vec4(Normal, 0.0);
    Normal0      = (gWorld * NormalL).xyz;
    WorldPos0    = (gWorld * PosL).xyz;                                
}
