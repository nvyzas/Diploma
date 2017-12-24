#ifndef SKINNED_MESH_H
#define	SKINNED_MESH_H

// Project
#include "ksensor.h"
#include "util.h"
#include "math_3d.h"

// Assimp
#include <assimp\Importer.hpp>      // C++ importer interface
#include <assimp\scene.h>			 // Output data structure
#include <assimp\postprocess.h>     // Post processing flags

// Qt
#include <QtCore\QVector>

// Standard C/C++
#include <map>
#include <vector>
#include <bitset>

#define ASSIMP_LOAD_FLAGS aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices | aiProcess_FlipWindingOrder | aiProcess_LimitBoneWeights 
#define NUM_PARAMETERS 10

#define NUM_BONES_PER_VERTEX 4
struct VertexBoneData
{
	uint IDs[NUM_BONES_PER_VERTEX];
	float Weights[NUM_BONES_PER_VERTEX];
	float OldWeights[NUM_BONES_PER_VERTEX];

	VertexBoneData()
	{
		Reset();
	};

	void Reset()
	{
		ZERO_MEM(IDs);
		ZERO_MEM(Weights);
		ZERO_MEM(OldWeights); // Not used
	}

	void AddBoneData(uint BoneID, float Weight);
	void AdjustBoneData(); // Not used
	void RestoreBoneData(); // Not used
};
#define INVALID_MATERIAL 0xFFFFFFFF
struct MeshEntry {
	MeshEntry()
	{
		NumIndices = 0;
		BaseVertex = 0;
		BaseIndex = 0;
		MaterialIndex = INVALID_MATERIAL;
	}

	uint NumIndices;
	uint BaseVertex;
	uint BaseIndex;
	uint MaterialIndex;
};
struct BoneInfo
{
	Matrix4f local;
	Matrix4f corrected;
	Matrix4f offset;
	Matrix4f global;
	Matrix4f combined;

	bool visible;
	float xRot, yRot, zRot;
	
	BoneInfo()
	{
		offset.SetZero();
		combined.SetZero();
		local.SetZero();
		xRot = yRot = zRot = 0.f;
		visible = true;
	}
};

class SkinnedMesh: protected QOpenGLFunctions_3_3_Core
{
public:
	SkinnedMesh();
	~SkinnedMesh();	
	bool LoadMesh(const string& Filename);
	void BoneTransform(float TimeInSeconds, vector<Matrix4f>& Transforms);
	void GetBoneTransforms(vector<Matrix4f>& Transforms);
	void traverseNodeHierarchy(const aiNode* pNode, const Matrix4f& P);
	void initCorrectedMatrices();
	void PrintInfo() const;
	void PrintNodeHierarchy(const aiNode* pNode) const;
	void PrintNodeMatching(const aiNode* pNode) const;
	void PrintParameters() const;
	void PrintSceneInfo() const;

	
	bool m_SuccessfullyLoaded;

	void initBoneMapping();
	float boneRotationX(const QString &boneName) const;
	float boneRotationY(const QString &boneName) const;
	float boneRotationZ(const QString &boneName) const;
	void setBoneRotationX(const QString &boneName, float value);
	void setBoneRotationY(const QString &boneName, float value);
	void setBoneRotationZ(const QString &boneName, float value);
	

	uint numBones() const;
	const map<string, uint>& Bones() const;
	uint findBoneId(const QString &boneName) const;
	bool boneVisibility(uint boneIndex) const;
	bool boneVisibility(const QString &boneName) const;
	void setBoneVisibility(uint boneIndex, bool state);
	void setBoneVisibility(const QString &boneName, bool state);
	QString boneTransformInfo(const QString& boneName) const;
	void flipParameter(uint i);

	// Get vertex attribute functions
	vector<MeshEntry>& entries();
	vector<Vector3f>& positions();
	vector<Vector3f>& normals();
	QVector<QVector2D>& texCoords();
	vector<VertexBoneData>& vertexBoneData();
	vector<uint>& indices();	
	vector<QImage>& images();

	void loadAxesToGPU();
	void drawBoneAxis();
	bool initOGL();
	const Matrix4f& boneGlobal(uint boneIndex) const;

	QQuaternion worldRotation();
	QVector3D worldPosition();
private:
	void Clear();
	void InitMesh(uint MeshIndex, const aiMesh* paiMesh);
	void LoadBones(uint MeshIndex, const aiMesh* paiMesh, vector<VertexBoneData>& Bones);
	bool initImages(const aiScene* pScene, const string& Filename);
	void CalcInterpolatedScaling(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);
	void CalcInterpolatedRotation(aiQuaternion& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);
	void CalcInterpolatedPosition(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);
	uint FindScaling(float AnimationTime, const aiNodeAnim* pNodeAnim);
	uint FindRotation(float AnimationTime, const aiNodeAnim* pNodeAnim);
	uint FindPosition(float AnimationTime, const aiNodeAnim* pNodeAnim);
	const aiNodeAnim* FindNodeAnim(const aiAnimation* pAnimation, const string NodeName);
	void ReadNodeHierarchy(float AnimationTime, const aiNode* pNode, const Matrix4f& ParentTransform);
	bool InitFromScene(const aiScene* pScene, const string& Filename);

	// Data loaded in CPU by LoadMesh		
	vector<MeshEntry> m_entries;
	// Vertex Attributes
	vector<Vector3f> m_positions;
	vector<Vector3f> m_normals;
	QVector<QVector2D> m_texCoords;
	vector<VertexBoneData> m_vertexBoneData;
	vector<uint> m_indices;
	// Bones
	vector<BoneInfo> m_boneInfo;
	// Textures
	vector<QImage> m_images;

	map<string, uint> m_boneMap; // maps a mesh's bone name to its index (key = bone name, value = index)
	map<string, uint> m_kboneMap; // maps a mesh's bone name to its kinect JointType index
	const KJoint *m_pKBones = NULL;

	// Control pose
	vector<Vector3f> m_controlVecs; 
	vector<QQuaternion> m_controlQuats; 
	vector<Matrix4f> m_controlMats; 
	// Default pose
	vector<Vector3f> m_correctionVecs; 
	QVector<QQuaternion> m_correctionQuats;
	vector<Matrix4f> m_correctionMats; 
	void initCorrectionQuats();
	void initLocalMatrices(const aiNode* node);

	vector<QString> m_boneTransformInfo;
	vector<bool> m_hasCoordinates; // #?

	static const uint m_numCoordinates = 37;
	enum Coordinates{
		pelvis_tilt, //z
		pelvis_list, //x
		pelvis_rotation, //y
		pelvis_tx,
		pelvis_ty,
		pelvis_tz,

		hip_flexion_r,
		hip_adduction_r,
		hip_rotation_r,
		knee_angle_r,
		ankle_angle_r,
		subtalar_angle_r,
		mtp_angle_r,

		hip_flexion_l,
		hip_adduction_l,
		hip_rotation_l,
		knee_angle_l,
		ankle_angle_l,
		subtalar_angle_l,
		mtp_angle_l,

		lumbar_extension,
		lumbar_bending,
		lumbar_rotation,

		arm_flex_r,
		arm_add_r,
		arm_rot_r,
		elbow_flex_r,
		pro_sup_r,
		wrist_flex_r,
		wrist_dev_r,

		arm_flex_l,
		arm_add_l,
		arm_rot_l,
		elbow_flex_l,
		pro_sup_l,
		wrist_flex_l,
		wrist_dev_l
	};
	array<float, m_numCoordinates> m_modelCoordinateNames;
	array<float, m_numCoordinates> m_modelCoordinates;
	void initCoordinates();
	QQuaternion coordinateAngles(uint i);


	uint m_numBones = 0; // crash if not 0
	uint m_numVertices; // total number of vertices
	unsigned long long m_vertexArrayBytes;
	Matrix4f m_GlobalInverseTransform;

	const aiScene* m_pScene;
	Assimp::Importer m_Importer;

	bitset<NUM_PARAMETERS> m_parameters = bitset<NUM_PARAMETERS>().set();
	const string m_parameterInfo[NUM_PARAMETERS] = { "",  "Corrected Pose", "OpenSim Pose", "Controlled Pose" };	

	GLuint m_axesVAO;
	GLuint m_axesVBO;
	GLuint m_axesIBO; 
};

#endif	/* SKINNED_MESH_H */
