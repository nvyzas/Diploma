#ifndef SKINNED_MESH_H
#define	SKINNED_MESH_H

// Project
#include "sensor.h"
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
	enum RenderPose
	{
		Bind,
		Kinect,
		Offset
	};
	Matrix4f BoneOffset;
	Matrix4f FinalTransformation;
	bool Visible;
	RenderPose Pose;
	float xRot, yRot, zRot;
	
	BoneInfo()
	{
		BoneOffset.SetZero();
		FinalTransformation.SetZero();
		Visible = true;
		Pose = Bind;
		xRot = yRot = zRot = 0;
	}
};

class SkinnedMesh
{
public:
	SkinnedMesh();
	~SkinnedMesh();	
	bool LoadMesh(const string& Filename);
	void BoneTransform(float TimeInSeconds, vector<Matrix4f>& Transforms);
	void GetBoneTransforms(vector<Matrix4f>& Transforms);
	void TraverseNodeHierarchy(const aiNode* pNode, const Matrix4f& P);
	void setConMats();
	void PrintInfo() const;
	void PrintNodeHierarchy(const aiNode* pNode) const;
	void PrintNodeMatching(const aiNode* pNode) const;
	void PrintParameters() const;
	void PrintSceneInfo() const;

	// TODO: implement skinning on/off
	void AdjustBones(); 
	void ToggleSkinning();
	
	bool m_SuccessfullyLoaded;

	void setKSensor(const KSensor &ks);
	void initBoneMapping();
	void initKBoneMapping();
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

	map<string, uint> m_boneMap; // maps a model bone name to its index (key = bone name, value = index)
	map<string, uint> m_kboneMap; // maps a model bone name to its kinect JointType index
	const KJoint *m_pKBones;

	vector<Vector3f> m_relVecs; // relative vectors (for hierarchical translation)
	vector<QQuaternion> m_relQuats; // relative quaternions (for hierarchical rotation)
	vector<Matrix4f> m_relMats; // relative matrices (for hierarchical transforms)
	vector<Vector3f> m_absVecs; 
	vector<QQuaternion> m_absQuats; 
	vector<Matrix4f> m_absMats; 	
	vector<Vector3f> m_conVecs; // control vectors (for controlled translation)
	vector<QQuaternion> m_conQuats; // control quaternions (for controlled rotation)
	vector<Matrix4f> m_conMats;
	vector<Vector3f> m_corVecs; // correction vectors
	vector<QQuaternion> m_corQuats; // correction quaternions 
	vector<Matrix4f> m_corMats;
	uint m_numBones;
	uint m_numKBones;
	uint m_numVertices; // total number of vertices
	unsigned long long m_vertexArrayBytes;
	Matrix4f m_GlobalInverseTransform;

	const aiScene* m_pScene;
	Assimp::Importer m_Importer;

	bitset<NUM_PARAMETERS> m_Parameters;
	const string m_ParametersStringTrue[NUM_PARAMETERS] = { "",  "My local", "My quaternion", "My matrix", "qRel=qAbs*qAbsParInv","qAbs=qAbsPar*qRel", "Bind pose", "Offset pose", "No control", "Isolated control" };
	const string m_ParametersStringFalse[NUM_PARAMETERS] = { "", "AI local", "AI quaternion", "AI matrix", "qRel=qAbsParInv*qAbs","qAbs=qRel*qAbsPar", "Kinect pose", "Kinect pose", "Control", "Hierarchical control" };
	vector<string> m_bonesTransformInfo;
};

#endif	/* SKINNED_MESH_H */
