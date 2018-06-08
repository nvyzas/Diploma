#ifndef SKINNED_MESH_H
#define	SKINNED_MESH_H

// Project
#include "ksensor.h"
#include "util.h"

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

#define ASSIMP_LOAD_FLAGS aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices  | aiProcess_LimitBoneWeights 
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
		numIndices = 0;
		baseVertex = 0;
		baseIndex = 0;
		materialIndex = INVALID_MATERIAL;
	}

	uint numIndices;
	uint baseVertex;
	uint baseIndex;
	uint materialIndex;

	void print() {
		cout << "MeshEntry: ";
		cout << " numIndices=" << numIndices;
		cout << " baseVertex=" << baseVertex;
		cout << " baseIndex=" << baseIndex;
		cout << " materialIndex=" << materialIndex << endl;
	}
};
struct BoneInfo
{
	// Transformations
	QMatrix4x4 defaultLocal;
	QMatrix4x4 localCorrection;
	QMatrix4x4 correctedLocal;
	QMatrix4x4 offset;
	QMatrix4x4 global;
	QMatrix4x4 combined;

	bool visible;
	float xRot, yRot, zRot;
	
	QVector3D endPosition;
	float length;

	BoneInfo()
	{
		xRot = yRot = zRot = 0.f;
		visible = true;
	}
};

class SkinnedMesh
{
public:
	SkinnedMesh();
	~SkinnedMesh();	
	bool loadFromFile(const string& basename);

	void calculateBoneTransforms(const aiNode* pNode, const QMatrix4x4& P, const array<KJoint, JointType_Count>& joints);
	void correctLocalMatrices();
	void printInfo() const;
	void printNodeHierarchy(const aiNode* pNode) const;
	void printParameters() const;
	void printSceneInfo() const;
	
	bool m_successfullyLoaded;

	float boneRotationX(const QString &boneName) const;
	float boneRotationY(const QString &boneName) const;
	float boneRotationZ(const QString &boneName) const;
	void setBoneRotationX(const QString &boneName, float value);
	void setBoneRotationY(const QString &boneName, float value);
	void setBoneRotationZ(const QString &boneName, float value);
	
	uint numBones() const;
	const map<string, uint>& boneMap() const;
	const QMatrix4x4& boneGlobal(uint boneIndex) const;
	const QVector3D& boneEndPosition(uint boneIndex) const;
	const BoneInfo& boneInfo(uint boneIndex) const;

	uint findBoneId(const QString &boneName) const;
	bool boneVisibility(uint boneIndex) const;
	bool boneVisibility(const QString &boneName) const;
	void setBoneVisibility(uint boneIndex, bool state);
	void setBoneVisibility(const QString &boneName, bool state);
	QString boneTransformInfo(const QString& boneName) const;
	void flipParameter(uint i);

	// Getters
	QVector<MeshEntry>& meshEntries();
	QVector<QVector3D>& positions();
	QVector<QVector3D>& normals();
	QVector<QVector2D>& texCoords();
	QVector<VertexBoneData>& vertexBoneData();
	QVector<uint>& indices();
	QVector<QImage>& images();

	QVector3D getPelvisOffset();
	void setKSkeleton(KSkeleton* ks);
	bool parameter(uint i) const;
	const aiScene* m_pScene;

private:
	void clear();
	void initMesh(uint meshIndex, const aiMesh* paiMesh);
	void loadBones(uint meshIndex, const aiMesh* paiMesh, QVector<VertexBoneData>& bones);
	void checkWeights(uint meshIndex, const aiMesh* pMesh);
	bool initImages(const aiScene* pScene, const string& filename);
	bool initFromScene(const aiScene* pScene, const string& filename);
	
	// Mesh entries
	QVector<MeshEntry> m_meshEntries;
	// Vertex attributes
	QVector<QVector3D> m_positions;
	QVector<QVector3D> m_normals;
	QVector<QVector2D> m_texCoords;
	QVector<VertexBoneData> m_vertexBoneData;
	QVector<uint> m_indices;
	// Bones
	QVector<BoneInfo> m_boneInfo;
	// Textures
	QVector<QImage> m_images;

	map<string, uint> m_boneMap; // maps a mesh's bone name to its index (key = bone name, value = index)
	map<string, uint> m_kboneMap; // maps a mesh's bone name to its kinect JointType index
	void initKBoneMap();
	const KJoint *m_pKBones = NULL;
	KSkeleton* m_kskelie;

	// Control pose
	vector<QQuaternion> m_controlQuats; 
	vector<QMatrix4x4> m_controlMats;

	void initDefaultLocalMatrices(const aiNode* node);

	vector<QString> m_boneTransformInfo;

	uint m_numBones = 0; // crash if not 0
	uint m_numVertices; // total number of vertices

	Assimp::Importer m_Importer;

	bitset<NUM_PARAMETERS> m_parameters = bitset<NUM_PARAMETERS>().set();
	const string m_parameterInfo[NUM_PARAMETERS] = { 
		"Offset Pose Disabled"  , // 0
		"Kinect Pose"           , // 1
		"Corrected Pose"        , // 2
		"Controlled Pose"       , // 3
		"Identity Pose Disabled", // 4
		"Handmade quaternions"	, // 5
		"Front Direction"		, // 6
		"+ Direction"    		, // 7
	};	
};

#endif	/* SKINNED_MESH_H */
