#include "skinned_mesh.h"
#include <iostream>
#include <iomanip>

#define POSITION_LOCATION    0
#define TEX_COORD_LOCATION   1
#define NORMAL_LOCATION      2
#define BONE_ID_LOCATION     3
#define BONE_WEIGHT_LOCATION 4
#define NUM_MODELS 5


SkinnedMesh::SkinnedMesh()
{
}
SkinnedMesh::SkinnedMesh(const KSensor &ks)
{
	m_VAO = 0;
	ZERO_MEM(m_Buffers);
	m_pScene = NULL;
	KBoneMapper();
	m_pKBones = ks.m_Joints;
	m_BindPose = false;
	m_NumBones = 0;
	m_NumNodes = 0;
	m_ActiveModel = 0;
	m_ActiveBoneTransformInfo = 0;
	m_SuccessfullyLoaded = false;
}
SkinnedMesh::~SkinnedMesh()
{
    Clear();
}
void SkinnedMesh::Clear()
{
	m_NumBones = 0;
	m_NumNodes = 0;
	m_BoneMapping.clear();
    for (uint i = 0 ; i < m_Textures.size() ; i++) {
        SAFE_DELETE(m_Textures[i]);
    }

    if (m_Buffers[0] != 0) {
        glDeleteBuffers(ARRAY_SIZE_IN_ELEMENTS(m_Buffers), m_Buffers);
    }
       
    if (m_VAO != 0) {
        glDeleteVertexArrays(1, &m_VAO);
        m_VAO = 0;
    }
	m_SuccessfullyLoaded = false;
}
bool SkinnedMesh::LoadMesh(const string& Filename)
{
    // Release the previously loaded mesh (if it exists)
    Clear();
 
    // Create the VAO
    glGenVertexArrays(1, &m_VAO);   
    glBindVertexArray(m_VAO);
    
    // Create the buffers for the vertices attributes
    glGenBuffers(ARRAY_SIZE_IN_ELEMENTS(m_Buffers), m_Buffers);

    bool Ret = false;    
	cout << endl;
	cout << "Loading model " << m_ActiveModel << ": " << Filename.c_str() << endl;
    m_pScene = m_Importer.ReadFile(Filename.c_str(), ASSIMP_LOAD_FLAGS);    
    if (m_pScene) {  
        m_GlobalInverseTransform = m_pScene->mRootNode->mTransformation;
        m_GlobalInverseTransform.Invert();		
        Ret = InitFromScene(m_pScene, Filename);
		PrintSceneInfo();
    }
    else {
        printf("Error parsing '%s': '%s'\n", Filename.c_str(), m_Importer.GetErrorString());
    }
	m_SuccessfullyLoaded = Ret;
	m_Skinned = true;

    // Make sure the VAO is not changed from the outside
    glBindVertexArray(0);

    return Ret;
}
bool SkinnedMesh::InitFromScene(const aiScene* pScene, const string& Filename)
{  
    m_Entries.resize(pScene->mNumMeshes);
    m_Textures.resize(pScene->mNumMaterials);
	vector<Vector3f> Positions;
	vector<Vector3f> Normals;
	vector<Vector2f> TexCoords;
	vector<VertexBoneData> Bones;
	vector<uint> Indices;
       
    uint NumVertices = 0;
    uint NumIndices = 0;
    
    // Count the number of vertices and indices
    for (uint i = 0 ; i < m_Entries.size() ; i++) {
        m_Entries[i].MaterialIndex = pScene->mMeshes[i]->mMaterialIndex;        
        m_Entries[i].NumIndices    = pScene->mMeshes[i]->mNumFaces * 3;
        m_Entries[i].BaseVertex    = NumVertices;
        m_Entries[i].BaseIndex     = NumIndices;	
        NumVertices += pScene->mMeshes[i]->mNumVertices;
        NumIndices  += m_Entries[i].NumIndices;
    }
	m_NumVertices = NumVertices;

	// Reserve space in the vectors for the vertex attributes and indices
	Positions.reserve(NumVertices);
	Normals.reserve(NumVertices);
	TexCoords.reserve(NumVertices);
	Bones.resize(NumVertices);
	Indices.reserve(NumIndices);

	// Initialize the meshes in the scene one by one
	for (uint i = 0; i < m_Entries.size(); i++) {
		const aiMesh* paiMesh = pScene->mMeshes[i];
		InitMesh(i, paiMesh, Positions, Normals, TexCoords, Bones, Indices);
	}
	m_ActiveBone = m_BoneMapping.begin();
	m_relQuats.resize(m_NumBones);
	m_relVecs.resize(m_NumBones);
	m_relMats.resize(m_NumBones);
	m_absQuats.resize(m_NumBones);
	m_absVecs.resize(m_NumBones);
	m_absMats.resize(m_NumBones);
	m_corQuats.resize(m_NumBones);
	m_corVecs.resize(m_NumBones);
	m_corMats.resize(m_NumBones);
	m_conQuats.resize(m_NumBones);
	m_conVecs.resize(m_NumBones);
	m_conMats.resize(m_NumBones);	
	m_BoneTransformInfo.resize(m_NumBones);
	SetConQuats();

	if (!InitMaterials(pScene, Filename)) {
		return false;
	}

	// Generate and populate the buffers with vertex attributes and the indices
	glBindBuffer(GL_ARRAY_BUFFER, m_Buffers[POS_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Positions[0]) * Positions.size(), &Positions[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(POSITION_LOCATION);
	glVertexAttribPointer(POSITION_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_Buffers[TEXCOORD_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(TexCoords[0]) * TexCoords.size(), &TexCoords[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(TEX_COORD_LOCATION);
	glVertexAttribPointer(TEX_COORD_LOCATION, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_Buffers[NORMAL_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Normals[0]) * Normals.size(), &Normals[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(NORMAL_LOCATION);
	glVertexAttribPointer(NORMAL_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_Buffers[BONE_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Bones[0]) * Bones.size(), &Bones[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(BONE_ID_LOCATION);
	glVertexAttribIPointer(BONE_ID_LOCATION, 4, GL_INT, sizeof(VertexBoneData), (const GLvoid*)0);
	glEnableVertexAttribArray(BONE_WEIGHT_LOCATION);
	glVertexAttribPointer(BONE_WEIGHT_LOCATION, 4, GL_FLOAT, GL_FALSE, sizeof(VertexBoneData), (const GLvoid*)16);

	m_vertexArrayBytes = sizeof(Positions[0]) * Positions.size() + sizeof(TexCoords[0]) * TexCoords.size()
		+ sizeof(Normals[0]) * Normals.size() + sizeof(Bones[0]) * Bones.size();

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_Buffers[INDEX_BUFFER]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Indices[0]) * Indices.size(), &Indices[0], GL_STATIC_DRAW);

	return GLCheckError();
}
void SkinnedMesh::InitMesh(uint MeshIndex,
	const aiMesh* paiMesh,
	vector<Vector3f>& Positions,
	vector<Vector3f>& Normals,
	vector<Vector2f>& TexCoords,
	vector<VertexBoneData>& Bones,
	vector<uint>& Indices)
{
	const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);

	// Populate the vertex attribute vectors
	for (uint i = 0; i < paiMesh->mNumVertices; i++) {
		const aiVector3D* pPos = &(paiMesh->mVertices[i]);
		const aiVector3D* pNormal = &(paiMesh->mNormals[i]);
		const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &Zero3D;

		Positions.push_back(Vector3f(pPos->x, pPos->y, pPos->z));
		Normals.push_back(Vector3f(pNormal->x, pNormal->y, pNormal->z));
		TexCoords.push_back(Vector2f(pTexCoord->x, pTexCoord->y));
	}

	LoadBones(MeshIndex, paiMesh, Bones);
	bool SumNot1 = false;
	for (uint i = 0; i < paiMesh->mNumVertices; i++) {
		float sum = 0;
		uint VertexID = m_Entries[MeshIndex].BaseVertex + i;
		for (uint i = 0; i < NUM_BONES_PER_VERTEX; i++) {
			sum += Bones[VertexID].Weights[i];
		}
		if (sum<0.99 || sum >1.01) SumNot1 = true; // Must be equal to 1
	}
	if (SumNot1) cout << "Sum of weights for some vertices is different than 1!" << endl;

	// Populate the index buffer
	for (uint i = 0; i < paiMesh->mNumFaces; i++) {
		const aiFace& Face = paiMesh->mFaces[i];
		assert(Face.mNumIndices == 3);
		Indices.push_back(Face.mIndices[0]);
		Indices.push_back(Face.mIndices[1]);
		Indices.push_back(Face.mIndices[2]);
	}
}
void SkinnedMesh::LoadBones(uint MeshIndex, const aiMesh* pMesh, vector<VertexBoneData>& Bones)
{
	for (uint i = 0; i < pMesh->mNumBones; i++) {
		uint BoneIndex = 0;
		string BoneName(pMesh->mBones[i]->mName.data);
		if (m_BoneMapping.find(BoneName) == m_BoneMapping.end()) {
			// Allocate an index for a new bone
			BoneIndex = m_NumBones;
			m_NumBones++;
			BoneInfo bi;
			m_BoneInfo.push_back(bi);
			m_BoneInfo[BoneIndex].BoneOffset = pMesh->mBones[i]->mOffsetMatrix;
			m_BoneMapping[BoneName] = BoneIndex;
		}
		else {
			BoneIndex = m_BoneMapping[BoneName];
		}
		
		for (uint j = 0; j < pMesh->mBones[i]->mNumWeights; j++) {
			uint VertexID = m_Entries[MeshIndex].BaseVertex + pMesh->mBones[i]->mWeights[j].mVertexId;
			float Weight = pMesh->mBones[i]->mWeights[j].mWeight;
			Bones[VertexID].AddBoneData(BoneIndex, Weight);
		}
		m_VertexBoneData = Bones; // keep vertex bone data copy in class member (necessary?)
	}
}
void SkinnedMesh::VertexBoneData::AddBoneData(uint BoneID, float Weight)
{
	for (uint i = 0; i < ARRAY_SIZE_IN_ELEMENTS(IDs); i++) {
		if (Weights[i] == 0.0) {
			IDs[i] = BoneID;
			Weights[i] = Weight;
			return;
		}
	}
	cout << "Should never get here!" << endl;
	system("PAUSE");
	// should never get here - more bones than we have space for
	assert(0);
}
void SkinnedMesh::VertexBoneData::AdjustBoneData()
{
	float max = 0.0f;
	for (uint i = 0; i < ARRAY_SIZE_IN_ELEMENTS(IDs); i++) {
		if (Weights[i] > max) max = Weights[i];
		OldWeights[i] = Weights[i]; // backup weights
	}

	for (uint i = 0; i < ARRAY_SIZE_IN_ELEMENTS(IDs); i++) {
		if (Weights[i] >= max) {
			Weights[i] = 1;
			for (uint j = 0; j < ARRAY_SIZE_IN_ELEMENTS(IDs); j++)
			{
				if (j != i) Weights[j] = 0.0f;
			}
			return;
		}	
	}
	cout << "Should never get here!" << endl;
	system("PAUSE");
	// should never get here
	assert(0);
}
void SkinnedMesh::ToggleSkinning()
{ 
	unsigned long long vertexBoneDataBytes = sizeof(m_VertexBoneData[0]) * m_VertexBoneData.size();
	if (m_Skinned){		
		for (uint i = 0; i < m_VertexBoneData.size(); i++) {
			m_VertexBoneData[i].AdjustBoneData();
		}		
		glBindBuffer(GL_ARRAY_BUFFER, m_Buffers[BONE_VB]);
		glBufferSubData(GL_ARRAY_BUFFER, m_vertexArrayBytes - vertexBoneDataBytes, vertexBoneDataBytes, &m_VertexBoneData);
		/*
		glEnableVertexAttribArray(BONE_ID_LOCATION);
		glVertexAttribIPointer(BONE_ID_LOCATION, 4, GL_INT, sizeof(VertexBoneData), (const GLvoid*)0);
		glEnableVertexAttribArray(BONE_WEIGHT_LOCATION);
		glVertexAttribPointer(BONE_WEIGHT_LOCATION, 4, GL_FLOAT, GL_FALSE, sizeof(VertexBoneData), (const GLvoid*)16);
		//*/
		m_Skinned = false;
	}
	else {
		for (uint i = 0; i < m_VertexBoneData.size(); i++) {
			m_VertexBoneData[i].RestoreBoneData();
		}
		glBindBuffer(GL_ARRAY_BUFFER, m_Buffers[BONE_VB]);
		glBufferSubData(GL_ARRAY_BUFFER, m_vertexArrayBytes - vertexBoneDataBytes, vertexBoneDataBytes, &m_VertexBoneData);
		m_Skinned = true;
	}
}
void SkinnedMesh::VertexBoneData::RestoreBoneData()
{
	for (uint i = 0; i < ARRAY_SIZE_IN_ELEMENTS(IDs); i++)
	{
		Weights[i] = OldWeights[i];
	}
}
bool SkinnedMesh::InitMaterials(const aiScene* pScene, const string& Filename)
{
	// Extract the directory part from the file name
	string::size_type SlashIndex = Filename.find_last_of("/");
	string Dir;

	if (SlashIndex == string::npos) {
		Dir = ".";
	}
	else if (SlashIndex == 0) {
		Dir = "/";
	}
	else {
		Dir = Filename.substr(0, SlashIndex);
	}

	bool Ret = true;

	// Initialize the materials
	for (uint i = 0; i < pScene->mNumMaterials; i++) {
		const aiMaterial* pMaterial = pScene->mMaterials[i];

		m_Textures[i] = NULL;

		if (pMaterial->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
			aiString Path;

			if (pMaterial->GetTexture(aiTextureType_DIFFUSE, 0, &Path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS) {
				string p(Path.data);

				if (p.substr(0, 2) == ".\\") {
					p = p.substr(2, p.size() - 2);
				}

				string FullPath = Dir + "/" + p;

				m_Textures[i] = new Texture(GL_TEXTURE_2D, FullPath.c_str());

				if (!m_Textures[i]->Load()) {
					printf("Error loading texture '%s'\n", FullPath.c_str());
					delete m_Textures[i];
					m_Textures[i] = NULL;
					Ret = false;
				}
				else {
					printf("Material[%d]: - loaded texture '%s'\n", i, FullPath.c_str());
				}
			}
		}
	}

	return Ret;
}
void SkinnedMesh::Render()
{
	if (m_SuccessfullyLoaded) {
		glBindVertexArray(m_VAO);

		for (uint i = 0; i < m_Entries.size(); i++) {
			const uint MaterialIndex = m_Entries[i].MaterialIndex;

			assert(MaterialIndex < m_Textures.size());

			if (m_Textures[MaterialIndex]) {
				m_Textures[MaterialIndex]->Bind(GL_TEXTURE0);
			}

			glDrawElementsBaseVertex(GL_TRIANGLES,
				m_Entries[i].NumIndices,
				GL_UNSIGNED_INT,
				(void*)(sizeof(uint) * m_Entries[i].BaseIndex),
				m_Entries[i].BaseVertex);
		}

		// Make sure the VAO is not changed from the outside    
		glBindVertexArray(0);
	}
}
uint SkinnedMesh::FindPosition(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	for (uint i = 0; i < pNodeAnim->mNumPositionKeys - 1; i++) {
		if (AnimationTime < (float)pNodeAnim->mPositionKeys[i + 1].mTime) {
			return i;
		}
	}

	assert(0);

	return 0;
}
uint SkinnedMesh::FindRotation(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	assert(pNodeAnim->mNumRotationKeys > 0);

	for (uint i = 0; i < pNodeAnim->mNumRotationKeys - 1; i++) {
		if (AnimationTime < (float)pNodeAnim->mRotationKeys[i + 1].mTime) {
			return i;
		}
	}

	assert(0);

	return 0;
}
uint SkinnedMesh::FindScaling(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	assert(pNodeAnim->mNumScalingKeys > 0);

	for (uint i = 0; i < pNodeAnim->mNumScalingKeys - 1; i++) {
		if (AnimationTime < (float)pNodeAnim->mScalingKeys[i + 1].mTime) {
			return i;
		}
	}

	assert(0);

	return 0;
}
void SkinnedMesh::CalcInterpolatedPosition(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	if (pNodeAnim->mNumPositionKeys == 1) {
		Out = pNodeAnim->mPositionKeys[0].mValue;
		return;
	}

	uint PositionIndex = FindPosition(AnimationTime, pNodeAnim);
	uint NextPositionIndex = (PositionIndex + 1);
	assert(NextPositionIndex < pNodeAnim->mNumPositionKeys);
	float DeltaTime = (float)(pNodeAnim->mPositionKeys[NextPositionIndex].mTime - pNodeAnim->mPositionKeys[PositionIndex].mTime);
	float Factor = (AnimationTime - (float)pNodeAnim->mPositionKeys[PositionIndex].mTime) / DeltaTime;
	assert(Factor >= 0.0f && Factor <= 1.0f);
	const aiVector3D& Start = pNodeAnim->mPositionKeys[PositionIndex].mValue;
	const aiVector3D& End = pNodeAnim->mPositionKeys[NextPositionIndex].mValue;
	aiVector3D Delta = End - Start;
	Out = Start + Factor * Delta;
}
void SkinnedMesh::CalcInterpolatedRotation(aiQuaternion& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	// we need at least two values to interpolate...
	if (pNodeAnim->mNumRotationKeys == 1) {
		Out = pNodeAnim->mRotationKeys[0].mValue;
		return;
	}

	uint RotationIndex = FindRotation(AnimationTime, pNodeAnim);
	uint NextRotationIndex = (RotationIndex + 1);
	assert(NextRotationIndex < pNodeAnim->mNumRotationKeys);
	float DeltaTime = (float)(pNodeAnim->mRotationKeys[NextRotationIndex].mTime - pNodeAnim->mRotationKeys[RotationIndex].mTime);
	float Factor = (AnimationTime - (float)pNodeAnim->mRotationKeys[RotationIndex].mTime) / DeltaTime;
	assert(Factor >= 0.0f && Factor <= 1.0f);
	const aiQuaternion& StartRotationQ = pNodeAnim->mRotationKeys[RotationIndex].mValue;
	const aiQuaternion& EndRotationQ = pNodeAnim->mRotationKeys[NextRotationIndex].mValue;
	aiQuaternion::Interpolate(Out, StartRotationQ, EndRotationQ, Factor);
	Out = Out.Normalize();
}
void SkinnedMesh::CalcInterpolatedScaling(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	if (pNodeAnim->mNumScalingKeys == 1) {
		Out = pNodeAnim->mScalingKeys[0].mValue;
		return;
	}

	uint ScalingIndex = FindScaling(AnimationTime, pNodeAnim);
	uint NextScalingIndex = (ScalingIndex + 1);
	assert(NextScalingIndex < pNodeAnim->mNumScalingKeys);
	float DeltaTime = (float)(pNodeAnim->mScalingKeys[NextScalingIndex].mTime - pNodeAnim->mScalingKeys[ScalingIndex].mTime);
	float Factor = (AnimationTime - (float)pNodeAnim->mScalingKeys[ScalingIndex].mTime) / DeltaTime;
	assert(Factor >= 0.0f && Factor <= 1.0f);
	const aiVector3D& Start = pNodeAnim->mScalingKeys[ScalingIndex].mValue;
	const aiVector3D& End = pNodeAnim->mScalingKeys[NextScalingIndex].mValue;
	aiVector3D Delta = End - Start;
	Out = Start + Factor * Delta;
}
void SkinnedMesh::ReadNodeHierarchy(float AnimationTime, const aiNode* pNode, const Matrix4f& ParentTransform)
{
	string NodeName(pNode->mName.data);

	const aiAnimation* pAnimation = m_pScene->mAnimations[0];

	Matrix4f NodeTransformation(pNode->mTransformation);

	const aiNodeAnim* pNodeAnim = FindNodeAnim(pAnimation, NodeName);

	if (pNodeAnim) {
		// Interpolate scaling and generate scaling transformation matrix
		aiVector3D Scaling;
		CalcInterpolatedScaling(Scaling, AnimationTime, pNodeAnim);
		Matrix4f ScalingM;
		ScalingM.InitScaleTransform(Scaling.x, Scaling.y, Scaling.z);

		// Interpolate rotation and generate rotation transformation matrix
		aiQuaternion RotationQ;
		CalcInterpolatedRotation(RotationQ, AnimationTime, pNodeAnim);
		Matrix4f RotationM(RotationQ.GetMatrix());

		// Interpolate translation and generate translation transformation matrix
		aiVector3D Translation;
		CalcInterpolatedPosition(Translation, AnimationTime, pNodeAnim);
		Matrix4f TranslationM;
		TranslationM.InitTranslateTransform(Translation.x, Translation.y, Translation.z);

		// Combine the above transformations
		NodeTransformation = TranslationM * RotationM * ScalingM;
	}

	Matrix4f GlobalTransformation = ParentTransform * NodeTransformation;

	if (m_BoneMapping.find(NodeName) != m_BoneMapping.end()) {
		uint BoneIndex = m_BoneMapping[NodeName];
		m_BoneInfo[BoneIndex].FinalTransformation = m_GlobalInverseTransform * GlobalTransformation * m_BoneInfo[BoneIndex].BoneOffset;
	}

	for (uint i = 0; i < pNode->mNumChildren; i++) {
		ReadNodeHierarchy(AnimationTime, pNode->mChildren[i], GlobalTransformation);
	}
}
void SkinnedMesh::BoneTransform(float TimeInSeconds, vector<Matrix4f>& Transforms)
{
	Matrix4f Identity;
	Identity.InitIdentity();

	float TicksPerSecond = (float)(m_pScene->mAnimations[0]->mTicksPerSecond != 0 ? m_pScene->mAnimations[0]->mTicksPerSecond : 25.0f);
	float TimeInTicks = TimeInSeconds * TicksPerSecond;
	float AnimationTime = fmod(TimeInTicks, (float)m_pScene->mAnimations[0]->mDuration);

	ReadNodeHierarchy(AnimationTime, m_pScene->mRootNode, Identity);

	Transforms.resize(m_NumBones);

	for (uint i = 0; i < m_NumBones; i++) {
		Transforms[i] = m_BoneInfo[i].FinalTransformation;
	}
}
void SkinnedMesh::PrintInfo()
{
	cout << endl;
	PrintSceneInfo();
	cout << "Model skeleton" << endl;
	PrintNodeHierarchy(m_pScene->mRootNode);	
	cout << "Bone matching" << endl;
	PrintNodeMatching(m_pScene->mRootNode);	
}
void SkinnedMesh::PrintSceneInfo()
{
	cout << "Mesh:";
	cout << " Entries:" << m_pScene->mNumMeshes;
	cout << " Materials:" << m_pScene->mNumMaterials;
	cout << " Textures:" << m_pScene->mNumTextures;
	cout << " Lights:" << m_pScene->mNumLights;
	cout << " Animations:" << m_pScene->mNumAnimations;
	cout << endl;
	for (uint i = 0; i < m_pScene->mNumMeshes; i++) {
		cout << "Entry " << i << ":";
		cout << " Name:" << setw(15) << m_pScene->mMeshes[i]->mName.C_Str();
		cout << " Vertices:" << setw(6) << m_pScene->mMeshes[i]->mNumVertices;
		cout << " Faces:" << setw(6) << m_pScene->mMeshes[i]->mNumFaces;
		cout << " Bones:" << setw(3) << m_pScene->mMeshes[i]->mNumBones;
		cout << endl;
	}
	cout << "Total: Vertices:" << m_NumVertices;
	cout << " Nodes:" << m_NumNodes;
	cout << " Bones:" << m_NumBones;
	cout << endl;
}
void SkinnedMesh::PrintNodeHierarchy(const aiNode* pNode)
{	
	string NodeName(pNode->mName.data);
	string ParentName(pNode->mParent ? pNode->mParent->mName.data : "Root");
	if (!pNode->mParent) m_NumNodes = 0;
	m_NumNodes += 1;
	cout << "Node " << setw(2) << m_NumNodes << ": " << setw(20) << left << NodeName << setw(10) << left;
	if (m_KBoneMapping.find(NodeName) != m_KBoneMapping.end()) cout << "(KBone)";
	else if (m_BoneMapping.find(NodeName) != m_BoneMapping.end()) cout << " (Bone)";
	else cout << "";
	cout << " Parent:" << setw(20) << left << ParentName;
	cout << " Children (" << pNode->mNumChildren << "): ";
	for (uint i = 0; i < pNode->mNumChildren; i++) {
		string ChildName(pNode->mChildren[i]->mName.data);
		cout << ChildName << " ";	
	}
	cout << endl;
	for (uint i = 0; i < pNode->mNumChildren; i++) {
		PrintNodeHierarchy(pNode->mChildren[i]);
	}
	
}
void SkinnedMesh::PrintNodeMatching(const aiNode* pNode)
{
	string NodeName(pNode->mName.data);
	if (m_KBoneMapping.find(NodeName) != m_KBoneMapping.end()) {
		uint i = m_KBoneMapping[NodeName];
		KJoint j = m_pKBones[i];
		cout << setw(20) << left << NodeName << " -> " << j.name << " (" << i << ")" << endl;
	}

	for (uint i = 0; i < pNode->mNumChildren; i++) {
		PrintNodeMatching(pNode->mChildren[i]);
	}
}
void SkinnedMesh::GetBoneTransforms(vector<Matrix4f>& Transforms)
{
	TraverseNodeHierarchy(m_pScene->mRootNode, Matrix4f::Identity());
	Transforms.resize(m_NumBones);
	for (uint i = 0; i < m_NumBones; i++) {
		Transforms[i] = m_BoneInfo[i].FinalTransformation;
	}
}
void SkinnedMesh::TraverseNodeHierarchy(const aiNode* pNode, const Matrix4f& P)
{
	static int counter;
	string NodeName(pNode->mName.data);
	if (NodeName == "Hips") counter = 0;
	Matrix4f L(pNode->mTransformation);
	Matrix4f G;
	Quaternion q;
	stringstream sso;
	sso << endl << "Node name " << counter << ": " << NodeName << endl;
	std::map<std::string, uint>::iterator it = m_BoneMapping.find(NodeName);
	if (m_BindPose || (it == m_BoneMapping.end())){
		if (m_Parameters[1]) {
			q = (m_Parameters[2]) ? L.ExtractQuaternion1() : L.ExtractQuaternion2();
			sso << "q rel from model " << ": " << q.ToString() << q.ToEulerAnglesString() << q.ToAxisAngleString() << endl;
			Matrix4f R = m_Parameters[3] ? Matrix4f(q, true) : Matrix4f(q, false);
			sso << "Rotation matrix (local) from q:" << endl << R;
			Matrix4f T = L.GetTranslationPart();
			sso << "Translation matrix (local) from model:" << endl << T;
			L = T * R;
		}
		else {
			L = Matrix4f(pNode->mTransformation);
		}
		G = P * L;
		sso << "Local transformation:" << endl << L;
		sso << "Parent transformation (global):" << endl << P;		
		sso << "Global transformation:" << endl << G;
		if (it != m_BoneMapping.end()) {
			uint i = it->second;
			m_BoneInfo[i].FinalTransformation = G * m_BoneInfo[i].BoneOffset;
		}
	}
	else {
		uint i = it->second;
		std::map<std::string, uint>::iterator kit = m_KBoneMapping.find(NodeName);
		uint c;
		if (kit != m_KBoneMapping.end() && (c = kit->second) != INVALID_JOINT_ID) {			
			q = m_pKBones[c].Orientation;
			sso << "q abs from kinect (" << m_pKBones[c].name << "): " << q.ToString() << q.ToEulerAnglesString() << q.ToAxisAngleString() << endl;
			m_absQuats[i] = q;
			if (NodeName == "Hips") {
				m_relQuats[i] = m_absQuats[i];				
			}
			else {
				uint p = m_BoneMapping[pNode->mParent->mName.data];
				m_relQuats[i] = m_Parameters[4] ? m_absQuats[i]*m_absQuats[p].Inverted(): m_absQuats[p].Inverted()*m_absQuats[i];
				q = m_absQuats[p];
				sso << "q abs parent: " << q.ToString() << q.ToEulerAnglesString() << q.ToAxisAngleString() << endl;
				q = m_absQuats[p].Inverted();
				sso << "q abs parent inverted: " << q.ToString() << q.ToEulerAnglesString() << q.ToAxisAngleString() << endl;
			}
			
			sso << "q relative: " << m_relQuats[i].ToString() << m_relQuats[i].ToEulerAnglesString() << m_relQuats[i].ToAxisAngleString() << endl;
			sso << "q absolute: " << m_absQuats[i].ToString() << m_absQuats[i].ToEulerAnglesString() << m_absQuats[i].ToAxisAngleString() << endl;
			Matrix4f R = m_Parameters[3] ? Matrix4f(m_relQuats[i], true) : Matrix4f(m_relQuats[i], false);
			Matrix4f T = L.GetTranslationPart();
			L = T*R;
			G = P * L;
			sso << "Rotation matrix (local) from q:" << endl << R;
			sso << "Translation matrix (local) from model:" << endl << T;
			sso << "Local transformation:" << endl << L;
			sso << "Global transformation:" << endl << G;
			m_BoneInfo[i].FinalTransformation = G * m_BoneInfo[i].BoneOffset;
			sso << "Final Transformation:" << endl << m_BoneInfo[i].FinalTransformation;
			m_BoneTransformInfo[counter] = sso.str();
			sso.clear();
			counter++;
		}else{ // bones or kbones with invalid id
			q = (m_Parameters[2]) ? L.ExtractQuaternion1() : L.ExtractQuaternion2();
			sso << "q rel from model " << ": " << q.ToString() << q.ToEulerAnglesString() << q.ToAxisAngleString() << endl;
			m_relQuats[i] = q;
			if (NodeName == "Hips") {
				m_absQuats[i] = m_relQuats[i];
			}
			else {
				uint p = m_BoneMapping[pNode->mParent->mName.data];
				m_absQuats[i] = m_Parameters[5] ? m_absQuats[p] * m_relQuats[i] : m_relQuats[i] * m_absQuats[p];
				q = m_absQuats[p];
				sso << "q rel parent: " << q.ToString() << q.ToEulerAnglesString() << q.ToAxisAngleString() << endl;
				q = m_absQuats[p].Inverted();
				sso << "q rel parent inverted: " << q.ToString() << q.ToEulerAnglesString() << q.ToAxisAngleString() << endl;
			}
			sso << "q relative: " << m_relQuats[i].ToString() << m_relQuats[i].ToEulerAnglesString() << m_relQuats[i].ToAxisAngleString() << endl;
			sso << "q absolute: " << m_absQuats[i].ToString() << m_absQuats[i].ToEulerAnglesString() << m_absQuats[i].ToAxisAngleString() << endl;
			Matrix4f R = m_Parameters[3] ? Matrix4f(m_relQuats[i], true) : Matrix4f(m_relQuats[i], false);
			Matrix4f T = L.GetTranslationPart();
			L = m_Parameters[1] ? Matrix4f(pNode->mTransformation) : T*R;
			G = P * L;
			sso << "Rotation matrix (local) from q:" << endl << R;
			sso << "Translation matrix (local) from model:" << endl << T;
			sso << "Local transformation:" << endl << L;			
			sso << "Global transformation:" << endl << G;
			m_BoneInfo[i].FinalTransformation = m_Parameters[0] ? Matrix4f::Zero() : G * m_BoneInfo[i].BoneOffset;				
			sso << "Final Transformation:" << endl << m_BoneInfo[i].FinalTransformation;
			m_BoneTransformInfo[counter] = sso.str();
			sso.clear();
			counter++;
		}
	}
		
	for (uint i = 0; i < pNode->mNumChildren; i++) {
		TraverseNodeHierarchy(pNode->mChildren[i], G);
	}
}
void SkinnedMesh::SetConQuats()
{
	/*
	m_Corrections.resize(m_NumBones);
	Vector3f zeroAngle(0.0f, 0.0f, 0.0f);
	for (uint i = 0; i < m_Corrections.size(); i++) m_Corrections[i]=Quaternion(0,0,0,1);

	
	Quaternion q;
	q.FromAxisAngle(Vector4f(0, 1, 0, 180));
	m_Corrections[JointType_SpineMid] = q;
	q.FromAxisAngle(Vector4f(0, 1, 0, 180));
	m_Corrections[JointType_SpineShoulder] = q;
	q.FromAxisAngle(Vector4f(0, 1, 0, 90));
	//*/
}
void SkinnedMesh::KBoneMapper()
{
	// JointType SpineBase, Head, HandTipLeft/Right, ThumbLeft/Right, FootLeft/Right are always q(0,0,0,0)
	// core
	m_KBoneMapping["Hips"]					= JointType_SpineMid; 
	//m_KBoneMapping["LowerBack"]			= INVALID_JOINT_ID;
	m_KBoneMapping["Spine"]					= JointType_SpineShoulder;
	//m_KBoneMapping["Spine1"]				= INVALID_JOINT_ID;
	m_KBoneMapping["Neck"]					= JointType_Neck;
	//m_KBoneMapping["Neck1"]				= INVALID_JOINT_ID;
	//m_KBoneMapping["Head"]				= INVALID_JOINT_ID;

	// legs
	m_KBoneMapping["LHipJoint"]				= JointType_HipLeft; 
	m_KBoneMapping["RHipJoint"]				= JointType_HipRight; 
	m_KBoneMapping["LeftUpLeg"]				= JointType_KneeLeft;
	m_KBoneMapping["RightUpLeg"]			= JointType_KneeRight;
	m_KBoneMapping["LeftLeg"]				= JointType_AnkleLeft;
	m_KBoneMapping["RightLeg"]				= JointType_AnkleRight;
	//m_KBoneMapping["LeftFoot"]			= INVALID_JOINT_ID;
	//m_KBoneMapping["RightFoot"]			= INVALID_JOINT_ID;
	//m_KBoneMapping["LeftToeBase"]			= INVALID_JOINT_ID;
	//m_KBoneMapping["RightToeBase"]		= INVALID_JOINT_ID;
	
	// arms
	m_KBoneMapping["LeftShoulder"]			= JointType_ShoulderLeft;
	m_KBoneMapping["RightShoulder"]			= JointType_ShoulderRight;
	m_KBoneMapping["LeftArm"]				= JointType_ElbowLeft;
	m_KBoneMapping["RightArm"]				= JointType_ElbowRight;
	m_KBoneMapping["LeftForeArm"]			= JointType_WristLeft;
	m_KBoneMapping["RightForeArm"]			= JointType_WristRight;
	m_KBoneMapping["LeftHand"]				= JointType_HandLeft;
	m_KBoneMapping["RightHand"]				= JointType_HandRight;
	//m_KBoneMapping["LeftThumb"]			= INVALID_JOINT_ID;
	//m_KBoneMapping["RightThumb"]			= INVALID_JOINT_ID;
	//m_KBoneMapping["LeftFingerBase"]		= INVALID_JOINT_ID;
	//m_KBoneMapping["RightFingerBase"]	    = INVALID_JOINT_ID;
	//m_KBoneMapping["LeftHandFinger1"]	    = INVALID_JOINT_ID;
	//m_KBoneMapping["RightHandFinger1"]	= INVALID_JOINT_ID;
}
const aiNodeAnim* SkinnedMesh::FindNodeAnim(const aiAnimation* pAnimation, const string NodeName)
{
	for (uint i = 0; i < pAnimation->mNumChannels; i++) {
		const aiNodeAnim* pNodeAnim = pAnimation->mChannels[i];

		if (string(pNodeAnim->mNodeName.data) == NodeName) {
			return pNodeAnim;
		}
	}
	return NULL;
}
void SkinnedMesh::NextModel(int step)
{	
	m_ActiveModel = Mod(m_ActiveModel, NUM_MODELS, step);
	string MeshNames[NUM_MODELS] = { "cmu_test","cmu", "cmumb_localy_180","bobby",""};
	LoadMesh("models/" + MeshNames[m_ActiveModel] + ".dae");
}
void SkinnedMesh::NextJoint(int step)
{
	if (step < 0) {
		if (m_ActiveBone == m_BoneMapping.begin()) m_ActiveBone = m_BoneMapping.end();
		m_ActiveBone--;
	}
	if (step > 0) {
		m_ActiveBone++;
		if (m_ActiveBone == m_BoneMapping.end()) m_ActiveBone = m_BoneMapping.begin();
	}
	uint i = m_ActiveBone->second;// m_BoneMapIterator->second;
	cout << "Active joint " << setw(2) << m_ActiveBone->second << ": " << setw(15) << m_ActiveBone->first;
	cout << " Visible:" << m_BoneInfo[i].Visible;
	cout << " Offset:" << m_BoneInfo[i].Offset;
	cout << " BindPose:" << m_BoneInfo[i].BindPose;
	cout << endl;
	//cout << m_BoneMapping.size() << endl;
}
void SkinnedMesh::FlipParameter(uint i)
{
	m_Parameters[i].flip();
	cout << "Parameter " << i << (m_Parameters[i] ? " ON: " : " OFF: ");
	cout << (m_Parameters[i] ? m_ParametersStringTrue[i] : m_ParametersStringFalse[i]) << endl;
}
void SkinnedMesh::NextBoneTransformInfo(int step)
{
	m_ActiveBoneTransformInfo = Mod(m_ActiveBoneTransformInfo, m_NumBones, step);
	cout << m_BoneTransformInfo[m_ActiveBoneTransformInfo] << endl;
}
void SkinnedMesh::PrintParameters()
{
	for (uint i = 0; i < NUM_PARAMETERS; i++) {
		cout << "Parameter " << i << (m_Parameters[i] ? " ON: " : " OFF: ");
		cout <<(m_Parameters[i] ? m_ParametersStringTrue[i] : m_ParametersStringFalse[i]) << endl;
	}
}
void SkinnedMesh::ToggleActiveBoneVisibility()
{
	bool &vis = m_BoneInfo[m_ActiveBone->second].Visible;
	vis = !vis;
	cout << m_ActiveBone->first << " visibility " << (vis ? " ON: " : " OFF: ") << endl;
}
bool SkinnedMesh::GetBoneVisibility(uint BoneIndex) const
{
	return m_BoneInfo[BoneIndex].Visible;
}
uint SkinnedMesh::GetActiveBoneID() const
{
	return m_ActiveBone->second;
}
uint SkinnedMesh::GetNumBones() const
{
	return m_NumBones;
}