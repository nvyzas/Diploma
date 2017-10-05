// Own
#include "skinned_mesh.h"

// Qt
#include <QtGui\QVector2D>

// Standard C/C++
#include <cassert>
#include <sstream>

SkinnedMesh::SkinnedMesh()
{
	Clear();
	m_SuccessfullyLoaded = false;
	m_pScene = NULL;
	LoadMesh("cmu_test");
	//initBoneMapping();
	initKBoneMapping();
	PrintInfo();
}
void SkinnedMesh::setKSensor(const KSensor &ks)
{
	m_pKBones = ks.getKJoints();
}
SkinnedMesh::~SkinnedMesh()
{
    Clear();
}
// delete container contents and resize to 0
void SkinnedMesh::Clear()
{	
	m_entries.clear();
	m_positions.clear();
	m_normals.clear();
	m_texCoords.clear();
	m_vertexBoneData.clear();
	m_indices.clear();
	m_boneInfo.clear();
}
bool SkinnedMesh::LoadMesh(const string& basename)
{
	Clear();

    bool Ret = false;
	string Filename = "models/" + basename + ".dae";
	cout << endl;
	cout << "Loading model: " << Filename << endl;
    m_pScene = m_Importer.ReadFile(Filename.c_str(), ASSIMP_LOAD_FLAGS);    
    if (m_pScene) {  
        m_GlobalInverseTransform = m_pScene->mRootNode->mTransformation;
        m_GlobalInverseTransform.Invert();		
        Ret = InitFromScene(m_pScene, Filename);
    }
    else {
        printf("Error parsing '%s': '%s'\n", Filename.c_str(), m_Importer.GetErrorString());
    }
	m_SuccessfullyLoaded = Ret;

    return Ret;
}
bool SkinnedMesh::InitFromScene(const aiScene* pScene, const string& Filename)
{  
    m_entries.resize(pScene->mNumMeshes);
       
    uint NumVertices = 0;
    uint NumIndices = 0;
    
    // Count the number of vertices and indices
    for (uint i = 0 ; i < m_entries.size() ; i++) {
        m_entries[i].MaterialIndex = pScene->mMeshes[i]->mMaterialIndex;        
        m_entries[i].NumIndices    = pScene->mMeshes[i]->mNumFaces * 3;
        m_entries[i].BaseVertex    = NumVertices;
        m_entries[i].BaseIndex     = NumIndices;	
        NumVertices += pScene->mMeshes[i]->mNumVertices;
        NumIndices  += m_entries[i].NumIndices;
    }
	m_numVertices = NumVertices;

	// Reserve space in the vectors for the vertex attributes and indices
	m_positions.reserve(NumVertices);
	m_normals.reserve(NumVertices);
	m_texCoords.reserve(NumVertices);
	m_vertexBoneData.resize(NumVertices);
	m_indices.reserve(NumIndices);

	// Initialize the meshes in the scene one by one
	for (uint i = 0; i < m_entries.size(); i++) {
		const aiMesh* paiMesh = pScene->mMeshes[i];
		InitMesh(i, paiMesh);
	}
	// TODO: check that mesh is correctly skinned
	m_relQuats.resize(m_numBones);
	m_relVecs.resize(m_numBones);
	m_relMats.resize(m_numBones);
	m_absQuats.resize(m_numBones);
	m_absVecs.resize(m_numBones);
	m_absMats.resize(m_numBones);
	m_corQuats.resize(m_numBones);
	m_corVecs.resize(m_numBones);
	m_corMats.resize(m_numBones);
	m_conQuats.resize(m_numBones);
	m_conVecs.resize(m_numBones);
	m_conMats.resize(m_numBones);
	m_boneInfo.resize(m_numBones);
	m_BoneTransformInfo.resize(m_numBones);
	SetConQuats();
	return true;
}
void SkinnedMesh::InitMesh(uint MeshIndex, const aiMesh* paiMesh)
{
	const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);

	// Populate the vertex attribute vectors
	for (uint i = 0; i < paiMesh->mNumVertices; i++) {
		const aiVector3D* pPos = &(paiMesh->mVertices[i]);
		const aiVector3D* pNormal = &(paiMesh->mNormals[i]);
		const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &Zero3D;

		m_positions.push_back(Vector3f(pPos->x, pPos->y, pPos->z));
		m_normals.push_back(Vector3f(pNormal->x, pNormal->y, pNormal->z));
		m_texCoords.push_back(QVector2D(pTexCoord->x, pTexCoord->y));
	}

	LoadBones(MeshIndex, paiMesh, m_vertexBoneData);
	bool SumNot1 = false;
	for (uint i = 0; i < paiMesh->mNumVertices; i++) {
		float sum = 0;
		uint VertexID = m_entries[MeshIndex].BaseVertex + i;
		for (uint i = 0; i < NUM_BONES_PER_VERTEX; i++) {
			sum += m_vertexBoneData[VertexID].Weights[i];
		}
		if (sum<0.99 || sum >1.01) SumNot1 = true; // Must be equal to 1
	}
	if (SumNot1) cout << "Sum of weights for some vertices is different than 1!" << endl;

	// Populate the index buffer
	for (uint i = 0; i < paiMesh->mNumFaces; i++) {
		const aiFace& Face = paiMesh->mFaces[i];
		assert(Face.mNumIndices == 3);
		m_indices.push_back(Face.mIndices[0]);
		m_indices.push_back(Face.mIndices[1]);
		m_indices.push_back(Face.mIndices[2]);
	}
}
void SkinnedMesh::LoadBones(uint MeshIndex, const aiMesh* pMesh, vector<VertexBoneData>& Bones)
{
	for (uint i = 0; i < pMesh->mNumBones; i++) {
		uint BoneIndex = 0;
		string BoneName(pMesh->mBones[i]->mName.data);
		if (m_boneMap.find(BoneName) == m_boneMap.end()) {
			// Allocate an index for a new bone
			BoneIndex = m_numBones;
			m_numBones++;
			BoneInfo bi;
			m_boneInfo.push_back(bi);
			m_boneInfo[BoneIndex].BoneOffset = pMesh->mBones[i]->mOffsetMatrix;
			m_boneMap[BoneName] = BoneIndex;
		}
		else {
			BoneIndex = m_boneMap[BoneName];
		}

		for (uint j = 0; j < pMesh->mBones[i]->mNumWeights; j++) {
			uint VertexID = m_entries[MeshIndex].BaseVertex + pMesh->mBones[i]->mWeights[j].mVertexId;
			float Weight = pMesh->mBones[i]->mWeights[j].mWeight;
			Bones[VertexID].AddBoneData(BoneIndex, Weight);
		}
	}
}
void VertexBoneData::AddBoneData(uint BoneID, float Weight)
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
void VertexBoneData::AdjustBoneData()
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

//void SkinnedMesh::ToggleSkinning()
//{ 
//	unsigned long long vertexBoneDataBytes = sizeof(m_vertexBoneData[0]) * m_vertexBoneData.size();
//	if (m_Skinning){		
//		for (uint i = 0; i < m_vertexBoneData.size(); i++) {
//			m_vertexBoneData[i].AdjustBoneData();
//		}		
//		glBindBuffer(GL_ARRAY_BUFFER, m_Buffers[BONE_VB]); 
//		glBufferSubData(GL_ARRAY_BUFFER, m_vertexArrayBytes - vertexBoneDataBytes, vertexBoneDataBytes, &m_vertexBoneData);
//		/*
//		glEnableVertexAttribArray(BONE_ID_LOCATION);
//		glVertexAttribIPointer(BONE_ID_LOCATION, 4, GL_INT, sizeof(VertexBoneData), (const GLvoid*)0);
//		glEnableVertexAttribArray(BONE_WEIGHT_LOCATION);
//		glVertexAttribPointer(BONE_WEIGHT_LOCATION, 4, GL_FLOAT, GL_FALSE, sizeof(VertexBoneData), (const GLvoid*)16);
//		//*/
//		m_Skinning = false;
//	}
//	else {
//		for (uint i = 0; i < m_vertexBoneData.size(); i++) {
//			m_vertexBoneData[i].RestoreBoneData();
//		}
//		glBindBuffer(GL_ARRAY_BUFFER, m_Buffers[BONE_VB]);
//		glBufferSubData(GL_ARRAY_BUFFER, m_vertexArrayBytes - vertexBoneDataBytes, vertexBoneDataBytes, &m_vertexBoneData);
//		m_Skinning = true;
//	}
//}
void VertexBoneData::RestoreBoneData()
{
	for (uint i = 0; i < ARRAY_SIZE_IN_ELEMENTS(IDs); i++)
	{
		Weights[i] = OldWeights[i];
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

	if (m_boneMap.find(NodeName) != m_boneMap.end()) {
		uint BoneIndex = m_boneMap[NodeName];
		m_boneInfo[BoneIndex].FinalTransformation = m_GlobalInverseTransform * GlobalTransformation * m_boneInfo[BoneIndex].BoneOffset;
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

	Transforms.resize(m_numBones);

	for (uint i = 0; i < m_numBones; i++) {
		Transforms[i] = m_boneInfo[i].FinalTransformation;
	}
}
void SkinnedMesh::PrintInfo() const
{
	cout << endl;
	PrintSceneInfo();
	cout << "Model nodes:" << endl;
	PrintNodeHierarchy(m_pScene->mRootNode);	
	cout << "Bone matching:" << endl;
	PrintNodeMatching(m_pScene->mRootNode);	
}
void SkinnedMesh::PrintSceneInfo() const
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
	cout << "Total: Vertices:" << m_numVertices;
	cout << " Bones:" << m_numBones;
	cout << " KBones:" << m_numKBones;
	cout << endl;
}
void SkinnedMesh::PrintNodeHierarchy(const aiNode* pNode) const
{
	static uint counter = 0;
	if (!pNode->mParent) counter = 0;
	string NodeName(pNode->mName.data);
	string ParentName(pNode->mParent ? pNode->mParent->mName.data : "Root");
	cout << "Node " << setw(2) << counter << ": " << setw(20) << left << NodeName << setw(10) << left;
	if (m_kboneMap.find(NodeName) != m_kboneMap.end()) cout << "(KBone)";
	else if (m_boneMap.find(NodeName) != m_boneMap.end()) cout << " (Bone)";
	else cout << "";
	counter++;
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
void SkinnedMesh::PrintNodeMatching(const aiNode* pNode) const
{
	string nodeName(pNode->mName.data);
	const auto& kit = m_kboneMap.find(nodeName);
	if (kit != m_kboneMap.end()) {		
		uint kboneIndex = kit->second;
		cout << setw(20) << left << nodeName << " -> " << kboneIndex << endl;
	}
	for (uint i = 0; i < pNode->mNumChildren; i++) {
		PrintNodeMatching(pNode->mChildren[i]);
	}
}
void SkinnedMesh::GetBoneTransforms(vector<Matrix4f>& Transforms)
{
	TraverseNodeHierarchy(m_pScene->mRootNode, Matrix4f::Identity());
	Transforms.resize(m_numBones);
	for (uint i = 0; i < m_numBones; i++) {
		Transforms[i] = m_boneInfo[i].FinalTransformation;
	}
}
void SkinnedMesh::TraverseNodeHierarchy(const aiNode* pNode, const Matrix4f& P)
{
	static int counter;
	string NodeName(pNode->mName.data);
	if (NodeName == "Hips") counter = 0;
	Matrix4f L(pNode->mTransformation);
	Matrix4f G;
	QQuaternion q;
	stringstream sso;
	sso << endl << "Node name " << counter << ": " << NodeName << endl;
	auto it = m_boneMap.find(NodeName);
	if (m_Parameters[6] || (it == m_boneMap.end())){
		// bind pose on
		if (m_Parameters[1]) {
			q = (m_Parameters[2]) ? L.ExtractQuaternion1() : L.ExtractQuaternion2();
			sso << "q rel from model " << ": " << printQuaternion1(q) << printQuaternion2(q) << printQuaternion3(q) << endl;
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
		if (it != m_boneMap.end()) {
			uint i = it->second;
			m_boneInfo[i].FinalTransformation = (m_Parameters[7] ? m_boneInfo[i].BoneOffset : G * m_boneInfo[i].BoneOffset);
		}
	}
	else {
		uint i = it->second;
		auto kit = m_kboneMap.find(NodeName);
		uint c;
		if (kit != m_kboneMap.end() && (c = kit->second) != INVALID_JOINT_ID) {			
			q = m_pKBones[c].Orientation;
			sso << "q abs from kinect (" << m_pKBones[c].name << "): " << printQuaternion1(q) << printQuaternion2(q) << printQuaternion3(q) << endl;
			m_absQuats[i] = q;
			if (NodeName == "Hips") {
				m_relQuats[i] = m_absQuats[i];				
			}
			else {
				uint p = m_boneMap[pNode->mParent->mName.data];
				m_relQuats[i] = m_Parameters[4] ? m_absQuats[i]*m_absQuats[p].inverted(): m_absQuats[p].inverted()*m_absQuats[i];
				q = m_absQuats[p];
				sso << "q abs parent: " << printQuaternion1(q) << printQuaternion2(q) << printQuaternion3(q) << endl;
				q = m_absQuats[p].inverted();
				sso << "q abs parent inverted: " << printQuaternion1(q) << printQuaternion2(q) << printQuaternion3(q) << endl;
			}
			
			sso << "q relative: " << printQuaternion1(m_relQuats[i]) << printQuaternion2(m_relQuats[i]) << printQuaternion3(m_relQuats[i]) << endl;
			sso << "q absolute: " << printQuaternion1(m_absQuats[i]) << printQuaternion2(m_absQuats[i]) << printQuaternion3(m_absQuats[i]) << endl;
			Matrix4f R = m_Parameters[3] ? Matrix4f(m_relQuats[i], true) : Matrix4f(m_relQuats[i], false);
			Matrix4f T = L.GetTranslationPart();
			L = T*R;
			G = P * L;
			sso << "Rotation matrix (local) from q:" << endl << R;
			sso << "Translation matrix (local) from model:" << endl << T;
			sso << "Local transformation:" << endl << L;
			sso << "Global transformation:" << endl << G;
			m_boneInfo[i].FinalTransformation = m_Parameters[7] ? m_boneInfo[i].BoneOffset: G * m_boneInfo[i].BoneOffset;
			sso << "Final Transformation:" << endl << m_boneInfo[i].FinalTransformation;
			m_BoneTransformInfo[counter] = sso.str();
			sso.clear();
			counter++;
		}else{ // bones or kbones with invalid id
			q = (m_Parameters[2]) ? L.ExtractQuaternion1() : L.ExtractQuaternion2();
			sso << "q rel from model " << ": " << printQuaternion1(q) << printQuaternion2(q) << printQuaternion3(q) << endl;
			m_relQuats[i] = q;
			if (NodeName == "Hips") {
				m_absQuats[i] = m_relQuats[i];
			}
			else {
				uint p = m_boneMap[pNode->mParent->mName.data];
				m_absQuats[i] = m_Parameters[5] ? m_absQuats[p] * m_relQuats[i] : m_relQuats[i] * m_absQuats[p];
				q = m_absQuats[p];
				sso << "q rel parent: " << printQuaternion1(q) << printQuaternion2(q) << printQuaternion3(q) << endl;
				q = m_absQuats[p].inverted();
				sso << "q rel parent inverted: " << printQuaternion1(q) << printQuaternion2(q) << printQuaternion3(q) << endl;
			}
			sso << "q relative: " << printQuaternion1(m_relQuats[i]) << printQuaternion2(m_relQuats[i]) << printQuaternion3(m_relQuats[i]) << endl;
			sso << "q absolute: " << printQuaternion1(m_absQuats[i]) << printQuaternion2(m_absQuats[i]) << printQuaternion3(m_absQuats[i]) << endl;
			Matrix4f R = m_Parameters[3] ? Matrix4f(m_relQuats[i], true) : Matrix4f(m_relQuats[i], false);
			Matrix4f T = L.GetTranslationPart();
			L = m_Parameters[1] ? Matrix4f(pNode->mTransformation) : T*R;
			G = P * L;
			sso << "Rotation matrix (local) from q:" << endl << R;
			sso << "Translation matrix (local) from model:" << endl << T;
			sso << "Local transformation:" << endl << L;			
			sso << "Global transformation:" << endl << G;
			m_boneInfo[i].FinalTransformation = m_Parameters[7] ? m_boneInfo[i].BoneOffset : G * m_boneInfo[i].BoneOffset;
			sso << "Final Transformation:" << endl << m_boneInfo[i].FinalTransformation;
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
void SkinnedMesh::initBoneMapping()
{
	uint counter = 0;
	// core
	m_boneMap["Hips"]					= counter; counter++;
	m_boneMap["LowerBack"]				= counter; counter++;
	m_boneMap["Spine"]					= counter; counter++;
	m_boneMap["Spine1"]					= counter; counter++;
	m_boneMap["Neck"]					= counter; counter++;
	m_boneMap["Neck1"]					= counter; counter++;
	m_boneMap["Head"]					= counter; counter++;											  
	// legs									  
	m_boneMap["LHipJoint"]				= counter; counter++;
	m_boneMap["RHipJoint"]				= counter; counter++;
	m_boneMap["LeftUpLeg"]				= counter; counter++;
	m_boneMap["RightUpLeg"]				= counter; counter++;
	m_boneMap["LeftLeg"]				= counter; counter++;
	m_boneMap["RightLeg"]				= counter; counter++;
	m_boneMap["LeftFoot"]				= counter; counter++;
	m_boneMap["RightFoot"]				= counter; counter++;
	m_boneMap["LeftToeBase"]			= counter; counter++;
	m_boneMap["RightToeBase"]			= counter; counter++;											  
	// arms									  
	m_boneMap["LeftShoulder"]			= counter; counter++;
	m_boneMap["RightShoulder"]			= counter; counter++;
	m_boneMap["LeftArm"]				= counter; counter++;
	m_boneMap["RightArm"]				= counter; counter++;
	m_boneMap["LeftForeArm"]			= counter; counter++;
	m_boneMap["RightForeArm"]			= counter; counter++;
	m_boneMap["LeftHand"]				= counter; counter++;
	m_boneMap["RightHand"]				= counter; counter++;
	m_boneMap["LeftThumb"]				= counter; counter++;
	m_boneMap["RightThumb"]				= counter; counter++;
	m_boneMap["LeftFingerBase"]			= counter; counter++;
	m_boneMap["RightFingerBase"]	    = counter; counter++;
	m_boneMap["LeftHandFinger1"]	    = counter; counter++;
	m_boneMap["RightHandFinger1"]		= counter; counter++;
	m_numBones = m_boneMap.size();
	m_boneInfo.resize(m_numBones);
}
void SkinnedMesh::initKBoneMapping()
{
	// JointType SpineBase, Head, HandTipLeft/Right, ThumbLeft/Right, FootLeft/Right are always q(0,0,0,0)
	// core
	m_kboneMap["Hips"]					= JointType_SpineMid; 
	//m_kboneMap["LowerBack"]			= INVALID_JOINT_ID;
	m_kboneMap["Spine"]					= JointType_SpineShoulder;
	//m_kboneMap["Spine1"]				= INVALID_JOINT_ID;
	m_kboneMap["Neck"]					= JointType_Neck;
	//m_kboneMap["Neck1"]				= INVALID_JOINT_ID;
	//m_kboneMap["Head"]				= INVALID_JOINT_ID;

	// legs
	m_kboneMap["LHipJoint"]				= JointType_HipLeft; 
	m_kboneMap["RHipJoint"]				= JointType_HipRight; 
	m_kboneMap["LeftUpLeg"]				= JointType_KneeLeft;
	m_kboneMap["RightUpLeg"]			= JointType_KneeRight;
	m_kboneMap["LeftLeg"]				= JointType_AnkleLeft;
	m_kboneMap["RightLeg"]				= JointType_AnkleRight;
	//m_kboneMap["LeftFoot"]			= INVALID_JOINT_ID;
	//m_kboneMap["RightFoot"]			= INVALID_JOINT_ID;
	//m_kboneMap["LeftToeBase"]			= INVALID_JOINT_ID;
	//m_kboneMap["RightToeBase"]		= INVALID_JOINT_ID;
	
	// arms
	m_kboneMap["LeftShoulder"]			= JointType_ShoulderLeft;
	m_kboneMap["RightShoulder"]			= JointType_ShoulderRight;
	m_kboneMap["LeftArm"]				= JointType_ElbowLeft;
	m_kboneMap["RightArm"]				= JointType_ElbowRight;
	m_kboneMap["LeftForeArm"]			= JointType_WristLeft;
	m_kboneMap["RightForeArm"]			= JointType_WristRight;
	m_kboneMap["LeftHand"]				= JointType_HandLeft;
	m_kboneMap["RightHand"]				= JointType_HandRight;
	//m_kboneMap["LeftThumb"]			= INVALID_JOINT_ID;
	//m_kboneMap["RightThumb"]			= INVALID_JOINT_ID;
	//m_kboneMap["LeftFingerBase"]		= INVALID_JOINT_ID;
	//m_kboneMap["RightFingerBase"]	    = INVALID_JOINT_ID;
	//m_kboneMap["LeftHandFinger1"]	    = INVALID_JOINT_ID;
	//m_kboneMap["RightHandFinger1"]	= INVALID_JOINT_ID;
	m_numKBones = m_kboneMap.size();
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
//void SkinnedMesh::NextJoint(int step)
//{
//	if (step < 0) {
//		if (m_activeBone == m_boneMap.begin()) m_activeBone = m_boneMap.end();
//		m_activeBone--;
//	}
//	if (step > 0) {
//		m_activeBone++;
//		if (m_activeBone == m_boneMap.end()) m_activeBone = m_boneMap.begin();
//	}
//	uint i = m_activeBone->second;// m_BoneMapIterator->second;
//	cout << "Active joint " << setw(2) << m_activeBone->second << ": " << setw(15) << m_activeBone->first;
//	cout << " Visible:" << m_boneInfo[i].Visible;
//	cout << " Offset:" << m_boneInfo[i].Offset;
//	cout << " BindPose:" << m_boneInfo[i].BindPose;
//	cout << endl;
//	cout << m_boneMap.size() << endl;
//}
void SkinnedMesh::FlipParameter(uint i)
{
	m_Parameters[i].flip();
	cout << "Parameter " << i << (m_Parameters[i] ? " ON: " : " OFF: ");
	cout << (m_Parameters[i] ? m_ParametersStringTrue[i] : m_ParametersStringFalse[i]) << endl;
}
void SkinnedMesh::PrintParameters() const
{
	for (uint i = 0; i < NUM_PARAMETERS; i++) {
		cout << "Parameter " << i << (m_Parameters[i] ? " ON: " : " OFF: ");
		cout <<(m_Parameters[i] ? m_ParametersStringTrue[i] : m_ParametersStringFalse[i]) << endl;
	}
}
bool SkinnedMesh::boneVisibility(uint boneIndex) const
{
	return m_boneInfo[boneIndex].Visible;
}
uint SkinnedMesh::findBoneId(const QString &boneName) const
{
	string name(boneName.toLocal8Bit());
	const auto& it = m_boneMap.find(name);
	if (it == m_boneMap.cend()) {
		cerr << "std::find could not locate bone " << name << endl;
		return m_numBones;
	}
	return it->second;
}
bool SkinnedMesh::boneVisibility(const QString &boneName) const
{
	return m_boneInfo[findBoneId(boneName)].Visible;
}
void SkinnedMesh::setBoneVisibility(const QString &boneName, bool state)
{
	m_boneInfo[findBoneId(boneName)].Visible = state;
}
QString SkinnedMesh::boneTransformInfo(const QString& boneName) const
{
	return QString::fromLocal8Bit(m_BoneTransformInfo[findBoneId(boneName)].c_str());
}
void SkinnedMesh::flipBonesVisibility()
{
	for (uint i = 0; i < m_boneInfo.size(); i++) m_boneInfo[i].Visible = !m_boneInfo[i].Visible;
}
vector<Vector3f>& SkinnedMesh::positions()
{
	return m_positions;
}
vector<Vector3f>& SkinnedMesh::normals()
{
	return m_normals;
}
QVector<QVector2D>& SkinnedMesh::texCoords()
{
	return m_texCoords;
}
vector<VertexBoneData>& SkinnedMesh::vertexBoneData()
{
	return m_vertexBoneData;
}
vector<uint>& SkinnedMesh::indices()
{
	return m_indices;
}
vector<MeshEntry>& SkinnedMesh::entries()
{
	return m_entries;
}
uint SkinnedMesh::numBones() const
{
	return m_numBones;
}
const map<string, uint>& SkinnedMesh::Bones() const
{
	return m_boneMap;
}


