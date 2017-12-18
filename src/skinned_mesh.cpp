// Own
#include "skinned_mesh.h"

// Qt
#include <QtGui\QVector2D>
#include <QtGui\QImage>
#include <QtGui\QMatrix4x4>

// Standard C/C++
#include <cassert>
#include <sstream>

SkinnedMesh::SkinnedMesh()
{
	Clear();
	m_SuccessfullyLoaded = false;
	m_pScene = NULL;
	LoadMesh("cmu_test");
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
	m_images.clear();

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
	if (m_SuccessfullyLoaded) PrintInfo();

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

	m_boneTransformInfo.resize(m_numBones);
	m_controlVecs.resize(m_numBones);
	m_controlQuats.resize(m_numBones);
	m_controlMats.resize(m_numBones);
	m_correctionVecs.resize(m_numBones);
	m_correctionQuats.resize(m_numBones);
	m_correctionMats.resize(m_numBones);

	m_hasCoordinates.resize(m_numBones);
	m_boneInfo.resize(m_numBones);

	initLocalMatrices(m_pScene->mRootNode);
	initCorrectionQuats();
	initCorrectedMatrices();
	initCoordinates();
	initImages(m_pScene, Filename);

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
			BoneIndex = m_numBones;
			m_numBones++;
			BoneInfo bi;
			m_boneInfo.push_back(bi);
		}
		else {
			BoneIndex = m_boneMap[BoneName];
		}

		m_boneInfo[BoneIndex].offset = pMesh->mBones[i]->mOffsetMatrix;
		m_boneMap[BoneName] = BoneIndex;

		for (uint j = 0; j < pMesh->mBones[i]->mNumWeights; j++) {
			uint VertexID = m_entries[MeshIndex].BaseVertex + pMesh->mBones[i]->mWeights[j].mVertexId;
			float Weight = pMesh->mBones[i]->mWeights[j].mWeight;
			Bones[VertexID].AddBoneData(BoneIndex, Weight);
		}
	}
}
bool SkinnedMesh::initImages(const aiScene* pScene, const string& Filename)
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

	bool ret;

	// Initialize the materials
	for (uint i = 0; i < pScene->mNumMaterials; i++) {
		ret = false;
		const aiMaterial* pMaterial = pScene->mMaterials[i];
		if (pMaterial->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
			aiString Path;
			if (pMaterial->GetTexture(aiTextureType_DIFFUSE, 0, &Path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS) {
				string p(Path.data);
				if (p.substr(0, 2) == ".\\") {
					p = p.substr(2, p.size() - 2);
				}
				string foolPath(Dir + "\\" + p);
				QString fullPath = QString::fromLocal8Bit(foolPath.c_str());
				cout << "Loading image: " << foolPath << endl;
				m_images.push_back(QImage(QString(fullPath)));
				ret = true;
			}
		}
	}
	return ret;
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
		m_boneInfo[BoneIndex].final = m_GlobalInverseTransform * GlobalTransformation * m_boneInfo[BoneIndex].offset;
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
		Transforms[i] = m_boneInfo[i].final;
	}
}
void SkinnedMesh::PrintInfo() const
{
	if (m_pScene) {
		cout << endl;
		PrintSceneInfo();
		if (m_pScene->mRootNode) {
			cout << endl;
			cout << "Model node hierarchy:" << endl;
			PrintNodeHierarchy(m_pScene->mRootNode);
			cout << endl;
			cout << "Model bone -> Kinect joint id matching:" << endl;
			PrintNodeMatching(m_pScene->mRootNode);
		}
	}
}
void SkinnedMesh::PrintSceneInfo() const
{
	cout << "Scene info:" << endl;
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
	cout << endl;
}
void SkinnedMesh::PrintNodeHierarchy(const aiNode* pNode) const
{
	static uint counter = 0;
	if (!pNode->mParent) counter = 0;
	string NodeName(pNode->mName.data);
	string ParentName(pNode->mParent ? pNode->mParent->mName.data : "Root");
	cout << "Node " << setw(2) << left << counter << ": " << setw(25) << left << NodeName << setw(10) << left;
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
	traverseNodeHierarchy(m_pScene->mRootNode, Matrix4f::Identity());
	Transforms.resize(m_numBones);
	for (uint i = 0; i < m_numBones; i++) {
		Transforms[i] = m_boneInfo[i].final;
	}
}
void SkinnedMesh::initBoneMapping()
{
	uint counter = 0;
	// core
	m_boneMap["Hips"] = counter; counter++;
	m_boneMap["LowerBack"] = counter; counter++;
	m_boneMap["Spine"] = counter; counter++;
	m_boneMap["Spine1"] = counter; counter++;
	m_boneMap["Neck"] = counter; counter++;
	m_boneMap["Neck1"] = counter; counter++;
	m_boneMap["Head"] = counter; counter++;
	// legs									  
	m_boneMap["LHipJoint"] = counter; counter++;
	m_boneMap["RHipJoint"] = counter; counter++;
	m_boneMap["LeftUpLeg"] = counter; counter++;
	m_boneMap["RightUpLeg"] = counter; counter++;
	m_boneMap["LeftLeg"] = counter; counter++;
	m_boneMap["RightLeg"] = counter; counter++;
	m_boneMap["LeftFoot"] = counter; counter++;
	m_boneMap["RightFoot"] = counter; counter++;
	m_boneMap["LeftToeBase"] = counter; counter++;
	m_boneMap["RightToeBase"] = counter; counter++;
	// arms									  
	m_boneMap["LeftShoulder"] = counter; counter++;
	m_boneMap["RightShoulder"] = counter; counter++;
	m_boneMap["LeftArm"] = counter; counter++;
	m_boneMap["RightArm"] = counter; counter++;
	m_boneMap["LeftForeArm"] = counter; counter++;
	m_boneMap["RightForeArm"] = counter; counter++;
	m_boneMap["LeftHand"] = counter; counter++;
	m_boneMap["RightHand"] = counter; counter++;
	m_boneMap["LeftThumb"] = counter; counter++;
	m_boneMap["RightThumb"] = counter; counter++;
	m_boneMap["LeftFingerBase"] = counter; counter++;
	m_boneMap["RightFingerBase"] = counter; counter++;
	m_boneMap["LeftHandFinger1"] = counter; counter++;
	m_boneMap["RightHandFinger1"] = counter; counter++;
	m_numBones = m_boneMap.size();
	m_boneInfo.resize(m_numBones);
}
void SkinnedMesh::initLocalMatrices(const aiNode* node)
{
	const auto& it = m_boneMap.find(node->mName.data);
	if (it != m_boneMap.cend()) {
		m_boneInfo[it->second].local = Matrix4f(node->mTransformation);
	}
	for (uint i = 0; i < node->mNumChildren; i++) {
		initLocalMatrices(node->mChildren[i]);
	}
}
void SkinnedMesh::initCorrectedMatrices()
{
	for (int i = 0; i < m_numBones; i++) {
		m_correctionMats[i].InitRotateTransform2(m_correctionQuats[i]);
		m_boneInfo[i].corrected = m_correctionMats[i] * m_boneInfo[i].local;
	}
}
void SkinnedMesh::initCorrectionQuats() // OpenSim crash if calling fromEulerAngles from instance
{
	QQuaternion q; // identity quaternion
	for (auto& p: m_boneMap) {
		m_correctionQuats[p.second] = q;
	}
	m_correctionQuats[findBoneId("Hips")            ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LowerBack")       ] = QQuaternion::fromEulerAngles(0, 0, 0); 
	m_correctionQuats[findBoneId("Spine")           ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("Spine1")          ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("Neck")            ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("Neck1")           ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("Head")            ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LHipJoint")       ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("RHipJoint")       ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LeftUpLeg")       ] = QQuaternion::fromEulerAngles(0, 0, 0); // all 10 ?
	m_correctionQuats[findBoneId("RightUpLeg")      ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LeftLeg")         ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("RightLeg")        ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LeftFoot")        ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("RightFoot")       ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LeftToeBase")     ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("RightToeBase")    ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LeftShoulder")    ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("RightShoulder")   ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LeftArm")         ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("RightArm")        ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LeftForeArm")     ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("RightForeArm")    ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LeftHand")        ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("RightHand")       ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LeftThumb")       ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("RightThumb")      ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LeftFingerBase")  ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("RightFingerBase") ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("LeftHandFinger1") ] = QQuaternion::fromEulerAngles(0, 0, 0);
	m_correctionQuats[findBoneId("RightHandFinger1")] = QQuaternion::fromEulerAngles(0, 0, 0);
}
void SkinnedMesh::initCoordinates()
{
	m_modelCoordinates[hip_flexion_l] = 0.f; 
	m_modelCoordinates[hip_adduction_l] = 0.f; // LHipJoint -z or LeftUpLeg -x 
	m_modelCoordinates[hip_rotation_l] = 0.f;
	m_modelCoordinates[knee_angle_l] = 0.f;
	m_hasCoordinates[m_boneMap.find("RightUpLeg")->second] = true;
}
QVector3D SkinnedMesh::coordinateAngles(uint i)
{
	if (i == m_boneMap.find("Hips")->second) {
		return QVector3D(0, 0, 0);
	}
	else if (i == m_boneMap.find("LHipJoint")->second) {
		return QVector3D(0, m_modelCoordinates[hip_flexion_l], m_modelCoordinates[hip_adduction_l]);
	}
	else if (i == m_boneMap.find("LeftUpLeg")->second){
		return QVector3D(0, 0, 0);
	}
	else if (i == m_boneMap.find("LeftLeg")->second) {
		return QVector3D(0, m_modelCoordinates[hip_rotation_l], 0);
	}
	else {
		return QVector3D(0, 0, 0);
	}
}
void SkinnedMesh::traverseNodeHierarchy(const aiNode* pNode, const Matrix4f& P)
{
	QString nodeName(pNode->mName.data);
	Matrix4f L(pNode->mTransformation);
	Matrix4f G;
	const auto& it = m_boneMap.find(nodeName.toStdString());
	if (it == m_boneMap.end()){ // is not a bone
		G = P * L;
	}
	else { // is a bone
		uint i = it->second;
		// #opt make the following static?
		QString qs;
		QTextStream qts(&qs);
		QQuaternion q;
		qts << "\n" << nodeName << " Index=" << i << endl;

		q = L.ExtractQuaternion2();
		qts << "Local quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Local transformation:\n" << toString(L);
		
		q = m_correctionQuats[i];
		qts << "Correction quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Correction transformation\n" << toString(m_correctionMats[i]);
		
		q = m_boneInfo[i].local.ExtractQuaternion2();
		qts << "Corrected quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Corrected transformation:\n" << toString(m_boneInfo[i].local);
		
		q = m_controlQuats[i];
		qts << "Control quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Control transformation:\n" << toString(m_controlMats[i]);
		
		QVector3D angles = coordinateAngles(i);
		qts << "Angles from coordinates: " << toStringCartesian(angles) << endl;
		q = QQuaternion::fromEulerAngles(angles); // z->x->y
		qts << "Quaternion from angles: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;		
		
		Matrix4f opensimRot, controlRot, correctedLocal; // initialized as identity
		if (m_parameters[1]) correctedLocal = m_boneInfo[i].corrected; else correctedLocal = m_boneInfo[i].local;
		if (m_parameters[2]) opensimRot = Matrix4f(q, false);
		if (m_parameters[3]) controlRot = m_controlMats[i];

		if (m_parameters[4]) G = P * correctedLocal * opensimRot * controlRot;
		else G = P * controlRot * opensimRot * correctedLocal;
		m_boneInfo[i].final = G * m_boneInfo[i].offset;
		qts << "Bone Offset:\n" << toString(m_boneInfo[i].offset);
		qts << "Global transformation:\n" << toString(G);
		qts << "Final Transformation:\n" << toString(m_boneInfo[i].final);
		qts << flush;
		m_boneTransformInfo[findBoneId(nodeName)] = qs;
	}

	for (uint i = 0; i < pNode->mNumChildren; i++) {
		traverseNodeHierarchy(pNode->mChildren[i], G);
	}
}
float SkinnedMesh::boneRotationX(const QString &boneName) const
{
	return m_boneInfo[findBoneId(boneName)].xRot;
}
float SkinnedMesh::boneRotationY(const QString &boneName) const
{
	return m_boneInfo[findBoneId(boneName)].yRot;
}
float SkinnedMesh::boneRotationZ(const QString &boneName) const
{
	return m_boneInfo[findBoneId(boneName)].zRot;
}
void SkinnedMesh::setBoneRotationX(const QString &boneName, float value)
{
	uint boneId = findBoneId(boneName);
	assert(boneId < m_boneInfo.size());
	BoneInfo &bi = m_boneInfo[boneId];
	QQuaternion q = QQuaternion::fromEulerAngles(bi.xRot = value, bi.yRot, bi.zRot);
	m_controlMats[boneId].InitRotateTransform2(q);
	//m_controlMats[boneId].InitRotateTransform(bi.xRot = value, bi.yRot, bi.zRot);
}
void SkinnedMesh::setBoneRotationY(const QString &boneName, float value)
{
	uint boneId = findBoneId(boneName);
	assert(boneId < m_boneInfo.size());
	BoneInfo &bi = m_boneInfo[boneId];
	QQuaternion q = QQuaternion::fromEulerAngles(bi.xRot, bi.yRot = value, bi.zRot);
	m_controlMats[boneId].InitRotateTransform2(q);
	//m_controlMats[boneId].InitRotateTransform(bi.xRot, bi.yRot = value, bi.zRot);
}
void SkinnedMesh::setBoneRotationZ(const QString &boneName, float value)
{
	uint boneId = findBoneId(boneName);
	assert(boneId < m_boneInfo.size());
	BoneInfo &bi = m_boneInfo[boneId];
	QQuaternion q = QQuaternion::fromEulerAngles(bi.xRot, bi.yRot, bi.zRot = value);
	m_controlMats[boneId].InitRotateTransform2(q);
	//m_controlMats[boneId].InitRotateTransform(bi.xRot, bi.yRot, bi.zRot = value);
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
void SkinnedMesh::flipParameter(uint i)
{
	m_parameters[i].flip();
	cout << "Parameter " << i << ": " << m_parameterInfo[i] << (m_parameters[i] ? " ON" : " OFF") << endl;
}
void SkinnedMesh::PrintParameters() const
{
	for (uint i = 0; i < NUM_PARAMETERS; i++) {
		cout << "Parameter " << i << ": " << m_parameterInfo[i] << (m_parameters[i] ? " ON" : " OFF") << endl;
	}
}
uint SkinnedMesh::findBoneId(const QString &boneName) const
{
	string name(boneName.toStdString()); // #? use toStdString() instead of toLocal8Bit()
	const auto& it = m_boneMap.find(name);	
	if (it == m_boneMap.cend()) {
		cerr << "std::find could not locate bone " << name << endl;
		return m_numBones;
	}
	return it->second;
}
bool SkinnedMesh::boneVisibility(uint boneIndex) const
{
	return m_boneInfo[boneIndex].visible;
}
bool SkinnedMesh::boneVisibility(const QString &boneName) const
{
	return m_boneInfo[findBoneId(boneName)].visible;
}
void SkinnedMesh::setBoneVisibility(uint boneIndex, bool state)
{
	m_boneInfo[boneIndex].visible = state;
}
void SkinnedMesh::setBoneVisibility(const QString &boneName, bool state)
{
	m_boneInfo[findBoneId(boneName)].visible = state;
}
QString SkinnedMesh::boneTransformInfo(const QString& boneName) const
{
	return m_boneTransformInfo[findBoneId(boneName)];
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
vector<QImage>& SkinnedMesh::images()
{
	return m_images;
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


