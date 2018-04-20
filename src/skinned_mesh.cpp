// Own
#include "skinned_mesh.h"

// Qt
#include <QtGui\QVector2D>
#include <QtGui\QImage>
#include <QtGui\QMatrix4x4>

// Standard C/C++
#include <cassert>
#include <sstream>
#include <iomanip>

SkinnedMesh::SkinnedMesh()
{
	cout << "SkinnedMesh constructor start." << endl;
	clear();
	m_SuccessfullyLoaded = false;
	m_pScene = NULL;
	loadMesh("cmu");
	printInfo();
	loadMotion("motion_1.mot");
	cout << "SkinnedMesh constructor end." << endl;
}
SkinnedMesh::~SkinnedMesh()
{
    clear();
}
// delete container contents and resize to 0
void SkinnedMesh::clear()
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
bool SkinnedMesh::loadMesh(const string& basename)
{
	clear();

    bool Ret = false;
	string filename = "models/" + basename + ".dae";
    m_pScene = m_Importer.ReadFile(filename.c_str(), ASSIMP_LOAD_FLAGS);    
    if (m_pScene) {  
        Ret = initFromScene(m_pScene, filename);
    }
    else {
        printf("Error parsing '%s': '%s'\n", filename.c_str(), m_Importer.GetErrorString());
    }
	
	m_SuccessfullyLoaded = Ret;
	if (m_SuccessfullyLoaded) {
		cout << "Loaded SkinnedMesh from " << filename << endl;
	}
    return Ret;
}
bool SkinnedMesh::initFromScene(const aiScene* pScene, const string& Filename)
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
		initMesh(i, paiMesh);
	}

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
void SkinnedMesh::initMesh(uint MeshIndex, const aiMesh* paiMesh)
{
	const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);

	// Populate the vertex attribute vectors
	for (uint i = 0; i < paiMesh->mNumVertices; i++) {
		const aiVector3D* pPos = &(paiMesh->mVertices[i]);
		const aiVector3D* pNormal = &(paiMesh->mNormals[i]);
		const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &Zero3D;

		m_positions.push_back(QVector3D(pPos->x, pPos->y, pPos->z));
		m_normals.push_back(QVector3D(pNormal->x, pNormal->y, pNormal->z));
		m_texCoords.push_back(QVector2D(pTexCoord->x, pTexCoord->y));
	}

	loadBones(MeshIndex, paiMesh, m_vertexBoneData);
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
void SkinnedMesh::loadBones(uint MeshIndex, const aiMesh* pMesh, vector<VertexBoneData>& Bones)
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

		m_boneMap[BoneName] = BoneIndex;
		m_boneInfo[BoneIndex].offset = toQMatrix(pMesh->mBones[i]->mOffsetMatrix);

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
void SkinnedMesh::printInfo() const
{
	if (m_pScene) {
		cout << endl;
		printSceneInfo();
		if (m_pScene->mRootNode) {
			cout << endl;
			cout << "Model node hierarchy:" << endl;
			printNodeHierarchy(m_pScene->mRootNode);
			cout << endl;
		}
	}
}
void SkinnedMesh::printSceneInfo() const
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
	cout << "Total: Vertices:" << m_numVertices << " Bones:" << m_numBones;
	cout << endl;
}
void SkinnedMesh::printNodeHierarchy(const aiNode* pNode) const
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
		printNodeHierarchy(pNode->mChildren[i]);
	}
	
}
void SkinnedMesh::getBoneTransforms(vector<QMatrix4x4>& transforms)
{
	traverseNodeHierarchy(m_pScene->mRootNode, QMatrix4x4());
	transforms.resize(m_numBones);
	for (uint i = 0; i < m_numBones; i++) {
		transforms[i] = m_boneInfo[i].combined;
	}
}
void SkinnedMesh::initLocalMatrices(const aiNode* node)
{
	const auto& it = m_boneMap.find(node->mName.data);
	if (it != m_boneMap.cend()) {
		m_boneInfo[it->second].local = toQMatrix(node->mTransformation);
	}
	for (uint i = 0; i < node->mNumChildren; i++) {
		initLocalMatrices(node->mChildren[i]);
	}
}
void SkinnedMesh::initCorrectedMatrices()
{
	for (int i = 0; i < m_numBones; i++) {
		m_correctionMats[i].rotate(m_correctionQuats[i]); // #? was using aiQuat for rotation
		m_boneInfo[i].corrected =  m_boneInfo[i].local * m_correctionMats[i]; // #? maybe opposite order of multiplication
	}
}
void SkinnedMesh::initCorrectionQuats() // OpenSim crash if calling fromEulerAngles from instance
{
	QQuaternion q; // identity quaternion
	for (auto& p: m_boneMap) {
		m_correctionQuats[p.second] = q;
	}
	// left side
	// try 1
	//m_correctionQuats[findBoneId("LeftArm")]      = QQuaternion::fromEulerAngles(0, 0, -25);
	//m_correctionQuats[findBoneId("LeftForeArm")]  = QQuaternion::fromEulerAngles(0, 100, -50);
	// try 2
	m_correctionQuats[findBoneId("LeftArm")]	    = QQuaternion::fromEulerAngles(0, 0, -40); // #? z=-35
	m_correctionQuats[findBoneId("LeftForeArm")]    = QQuaternion::fromEulerAngles(0, -90, 50);
	m_correctionQuats[findBoneId("LeftUpLeg")]		= QQuaternion::fromEulerAngles(0, 0, 5);
	// right side
	m_correctionQuats[findBoneId("RightArm")]		= QQuaternion::fromEulerAngles(0, 0, 40); // #? z=35
	m_correctionQuats[findBoneId("RightForeArm")]	= QQuaternion::fromEulerAngles(0, 90, -50);
	m_correctionQuats[findBoneId("RightUpLeg")]		= QQuaternion::fromEulerAngles(0, 0, -5);
}
QQuaternion SkinnedMesh::worldRotation()
{
	QQuaternion q;
	q = QQuaternion::fromEulerAngles(0, 0, m_modelCoordinates[pelvis_tilt]);
	q *= QQuaternion::fromEulerAngles(m_modelCoordinates[pelvis_list], 0, 0);
	q *= QQuaternion::fromEulerAngles(0, m_modelCoordinates[pelvis_rotation], 0);
	q *= QQuaternion::fromEulerAngles(0, 90, 0);
	return q;
}
QVector3D SkinnedMesh::worldPosition()
{
	return QVector3D(m_modelCoordinates[pelvis_tx], m_modelCoordinates[pelvis_ty], m_modelCoordinates[pelvis_tz]);
}
double SkinnedMesh::timestamp(uint index)
{
	return m_timestamps[index];
}
void SkinnedMesh::initCoordinates()
{
	// core
	m_modelCoordinates[pelvis_tilt]		 = 0.f;
	m_modelCoordinates[pelvis_list]		 = 0.f;
	m_modelCoordinates[pelvis_rotation]  = 0.f;
	m_modelCoordinates[pelvis_tx]		 = 0.f;
	m_modelCoordinates[pelvis_ty]		 = 0.f;
	m_modelCoordinates[pelvis_tz]		 = 0.f;

	m_modelCoordinates[lumbar_extension] = 0.f; // 
	m_modelCoordinates[lumbar_bending]	 = 0.f; // 
	m_modelCoordinates[lumbar_rotation]  = 0.f; // 
	// left
	m_modelCoordinates[hip_flexion_l]    = 0.f; //  LHipJoint -y (worse)
	m_modelCoordinates[hip_adduction_l]  = 0.f; //  LHipJoint x (worse)
	m_modelCoordinates[hip_rotation_l]   = 0.f; //  LHipJoint z (worse)
	m_modelCoordinates[knee_angle_l]     = 0.f;
	m_modelCoordinates[arm_flex_l]       = 0.f; // 
	m_modelCoordinates[arm_add_l]        = 0.f; // 
	m_modelCoordinates[arm_rot_l]        = 0.f; // 
	m_modelCoordinates[elbow_flex_l]	 = 0.f;
	m_modelCoordinates[pro_sup_l]		 = 0.f;
	m_modelCoordinates[subtalar_angle_l] = 0.f; // #? not used
	m_modelCoordinates[wrist_flex_l]	 = 0.f;
	m_modelCoordinates[wrist_dev_l]		 = 0.f;
	// right
	m_modelCoordinates[hip_flexion_r]    = 0.f; //  LHipJoint -y (worse)
	m_modelCoordinates[hip_adduction_r]  = 0.f; //  LHipJoint x (worse)
	m_modelCoordinates[hip_rotation_r]   = 0.f; //  LHipJoint z (worse)
	m_modelCoordinates[knee_angle_r]     = 0.f;
	m_modelCoordinates[arm_flex_r]       = 0.f; 
	m_modelCoordinates[arm_add_r]        = 0.f;  
	m_modelCoordinates[arm_rot_r]        = 0.f; 
	m_modelCoordinates[elbow_flex_r]     = 0.f;
	m_modelCoordinates[pro_sup_r]        = 0.f;
	m_modelCoordinates[subtalar_angle_r] = 0.f; // #? not used
	m_modelCoordinates[wrist_flex_r]	 = 0.f;
	m_modelCoordinates[wrist_dev_r]		 = 0.f;
}
void SkinnedMesh::setActiveCoordinates(uint frameIndex)
{
	m_modelCoordinates = m_modelCoordinateSequence[frameIndex];
}
uint SkinnedMesh::sequenceSize()
{
	return m_modelCoordinateSequence.size();
}
void SkinnedMesh::updateCoordinates()
{
}
QQuaternion SkinnedMesh::boneOrientation(uint boneIndex)
{
	QQuaternion q;
	// core
	if (boneIndex == m_boneMap.find("LowerBack")->second) {
		q = QQuaternion::fromEulerAngles(-m_modelCoordinates[lumbar_extension], 0, 0);
		q *= QQuaternion::fromEulerAngles(0, 0, m_modelCoordinates[lumbar_bending]);
		q *= QQuaternion::fromEulerAngles(0, m_modelCoordinates[lumbar_rotation], 0);
	}
	// left side
	else if (boneIndex == m_boneMap.find("LeftUpLeg")->second) {
		q = QQuaternion::fromEulerAngles(-m_modelCoordinates[hip_flexion_l], 0, 0);
		q *= QQuaternion::fromEulerAngles(0, 0, m_modelCoordinates[hip_adduction_l]);
		q *= QQuaternion::fromEulerAngles(0, m_modelCoordinates[hip_rotation_l], 0);
	}
	else if (boneIndex == m_boneMap.find("LeftLeg")->second) {
		q = QQuaternion::fromEulerAngles(-m_modelCoordinates[knee_angle_l], 0, 0);
	}
	else if (boneIndex == m_boneMap.find("LeftFoot")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, m_modelCoordinates[subtalar_angle_l]);
	}
	else if (boneIndex == m_boneMap.find("LeftArm")->second) {
		q = QQuaternion::fromEulerAngles(m_modelCoordinates[arm_flex_l], 0, 0);
		q *= QQuaternion::fromEulerAngles(0, 0, -m_modelCoordinates[arm_add_l]);
		q *= QQuaternion::fromEulerAngles(0, m_modelCoordinates[arm_rot_l], 0);
	}
	else if (boneIndex == m_boneMap.find("LeftForeArm")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, -m_modelCoordinates[elbow_flex_l]);
		q *= QQuaternion::fromEulerAngles(0, m_modelCoordinates[pro_sup_l], 0);
	}
	else if (boneIndex == m_boneMap.find("LeftHand")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, -m_modelCoordinates[wrist_flex_l]);
		q *= QQuaternion::fromEulerAngles(-m_modelCoordinates[wrist_dev_l], 0, 0);
	}
	// right side
	else if (boneIndex == m_boneMap.find("RightUpLeg")->second) {
		q = QQuaternion::fromEulerAngles(-m_modelCoordinates[hip_flexion_r], 0, 0);
		q *= QQuaternion::fromEulerAngles(0, 0, -m_modelCoordinates[hip_adduction_r]);
		q *= QQuaternion::fromEulerAngles(0, -m_modelCoordinates[hip_rotation_r], 0);
	}
	else if (boneIndex == m_boneMap.find("RightLeg")->second) {
		q = QQuaternion::fromEulerAngles(-m_modelCoordinates[knee_angle_r], 0, 0);
	}
	else if (boneIndex == m_boneMap.find("RightFoot")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, m_modelCoordinates[subtalar_angle_r]);
	}
	else if (boneIndex == m_boneMap.find("RightArm")->second) {
		q = QQuaternion::fromEulerAngles(m_modelCoordinates[arm_flex_r], 0, 0);
		q *= QQuaternion::fromEulerAngles(0, 0, m_modelCoordinates[arm_add_r]);
		q *= QQuaternion::fromEulerAngles(0, -m_modelCoordinates[arm_rot_r], 0);
	}
	else if (boneIndex == m_boneMap.find("RightForeArm")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, m_modelCoordinates[elbow_flex_r]);
		q *= QQuaternion::fromEulerAngles(0, -m_modelCoordinates[pro_sup_r], 0);
	}
	else if (boneIndex == m_boneMap.find("RightHand")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, m_modelCoordinates[wrist_flex_r]);
		q *= QQuaternion::fromEulerAngles(-m_modelCoordinates[wrist_dev_r], 0, 0);
	}
	return q;
}
void SkinnedMesh::traverseNodeHierarchy(const aiNode* pNode, const QMatrix4x4& P)
{
	QString nodeName(pNode->mName.data);
	QMatrix4x4 L = toQMatrix(pNode->mTransformation);
	QMatrix4x4 G;
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

		q = extractQuaternion(m_boneInfo[i].local);
		qts << "Local quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Local transformation:\n" << toString(m_boneInfo[i].local);
		
		q = m_correctionQuats[i];
		qts << "Correction quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Correction transformation\n" << toString(m_correctionMats[i]);
		
		q = extractQuaternion(m_boneInfo[i].corrected);
		qts << "Corrected quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Corrected transformation:\n" << toString(m_boneInfo[i].corrected);
		
		q = m_controlQuats[i];
		qts << "Control quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Control transformation:\n" << toString(m_controlMats[i]);
		
		q = boneOrientation(i); // #todo make it a container rather than a function
		qts << "Quaternion from angles: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;		
		
		QMatrix4x4 opensimRot, controlRot, correctedLocal; // initialized as identity
		if (m_parameters[1]) correctedLocal = m_boneInfo[i].corrected; else correctedLocal = m_boneInfo[i].local;
		if (m_parameters[2]) opensimRot = fromRotation(q);
		if (m_parameters[3]) controlRot = m_controlMats[i];

		// correctedLocal next to P, control near correctedLocal
		if (m_parameters[4]) G = P * correctedLocal * controlRot * opensimRot; 
		else G = P * controlRot * opensimRot * correctedLocal;
		m_boneInfo[i].global = G;
		m_boneInfo[i].combined = G * m_boneInfo[i].offset;
		qts << "Bone Offset:\n" << toString(m_boneInfo[i].offset);
		qts << "Global transformation:\n" << toString(G);
		qts << "Final Transformation:\n" << toString(m_boneInfo[i].combined);
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
	bi.xRot = value;
	QQuaternion q;
	q  = QQuaternion::fromEulerAngles(bi.xRot, 0, 0);
	q *= QQuaternion::fromEulerAngles(0, bi.yRot, 0);
	q *= QQuaternion::fromEulerAngles(0, 0, bi.zRot);
	m_controlQuats[boneId] = q;
	m_controlMats[boneId] = fromRotation(q);
}
void SkinnedMesh::setBoneRotationY(const QString &boneName, float value)
{
	uint boneId = findBoneId(boneName);
	assert(boneId < m_boneInfo.size());
	BoneInfo &bi = m_boneInfo[boneId];
	bi.yRot = value;
	QQuaternion q;
	q = QQuaternion::fromEulerAngles(bi.xRot, 0, 0);
	q *= QQuaternion::fromEulerAngles(0, bi.yRot, 0);
	q *= QQuaternion::fromEulerAngles(0, 0, bi.zRot);
	m_controlQuats[boneId] = q;
	m_controlMats[boneId] = fromRotation(q);
}
void SkinnedMesh::setBoneRotationZ(const QString &boneName, float value)
{
	uint boneId = findBoneId(boneName);
	assert(boneId < m_boneInfo.size());
	BoneInfo &bi = m_boneInfo[boneId];
	bi.zRot = value;
	QQuaternion q;
	q = QQuaternion::fromEulerAngles(bi.xRot, 0, 0);
	q *= QQuaternion::fromEulerAngles(0, bi.yRot, 0);
	q *= QQuaternion::fromEulerAngles(0, 0, bi.zRot);
	m_controlQuats[boneId] = q;
	m_controlMats[boneId] = fromRotation(q);
}
void SkinnedMesh::flipParameter(uint i)
{
	m_parameters[i].flip();
	cout << "Parameter " << i << ": " << m_parameterInfo[i] << (m_parameters[i] ? " ON" : " OFF") << endl;
}
void SkinnedMesh::printParameters() const
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
vector<QVector3D>& SkinnedMesh::positions()
{
	return m_positions;
}
vector<QVector3D>& SkinnedMesh::normals()
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
const QMatrix4x4& SkinnedMesh::boneGlobal(uint boneIndex) const
{
	return m_boneInfo[boneIndex].global;
}
bool SkinnedMesh::loadMotion(const QString& filename)
{
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		cout << "Could not open " << filename.toStdString() << endl;
		return false;
	}

	QString line;
	QTextStream in(&file);
	QStringList list;

	uint lineCounter = 0;
	do {
		line = in.readLine();
		lineCounter++;
		list = line.split("\t");

		bool ok;
		list[0].toDouble(&ok);
		if (!ok) {
			cout << "StringList element could not be converted to double.";
			cout << " Element=" << list[0].toStdString();
			cout << " Index=0";
			cout << " Line=" << lineCounter << endl;
			continue;
		}
		m_timestamps.push_back(list[0].toDouble(&ok));
		array<float, m_numCoordinates> coords;
		for (uint i = 0; i < m_numCoordinates; i++) {
			coords[i] = list[i + 1].toFloat(&ok);
			if (!ok) {
				cout << "StringList element could not be converted to double.";
				cout << " Element=" << list[i+1].toStdString();
				cout << " Index=0";
				cout << " Line=" << lineCounter << endl;
				continue;
			}
		}
		m_modelCoordinateSequence.push_back(coords);
	} while (!line.isNull());

	/*cout << "Lines in .mot file: " << lineCounter << endl;
	cout << "Timestamp = " << m_timestamps[0] << endl;
	cout << "Coordinates: " << list.size() << endl;
	for (const auto& p : m_modelCoordinateSequence[0]) {
		cout << p << " ";
	}
	cout << endl;*/
	cout << "Loaded SkinnedMesh motion from " << filename.toStdString() << endl;
	return true;
}