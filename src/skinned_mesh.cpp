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
	// should never get here - more vertexBoneData than we have space for
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

SkinnedMesh::SkinnedMesh()
	:
	m_successfullyLoaded(false),
	m_pScene(NULL)
{
	cout << "SkinnedMesh constructor start." << endl;
	loadFromFile("cmu.dae");
	initMotionFromFile("motion.mot");
	printInfo();
	cout << "SkinnedMesh constructor end.\n" << endl;
}
SkinnedMesh::~SkinnedMesh()
{
    clear();
}
// delete container contents and resize to 0
void SkinnedMesh::clear()
{	
	m_meshEntries.clear();
	m_positions.clear();
	m_normals.clear();
	m_texCoords.clear();
	m_vertexBoneData.clear();
	m_indices.clear();
	m_images.clear();
	m_boneInfo.clear();
}
bool SkinnedMesh::loadFromFile(const string& fileName)
{
	clear();

    bool ret = false;
	string filePath = "models/" + fileName;
    m_pScene = m_Importer.ReadFile(filePath.c_str(), ASSIMP_LOAD_FLAGS);    
    if (m_pScene) {  
        ret = initFromScene(m_pScene, filePath);
    }
    else {
       cout << "Error parsing '" << filePath << "' -> '\n" << string(m_Importer.GetErrorString()) << endl;
    }
	
	m_successfullyLoaded = ret;
	if (m_successfullyLoaded) {
		cout << "Loaded SkinnedMesh from " << filePath << endl;
	}
    return ret;
}
bool SkinnedMesh::initFromScene(const aiScene* pScene, const string& filename)
{  
	// Update mesh entries
	uint numVertices = 0;
    uint numIndices = 0;
	m_meshEntries.resize(pScene->mNumMeshes);
	for (uint i = 0 ; i < m_meshEntries.size() ; i++) {
        m_meshEntries[i].materialIndex = pScene->mMeshes[i]->mMaterialIndex;        
        m_meshEntries[i].numIndices    = pScene->mMeshes[i]->mNumFaces * 3;
        m_meshEntries[i].baseVertex    = numVertices;
        m_meshEntries[i].baseIndex     = numIndices;	
        numVertices += pScene->mMeshes[i]->mNumVertices;
        numIndices  += m_meshEntries[i].numIndices;
    }
	m_numVertices = numVertices;

	// Reserve space in the vectors for the vertex attributes and indices
	m_positions.reserve(numVertices);
	m_normals.reserve(numVertices);
	m_texCoords.reserve(numVertices);
	m_indices.reserve(numIndices);
	m_vertexBoneData.resize(numVertices); // elements will be accessed without being push_backed first

	// Initialize the meshes in the scene one by one
	for (uint i = 0; i < m_meshEntries.size(); i++) {
		const aiMesh* paiMesh = pScene->mMeshes[i];
		initMesh(i, paiMesh);
	}

	m_controlQuats.resize(m_numBones);
	m_controlMats.resize(m_numBones);
	m_correctionVecs.resize(m_numBones);
	m_correctionQuats.resize(m_numBones);
	m_correctionMats.resize(m_numBones);
	m_boneInfo.resize(m_numBones);
	m_boneTransformInfo.resize(m_numBones);

	initLocalMatrices(m_pScene->mRootNode);
	initCorrectionQuats();
	initCorrectedMatrices();
	initCoordinates();
	initImages(m_pScene, filename);

	return true;
}
void SkinnedMesh::initMesh(uint meshIndex, const aiMesh* paiMesh)
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

	loadBones(meshIndex, paiMesh, m_vertexBoneData);
	checkWeights(meshIndex, paiMesh);

	// Populate the index buffer
	for (uint i = 0; i < paiMesh->mNumFaces; i++) {
		const aiFace& face = paiMesh->mFaces[i];
		assert(face.mNumIndices == 3);
		m_indices.push_back(face.mIndices[0]);
		m_indices.push_back(face.mIndices[1]);
		m_indices.push_back(face.mIndices[2]);
	}
}
void SkinnedMesh::loadBones(uint meshIndex, const aiMesh* pMesh, QVector<VertexBoneData>& vertexBoneData)
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
			uint VertexID = m_meshEntries[meshIndex].baseVertex + pMesh->mBones[i]->mWeights[j].mVertexId;
			float Weight = pMesh->mBones[i]->mWeights[j].mWeight;
			vertexBoneData[VertexID].AddBoneData(BoneIndex, Weight);
		}
	}
}
void SkinnedMesh::checkWeights(uint meshIndex, const aiMesh* pMesh) {
	bool SumNot1 = false;
	for (uint i = 0; i < pMesh->mNumVertices; i++) {
		float sum = 0;
		uint VertexID = m_meshEntries[meshIndex].baseVertex + i;
		for (uint i = 0; i < NUM_BONES_PER_VERTEX; i++) {
			sum += m_vertexBoneData[VertexID].Weights[i];
		}
		if (sum<0.99 || sum >1.01) SumNot1 = true; // Must be equal to 1
	}
	if (SumNot1) cout << "Sum of weights is not 1 for all vertices!" << endl;
}
bool SkinnedMesh::initImages(const aiScene* pScene, const string& fileName)
{
	// Extract the directory part from the file name
	string::size_type slashIndex = fileName.find_last_of("/");
	string directory;

	if (slashIndex == string::npos) {
		directory = ".";
	}
	else if (slashIndex == 0) {
		directory = "/";
	}
	else {
		directory = fileName.substr(0, slashIndex);
	}

	bool ret;
	// Initialize the materials
	for (uint i = 0; i < pScene->mNumMaterials; i++) {
		ret = false;
		const aiMaterial* pMaterial = pScene->mMaterials[i];
		if (pMaterial->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
			aiString path;
			if (pMaterial->GetTexture(aiTextureType_DIFFUSE, 0, &path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS) {
				string p(path.data);
				if (p.substr(0, 2) == ".\\") {
					p = p.substr(2, p.size() - 2);
				}
				string fullPath = string(directory + "\\" + p).c_str();
				cout << "Loading image: " << fullPath << endl;
				m_images.push_back(QImage(QString::fromStdString(fullPath)));
				ret = true;
			}
		}
	}
	return ret;
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
	cout << "Scene info: ";
	cout << " MeshEntries:"	 << setw( 3) << m_pScene->mNumMeshes;
	cout << " Materials:"	 << setw( 3) << m_pScene->mNumMaterials;
	cout << " Textures:"	 << setw( 3) << m_pScene->mNumTextures;
	cout << " Lights:"		 << setw( 3) << m_pScene->mNumLights;
	cout << " Animations:"   << setw( 3) << m_pScene->mNumAnimations;
	cout << endl;
	for (uint i = 0; i < m_pScene->mNumMeshes; i++) {
		cout << "MeshId:"	 << i;
		cout << " Name:"	 << setw(15) << m_pScene->mMeshes[i]->mName.C_Str();
		cout << " Vertices:" << setw( 6) << m_pScene->mMeshes[i]->mNumVertices;
		cout << " Faces:"	 << setw( 6) << m_pScene->mMeshes[i]->mNumFaces;
		cout << " Bones:"	 << setw( 6) << m_pScene->mMeshes[i]->mNumBones;
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
void SkinnedMesh::calculateBoneLength(const aiNode* pNode)
{
	QString nodeName(pNode->mName.data);
	const auto& it = m_boneMap.find(nodeName.toStdString());
	if (it != m_boneMap.end()) {
		//m_boneInfo[it->second].length = m_boneInfo[it->second].endPosition;
	}

	for (uint i = 0; i < pNode->mNumChildren; i++) {
		calculateBoneLength(pNode->mChildren[i]);
	}
}
void SkinnedMesh::initCorrectionVecs(const array<KLimb, NUM_LIMBS>& limbs)
{
	// init all as unit vectors
	for (auto& p : m_boneMap) {
		m_correctionVecs[p.second] = QVector3D(1.f, 1.f, 1.f);
	}
	uint i, j;
	float meshLength, kinectLength, factor;

	i = m_boneMap.find("Hips")->second;
	j = m_boneMap.find("LowerBack")->second;
	meshLength = m_boneInfo[i].endPosition.distanceToPoint(m_boneInfo[j].endPosition);
	kinectLength = limbs[JointType_SpineBase].lengthAverage;

	factor =  kinectLength / meshLength;
	cout << "MeshLength=" << meshLength << " KinectLength=" << kinectLength << " Factor=" << factor << endl;

	m_correctionVecs[m_boneMap.find("LowerBack")->second] = QVector3D(1.f, 1.f, 1.f);
	m_correctionVecs[m_boneMap.find("RightArm")->second] = QVector3D(1.f, 1.f, 1.f);
	for (int i = 0; i < m_numBones; i++) {
		m_boneInfo[i].scaling.scale(m_correctionVecs[i]);
	}
}
QVector3D SkinnedMesh::getOffset()
{
	float x = m_coordinateSequence[0][pelvis_tx];
	float y = m_coordinateSequence[0][pelvis_ty];
	float z = m_coordinateSequence[0][pelvis_tz];
	return QVector3D(x, y, z);
}
void SkinnedMesh::initCorrectionQuats() // OpenSim crash if calling fromEulerAngles from instance
{
	// init all as identity quaternions
	for (auto& p: m_boneMap) {
		m_correctionQuats[p.second] = QQuaternion();
	}

	// left side
	m_correctionQuats[findBoneId("LeftArm")]	    = QQuaternion::fromEulerAngles(0, 0, -40); // #? z=-35
	m_correctionQuats[findBoneId("LeftForeArm")]    = QQuaternion::fromEulerAngles(0, -90, 50);
	m_correctionQuats[findBoneId("LeftUpLeg")]		= QQuaternion::fromEulerAngles(0, 0, 5);
	// right side
	m_correctionQuats[findBoneId("RightArm")]		= QQuaternion::fromEulerAngles(0, 0, 40); // #? z=35
	m_correctionQuats[findBoneId("RightForeArm")]	= QQuaternion::fromEulerAngles(0, 90, -50);
	m_correctionQuats[findBoneId("RightUpLeg")]		= QQuaternion::fromEulerAngles(0, 0, -5);
}
QQuaternion SkinnedMesh::pelvisRotation()
{
	QQuaternion q;
	q = QQuaternion::fromEulerAngles(0, 0, m_activeCoordinates[pelvis_tilt]);
	q *= QQuaternion::fromEulerAngles(m_activeCoordinates[pelvis_list], 0, 0);
	q *= QQuaternion::fromEulerAngles(0, m_activeCoordinates[pelvis_rotation], 0);
	q *= QQuaternion::fromEulerAngles(0, 90, 0);
	return q;
}
QVector3D SkinnedMesh::pelvisPosition()
{
	return QVector3D(m_activeCoordinates[pelvis_tx], m_activeCoordinates[pelvis_ty], m_activeCoordinates[pelvis_tz]);
}
double SkinnedMesh::timestamp(uint index)
{
	return m_timestamps[index];
}
void SkinnedMesh::initCoordinates()
{
	// core
	m_activeCoordinates[pelvis_tilt]		 = 0.f;
	m_activeCoordinates[pelvis_list]		 = 0.f;
	m_activeCoordinates[pelvis_rotation]  = 0.f;
	m_activeCoordinates[pelvis_tx]		 = 0.f;
	m_activeCoordinates[pelvis_ty]		 = 0.f;
	m_activeCoordinates[pelvis_tz]		 = 0.f;
	m_activeCoordinates[lumbar_extension] = 0.f; // 
	m_activeCoordinates[lumbar_bending]	 = 0.f; // 
	m_activeCoordinates[lumbar_rotation]  = 0.f; // 
	// left
	m_activeCoordinates[hip_flexion_l]    = 0.f; //  LHipJoint -y (worse)
	m_activeCoordinates[hip_adduction_l]  = 0.f; //  LHipJoint x (worse)
	m_activeCoordinates[hip_rotation_l]   = 0.f; //  LHipJoint z (worse)
	m_activeCoordinates[knee_angle_l]     = 0.f;
	m_activeCoordinates[arm_flex_l]       = 0.f; // 
	m_activeCoordinates[arm_add_l]        = 0.f; // 
	m_activeCoordinates[arm_rot_l]        = 0.f; // 
	m_activeCoordinates[elbow_flex_l]	 = 0.f;
	m_activeCoordinates[pro_sup_l]		 = 0.f;
	m_activeCoordinates[subtalar_angle_l] = 0.f; // #? not used
	m_activeCoordinates[wrist_flex_l]	 = 0.f;
	m_activeCoordinates[wrist_dev_l]		 = 0.f;
	// right
	m_activeCoordinates[hip_flexion_r]    = 0.f; //  LHipJoint -y (worse)
	m_activeCoordinates[hip_adduction_r]  = 0.f; //  LHipJoint x (worse)
	m_activeCoordinates[hip_rotation_r]   = 0.f; //  LHipJoint z (worse)
	m_activeCoordinates[knee_angle_r]     = 0.f;
	m_activeCoordinates[arm_flex_r]       = 0.f; 
	m_activeCoordinates[arm_add_r]        = 0.f;  
	m_activeCoordinates[arm_rot_r]        = 0.f; 
	m_activeCoordinates[elbow_flex_r]     = 0.f;
	m_activeCoordinates[pro_sup_r]        = 0.f;
	m_activeCoordinates[subtalar_angle_r] = 0.f; // #? not used
	m_activeCoordinates[wrist_flex_r]	 = 0.f;
	m_activeCoordinates[wrist_dev_r]		 = 0.f;
}
void SkinnedMesh::setActiveCoordinates(uint frameIndex)
{
	if (frameIndex < m_coordinateSequence.size()) {
		m_activeCoordinates = m_coordinateSequence[frameIndex];
	}
	else {
		m_activeCoordinates = m_coordinateSequence.last();
	}
}
uint SkinnedMesh::sequenceSize()
{
	return m_coordinateSequence.size();
}
// #? make it a container rather than a function
QQuaternion SkinnedMesh::boneOrientation(uint boneIndex)
{
	QQuaternion q;
	// core
	if (boneIndex == m_boneMap.find("LowerBack")->second) {
		q = QQuaternion::fromEulerAngles(-m_activeCoordinates[lumbar_extension], 0, 0);
		q *= QQuaternion::fromEulerAngles(0, 0, m_activeCoordinates[lumbar_bending]);
		q *= QQuaternion::fromEulerAngles(0, m_activeCoordinates[lumbar_rotation], 0);
	}
	// left side
	else if (boneIndex == m_boneMap.find("LeftUpLeg")->second) {
		q = QQuaternion::fromEulerAngles(-m_activeCoordinates[hip_flexion_l], 0, 0);
		q *= QQuaternion::fromEulerAngles(0, 0, m_activeCoordinates[hip_adduction_l]);
		q *= QQuaternion::fromEulerAngles(0, m_activeCoordinates[hip_rotation_l], 0);
	}
	else if (boneIndex == m_boneMap.find("LeftLeg")->second) {
		q = QQuaternion::fromEulerAngles(-m_activeCoordinates[knee_angle_l], 0, 0);
	}
	else if (boneIndex == m_boneMap.find("LeftFoot")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, m_activeCoordinates[subtalar_angle_l]);
	}
	else if (boneIndex == m_boneMap.find("LeftArm")->second) {
		q = QQuaternion::fromEulerAngles(m_activeCoordinates[arm_flex_l], 0, 0);
		q *= QQuaternion::fromEulerAngles(0, 0, -m_activeCoordinates[arm_add_l]);
		q *= QQuaternion::fromEulerAngles(0, m_activeCoordinates[arm_rot_l], 0);
	}
	else if (boneIndex == m_boneMap.find("LeftForeArm")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, -m_activeCoordinates[elbow_flex_l]);
		q *= QQuaternion::fromEulerAngles(0, m_activeCoordinates[pro_sup_l], 0);
	}
	else if (boneIndex == m_boneMap.find("LeftHand")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, -m_activeCoordinates[wrist_flex_l]);
		q *= QQuaternion::fromEulerAngles(-m_activeCoordinates[wrist_dev_l], 0, 0);
	}
	// right side
	else if (boneIndex == m_boneMap.find("RightUpLeg")->second) {
		q = QQuaternion::fromEulerAngles(-m_activeCoordinates[hip_flexion_r], 0, 0);
		q *= QQuaternion::fromEulerAngles(0, 0, -m_activeCoordinates[hip_adduction_r]);
		q *= QQuaternion::fromEulerAngles(0, -m_activeCoordinates[hip_rotation_r], 0);
	}
	else if (boneIndex == m_boneMap.find("RightLeg")->second) {
		q = QQuaternion::fromEulerAngles(-m_activeCoordinates[knee_angle_r], 0, 0);
	}
	else if (boneIndex == m_boneMap.find("RightFoot")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, m_activeCoordinates[subtalar_angle_r]);
	}
	else if (boneIndex == m_boneMap.find("RightArm")->second) {
		q = QQuaternion::fromEulerAngles(m_activeCoordinates[arm_flex_r], 0, 0);
		q *= QQuaternion::fromEulerAngles(0, 0, m_activeCoordinates[arm_add_r]);
		q *= QQuaternion::fromEulerAngles(0, -m_activeCoordinates[arm_rot_r], 0);
	}
	else if (boneIndex == m_boneMap.find("RightForeArm")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, m_activeCoordinates[elbow_flex_r]);
		q *= QQuaternion::fromEulerAngles(0, -m_activeCoordinates[pro_sup_r], 0);
	}
	else if (boneIndex == m_boneMap.find("RightHand")->second) {
		q = QQuaternion::fromEulerAngles(0, 0, m_activeCoordinates[wrist_flex_r]);
		q *= QQuaternion::fromEulerAngles(-m_activeCoordinates[wrist_dev_r], 0, 0);
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

		//q = extractQuaternion(m_boneInfo[i].local);
		//qts << "Local quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Local transformation:\n" << toString(m_boneInfo[i].local);
		
		q = m_correctionQuats[i];
		qts << "Correction quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Correction transformation\n" << toString(m_correctionMats[i]);
		
		//q = extractQuaternion(m_boneInfo[i].corrected);
		//qts << "Corrected quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Corrected transformation:\n" << toString(m_boneInfo[i].corrected);
		
		q = m_controlQuats[i];
		qts << "Control quaternion: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;
		qts << "Control transformation:\n" << toString(m_controlMats[i]);
		
		q = boneOrientation(i); 
		qts << "Quaternion from angles: " << toString(q) << toStringEulerAngles(q) << toStringAxisAngle(q) << endl;		
		
		QMatrix4x4 opensimRot, controlRot, opensimScaling; // initialized as identity
		QMatrix4x4 localTransformation = m_boneInfo[i].local; 
		if (m_parameters[1]) localTransformation = m_boneInfo[i].corrected;
		if (m_parameters[2]) opensimRot = fromRotation(q);
		if (m_parameters[3]) controlRot = m_controlMats[i];
		if (m_parameters[4]) opensimScaling = m_boneInfo[i].scaling;

		// localTransformation next to P, control next to localTransformation
		G = P * localTransformation * controlRot * opensimRot * opensimScaling;


		m_boneInfo[i].global = G;
		m_boneInfo[i].combined = G * m_boneInfo[i].offset;
		qts << "Offset transformation:\n" << toString(m_boneInfo[i].offset);
		qts << "Global transformation:\n" << toString(G);
		qts << "Combined transformation:\n" << toString(m_boneInfo[i].combined);
		QVector3D positionGlobal = m_boneInfo[i].global * QVector3D(QVector4D(0.f, 0.f, 0.f, 1.f));
		m_boneInfo[i].endPosition = positionGlobal;
		qts << "Bone position (from global): " << toStringCartesian(positionGlobal);
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
	if (boneIndex > m_boneInfo.size() || boneIndex < 0) {
		cout << "setBoneVisibility: bone index out of bounds! Index=" << boneIndex << endl;
		return;
	}
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
QVector<QVector3D>& SkinnedMesh::positions()
{
	return m_positions;
}
QVector<QVector3D>& SkinnedMesh::normals()
{
	return m_normals;
}
QVector<QVector2D>& SkinnedMesh::texCoords()
{
	return m_texCoords;
}
QVector<VertexBoneData>& SkinnedMesh::vertexBoneData()
{
	return m_vertexBoneData;
}
QVector<uint>& SkinnedMesh::indices()
{
	return m_indices;
}
QVector<QImage>& SkinnedMesh::images()
{
	return m_images;
}
QVector<MeshEntry>& SkinnedMesh::meshEntries()
{
	return m_meshEntries;
}
uint SkinnedMesh::numBones() const
{
	return m_numBones;
}
const map<string, uint>& SkinnedMesh::boneMap() const
{
	return m_boneMap;
}
const QMatrix4x4& SkinnedMesh::boneGlobal(uint boneIndex) const
{
	if (boneIndex > m_boneInfo.size() || boneIndex < 0) {
		cout << "boneGlobal: bone index out of bounds! Index=" << boneIndex << endl;
		return QMatrix4x4();
	}
	return m_boneInfo[boneIndex].global;
}
const QVector3D& SkinnedMesh::boneEndPosition(uint boneIndex) const
{
	if (boneIndex > m_boneInfo.size() || boneIndex < 0) {
		cout << "boneEndPosition: bone index out of bounds! Index=" << boneIndex << endl;
		return QVector3D();
	}
	return m_boneInfo[boneIndex].endPosition;
}
bool SkinnedMesh::initMotionFromFile(const QString& filename)
{
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		cout << "Could not open " << filename.toStdString() << endl;
		return false;
	}

	QTextStream in(&file);
	QString line;
	QStringList list;
	bool ok;
	uint linesCounter = 0;
	uint framesCounter = 0;
	bool endOfHeaders = false;
	do {
		linesCounter++;
		line = in.readLine();
		list = line.split("\t");

		if (list[0] == "time") {
			endOfHeaders = true;
			continue;
		}
		if (!endOfHeaders || list[0]=="") continue;

		// Add timestamp
		double timestamp = list[0].toDouble(&ok);
		if (!ok) {
			//*
			cout << "Timestamp could not be converted to double.";
			cout << " Element=" << list[0].toStdString();
			cout << " Index=0";
			cout << " Line=" << linesCounter << endl;
			//*/
			continue;
		}
		m_timestamps.push_back(timestamp);

		// Add coordinates
		array<float, m_numCoordinates> coordinates;
		for (uint i = 0; i < m_numCoordinates; i++) {
			coordinates[i] = list[i + 1].toFloat(&ok);
			if (!ok) {
				//*
				cout << "Coordinate could not be converted to double.";
				cout << " Element=" << list[i+1].toStdString();
				cout << " Index=0";
				cout << " Line=" << linesCounter << endl;
				//*/
				continue;
			}
		}
		m_coordinateSequence.push_back(coordinates);

		framesCounter++;
	} while (!line.isNull()); //

	//*
	cout << "Loaded SkinnedMesh motion from " << filename.toStdString() << endl;
	cout << "NumberOfLines=" << linesCounter << endl;
	cout << "NumberOfFrames=" << framesCounter << endl;
	cout << "Duration=" << m_timestamps.last() - m_timestamps.first() << endl;
	cout << "Coordinates/Frame=" << list.size()-1 << endl;
	cout << "Last frame:" << endl;
	cout << "Timestamp=" << m_timestamps.last() << endl;
	cout << "Coordinates=";
	for (const auto& p : m_coordinateSequence.last()) {
		cout << p << " ";
	}
	cout << endl;
	//*/
	return true;
}