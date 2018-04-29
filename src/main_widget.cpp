// Own
#include "main_widget.h"

// Project
#include "skinned_mesh.h"
#include "skinning_technique.h"
#include "pipeline.h"
#include "camera.h"
#include "kskeleton.h"

// Assimp
#include <assimp\Importer.hpp>      
#include <assimp\scene.h>			 
#include <assimp\postprocess.h>     

// Qt
#include <QtCore\QDebug>
#include <QtGui\QKeyEvent>
#include <QtGui\QOpenGLTexture>

// Standard C/C++
#include <cassert>
#include <iomanip>

MainWidget::MainWidget(QWidget *parent)
	: 
	QOpenGLWidget(parent),
	m_ksensor(new KSensor()),
	m_skinnedMesh(new SkinnedMesh()),
	m_camera(new Camera()),
	m_pipeline(new Pipeline())
{
	cout << "MainWidget class constructor start." << endl;
	setup();
	cout << "MainWidget class constructor end.\n" << endl;
}
MainWidget::~MainWidget()
{
	delete m_camera;
	delete m_pipeline;
	delete m_skinnedMesh;

	makeCurrent();

	unloadSkinnedMesh();
	delete m_technique;
	delete m_skinningTechnique;

	glDeleteVertexArrays(1, &m_axesVAO);
	glDeleteVertexArrays(1, &m_arrowVAO);
	glDeleteVertexArrays(1, &m_kinectSkeletonJointsVAO);
	glDeleteVertexArrays(1, &m_skinnedMeshJointsVAO);
	glDeleteVertexArrays(1, &m_cubeVAO);
	glDeleteVertexArrays(1, &m_planeVAO);

	glDeleteBuffers(1, &m_kinectSkeletonJointsVBO);
	glDeleteBuffers(1, &m_skinnedMeshJointsVBO);
	glDeleteBuffers(1, &m_cubeVBO);

	doneCurrent();
}
void MainWidget::setup()
{
	cout << "MainWidget setup start." << endl;

	// Setup mode
	m_mode = Mode::PLAYBACK;
	cout << "Mode: " << mode().toStdString() << endl;

	// Setup timer
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(updateIndirect()));
	m_timer.setTimerType(Qt::PreciseTimer);
	m_playbackInterval = m_ksensor->skeleton()->m_interpolationInterval * 1000;
	cout << "Playback interval: " << m_playbackInterval << endl;
	m_timer.setInterval(m_playbackInterval);
	m_timer.start();

	// Setup skinned mesh
	vector<QMatrix4x4> transforms;
	m_skinnedMesh->getBoneTransforms(transforms);
	m_skinnedMesh->initCorrectionVecs(m_ksensor->skeleton()->limbs());

	// #todo: Setup camera
	m_camera->printInfo();

	// Setup pipeline
	PerspectiveProjectionInfo perspectiveProjectionInfo;
	perspectiveProjectionInfo.fieldOfView = 60.f;
	perspectiveProjectionInfo.aspectRatio = m_camera->windowWidth() / m_camera->windowHeight();
	perspectiveProjectionInfo.nearPlane = 0.1f;
	perspectiveProjectionInfo.farPlane = 1000.f;
	m_pipeline->setPersProjInfo(perspectiveProjectionInfo);
	m_pipeline->setCamera(m_camera->GetPos(), m_camera->GetTarget(), m_camera->GetUp());

	// Setup offsets
	m_kinectSkeletonOffset = -m_ksensor->skeleton()->m_activeSequence->at(0).joints[JointType_SpineBase].position;
	cout << "Kinect skeleton offset=" << toStringCartesian(m_kinectSkeletonOffset).toStdString() << endl;
	m_skinnedMeshOffset = -m_skinnedMesh->getOffset();
	cout << "Skinned mesh offset=" << toStringCartesian(m_skinnedMeshOffset).toStdString() << endl;

	cout << "MainWidget setup end." << endl;
}
// This virtual function is called once before the first call to paintGL() or resizeGL().
// Instantiate and initialize context dependent classes in it.
void MainWidget::initializeGL()
{
	cout << "MainWidget initializeGL start." << endl;

	qDebug() << "Obtained format:" << format();
	initializeOpenGLFunctions();

	// Init plane shaders
	QOpenGLShader planeVS(QOpenGLShader::Vertex);
	QOpenGLShader planeFS(QOpenGLShader::Fragment);
	planeVS.compileSourceFile("shaders/plane.vs");
	planeFS.compileSourceFile("shaders/plane.fs");

	m_shaderProgram = new QOpenGLShaderProgram(context());
	if (m_shaderProgram->addShader(&planeVS)) cout << "Added plane vertex shader." << endl;
	else cout << "Could not add plane vertex shader." << endl;
	qDebug() << m_shaderProgram->log();
	if (m_shaderProgram->addShader(&planeFS)) cout << "Added plane fragment shader." << endl;
	else cout << "Could not add plane fragment shader." << endl;
	qDebug() << m_shaderProgram->log();
	if (m_shaderProgram->link()) cout << "Linked plane vertex shader." << endl;
	else cout << "Could not link plane vertex shader." << endl;
	qDebug() << m_shaderProgram->log();
	if (m_shaderProgram->bind()) cout << "Bound plane vertex shader." << endl;
	else cout << "Could not bind plane vertex shader." << endl;

	cout << "Shader program id: " << m_shaderProgram->programId() << endl;
	cout << "Is supported by system? " << m_shaderProgram->hasOpenGLShaderPrograms() << endl;
	m_positionLocation  = m_shaderProgram->attributeLocation("inPosition");
	m_colorLocation		= m_shaderProgram->attributeLocation("inColor");
	m_mvpLocation		= m_shaderProgram->uniformLocation("gMVP");
	m_specificLocation  = m_shaderProgram->uniformLocation("gSpecific");
	cout << "Locations in shader program:" << endl;
	cout << m_positionLocation << " ";
	cout << m_colorLocation << " ";
	cout << m_mvpLocation << " ";
	cout << m_specificLocation << endl;

	// Init technique
	m_technique = new Technique();
	m_technique->initDefault();
	m_technique->enable();
	m_technique->setMVP(m_pipeline->getWVPtrans());
	m_technique->setSpecific(QMatrix4x4());

	// Init skinning technique
	m_skinningTechnique = new SkinningTechnique();
	m_skinningTechnique->Init();
	m_skinningTechnique->enable();
	m_skinningTechnique->SetColorTextureUnit(0);
	DirectionalLight directionalLight;
	directionalLight.Color = QVector3D(1.f, 1.f, 1.f);
	directionalLight.AmbientIntensity = 0.7f;
	directionalLight.DiffuseIntensity = 0.9f;
	directionalLight.Direction = QVector3D(0.f, -1.f, 0.f);
	m_skinningTechnique->setDirectionalLight(directionalLight);
	m_skinningTechnique->setMatSpecularIntensity(0.0f);
	m_skinningTechnique->setMatSpecularPower(0);
	m_skinningTechnique->setSkinning(true);
	m_skinningTechnique->setWVP(m_pipeline->getWVPtrans());
	for (uint i = 0; i < m_skinnedMesh->numBones(); i++) {
		m_skinningTechnique->setBoneVisibility(i, m_skinnedMesh->boneVisibility(i));
	}
	
	loadSkinnedMesh();
	loadAxes();
	loadArrow();
	loadKinectSkeletonJoints();
	loadSkinnedMeshJoints();
	loadCube(0.03);
	loadPlane();

	glEnable(GL_TEXTURE_2D);
	glClearColor(0.0f, 0.0f, 0.0f, 1.f);               
	glClearDepth(1.0f);									
	glEnable(GL_POINT_SMOOTH);							
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	
	// glShadeModel(GL_SMOOTH); // #? deprecated

	//transformSkinnedMesh(true);
	cout << "MainWidget initializeGL end." << endl;
}
// #? must enable corresponding shading technique before using each drawing function. bad design?
void MainWidget::paintGL()
{
	glClearColor(0.f, 0.f, 0.f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_mode == Mode::CAPTURE) {
		m_ksensor->getBodyFrame();
	}
	else if (m_mode == Mode::PLAYBACK){
		m_ksensor->skeleton()->setActiveJoints(m_activeFrame);
		m_skinnedMesh->setActiveCoordinates(m_activeFrame);
	}
	else {
		cout << "Unknown mode of operation." << endl;
	}
	m_time = m_skinnedMesh->timestamp(m_activeFrame);

	transformSkinnedMesh(false);

	// prepare pipeline for drawing skinned mesh related stuff
	if (m_defaultPose) m_pipeline->setWorldRotation(QQuaternion());
	else m_pipeline->setWorldRotation(m_skinnedMesh->pelvisRotation());
	if (m_defaultPose) m_pipeline->setWorldPosition(QVector3D());
	else m_pipeline->setWorldPosition(m_skinnedMesh->pelvisPosition() + m_skinnedMeshOffset);
	m_skinningTechnique->setWVP(m_pipeline->getWVPtrans());

	// enable skinning technique
	m_skinningTechnique->enable();

	// draw skinned mesh
	if (m_mode == Mode::PLAYBACK && m_renderSkinnedMesh && m_skinnedMesh->m_successfullyLoaded) {
		drawSkinnedMesh();
	}

	// enable simple technique
	m_technique->enable();
	m_technique->setMVP(m_pipeline->getWVPtrans());

	// draw skinned mesh bone axes
	m_technique->setSpecific(m_skinnedMesh->boneGlobal(m_activeBone));
	if (m_renderAxes) drawAxes();
	
	// draw skinned mesh joints
	m_technique->setSpecific(QMatrix4x4());
	if (m_renderSkinnedMeshJoints) drawSkinnedMeshJoints();

	// draw basic axes
	m_technique->setMVP(m_pipeline->getVPtrans()); // only VP transformation! 
	m_technique->setSpecific(QMatrix4x4());
	if (m_renderAxes) drawAxes();

	// Prepare pipeline to draw kinect skeleton related stuff
	m_pipeline->setWorldRotation(QQuaternion());
	if (m_defaultPose) m_pipeline->setWorldPosition(QVector3D());
	else m_pipeline->setWorldPosition(m_kinectSkeletonOffset);
	m_technique->setMVP(m_pipeline->getWVPtrans());

	// draw kinect skeleton
	if (m_renderKinectSkeleton) {
		drawKinectSkeletonJoints();
		for (uint i = 0; i < JointType_Count; i++) {
			m_technique->setSpecific(fromTranslation(m_ksensor->skeleton()->activeJoints()[i].position));
			drawCube();
		}
		QVector3D HipsMid = (m_ksensor->skeleton()->activeJoints()[JointType_HipLeft].position + m_ksensor->skeleton()->activeJoints()[JointType_HipRight].position) / 2;
		m_technique->setSpecific(fromTranslation(HipsMid));
		drawCube();
	}

	
	// draw arrow
	QVector3D leftHand = (m_ksensor->skeleton()->activeJoints())[JointType_HandLeft].position;
	QVector3D rightHand = (m_ksensor->skeleton()->activeJoints())[JointType_HandRight].position;
	QVector3D barDirection = rightHand - leftHand;
	QMatrix4x4 S = fromScaling(QVector3D(1, (rightHand - leftHand).length(), 1));
	QMatrix4x4 R = fromRotation(QQuaternion::rotationTo(QVector3D(0, 1, 0), barDirection));
	QMatrix4x4 T = fromTranslation(leftHand);
	m_technique->setSpecific(T * R * S);
	drawArrow();

	// draw planes
	drawPlanes();

	// calculate bar horizontal angle
	m_barAngle = ToDegrees(atan2(barDirection.y(), sqrt(pow(barDirection.x(), 2) + pow(barDirection.z(), 2))));
	
	// calculate bar speed
	static QVector3D previousBarPosition, currentBarPosition;
	currentBarPosition = (leftHand + rightHand) * 0.5;
	m_barSpeed = (currentBarPosition - previousBarPosition) / m_playbackInterval * 1000;
	previousBarPosition = currentBarPosition;

	// calculate knee angle
	QVector3D hipRight = (m_ksensor->skeleton()->activeJoints())[JointType_HipRight].position;
	QVector3D kneeRight = (m_ksensor->skeleton()->activeJoints())[JointType_KneeRight].position;
	QVector3D ankleRight = (m_ksensor->skeleton()->activeJoints())[JointType_AnkleRight].position;
	QVector3D kneeToHip = (hipRight - kneeRight).normalized();
	QVector3D kneeToAnkle = (ankleRight - kneeRight).normalized();
	m_kneeAngle = 180.f-ToDegrees(acos(QVector3D::dotProduct(kneeToHip, kneeToAnkle)));
	
	if (m_play && m_shouldUpdate) {
		if (++m_activeFrame > m_ksensor->skeleton()->m_activeSequence->size())  m_activeFrame = 0;
		m_shouldUpdate = false;
		calculateFPS();
	}

}
void MainWidget::calculateFPS()
{
	static clock_t ticksThisTime;
	static clock_t ticksLastTime = clock();
	static uint framesPassed = 0;

	ticksThisTime = clock();
	clock_t ticksPassed = ticksThisTime - ticksLastTime;
	double millisecondsPassed = ticksToMilliseconds(ticksPassed);

	framesPassed++;
	if (millisecondsPassed > 1000.) {
		m_fpsCount = framesPassed / (millisecondsPassed / 1000.);
		framesPassed = 0;
		ticksLastTime = ticksThisTime;
	}
}
void MainWidget::keyPressEvent(QKeyEvent *event)
{
	int key = event->key();
	switch (key) {
	case Qt::Key_Down:
	case Qt::Key_Up:
	case Qt::Key_Left:
	case Qt::Key_Right:
		m_camera->onKeyboardArrow(key, false);
		m_pipeline->setCamera(m_camera->GetPos(), m_camera->GetTarget(), m_camera->GetUp());
		m_skinningTechnique->enable();
		m_skinningTechnique->SetEyeWorldPos(m_camera->GetPos());
		m_skinningTechnique->setWVP(m_pipeline->getWVPtrans());
		m_technique->enable();
		m_technique->setMVP(m_pipeline->getVPtrans());
		break;
	case Qt::Key_0:
	case Qt::Key_1:
	case Qt::Key_2:
	case Qt::Key_3:
	case Qt::Key_4:
	case Qt::Key_5:
	case Qt::Key_6:
	case Qt::Key_7:
	case Qt::Key_8:
	case Qt::Key_9:
		m_skinnedMesh->flipParameter(key - Qt::Key_0);
		transformSkinnedMesh(true);
		break;
	case Qt::Key_Y:
		m_playbackInterval *= 1.2f; // decrease playback speed
		cout << "Playback interval: " << m_playbackInterval << endl;
		m_timer.setInterval(m_playbackInterval);
		break;
	case Qt::Key_U: 
		m_playbackInterval /= 1.2f; // increase playback speed
		cout << "Playback interval: " << m_playbackInterval << endl;
		m_timer.setInterval(m_playbackInterval);
		break;
	case Qt::Key_A:
		m_ksensor->skeleton()->processFrames();
		break;
	case Qt::Key_C:
		m_ksensor->skeleton()->calculateLimbLengths(*m_ksensor->skeleton()->m_activeSequence);
		m_ksensor->skeleton()->printLimbLengths();
		break;
	case Qt::Key_D:
		m_defaultPose = !m_defaultPose;
		cout << "Default pause " << (m_defaultPose ? "ON" : "OFF") << endl;
		break;
	case Qt::Key_G:
		if (!m_ksensor->getBodyFrame()) cout << "Could not update kinect data." << endl;
		break;
	case Qt::Key_J:
		m_ksensor->skeleton()->printActiveJoints();
		break;
	case Qt::Key_L:
		m_ksensor->skeleton()->loadFrameSequences();
		break;
	case Qt::Key_M:
		changeMode();
		break;
	case Qt::Key_N:
		m_ksensor->skeleton()->nextActiveSequence();
		break;
	case Qt::Key_P:
		if (m_play) {
			m_play = false;
			cout << "Play OFF" << endl;
			m_timer.stop();
		}
		else {
			m_play = true;
			cout << "Play ON" << endl;
			m_timer.start();
		}
		break;
	case Qt::Key_R:
		if (m_mode==Mode::CAPTURE) m_ksensor->record();
		else cout << "Record does not work in this mode." << endl;
		break;
	case Qt::Key_S:
		m_ksensor->skeleton()->saveFrameSequences();
		break;
	case Qt::Key_Q:
		m_skinnedMesh->printInfo();
		break;
	case Qt::Key_T:
		m_ksensor->skeleton()->saveToTrc();
		break;
	case Qt::Key_Escape:
		event->ignore();	// event passed to MainWidget's parent (MainWindow)
		break;
	default:
		cout << "MainWidget: This key does not do anything." << endl;
		break;
	}
	update();
}
void MainWidget::mousePressEvent(QMouseEvent *event)
{
	m_lastMousePosition = event->pos();
}
void MainWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - m_lastMousePosition.x();
	int dy = event->y() - m_lastMousePosition.y();

	if (event->buttons() & Qt::LeftButton){
		if (event->modifiers() & Qt::ControlModifier) {
			m_camera->rotateRight(-dx);
		}
		else if (event->modifiers() & Qt::ShiftModifier) {
			m_camera->rotateUp(-dy);
		}
		else {
			m_camera->rotateRight(-dx);
			m_camera->rotateUp(-dy);
		}
	}
	m_lastMousePosition = event->pos();

	m_pipeline->setCamera(m_camera->GetPos(), m_camera->GetTarget(), m_camera->GetUp());
	m_skinningTechnique->enable();
	m_skinningTechnique->SetEyeWorldPos(m_camera->GetPos());
	m_skinningTechnique->setWVP(m_pipeline->getWVPtrans());
	m_technique->enable();
	m_technique->setMVP(m_pipeline->getVPtrans());
	update();
}
void MainWidget::wheelEvent(QWheelEvent *event)
{
	QPoint degrees = event->angleDelta() / 8;
	//cout << degrees.x() << " " << degrees.y() << endl;
	m_camera->onMouseWheel(degrees.y(), false);

	m_pipeline->setCamera(m_camera->GetPos(), m_camera->GetTarget(), m_camera->GetUp());
	m_skinningTechnique->enable();
	m_skinningTechnique->SetEyeWorldPos(m_camera->GetPos());
	m_skinningTechnique->setWVP(m_pipeline->getWVPtrans());
	m_technique->enable();
	m_technique->setMVP(m_pipeline->getVPtrans());
	update();
}
void MainWidget::transformSkinnedMesh(bool print)
{
	if (print) cout << "Transforming bones." << endl;
	vector<QMatrix4x4> transforms;
	m_skinnedMesh->getBoneTransforms(transforms);
	m_skinningTechnique->enable();
	for (uint i = 0; i < transforms.size(); i++) {
		m_skinningTechnique->setBoneTransform(i, transforms[i]);
	}
}
void MainWidget::setRenderAxes(bool state)
{
	if (m_renderAxes != state) {
		m_renderAxes = state;		
		update();
	}
}
KSensor * MainWidget::ksensor()
{
	return m_ksensor;
}
void MainWidget::setRenderModel(bool state)
{
	if (m_renderSkinnedMesh != state) {
		m_renderSkinnedMesh = state;
		update();
	}
}
void MainWidget::setRenderSkeleton(bool state)
{
	if (m_renderKinectSkeleton != state) {
		m_renderKinectSkeleton = state;
		update();
	}
}
bool MainWidget::renderAxes() const
{
	return m_renderAxes;
}
bool MainWidget::renderSkinnedMesh() const
{
	return m_renderSkinnedMesh;
}
bool MainWidget::renderKinectSkeleton() const
{
	return m_renderKinectSkeleton;
}
bool MainWidget::modelSkinning() const
{
	return m_skinningEnabled;
}
QStringList MainWidget::modelBoneList() const
{
	QStringList qsl;
	for (const auto& it : m_skinnedMesh->boneMap()) qsl << QString::fromLocal8Bit(it.first.c_str());
	return qsl;
}
QString MainWidget::mode() const
{
	return QString(m_mode==Mode::PLAYBACK ? "PLAYBACK" : "CAPTURE");
}
void MainWidget::setModelName(const QString &modelName)
{
	m_skinnedMeshModelName = modelName;
}
void MainWidget::setModelSkinning(bool state)
{
	m_skinningTechnique->enable();
	m_skinningTechnique->setSkinning(state);
	update();
}
SkinnedMesh* MainWidget::skinnedMesh()
{
	return m_skinnedMesh;
}
SkinningTechnique* MainWidget::skinningTechnique()
{
	return m_skinningTechnique;
}
Technique* MainWidget::technique()
{
	return m_technique;
}
void MainWidget::unloadSkinnedMesh()
{
	for (uint i = 0; i < m_textures.size(); i++) {
		SAFE_DELETE(m_textures[i]);
	}

	if (m_skinnedMeshVBOs[0] != 0) {
		glDeleteBuffers(ARRAY_SIZE_IN_ELEMENTS(m_skinnedMeshVBOs), m_skinnedMeshVBOs);
	}

	if (m_skinnedMeshVAO != 0) {
		glDeleteVertexArrays(1, &m_skinnedMeshVAO);
		m_skinnedMeshVAO = 0;
	}
}
void MainWidget::updateIndirect()
{
	m_shouldUpdate = true;
	update(); // #? use QWidget->update or QWidget->repaint
}
void MainWidget::setActiveFrame(uint index)
{
	m_activeFrame = (uint)(index / 100.f * m_ksensor->skeleton()->m_activeSequence->size());
}
void MainWidget::changeMode()
{
		if (m_mode == Mode::CAPTURE) {
			m_mode = Mode::PLAYBACK;
			m_timer.setInterval(m_playbackInterval);
		}
		else {
			m_mode = Mode::CAPTURE;
			m_timer.setInterval(m_captureInterval);
		}
		cout << "Mode: " << mode().toStdString() << endl;
		m_timer.start();
}
void MainWidget::setBoneAxes(const QString &boneName)
{
	uint boneId = m_skinnedMesh->findBoneId(boneName);
	assert(boneId < m_boneInfo.size());
	m_technique->enable();
	m_technique->setSpecific(m_skinnedMesh->boneGlobal(boneId));
}
void MainWidget::setActiveBone(const QString& boneName)
{
	m_activeBone = m_skinnedMesh->findBoneId(boneName);
}
void MainWidget::loadSkinnedMesh()
{
	// Release the previously loaded mesh (if it exists)
	unloadSkinnedMesh();

	for (int i = 0; i < m_skinnedMesh->images().size(); i++) m_textures.push_back(new QOpenGLTexture(m_skinnedMesh->images()[i]));

	// Create the VAO
	glGenVertexArrays(1, &m_skinnedMeshVAO);
	cout << "skinnedMeshVAO=" << m_skinnedMeshVAO << endl;
	glBindVertexArray(m_skinnedMeshVAO);

	// Create the buffers for the vertices attributes
	glGenBuffers(ARRAY_SIZE_IN_ELEMENTS(m_skinnedMeshVBOs), m_skinnedMeshVBOs);
	for (uint i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_skinnedMeshVBOs); i++) {
		cout << "skinnedMeshVBO=" << m_skinnedMeshVBOs[i] << endl;
	}

#define POSITION_LOCATION    0
#define TEX_COORD_LOCATION   1
#define NORMAL_LOCATION      2
#define BONE_ID_LOCATION     3
#define BONE_WEIGHT_LOCATION 4

	const auto& positions = m_skinnedMesh->positions();
	const auto& texCoords = m_skinnedMesh->texCoords();
	const auto& normals = m_skinnedMesh->normals();
	const auto& vertexBoneData = m_skinnedMesh->vertexBoneData();
	const auto& indices = m_skinnedMesh->indices();

	// Generate and populate the buffers with vertex attributes and the indices
	glBindBuffer(GL_ARRAY_BUFFER, m_skinnedMeshVBOs[POS_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(positions[0]) * positions.size(), &positions[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(POSITION_LOCATION);
	glVertexAttribPointer(POSITION_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_skinnedMeshVBOs[TEXCOORD_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(texCoords[0]) * texCoords.size(), texCoords.constData(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(TEX_COORD_LOCATION);
	glVertexAttribPointer(TEX_COORD_LOCATION, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_skinnedMeshVBOs[NORMAL_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(normals[0]) * normals.size(), &normals[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(NORMAL_LOCATION);
	glVertexAttribPointer(NORMAL_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_skinnedMeshVBOs[BONE_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertexBoneData[0]) * vertexBoneData.size(), &vertexBoneData[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(BONE_ID_LOCATION);
	glVertexAttribIPointer(BONE_ID_LOCATION, 4, GL_INT, sizeof(VertexBoneData), (const GLvoid*)0);
	glEnableVertexAttribArray(BONE_WEIGHT_LOCATION);
	glVertexAttribPointer(BONE_WEIGHT_LOCATION, 4, GL_FLOAT, GL_FALSE, sizeof(VertexBoneData), (const GLvoid*)16);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_skinnedMeshVBOs[INDEX_BUFFER]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0]) * indices.size(), &indices[0], GL_STATIC_DRAW);

	glBindVertexArray(0);

	if (GLNoError()) {
		cout << "Successfully loaded SkinnedMesh to GPU" << endl;
	}
	else {
		cout << "Error loading SkinnedMesh to GPU" << endl;
		GLPrintError();
	}
}
void MainWidget::drawSkinnedMesh()
{
	glBindVertexArray(m_skinnedMeshVAO);

	auto meshEntries = m_skinnedMesh->entries();
	for (uint i = 0; i < meshEntries.size(); i++) {
		const uint materialIndex = meshEntries[i].MaterialIndex;

		assert(materialIndex < m_textures.size());

		if (m_textures[materialIndex]) {
			m_textures[materialIndex]->bind();
		}
		glDrawElementsBaseVertex(GL_TRIANGLES,
			meshEntries[i].NumIndices,
			GL_UNSIGNED_INT,
			(void*)(sizeof(uint) * meshEntries[i].BaseIndex),
			meshEntries[i].BaseVertex);
	}

	glBindVertexArray(0);
}
void MainWidget::loadArrow()
{
	const float head = 0.1f;
	const float colorRed = 255.f;
	const float colorBlue = 255.f;
	const float colorGreen = 255.f;

	const GLfloat vertices[] =
	{
		0         ,   0, 0, // start
		colorRed  , colorGreen, colorBlue,
		0         ,   1, 0, // end
		colorRed  , colorGreen, colorBlue,
		head      , 1 - head, 0, // +x
		colorRed  , colorGreen, colorBlue,
		-head     , 1 - head, 0, // -x
		colorRed  , colorGreen, colorBlue,
		0         , 1 - head, head, // +z
		colorRed  , colorGreen, colorBlue,
		0         , 1 - head, -head, // -z
		colorRed  , colorGreen, colorBlue,
	};

	const GLushort indices[] =
	{
		0, 1,
		1, 2,
		1, 3,
		1, 4,
		1, 5,
	};
	glGenVertexArrays(1, &m_arrowVAO);
	cout << "arrowVAO=" << m_arrowVAO << endl;
	glBindVertexArray(m_arrowVAO);

	GLuint arrowIBO;
	glGenBuffers(1, &arrowIBO);
	cout << "arrowIBO=" << arrowIBO << endl;
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, arrowIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	GLuint arrowVBO;
	glGenBuffers(1, &arrowVBO);
	cout << "arrowVBO=" << arrowVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, arrowVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, BUFFER_OFFSET(sizeof(GLfloat) * 3));

	glBindVertexArray(0);
}
void MainWidget::drawArrow()
{
	glBindVertexArray(m_arrowVAO);
	glDrawElements(GL_LINES, 10, GL_UNSIGNED_SHORT, 0);
	glBindVertexArray(0);
}
void MainWidget::loadAxes()
{
	GLfloat vertices[] =
	{
		0.f  , 0.f, 0.f,  // origin position
		0.f  , 0.f, 0.f, // origin color
		1.f  , 0.f, 0.f, // x axis position
		255.f, 0.f  , 0.f, // x axis color
		0.f  , 1.f, 0.f, // y axis position
		0.f  , 255.f, 0.f, // y axis color
		0.f  , 0.f, 1.f, // z axis position
		0.f  , 0.f  , 255.f // z axis color
	};

	GLushort indices[] =
	{
		0, 1,
		0, 2,
		0, 3
	};

	glGenVertexArrays(1, &m_axesVAO);
	glBindVertexArray(m_axesVAO);
	cout << "axesVAO=" << m_axesVAO << endl;

	GLuint axesIBO;
	glGenBuffers(1, &axesIBO);
	cout << "axesIBO=" << axesIBO << endl;
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, axesIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	GLuint axesVBO;
	glGenBuffers(1, &axesVBO);
	cout << "axesVBO=" << axesVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, axesVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, BUFFER_OFFSET(sizeof(GLfloat) * 3));

	glBindVertexArray(0);
}
void MainWidget::drawAxes()
{
	glBindVertexArray(m_axesVAO);
	glDrawElements(GL_LINES, 6, GL_UNSIGNED_SHORT, 0);
	glBindVertexArray(0);
}
void MainWidget::loadKinectSkeletonJoints()
{
	GLushort indices[] =
	{
		// core (4 parts)
		JointType_SpineBase    , JointType_SpineMid,
		JointType_SpineMid     , JointType_SpineShoulder,
		JointType_SpineShoulder, JointType_Neck,
		JointType_Neck         , JointType_Head,
		// left side (10 parts)	   
		JointType_SpineShoulder, JointType_ShoulderLeft,
		JointType_ShoulderLeft , JointType_ElbowLeft,
		JointType_ElbowLeft    , JointType_WristLeft,
		JointType_WristLeft    , JointType_HandLeft,
		JointType_HandLeft     , JointType_ThumbLeft,
		JointType_HandLeft     , JointType_HandTipLeft,
		JointType_SpineBase    , JointType_HipLeft,
		JointType_HipLeft      , JointType_KneeLeft,
		JointType_KneeLeft     , JointType_AnkleLeft,
		JointType_AnkleLeft    , JointType_FootLeft,
		// Right side (10 parts) 
		JointType_SpineShoulder, JointType_ShoulderRight,
		JointType_ShoulderRight, JointType_ElbowRight,
		JointType_ElbowRight   , JointType_WristRight,
		JointType_WristRight   , JointType_HandRight,
		JointType_HandRight    , JointType_ThumbRight,
		JointType_HandRight    , JointType_HandTipRight,
		JointType_SpineBase    , JointType_HipRight,
		JointType_HipRight     , JointType_KneeRight,
		JointType_KneeRight    , JointType_AnkleRight,
		JointType_AnkleRight   , JointType_FootRight
	};

	glGenVertexArrays(1, &m_kinectSkeletonJointsVAO);
	glBindVertexArray(m_kinectSkeletonJointsVAO);
	cout << "kinectSkeletonJointsVAO=" << m_kinectSkeletonJointsVAO << endl;

	GLuint skeletonIBO;
	glGenBuffers(1, &skeletonIBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, skeletonIBO);
	cout << "kinectSkeletonJointsIBO=" << skeletonIBO << endl;
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_kinectSkeletonJointsVBO);
	cout << "kinectSkeletonJointsVBO=" << m_kinectSkeletonJointsVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, m_kinectSkeletonJointsVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * 3 * JointType_Count, NULL, GL_STREAM_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, BUFFER_OFFSET(sizeof(GLfloat) * 3));

	glBindVertexArray(0);
}
void MainWidget::drawKinectSkeletonJoints()
{
	for (uint i = 0; i < JointType_Count; i++) {
		m_kinectSkeletonJoints[6 * i    ] = m_ksensor->skeleton()->activeJoints()[i].position.x();
		m_kinectSkeletonJoints[6 * i + 1] = m_ksensor->skeleton()->activeJoints()[i].position.y();
		m_kinectSkeletonJoints[6 * i + 2] = m_ksensor->skeleton()->activeJoints()[i].position.z();
		m_kinectSkeletonJoints[6 * i + 3] = (m_ksensor->skeleton()->activeJoints()[i].trackingState == TrackingState_NotTracked ? 255.f : 0.f);
		m_kinectSkeletonJoints[6 * i + 4] = (m_ksensor->skeleton()->activeJoints()[i].trackingState == TrackingState_Tracked ? 255.f : 0.f);
		m_kinectSkeletonJoints[6 * i + 5] = (m_ksensor->skeleton()->activeJoints()[i].trackingState == TrackingState_Inferred ? 255.f : 0.f);
	}
	//makeCurrent();
	glBindBuffer(GL_ARRAY_BUFFER, m_kinectSkeletonJointsVBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_kinectSkeletonJoints), m_kinectSkeletonJoints);

	glBindVertexArray(m_kinectSkeletonJointsVAO);
	glDrawElements(GL_LINES, 48, GL_UNSIGNED_SHORT, 0);
	glBindVertexArray(0);
}
void MainWidget::loadSkinnedMeshJoints()
{
	glGenVertexArrays(1, &m_skinnedMeshJointsVAO);
	glBindVertexArray(m_skinnedMeshJointsVAO);
	cout << "skinnedMeshJointsVAO=" << m_skinnedMeshJointsVAO << endl;

	glGenBuffers(1, &m_skinnedMeshJointsVBO);
	glBindBuffer(GL_ARRAY_BUFFER, m_skinnedMeshJointsVBO);
	cout << "skinnedMeshJointsVBO=" << m_skinnedMeshJointsVBO << endl;
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * 3 * NUM_BONES, NULL, GL_STREAM_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, BUFFER_OFFSET(sizeof(GLfloat) * 3));

	glBindVertexArray(0);
}
void MainWidget::drawSkinnedMeshJoints()
{
	for (uint i = 0; i < NUM_BONES; i++) {
		m_skinnedMeshJoints[6 * i    ] = m_skinnedMesh->boneEndPosition(i).x();
		m_skinnedMeshJoints[6 * i + 1] = m_skinnedMesh->boneEndPosition(i).y();
		m_skinnedMeshJoints[6 * i + 2] = m_skinnedMesh->boneEndPosition(i).z();
		m_skinnedMeshJoints[6 * i + 3] = 0.f;
		m_skinnedMeshJoints[6 * i + 4] = 255.f;
		m_skinnedMeshJoints[6 * i + 5] = 0.f;	 
	}

	glBindBuffer(GL_ARRAY_BUFFER, m_skinnedMeshJointsVBO);

	glBindVertexArray(m_skinnedMeshJointsVAO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_skinnedMeshJoints), m_skinnedMeshJoints);
	glDrawArrays(GL_POINTS, 0, NUM_BONES*60);
	glBindVertexArray(0);
}
void MainWidget::loadCube(float r)
{
	GLfloat vertices[] =
	{
		+r / 2, -r / 2, +r / 2, // 0
		+r / 2, +r / 2, +r / 2, // 1
		-r / 2, +r / 2, +r / 2, // 2
		-r / 2, -r / 2, +r / 2, // 3
		+r / 2, -r / 2, -r / 2, // 4
		+r / 2, +r / 2, -r / 2, // 5
		-r / 2, +r / 2, -r / 2, // 6
		-r / 2, -r / 2, -r / 2, // 7
		255.f, 255.f, 255.f,
		255.f, 255.f, 255.f,
		255.f, 255.f, 255.f,
		255.f, 255.f, 255.f,
		255.f, 255.f, 255.f,
		255.f, 255.f, 255.f,
		255.f, 255.f, 255.f,
		255.f, 255.f, 255.f
	};

	GLushort indices[] =
	{
		0, 1, 2, 3,
		0, 4, 5, 1,
		4, 7, 6, 5,
		7, 3, 2, 6,
		2, 1, 5, 6,
		0, 4, 7, 3
	};

	glGenVertexArrays(1, &m_cubeVAO);
	glBindVertexArray(m_cubeVAO);
	cout << "cubeVAO=" << m_cubeVAO << endl;

	GLuint cubeIBO;
	glGenBuffers(1, &cubeIBO);
	cout << "cubeIBO=" << cubeIBO << endl;
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_cubeVBO);
	cout << "cubeVBO=" << m_cubeVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, m_cubeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 3, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 3, BUFFER_OFFSET(sizeof(vertices) / 2));

	glBindVertexArray(0);
}
void MainWidget::drawCube()
{
	for (uint i = 0; i < JointType_Count; i++) {
		QVector3D color(0.f, 0.f, 0.f);
		if (m_ksensor->skeleton()->activeJoints()[i].trackingState == TrackingState_NotTracked) color.setX(255.f);
		else if (m_ksensor->skeleton()->activeJoints()[i].trackingState == TrackingState_Tracked) color.setY(255.f);
		else color.setZ(255.f);
		for (uint j = 0; j < 8; j++) {
			m_cubeColors[3 * j + 0] = color.x();
			m_cubeColors[3 * j + 1] = color.y();
			m_cubeColors[3 * j + 2] = color.z();
		}
	}

	glBindBuffer(GL_ARRAY_BUFFER, m_cubeVBO);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(m_cubeColors), sizeof(m_cubeColors), m_cubeColors);

	glBindVertexArray(m_cubeVAO);
	glDrawElements(GL_TRIANGLE_STRIP, 24, GL_UNSIGNED_SHORT, 0);
	glBindVertexArray(0);
}
void MainWidget::loadPlane()
{
	glGenVertexArrays(1, &m_planeVAO);
	glBindVertexArray(m_planeVAO);
	cout << "planeVAO=" << m_planeVAO << endl;

	//GLuint planeIBO;
	//glGenBuffers(1, &cubeIBO);
	//cout << "cubeIBO=" << cubeIBO << endl;
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeIBO);
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	GLuint planeVBO;
	glGenBuffers(1, &planeVBO);
	cout << "m_planeVBO=" << planeVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(m_planePositions)+sizeof(m_planeColors), NULL, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 3, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 3, BUFFER_OFFSET(sizeof(m_planePositions)));
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_planePositions), m_planePositions);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(m_planeColors), sizeof(m_planeColors), m_planeColors);

	glBindVertexArray(0);

	m_shaderProgram->bind();
	m_shaderProgram->setAttributeArray(m_positionLocation, m_planePositions, 3, 0);
	m_shaderProgram->setAttributeArray(m_colorLocation, m_planeColors, 3, 0);
	m_shaderProgram->release();
}

void MainWidget::drawPlanes()
{
	//m_shaderProgram->bind();
	m_shaderProgram->setUniformValue(m_mvpLocation, m_pipeline->getWVPtrans());
	m_shaderProgram->setUniformValue(m_specificLocation, QMatrix4x4());

	glBindVertexArray(m_planeVAO);
	glDrawArrays(GL_LINE_LOOP, 0, 3);
	glBindVertexArray(0);

	m_shaderProgram->disableAttributeArray(m_positionLocation);
	m_shaderProgram->disableAttributeArray(m_colorLocation);
}
