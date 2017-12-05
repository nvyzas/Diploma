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

MainWidget::MainWidget(QWidget *parent)
	: QOpenGLWidget(parent)
	,m_timer(this)
{
	m_Mesh = new SkinnedMesh();
	m_VAO = 0;
	ZERO_MEM(m_Buffers);


	m_timer.setTimerType(Qt::PreciseTimer);
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(updateIndirect()));
	m_modeOfOperation = Mode::CAPTURE;
	m_timer.setInterval(m_captureInterval);
	m_timer.start();
}
MainWidget::~MainWidget()
{
	makeCurrent();
	unloadFromGPU();
	doneCurrent();

	delete m_Mesh;
	delete m_Cam;
	delete m_Tech;
	delete m_Skin;
	delete m_Pipe;
}
// This virtual function is called once before the first call to paintGL() or resizeGL().
void MainWidget::initializeGL()
{
	m_playbackInterval = m_ksensor->skeleton()->timeStep() * 1000;
	m_Cam = new Camera();
	m_Tech = new Technique();
	m_Skin = new SkinningTechnique();
	m_Pipe = new Pipeline();

	qDebug() << "Obtained format:" << format();
	initializeOpenGLFunctions();
	m_ksensor->skeleton()->initOGL();
	//cout << "GL version: " << glGetString(GL_VERSION) << endl;
	//cout << "GL renderer: " << glGetString(GL_RENDERER) << endl;
	//cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;	
	GLint i;
	glGetIntegerv(GL_CONTEXT_FLAGS, &i);
	
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);                            // Enable Smooth Shading
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);               // Black Background
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_POINT_SMOOTH);							// Points represented as circles
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glDisable(GL_CULL_FACE);
	
	MySetup();
}
void MainWidget::MySetup()
{
	// 1) Init KSensor
	//m_Sensor->init();
	//m_Sensor->PrintJointHierarchy();

	// 2) Init Mesh
	//m_Mesh->setKSensor(*m_Sensor);
	//m_successfullyLoaded = loadToGPU("cmu_test");
	//m_Sensor->GetKinectData(); // to successfully acquire frame init sensor before mesh and load mesh before getting data

	// 3) Init Camera
	//m_Cam->SetCam(); Camera init in its constructor
	m_Cam->PrintInfo();

	// 4) Init Pipeline
	m_Pipe->Scale(1.f, 1.f, 1.f);
	m_Pipe->Rotate(0.f, 0.f, 0.f);
	m_Pipe->WorldPos(0.f, 0.f, 2.f);
	/*
	if (mySensor.m_GotFrame) {
	const Vector3f &fl = mySensor.m_Joints[JointType_FootLeft].Position;
	const Vector3f &fr = mySensor.m_Joints[JointType_FootRight].Position;
	const Vector3f &sb = mySensor.m_Joints[JointType_SpineBase].Position;
	myPipe.WorldPos((fl.x + fr.x) / 2.0f, (fl.y + fr.y) / 2.0f, (fl.z + fr.z) / 2.0f);
	myPipe.WorldPos(sb.x, sb.y, sb.z);
	}
	else myPipe.WorldPos(0, 0, 1);
	//*/
	m_Pipe->SetCamera(m_Cam->GetPos(), m_Cam->GetTarget(), m_Cam->GetUp());
	PersProjInfo persProjInfo;
	persProjInfo.FOV = 60.0f;
	persProjInfo.Height = m_Cam->GetHeight();
	persProjInfo.Width = m_Cam->GetWidth();
	persProjInfo.zNear = 0.1f;
	persProjInfo.zFar = 1000.0f;
	m_Pipe->SetPerspectiveProj(persProjInfo);

	// 5) Init Technique
	if (!m_Tech->InitDefault()) cout << "Could not initialize default shaders" << endl;
	m_Tech->enable();
	m_Tech->SetDefault(m_Pipe->GetWVPTrans());

	// 6) Init SkinningTechnique
	if (!m_Skin->Init()) cout << "Could not initialize skinning shaders" << endl;
	m_Skin->enable();
	m_Skin->SetColorTextureUnit(0);
	DirectionalLight directionalLight;
	directionalLight.Color = QVector3D(1.f, 1.f, 1.f);
	directionalLight.AmbientIntensity = 0.7f;
	directionalLight.DiffuseIntensity = 0.9f;
	directionalLight.Direction = QVector3D(0.f, -1.f, 0.f);
	m_Skin->SetDirectionalLight(directionalLight);
	m_Skin->SetMatSpecularIntensity(0.0f);
	m_Skin->SetMatSpecularPower(0);
	m_Skin->setSkinning(true);
	m_Skin->SetWVP(m_Pipe->GetWVPTrans());
	for (uint i = 0; i < m_Mesh->numBones(); i++) {
		m_Skin->setBoneVisibility(i, m_Mesh->boneVisibility(i));
	}
	Transform(true);
}
void MainWidget::paintGL()
{
	glClearColor(0.f, 0.f, 0.f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_modeOfOperation==Mode::CAPTURE) m_ksensor->getBodyData();
	else m_ksensor->skeleton()->getActiveFrame();
	/*
	m_Skin->enable();
	m_Skin->SetWVP(m_Pipe->GetWVPTrans());
	if (m_play) {
		m_sensor->GetKinectData();
		Transform(false);
	}
	/*if (!m_modelName.isEmpty()) {
		m_Mesh->LoadMesh(string(m_modelName.toLocal8Bit()));
		m_modelName.clear();
	}*/
	if (m_renderModel && m_successfullyLoaded) {
		drawSkinnedMesh();
	}

	m_Tech->enable();
	m_Tech->SetDefault(m_Pipe->GetVPTrans());
	if (m_renderSkeleton) m_ksensor->skeleton()->drawSkeleton(0);
	if (m_renderAxes)	DrawAxes();

	//if (m_renderCameraVectors)		m_Cam->DrawCameraVectors();
	//*/
	if (m_modeOfOperation == Mode::PLAYBACK) m_ksensor->skeleton()->nextActiveFrame();
}
void MainWidget::keyPressEvent(QKeyEvent *event)
{
	int key = event->key();
	switch (key) {
	case Qt::Key_Down:
	case Qt::Key_Up:
	case Qt::Key_Left:
	case Qt::Key_Right:
		m_Cam->onKeyboardArrow(key, true);
		m_Pipe->SetCamera(m_Cam->GetPos(), m_Cam->GetTarget(), m_Cam->GetUp());
		m_Skin->enable();
		m_Skin->SetEyeWorldPos(m_Cam->GetPos());
		m_Skin->SetWVP(m_Pipe->GetWVPTrans());
		m_Tech->enable();
		m_Tech->SetDefault(m_Pipe->GetVPTrans());
		break;
	/*case Qt::Key_0:
	case Qt::Key_1:
	case Qt::Key_2:
	case Qt::Key_3:
	case Qt::Key_4:
	case Qt::Key_5:
	case Qt::Key_6:
	case Qt::Key_7:
	case Qt::Key_8:
	case Qt::Key_9:
		m_Mesh->flipParameter(key - Qt::Key_0);
		Transform(true);
		break;*/
	case Qt::Key_1:
		m_playbackInterval *= 4.f/3.f;
		cout << "Playback interval: " << m_playbackInterval << endl;
		m_timer.setInterval(m_playbackInterval);
		break;
	case Qt::Key_3:
		m_playbackInterval /= 4.f/3.f;
		cout << "Playback interval: " << m_playbackInterval << endl;
		m_timer.setInterval(m_playbackInterval);
		break;
	case Qt::Key_C:
		if (!m_ksensor->connect()) cout << "Could not connect to kinect sensor." << endl;
		break;
	case Qt::Key_F:
		m_ksensor->skeleton()->m_playbackFiltered = !m_ksensor->skeleton()->m_playbackFiltered;
		cout << "Filtered data playback " << (m_ksensor->skeleton()->m_playbackFiltered ? "ON" : "OFF") << endl;
		break;
	case Qt::Key_I:
		m_ksensor->skeleton()->m_playbackInterpolated = !m_ksensor->skeleton()->m_playbackInterpolated;
		cout << "Interpolated data playback " << (m_ksensor->skeleton()->m_playbackInterpolated ? "ON" : "OFF") << endl;
		break;
	case Qt::Key_G:
		if (!m_ksensor->getBodyData()) cout << "Could not update kinect data." << endl;
		break;
	case Qt::Key_L:
		m_ksensor->skeleton()->loadFromBinary();
		break;
	case Qt::Key_M:
		changeMode();
		break;
	case Qt::Key_P:
		if (m_play) {
			m_play = false;
			cout << "Play: OFF" << endl;
			m_timer.stop();
		}
		else {
			m_play = true;
			cout << "Play: ON" << endl;
			m_timer.start();
		}
		break;
	case Qt::Key_R:
		m_ksensor->record();
		break;
	case Qt::Key_S:
		m_ksensor->skeleton()->saveToBinary();
		break;
	case Qt::Key_Q:
		m_ksensor->skeleton()->printSequence();
		break;
	case Qt::Key_T:
		m_ksensor->skeleton()->createTRC();
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
	m_lastPos = event->pos();
}
void MainWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - m_lastPos.x();
	int dy = event->y() - m_lastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		m_Cam->rotateRight(dx);
	}
	else if (event->buttons() & Qt::RightButton) {
		m_Cam->rotateUp(dy);
	}
	m_lastPos = event->pos();

	m_Pipe->SetCamera(m_Cam->GetPos(), m_Cam->GetTarget(), m_Cam->GetUp());
	m_Skin->enable();
	m_Skin->SetEyeWorldPos(m_Cam->GetPos());
	m_Skin->SetWVP(m_Pipe->GetWVPTrans());
	m_Tech->enable();
	m_Tech->SetDefault(m_Pipe->GetVPTrans());
	update();
}
void MainWidget::wheelEvent(QWheelEvent *event)
{
	QPoint degrees = event->angleDelta() / 8;
	//cout << degrees.x() << " " << degrees.y() << endl;
	m_Cam->onMouseWheel(degrees.y(), true);

	m_Pipe->SetCamera(m_Cam->GetPos(), m_Cam->GetTarget(), m_Cam->GetUp());
	m_Skin->enable();
	m_Skin->SetEyeWorldPos(m_Cam->GetPos());
	m_Skin->SetWVP(m_Pipe->GetWVPTrans());
	m_Tech->enable();
	m_Tech->SetDefault(m_Pipe->GetVPTrans());
	update();
}
void MainWidget::DrawAxes()
{
	//*
	glBegin(GL_LINES);
	Vector3f origin(0.0f, 0.0f, 0.0f);
	float length = 1;
	//[X]
	glColor3f(0xFF, 0, 0);
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f(length, 0, 0);
	//[Y]
	glColor3f(0, 0xFF, 0);
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f(0, length, 0);
	//[Z]
	glColor3f(0, 0, 0xFF);
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f(0, 0, length);
	glEnd();
	//*/
}
void MainWidget::DrawAxes(Vector3f origin, Vector3f vx, Vector3f vy, Vector3f vz, float length)
{
	//*
	glBegin(GL_LINES);
	//[X]
	glColor3f(0xFF, 0, 0);
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f((vx.x)*length, (vx.y)*length, (vx.z)*length);
	//[Y]
	glColor3f(0, 0xFF, 0);
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f((vy.x)*length, (vy.y)*length, (vy.z)*length);
	//[Z]
	glColor3f(0, 0, 0xFF);
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f((vz.x)*length, (vz.y)*length, (vz.z)*length);
	glEnd();
	//*/
}
void MainWidget::DrawTestAxes()
{
	Vector3f origin(0.0f, 0.0f, 0.0f);
	Vector3f vx = Vector3f::UnitX;
	Vector3f vy = Vector3f::UnitY;
	Vector3f vz = Vector3f::UnitZ;
	DrawAxes(origin, vx, vy, vz, 1);
	Matrix4f R;
	R.InitRotateTransform(45, 0, 0);
}
void MainWidget::NextInfoBlock(int step)
{
	activeInfo = Mod(activeInfo, NUM_INFO_BLOCKS, step);
	cout << "Printing info block " << activeInfo << endl;
	if (activeInfo == 0) m_Mesh->PrintInfo();
	else m_Cam->PrintInfo();
}
void MainWidget::Transform(bool print)
{
	//if (!print) cout.setstate(std::ios_base::failbit);
	if (m_Mesh->m_SuccessfullyLoaded) {
		if (print) cout << "Transforming bones." << endl;
		vector<Matrix4f> Transforms;
		m_Mesh->GetBoneTransforms(Transforms); // update bone transforms from kinect
		m_Skin->enable();
		for (uint i = 0; i < Transforms.size(); i++) {
			m_Skin->setBoneTransform(i, Transforms[i]); // send transforms to vertex shader
		}
	}
	else {
		if (print) cout << "Mesh is not successfully loaded. Cannot transform bones." << endl;
	}
	//if (!print) cout.clear();
}
//*/

void MainWidget::setRenderAxes(bool state)
{
	if (m_renderAxes != state) {
		m_renderAxes = state;		
		update();
	}
}
bool MainWidget::renderAxes() const
{
	return m_renderAxes;
}
void MainWidget::setRenderModel(bool state)
{
	if (m_renderModel != state) {
		m_renderModel = state;
		update();
	}
}
bool MainWidget::renderModel() const
{
	return m_renderModel;
}
bool MainWidget::modelSkinning() const
{
	return m_modelSkinning;
}
QStringList MainWidget::modelBoneList() const
{
	QStringList qsl;
	for (const auto& it : m_Mesh->Bones()) qsl << QString::fromLocal8Bit(it.first.c_str());
	return qsl;
}
void MainWidget::setModelName(const QString &modelName)
{
	m_modelName = modelName;
}
void MainWidget::setModelSkinning(bool state)
{
	m_Skin->enable();
	m_Skin->setSkinning(state);
	update();
}
SkinnedMesh *MainWidget::skinnedMesh()
{
	return m_Mesh;
}

SkinningTechnique *MainWidget::skinningTechnique()
{
	return m_Skin;
}

bool MainWidget::loadToGPU(const string& basename)
{
	// Release the previously loaded mesh (if it exists)
	unloadFromGPU();

	bool Ret = false;
	cout << "Loading model to GPU." << endl;
	for (int i = 0; i < m_Mesh->images().size(); i++) m_textures.push_back(new QOpenGLTexture(m_Mesh->images()[i]));

	// Create the VAO
	glGenVertexArrays(1, &m_VAO);
	glBindVertexArray(m_VAO);

	// Create the buffers for the vertices attributes
	glGenBuffers(ARRAY_SIZE_IN_ELEMENTS(m_Buffers), m_Buffers);

#define POSITION_LOCATION    0
#define TEX_COORD_LOCATION   1
#define NORMAL_LOCATION      2
#define BONE_ID_LOCATION     3
#define BONE_WEIGHT_LOCATION 4

	const auto& Positions = m_Mesh->positions();
	const auto& TexCoords = m_Mesh->texCoords();
	const auto& Normals = m_Mesh->normals();
	const auto& Bones = m_Mesh->vertexBoneData();
	const auto& Indices = m_Mesh->indices();

	// Generate and populate the buffers with vertex attributes and the indices
	glBindBuffer(GL_ARRAY_BUFFER, m_Buffers[POS_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Positions[0]) * Positions.size(), &Positions[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(POSITION_LOCATION);
	glVertexAttribPointer(POSITION_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_Buffers[TEXCOORD_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(TexCoords[0]) * TexCoords.size(), TexCoords.constData(), GL_STATIC_DRAW);
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

	//m_vertexArrayBytes = sizeof(Positions[0]) * Positions.size() + sizeof(TexCoords[0]) * TexCoords.size()
	//	+ sizeof(Normals[0]) * Normals.size() + sizeof(Bones[0]) * Bones.size();

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_Buffers[INDEX_BUFFER]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Indices[0]) * Indices.size(), &Indices[0], GL_STATIC_DRAW);

	// Make sure the VAO is not changed from the outside
	glBindVertexArray(0);

	cout << "SkinnedMesh::InitFromScene: "; GLPrintError();
	return GLCheckError();
}
void MainWidget::unloadFromGPU()
{
	for (uint i = 0; i < m_textures.size(); i++) {
		SAFE_DELETE(m_textures[i]);
	}

	if (m_Buffers[0] != 0) {
		glDeleteBuffers(ARRAY_SIZE_IN_ELEMENTS(m_Buffers), m_Buffers);
	}

	if (m_VAO != 0) {
		glDeleteVertexArrays(1, &m_VAO);
		m_VAO = 0;
	}
	m_successfullyLoaded = false;
}
void MainWidget::drawSkinnedMesh()
{
	if (m_successfullyLoaded) {
		glBindVertexArray(m_VAO);
		auto Entries = m_Mesh->entries();
		for (uint i = 0; i < Entries.size(); i++) {
			const uint MaterialIndex = Entries[i].MaterialIndex;

			assert(MaterialIndex < m_textures.size());

			if (m_textures[MaterialIndex]) {
				m_textures[MaterialIndex]->bind();
			}
			glDrawElementsBaseVertex(GL_TRIANGLES,
				Entries[i].NumIndices,
				GL_UNSIGNED_INT,
				(void*)(sizeof(uint) * Entries[i].BaseIndex),
				Entries[i].BaseVertex);
		}

		// Make sure the VAO is not changed from the outside    
		glBindVertexArray(0);
	}
}
void MainWidget::setKSensor(KSensor &ksensor)
{
	m_ksensor = &ksensor;
}
void MainWidget::updateIndirect()
{
	update();
}
void MainWidget::changeMode()
{
		if (m_modeOfOperation == Mode::CAPTURE) {
			m_modeOfOperation = Mode::PLAYBACK;
			m_timer.setInterval(m_playbackInterval);
			cout << "Mode: PLAYBACK" << endl;
		}
		else {
			m_modeOfOperation = Mode::CAPTURE;
			m_timer.setInterval(m_captureInterval);
			cout << "Mode: CAPTURE" << endl;
		}
		m_timer.start();
}