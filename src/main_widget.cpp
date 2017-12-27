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
	, m_skinnedMesh(new SkinnedMesh())
{
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

	delete m_skinnedMesh;
	delete m_Cam;
	delete m_Tech;
	delete m_Skin;
	delete m_Pipe;
}
// This virtual function is called once before the first call to paintGL() or resizeGL().
void MainWidget::initializeGL()
{
	qDebug() << "Obtained format:" << format();
	initializeOpenGLFunctions();


	m_Cam = new Camera();
	m_Tech = new Technique();
	m_Skin = new SkinningTechnique();
	m_Pipe = new Pipeline();
	m_playbackInterval = m_ksensor->skeleton()->timeStep() * 1000;
	m_ksensor->skeleton()->initOGL();
	m_skinnedMesh->initOGL();
	m_skinnedMesh->loadAxesToGPU();

	//cout << "GL version: " << glGetString(GL_VERSION) << endl;
	//cout << "GL renderer: " << glGetString(GL_RENDERER) << endl;
	//cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;	
	GLint i;
	glGetIntegerv(GL_CONTEXT_FLAGS, &i);
	
	glEnable(GL_TEXTURE_2D);
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);               
	glClearDepth(1.0f);									
	glEnable(GL_POINT_SMOOTH);							
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
	m_successfullyLoaded = loadToGPU("cmu_test");
	m_Cam->printInfo();

	// Init Pipeline
	m_Pipe->SetCamera(m_Cam->GetPos(), m_Cam->GetTarget(), m_Cam->GetUp());

	PersProjInfo persProjInfo;
	persProjInfo.FOV = 60.0f;
	persProjInfo.Height = m_Cam->GetHeight();
	persProjInfo.Width = m_Cam->GetWidth();
	persProjInfo.zNear = 0.1f;
	persProjInfo.zFar = 1000.0f;
	m_Pipe->SetPerspectiveProj(persProjInfo);

	/*m_linesProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, "shaders/axes.vs");
	qDebug() << m_linesProgram.log();
	m_linesProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, "shaders/axes.fs");
	qDebug() << m_linesProgram.log();
	if (!m_linesProgram.link()) cout << "Could not link axes shader program" << endl;
	m_linesProgram.bind();*/

	// 5) Init Technique
	if (!m_Tech->InitDefault()) cout << "Could not initialize default shaders" << endl;
	m_Tech->enable();
	m_Tech->setMVP(m_Pipe->GetWVPTrans());
	m_Tech->setSpecific(Matrix4f::Identity());

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
	for (uint i = 0; i < m_skinnedMesh->numBones(); i++) {
		m_Skin->setBoneVisibility(i, m_skinnedMesh->boneVisibility(i));
	}
	Transform(true);
}
void MainWidget::paintGL()
{
	glClearColor(0.f, 0.f, 0.f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_modeOfOperation == Mode::CAPTURE) {
		m_ksensor->getBodyData();
	}
	else {
		m_ksensor->skeleton()->getActiveFrame();
	}
	
	// draw skinned mesh
	m_Skin->enable();
	QQuaternion& q = m_skinnedMesh->worldRotation();
	m_Pipe->setWorldRotation(q);
	QVector3D& p = m_skinnedMesh->worldPosition();
	m_Pipe->setWorldPosition(p);
	m_Skin->SetWVP(m_Pipe->GetWVPTrans());
	if (m_play) {
		Transform(false);
	}
	if (m_renderSkinnedMesh && m_successfullyLoaded) {
		drawSkinnedMesh();
	}

	// draw skinned mesh bone axis
	m_Tech->enable();
	m_Tech->setMVP(m_Pipe->GetWVPTrans());
	m_Tech->setSpecific(m_skinnedMesh->boneGlobal(m_activeBone));
	m_skinnedMesh->drawBoneAxis();
	
	// draw axis
	m_Tech->setSpecific(Matrix4f::Identity());
	m_Tech->setMVP(m_Pipe->GetVPTrans()); // only VP transformation!
	if (m_renderAxes) m_skinnedMesh->drawBoneAxis();
	if (m_renderSkeleton) m_ksensor->skeleton()->drawSkeleton();

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
		m_Cam->onKeyboardArrow(key, false);
		m_Pipe->SetCamera(m_Cam->GetPos(), m_Cam->GetTarget(), m_Cam->GetUp());
		m_Skin->enable();
		m_Skin->SetEyeWorldPos(m_Cam->GetPos());
		m_Skin->SetWVP(m_Pipe->GetWVPTrans());
		m_Tech->enable();
		m_Tech->setMVP(m_Pipe->GetVPTrans());
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
		Transform(true);
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
	case Qt::Key_C:
		if (!m_ksensor->connect()) cout << "Could not connect to kinect sensor." << endl;
		break;
	case Qt::Key_J:
		m_ksensor->skeleton()->printJoints();
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
		if (m_modeOfOperation==Mode::CAPTURE) m_ksensor->record();
		else cout << "Record does not work in this mode" << endl;
		break;
	case Qt::Key_S:
		m_ksensor->skeleton()->saveToBinary();
		break;
	case Qt::Key_Q:
		m_ksensor->skeleton()->printSequence();
		break;
	case Qt::Key_T:
		m_ksensor->skeleton()->writeTRC();
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

	if (event->buttons() & Qt::LeftButton){
		if (event->modifiers() & Qt::ControlModifier) {
			m_Cam->rotateRight(-dx);
		}
		else if (event->modifiers() & Qt::ShiftModifier) {
			m_Cam->rotateUp(-dy);
		}
		else {
			m_Cam->rotateRight(-dx);
			m_Cam->rotateUp(-dy);
		}
	}
	m_lastPos = event->pos();

	m_Pipe->SetCamera(m_Cam->GetPos(), m_Cam->GetTarget(), m_Cam->GetUp());
	m_Skin->enable();
	m_Skin->SetEyeWorldPos(m_Cam->GetPos());
	m_Skin->SetWVP(m_Pipe->GetWVPTrans());
	m_Tech->enable();
	m_Tech->setMVP(m_Pipe->GetVPTrans());
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
	m_Tech->setMVP(m_Pipe->GetVPTrans());
	update();
}
void MainWidget::NextInfoBlock(int step)
{
	activeInfo = Mod(activeInfo, NUM_INFO_BLOCKS, step);
	cout << "Printing info block " << activeInfo << endl;
	if (activeInfo == 0) m_skinnedMesh->PrintInfo();
	else m_Cam->printInfo();
}
void MainWidget::Transform(bool print)
{
	//if (!print) cout.setstate(std::ios_base::failbit);
	if (m_skinnedMesh->m_SuccessfullyLoaded) {
		if (print) cout << "Transforming bones." << endl;
		vector<Matrix4f> Transforms;
		m_skinnedMesh->GetBoneTransforms(Transforms); // update bone transforms from kinect
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
void MainWidget::setRenderModel(bool state)
{
	if (m_renderSkinnedMesh != state) {
		m_renderSkinnedMesh = state;
		update();
	}
}
void MainWidget::setRenderSkeleton(bool state)
{
	if (m_renderSkeleton != state) {
		m_renderSkeleton = state;
		update();
	}
}
bool MainWidget::renderAxes() const
{
	return m_renderAxes;
}
bool MainWidget::renderModel() const
{
	return m_renderSkinnedMesh;
}
bool MainWidget::renderSkeleton() const
{
	return m_renderSkeleton;
}
bool MainWidget::modelSkinning() const
{
	return m_modelSkinning;
}
QStringList MainWidget::modelBoneList() const
{
	QStringList qsl;
	for (const auto& it : m_skinnedMesh->Bones()) qsl << QString::fromLocal8Bit(it.first.c_str());
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
	return m_skinnedMesh;
}

SkinningTechnique *MainWidget::skinningTechnique()
{
	return m_Skin;
}
Technique* MainWidget::technique()
{
	return m_Tech;
}
bool MainWidget::loadToGPU(const string& basename)
{
	// Release the previously loaded mesh (if it exists)
	unloadFromGPU();

	bool Ret = false;
	cout << "Loading model to GPU." << endl;
	for (int i = 0; i < m_skinnedMesh->images().size(); i++) m_textures.push_back(new QOpenGLTexture(m_skinnedMesh->images()[i]));

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

	const auto& Positions = m_skinnedMesh->positions();
	const auto& TexCoords = m_skinnedMesh->texCoords();
	const auto& Normals = m_skinnedMesh->normals();
	const auto& Bones = m_skinnedMesh->vertexBoneData();
	const auto& Indices = m_skinnedMesh->indices();

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
		auto Entries = m_skinnedMesh->entries();
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
void MainWidget::setKSensor(KSensor& ksensor)
{
	m_ksensor = &ksensor;
}
void MainWidget::setBoneAxes(const QString &boneName)
{
	uint boneId = m_skinnedMesh->findBoneId(boneName);
	assert(boneId < m_boneInfo.size());
	m_Tech->enable();
	m_Tech->setSpecific(m_skinnedMesh->boneGlobal(boneId));
}
void MainWidget::setActiveBone(const QString& boneName)
{
	m_activeBone = m_skinnedMesh->findBoneId(boneName);
}