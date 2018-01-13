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
	, m_skinnedMesh(new SkinnedMesh())
{
	cout << "MainWidget constructor start." << endl;
	m_VAO = 0;
	ZERO_MEM(m_Buffers);

	cout << "MainWidget constructor end." << endl;
}
MainWidget::~MainWidget()
{
	makeCurrent();
	unloadSkinnedMesh();
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
	cout << "MainWidget initializeGL start." << endl;

	qDebug() << "Obtained format:" << format();
	initializeOpenGLFunctions();


	m_Cam = new Camera();
	m_Tech = new Technique();
	m_Skin = new SkinningTechnique();
	m_Pipe = new Pipeline();
	m_ksensor->skeleton()->initOGL();
	m_skinnedMesh->initOGL();
	loadAxes();
	loadArrow();

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
	
	// glShadeModel(GL_SMOOTH); // #? deprecated
	MySetup();

	m_timer.setTimerType(Qt::PreciseTimer);
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(updateIndirect()));
	m_modeOfOperation = Mode::PLAYBACK;
	m_playbackInterval = m_ksensor->skeleton()->timeStep() * 1000;
	m_timer.setInterval(m_playbackInterval);
	m_timer.start();

	cout << "MainWidget initializeGL end." << endl;
}
void MainWidget::MySetup()
{
	cout << "MainWidget setup start." << endl;
	m_successfullyLoaded = loadSkinnedMesh("cmu_test");

	// Init Pipeline
	QVector3D spineBase = (m_ksensor->skeleton()->joints())[JointType_SpineBase].position;
	//m_Cam->setOffset(spineBase);
	m_Cam->printInfo();
	m_Pipe->SetCamera(m_Cam->GetPos(), m_Cam->GetTarget(), m_Cam->GetUp());

	PersProjInfo persProjInfo;
	persProjInfo.FOV = 60.0f;
	persProjInfo.Height = m_Cam->windowHeight();
	persProjInfo.Width = m_Cam->windowWidth();
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
	m_Tech->setSpecific(QMatrix4x4());

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
	transformSkinnedMesh(true);

	cout << "MainWidget setup end." << endl;
}
// #? must enable corresponding shading technique before using each drawing function. bad design?
void MainWidget::paintGL()
{
	glClearColor(0.f, 0.f, 0.f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_modeOfOperation == Mode::CAPTURE) {
		m_ksensor->getBodyData();
	}
	else if (m_modeOfOperation == Mode::PLAYBACK){
		m_ksensor->skeleton()->setActiveJoints(m_activeFrame);
		m_skinnedMesh->setActiveCoordinates(m_activeFrame);
	}
	m_time = m_skinnedMesh->timestamp(m_activeFrame);
	QVector3D offset(0.f, 0.f, -2.f);

	// draw skinned mesh
	m_Skin->enable();
	QQuaternion& q = m_skinnedMesh->worldRotation();
	if (m_defaultPose) m_Pipe->setWorldRotation(QQuaternion()); else m_Pipe->setWorldRotation(q);
	QVector3D& v = m_skinnedMesh->worldPosition();
	if (m_defaultPose) m_Pipe->setWorldPosition(QVector3D()); else m_Pipe->setWorldPosition(v + offset);
	m_Skin->SetWVP(m_Pipe->GetWVPTrans());
	if (m_modeOfOperation == Mode::PLAYBACK && m_play) {
		transformSkinnedMesh(false);
	}
	if (m_renderSkinnedMesh && m_successfullyLoaded) {
		drawSkinnedMesh();
	}

	m_Tech->enable();

	// draw skinned mesh bone axes
	m_Tech->setMVP(m_Pipe->GetWVPTrans());
	m_Tech->setSpecific(m_skinnedMesh->boneGlobal(m_activeBone));
	if (m_renderAxes) drawAxes();
	
	// draw basic axes
	m_Tech->setMVP(m_Pipe->GetVPTrans()); // only VP transformation! #! changed view transform to displace all
	m_Tech->setSpecific(QMatrix4x4());
	if (m_renderAxes) drawAxes();

	// draw skeleton
	m_Pipe->setWorldRotation(QQuaternion());
	if (m_defaultPose) m_Pipe->setWorldPosition(QVector3D()); else m_Pipe->setWorldPosition(offset);
	m_Tech->setMVP(m_Pipe->GetWVPTrans()); 
	if (m_renderSkeleton) m_ksensor->skeleton()->drawSkeleton();

	// draw arrow
	QVector3D leftHand = (m_ksensor->skeleton()->joints())[JointType_HandLeft].position;
	QVector3D rightHand = (m_ksensor->skeleton()->joints())[JointType_HandRight].position;
	QVector3D barDirection = rightHand - leftHand;
	m_barAngle = ToDegrees(atan2(barDirection.y(), sqrt(pow(barDirection.x(), 2) + pow(barDirection.z(), 2))));
	
	static QVector3D previousBarPosition, currentBarPosition;
	currentBarPosition = (leftHand + rightHand) * 0.5;
	m_barSpeed = (currentBarPosition - previousBarPosition) / m_playbackInterval * 1000;
	previousBarPosition = currentBarPosition;

	QVector3D hipRight = (m_ksensor->skeleton()->joints())[JointType_HipRight].position;
	QVector3D kneeRight = (m_ksensor->skeleton()->joints())[JointType_KneeRight].position;
	QVector3D ankleRight = (m_ksensor->skeleton()->joints())[JointType_AnkleRight].position;
	QVector3D kneeToHip = (hipRight - kneeRight).normalized();
	QVector3D kneeToAnkle = (ankleRight - kneeRight).normalized();
	m_kneeAngle = 180.f-ToDegrees(acos(QVector3D::dotProduct(kneeToHip, kneeToAnkle)));
	
	// Scaling
	QMatrix4x4 S;
	S.scale(1, (rightHand - leftHand).length(), 1);
	// Rotation
	QMatrix4x4 R;
	R.rotate(QQuaternion::rotationTo(QVector3D(0, 1, 0), barDirection));
	// Translation
	QMatrix4x4 T;
	T.translate(leftHand);

	m_Tech->setSpecific(T * R * S);
	drawArrow();

	if (m_play) {
		if (++m_activeFrame > m_ksensor->skeleton()->sequenceSize())  m_activeFrame = 0;
	}

	calculateFPS();
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
	case Qt::Key_C:
		if (!m_ksensor->open()) cout << "Could not open kinect sensor." << endl;
		break;
	case Qt::Key_D:
		m_defaultPose = !m_defaultPose;
		cout << "Default pause " << (m_defaultPose ? "ON" : "OFF") << endl;
		break;
	case Qt::Key_J:
		m_ksensor->skeleton()->printJoints();
		m_ksensor->skeleton()->printJointBufferData();
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
	m_lastMousePosition = event->pos();

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
	if (activeInfo == 0) m_skinnedMesh->printInfo();
	else m_Cam->printInfo();
}
void MainWidget::transformSkinnedMesh(bool print)
{
	//if (!print) cout.setstate(std::ios_base::failbit);
	if (m_skinnedMesh->m_SuccessfullyLoaded) {
		if (print) cout << "Transforming bones." << endl;
		vector<QMatrix4x4> transforms;
		m_skinnedMesh->getBoneTransforms(transforms); // update bone transforms from kinect
		m_Skin->enable();
		for (uint i = 0; i < transforms.size(); i++) {
			m_Skin->setBoneTransform(i, transforms[i]); // send transforms to vertex shader
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
SkinnedMesh* MainWidget::skinnedMesh()
{
	return m_skinnedMesh;
}
SkinningTechnique* MainWidget::skinningTechnique()
{
	return m_Skin;
}
Technique* MainWidget::technique()
{
	return m_Tech;
}
bool MainWidget::loadSkinnedMesh(const string& basename)
{
	// Release the previously loaded mesh (if it exists)
	unloadSkinnedMesh();

	bool Ret = false;
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

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_Buffers[INDEX_BUFFER]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Indices[0]) * Indices.size(), &Indices[0], GL_STATIC_DRAW);

	// Make sure the VAO is not changed from the outside
	glBindVertexArray(0);

	Ret = GLCheckError();
	if (Ret) cout << "Successfully loaded SkinnedMesh to GPU (MainWidget)" << endl;
	return Ret;
}
void MainWidget::unloadSkinnedMesh()
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
void MainWidget::setActiveFrame(uint index)
{
	m_activeFrame = (uint)(index / 100.f * m_ksensor->skeleton()->sequenceSize());
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
void MainWidget::loadArrow()
{
	const float head = 0.1f;
	const float colorRed = 255.f;
	const float colorBlue = 255.f;
	const float colorGreen = 255.f;

	const GLfloat vertices[] =
	{
		0    , 0, 0, // start
		colorRed  , colorGreen, colorBlue,
		0    , 1, 0, // end
		colorRed  , colorGreen, colorBlue,
		head , 1 - head, 0, // +x
		colorRed  , colorGreen, colorBlue,
		-head, 1 - head, 0, // -x
		colorRed  , colorGreen, colorBlue,
		0    , 1 - head, head, // +z
		colorRed  , colorGreen, colorBlue,
		0    , 1 - head, -head, // -z
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
	glBindVertexArray(m_arrowVAO);

	GLuint arrowIBO;
	glGenBuffers(1, &arrowIBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, arrowIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	GLuint arrowVBO;
	glGenBuffers(1, &arrowVBO);
	glBindBuffer(GL_ARRAY_BUFFER, arrowVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, BUFFER_OFFSET(sizeof(GLfloat) * 3));

	glBindVertexArray(0); // break the existing vertex array object binding
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

	GLuint axesIBO;
	glGenBuffers(1, &axesIBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, axesIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	GLuint axesVBO;
	glGenBuffers(1, &axesVBO);
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