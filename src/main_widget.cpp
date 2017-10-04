#include "main_widget.h"
#include "skinned_mesh.h"
#include "skinning_technique.h"
#include "pipeline.h"
#include "camera.h"
#include "sensor.h"

// Qt
#include <QtCore\QDebug>
#include <QtGui\QKeyEvent>

MainWidget::MainWidget(QWidget *parent) : QOpenGLWidget(parent)
{
	m_Mesh = new SkinnedMesh();
}

// This virtual function is called once before the first call to paintGL() or resizeGL().
void MainWidget::initializeGL()
{
	m_Cam = new Camera();
	m_Sensor = new KSensor();
	m_Tech = new Technique();
	m_Skin = new SkinningTechnique();
	m_Pipe = new Pipeline();

	qDebug() << "Obtained format:" << format();
	initializeOpenGLFunctions();
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
void MainWidget::paintGL()
{
	glClearColor(0.f, 0.f, 0.f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//*
	m_Skin->enable();
	m_Skin->SetWVP(m_Pipe->GetWVPTrans());
	if (m_play) {
		m_Sensor->GetKinectData();
		Transform(false);
	}
	if (!m_modelName.isEmpty()) {
		m_Mesh->LoadMesh(string(m_modelName.toLocal8Bit()));
		m_modelName.clear();
	}
	if (m_renderModel) m_Mesh->Render();

	m_Tech->enable();
	//m_Tech->SetDefault(m_Pipe->GetWVPTrans());
	m_Tech->SetDefault(m_Pipe->GetVPTrans());
	if (m_renderAxes)				DrawAxes();		
	if (m_renderSkeleton)			m_Sensor->DrawSkeleton(JointType_SpineBase);
	if (m_renderActiveJoint)		m_Sensor->DrawActiveJoint();
	if (m_renderCloud)				m_Sensor->DrawCloud();
	//if (m_renderCameraVectors)		m_Cam->DrawCameraVectors();
	//*/
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
		m_Mesh->FlipParameter(key - Qt::Key_0);
		Transform(true);
		break;
	default:
		cout << "This key does not do anything." << endl;
		break;
	}
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
	Vector3f o(0.0f, 0.0f, 0.0f);
	Vector3f vx = Vector3f::UnitX;
	Vector3f vy = Vector3f::UnitY;
	Vector3f vz = Vector3f::UnitZ;
	DrawAxes(o, vx, vy, vz, 1);
	Matrix4f R;
	R.InitRotateTransform(45, 0, 0);
}
void MainWidget::NextInfoBlock(int step)
{
	activeInfo = Mod(activeInfo, NUM_INFO_BLOCKS, step);
	cout << "Printing info block " << activeInfo << endl;
	if (activeInfo == 0) m_Sensor->PrintJointData();
	else if (activeInfo == 1) m_Mesh->PrintInfo();
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
void MainWidget::MySetup()
{
	// 1) Init KSensor
	m_Sensor->Init();
	m_Sensor->PrintJointHierarchy();
	
	// 2) Init Mesh
	m_Mesh->setKSensor(*m_Sensor);
	m_Mesh->LoadMesh("cmu_test");
	m_Sensor->GetKinectData(); // to successfully acquire frame init sensor before mesh and load mesh before getting data

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
	directionalLight.Color = Vector3f(1.0f, 1.0f, 1.0f);
	directionalLight.AmbientIntensity = 0.7f;
	directionalLight.DiffuseIntensity = 0.9f;
	directionalLight.Direction = Vector3f(0.0f, -1.0, 0.0);
	m_Skin->SetDirectionalLight(directionalLight);
	m_Skin->SetMatSpecularIntensity(0.0f);
	m_Skin->SetMatSpecularPower(0);
	m_Skin->setSkinning(m_Mesh->m_Skinning);
	m_Skin->SetWVP(m_Pipe->GetWVPTrans());
	for (uint i = 0; i < m_Mesh->numBones(); i++) {
		m_Skin->setBoneVisibility(i, m_Mesh->boneVisibility(i));
	}
	Transform(true);
}
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
QStringList MainWidget::ModelBoneList() const
{
	QStringList qsl;
	for (const auto& it : m_Mesh->Bones()) {
		qsl << QString::fromLocal8Bit(it.first.c_str());
	}
	return qsl;
}
bool MainWidget::boneVisibility(const QString &boneName) const
{
	return m_Mesh->boneVisibility(boneName);
}
bool MainWidget::modelSkinning() const
{
	return m_modelSkinning;
}
void MainWidget::flipBonesVisibility()
{
	m_Mesh->flipBonesVisibility();
	m_Skin->enable();
	for (uint i = 0; i < m_Mesh->numBones(); i++) {
		m_Skin->setBoneVisibility(i, m_Mesh->boneVisibility(i));
	}
	update();
}
void MainWidget::setBoneVisibility(const QString &boneName, bool state)
{
	m_Mesh->setBoneVisibility(boneName, state);
	m_Skin->enable();
	m_Skin->setBoneVisibility(m_Mesh->findBoneId(boneName), state);
	update();
}
void MainWidget::setModelSkinning(bool state)
{
	m_Skin->enable();
	m_Skin->setSkinning(state);
	update();
}
void MainWidget::setModelName(const QString& modelName)
{
	m_modelName = modelName;
	update();
}



