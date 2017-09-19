#include "main_window.h"

MainWindow::MainWindow(QWidget *parent) : QOpenGLWidget(parent)
{
	// Setup scene and render it
	//initializeGL();
	//paintGL();	
}
void MainWindow::initializeGL()
{
	/*initializeOpenGLFunctions();
	glClearColor(1.0f, 0.0f, 0.0f, 1.0f);*/
	//m_context->makeCurrent(this);
	if (!initializeOpenGLFunctions()) cout << "Could not init OpenGL function." << endl;
	/*		 Create an OpenGL context
	m_context = new QOpenGLContext;
	m_context->create();
	//*/
	SetupOpenGL();
	m_Cam		= new Camera();
	m_Sensor	= new KSensor();
	m_Mesh		= new SkinnedMesh(*m_Sensor); //Mesh(mySensor)
	m_Tech		= new Technique();
	m_Skin		= new SkinningTechnique();
	m_Pipe		= new Pipeline();
	MySetup();
}
void MainWindow::paintGL()
{
	//glClear(GL_COLOR_BUFFER_BIT);
	glClearColor(0.f, 0.f, 0.f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//*
	m_Skin->Enable();
	m_Skin->SetWVP(m_Pipe->GetWVPTrans());
	if (m_play) {
		m_Sensor->GetKinectData();
		Transform(false);
	}
	if (m_renderObject) m_Mesh->Render();

	m_Tech->Enable();
	//m_Tech->SetDefault(m_Pipe->GetWVPTrans());
	m_Tech->SetDefault(m_Pipe->GetVPTrans());
	if (m_renderAxes)				DrawAxes();
	if (m_renderCameraVectors)		m_Cam->DrawCameraVectors();
	if (m_renderJoint)				m_Sensor->DrawActiveJoint();
	if (m_renderSkeleton)			m_Sensor->DrawSkeleton(JointType_SpineBase);
	if (m_renderCloud)				m_Sensor->DrawCloud();
	//*/
	//glutSwapBuffers();
}
void MainWindow::resizeGL(int w, int h)
{

}
void MainWindow::SetupOpenGL()
{
	cout << "GL version: " << glGetString(GL_VERSION) << endl;
	GLint i;
	glGetIntegerv(GL_CONTEXT_FLAGS, &i);
	cout << "GL renderer: " << glGetString(GL_RENDERER) << endl;
	cout << "Shaders version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
	/*
	glGetIntegerv(GL_NUM_EXTENSIONS, &i);
	for (uint j = 0; j < i; j++) cout << "Extension " << j << ": " << glGetStringi(GL_EXTENSIONS, j) << endl;
	cout << "Legacy extensions: " << glGetString(GL_EXTENSIONS) << endl;
	//*/
	glEnable(GL_TEXTURE_2D);
	//glShadeModel(GL_SMOOTH);                            // Enable Smooth Shading
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);               // Black Background
	//glClearDepth(1.0f);                                 // Depth Buffer Setup
	glEnable(GL_POINT_SMOOTH);							// Points represented as circles
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glDisable(GL_CULL_FACE);
}
void MainWindow::DrawAxes()
{
	/*
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
void MainWindow::DrawAxes(Vector3f origin, Vector3f vx, Vector3f vy, Vector3f vz, float length)
{
	/*
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
void MainWindow::DrawTestAxes()
{
	Vector3f o(0.0f, 0.0f, 0.0f);
	Vector3f vx = Vector3f::UnitX;
	Vector3f vy = Vector3f::UnitY;
	Vector3f vz = Vector3f::UnitZ;
	DrawAxes(o, vx, vy, vz, 1);
	Matrix4f R;
	R.InitRotateTransform(45, 0, 0);

}
void MainWindow::NextInfoBlock(int step)
{
	activeInfo = Mod(activeInfo, NUM_INFO_BLOCKS, step);
	cout << "Printing info block " << activeInfo << endl;
	if (activeInfo == 0) m_Sensor->PrintJointData();
	else if (activeInfo == 1) m_Mesh->PrintInfo();
	else m_Cam->PrintInfo();
}
void MainWindow::Transform(bool print)
{
	//if (!print) cout.setstate(std::ios_base::failbit);
	if (m_Mesh->m_SuccessfullyLoaded) {
		if (print) cout << "Transforming bones." << endl;
		vector<Matrix4f> Transforms;
		m_Mesh->GetBoneTransforms(Transforms); // update bone transforms from kinect
		m_Skin->Enable();
		for (uint i = 0; i < Transforms.size(); i++) {
			m_Skin->SetBoneTransform(i, Transforms[i]); // send transforms to vertex shader
		}
	}
	else {
		if (print) cout << "Mesh is not successfully loaded. Cannot transform bones." << endl;
	}
	//if (!print) cout.clear();
}
//*/
void MainWindow::MySetup()
{
	// 1) Init KSensor
	m_Sensor->Init();
	m_Sensor->PrintJointHierarchy();
	// 2) Init SkinnedMesh
	m_Mesh->NextModel(0);
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
	m_Tech->Enable();
	m_Tech->SetDefault(m_Pipe->GetWVPTrans());

	// 6) Init SkinningTechnique
	if (!m_Skin->Init()) cout << "Could not initialize skinning shaders" << endl;
	m_Skin->Enable();
	m_Skin->SetColorTextureUnit(0);
	DirectionalLight directionalLight;
	directionalLight.Color = Vector3f(1.0f, 1.0f, 1.0f);
	directionalLight.AmbientIntensity = 0.7f;
	directionalLight.DiffuseIntensity = 0.9f;
	directionalLight.Direction = Vector3f(0.0f, -1.0, 0.0);
	m_Skin->SetDirectionalLight(directionalLight);
	m_Skin->SetMatSpecularIntensity(0.0f);
	m_Skin->SetMatSpecularPower(0);
	m_Skin->SetSkinningSwitch(m_Mesh->m_Skinned);
	m_Skin->SetWVP(m_Pipe->GetWVPTrans());
	for (uint i = 0; i < m_Mesh->GetNumBones(); i++) {
		m_Skin->SetBoneVisibility(i, m_Mesh->GetBoneVisibility(i));
	}
	Transform(true);
}