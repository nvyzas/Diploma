#pragma once
#include "skinned_mesh.h"
#include "skinning_technique.h"
#include "pipeline.h"
#include "camera.h"
#include "math_3d.h"
#include "sensor.h"
#include <iostream>
#include <iomanip>
using namespace std;

void KeyPressed(unsigned char key, int x, int y);
void SpecialKeyPressed(int key, int x, int y);

Camera myCam(WIDTH, HEIGHT);
KSensor mySensor;
SkinnedMesh myMesh(mySensor);
Technique myTech;
SkinningTechnique mySkin;
Pipeline myPipe;
Matrix4f proj;
DirectionalLight directionalLight;
PersProjInfo persProjInfo;

bool renderCloud = false;
bool renderSkeleton = true;
bool renderAxes = true;
bool renderObject = true;
bool renderQuats = true;
bool play = false;

#define NUM_INFO_BLOCKS 3
uint activeJoint = JointType_SpineBase;
void NextJoint(int step);
uint activeInfo = 0;
void NextInfoBlock(int step);

void Transform(bool print);
void Draw();
void DrawAxes();
void DrawAxes(Vector3f center, Vector3f vx, Vector3f vy, Vector3f vz, float length);
void DrawTestAxes();
void DrawQuats();
void DrawSkeleton(uint id);


void SetupOGL() {
	// OpenGL setup
	printf("GL version: %s\n", glGetString(GL_VERSION));
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);                            // Enable Smooth Shading
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);               // Black Background
	glClearDepth(1.0f);                                 // Depth Buffer Setup
	glEnable(GL_POINT_SMOOTH);							// Points represented as circles
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	
	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glDisable(GL_CULL_FACE);

	// Lighting
	glEnable(GL_LIGHTING);

	// Set up Light 0
	GLfloat light0Ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat light0Diffuse[] = { 0.6f, 0.6f, 0.1f, 1.0f };
	GLfloat light0Specular[] = { 1.0f, 1.0f, 0.2f, 1.0f };
	GLfloat light0Position[] = { 50.0f, 30.0f, -200.0f, 1.0f };
	GLfloat light0Direction[] = { -1.0f, -1.0f, -1.0f, 0.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, light0Ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0Diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light0Specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light0Position);
	// Set Attenuation of the light a*x^2 + b*x + c
	//glLightf (GL_LIGHT0, GL_CONSTANT_ATTENUATION, 2);
	//glLightf (GL_LIGHT0, GL_LINEAR_ATTENUATION, 1);
	//glLightf (GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 1);
	// Make Light a spot light setting the cutoff angle and direction
	//glLightf (GL_LIGHT0, GL_SPOT_CUTOFF, 35.0);
	//glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light0Direction);
	//glLightf (GL_LIGHT0, GL_SPOT_EXPONENT, 2.0); // [0,128]

	glEnable(GL_LIGHT0); // Enable Light0

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE); // We use Ambient and Diffuse color of each object as given in color3f

	// Set Material parameters
	//*
	GLfloat ambref[] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat difref[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat specref[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	GLint shininess = 128.0;
	//*/

	// Material Parameters for Gold
	/*
	GLfloat ambref[]  = {0.247, 0.225, 0.065, 1.0};
	GLfloat difref[]  = {0.346, 0.314, 0.090, 1.0};
	GLfloat specref[] = {0.797, 0.724, 0.208, 1.0};
	GLint shininess   = 83.2;
	//*/
	// Material Parameters for Silver
	/*
	GLfloat ambref[]  = {0.231, 0.231, 0.231, 1.0};
	GLfloat difref[]  = {0.278, 0.278, 0.278, 1.0};
	GLfloat specref[] = {0.774, 0.774, 0.774, 1.0};
	GLint shininess   = 89.6;
	//*/
	// Material Parameters for Cooper
	/*
	GLfloat ambref[]  = {0.230, 0.089, 0.028, 1.0};
	GLfloat difref[]  = {0.551, 0.212, 0.066, 1.0};
	GLfloat specref[] = {0.581, 0.223, 0.070, 1.0};
	GLint shininess   = 51.2;
	//*/
	// Material Parameters for Ruby
	/*
	GLfloat ambref[]  = {0.175, 0.012, 0.012, 0.55};
	GLfloat difref[]  = {0.614, 0.041, 0.041, 0.55};
	GLfloat specref[] = {0.728, 0.308, 0.308, 0.55};
	GLint shininess   = 76.8;
	//*/
	// Material Parameters for Black Plastic
	/*
	GLfloat ambref[]  = {0.0,  0.0,  0.0,  1.0};
	GLfloat difref[]  = {0.01, 0.01, 0.01, 1.0};
	GLfloat specref[] = {0.5,  0.5,  0.5,  1.0};
	GLint shininess   = 32;
	//*/
	// Material Parameters for Black Rubber
	/*
	GLfloat ambref[]  = {0.02, 0.02, 0.02, 1.0};
	GLfloat difref[]  = {0.01, 0.01, 0.01, 1.0};
	GLfloat specref[] = {0.4,  0.4,  0.4,  1.0};
	GLint shininess   = 10;
	//*/

	// Set Material parameters
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambref);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, difref);
	glMaterialfv(GL_FRONT, GL_SPECULAR, specref);
	glMaterialf(GL_FRONT, GL_SHININESS, shininess);

	glEnable(GL_NORMALIZE);
}

void SpecialKeyPressed(int key, int x, int y)
{
	myCam.OnKeyboardSpecial(key, true);
	myPipe.SetCamera(myCam.GetPos(), myCam.GetTarget(), myCam.GetUp());
	mySkin.Enable();
	mySkin.SetEyeWorldPos(myCam.GetPos());
	mySkin.SetWVP(myPipe.GetWVPTrans());
	myTech.Enable();
	myTech.SetDefault(myPipe.GetVPTrans());
	glutPostRedisplay();
}

void MouseMotion(int x, int y)
{
	myCam.OnMouse(x, y);
	myPipe.SetCamera(myCam.GetPos(), myCam.GetTarget(), myCam.GetUp());
	mySkin.SetEyeWorldPos(myCam.GetPos());
	mySkin.SetWVP(myPipe.GetWVPTrans());
	glutPostRedisplay();
}

void MouseEvent(int button, int state, int x, int y)
{
	glutPostRedisplay();
}

void Idle()
{
	glutPostRedisplay();
}

void Execute()
{
	glutIgnoreKeyRepeat(0);
	glutKeyboardFunc(KeyPressed);
	glutSpecialFunc(SpecialKeyPressed);
	glutMouseFunc(MouseEvent);
	//glutMotionFunc(MouseMotion);
	glutIdleFunc(Idle);
	glutMainLoop();
}

bool InitGLUT(int argc, char* argv[]) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutCreateWindow("Diploma");
	glutDisplayFunc(Draw);
	glutIdleFunc(Draw);

	// Must be done after glut is initialized!
	GLenum res = glewInit();
	if (res != GLEW_OK) {
		printf("Error: '%s'\n", glewGetErrorString(res));
		return false;
	}
	return true;
}

void DrawAxes()
{
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
}
void DrawAxes(Vector3f origin, Vector3f vx, Vector3f vy, Vector3f vz, float length)
{
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
}
void DrawQuats()
{
	glBegin(GL_LINES);
	const Vector3f &p = mySensor.m_Joints[activeJoint].Position;	
	const Quaternion &q = mySensor.m_Joints[activeJoint].Orientation;
	Vector3f v;

	v = q.RotateVector(Vector3f(1, 0, 0));
	v += p;
	glColor3f(0xFF, 0xFF, 0);
	glVertex3f(p.x, p.y, p.z);
	glVertex3f(v.x, v.y, v.z);

	v = q.RotateVector(Vector3f(0, 1, 0));
	v += p;
	glColor3f(0, 0xFF, 0xFF);
	glVertex3f(p.x, p.y, p.z);
	glVertex3f(v.x, v.y, v.z);

	v = q.RotateVector(Vector3f(0, 0, 1));
	v += p;
	glColor3f(0xFF, 0, 0xFF);
	glVertex3f(p.x, p.y, p.z);
	glVertex3f(v.x, v.y, v.z);
	glEnd();
}
void DrawCameraVectors()
{
	glBegin(GL_LINES);
	const Vector3f &c = myCam.GetCenter();
	Vector3f v;
	float f=2;
	v = c + (myCam.GetTarget()*f);
	glColor3f(0, 0, 0xFF);
	glVertex3f(c.x, c.y, c.z);
	glVertex3f(v.x, v.y, v.z);

	v = c + (myCam.GetUp()*f);
	glColor3f(0, 0xFF, 0);
	glVertex3f(c.x, c.y, c.z);
	glVertex3f(v.x, v.y, v.z);

	v = c + (myCam.GetRight()*f);
	glColor3f(0xFF, 0, 0);
	glVertex3f(c.x, c.y, c.z);
	glVertex3f(v.x, v.y, v.z);
	glEnd();
}
void DrawSkeleton(uint id)
{	
	KJoint j = mySensor.m_Joints[id];	
	for (uint i = 0; i < j.children.size(); i++) {
		uint c = j.children[i];
		const KJoint &cj = mySensor.m_Joints[c];
		glBegin(GL_LINES);
		glColor3f(0xFF, 0xFF, 0xFF);
		glVertex3f(j.Position.x, j.Position.y, j.Position.z);
		glVertex3f(cj.Position.x, cj.Position.y, cj.Position.z);
		glEnd();
		DrawSkeleton(c);
	}
}

void DrawTestAxes()
{
	Vector3f o(0.0f, 0.0f, 0.0f);
	Vector3f vx = Vector3f::UnitX;
	Vector3f vy = Vector3f::UnitY;
	Vector3f vz = Vector3f::UnitZ;
	DrawAxes(o, vx, vy, vz, 1);
	Matrix4f R;
	R.InitRotateTransform(45, 0, 0);
	
}
void NextJoint(int step)
{
	activeJoint = Mod(activeJoint, JointType_Count, step);
	const KJoint &j = mySensor.m_Joints[activeJoint];
	cout << left << setw(2) << activeJoint << ": " << setw(15) << j.name << " p=" << left << setw(25) << j.Position.ToString() << " q=" << j.Orientation.ToString() << j.Orientation.ToEulerAnglesString() << j.Orientation.ToAxisAngleString() << endl;
}
void NextInfoBlock(int step)
{
	activeInfo = Mod(activeInfo, NUM_INFO_BLOCKS, step);
	cout << "Printing info block " << activeInfo << endl;
	if (activeInfo == 0) mySensor.PrintJointData(); 
	else if (activeInfo == 1) myMesh.PrintInfo();
	else myCam.PrintInfo();
}
void Transform(bool print)
{
	//if (!print) cout.setstate(std::ios_base::failbit);
	if (myMesh.m_SuccessfullyLoaded) {	
		if (print) cout << "Transforming bones" << endl;
		vector<Matrix4f> Transforms;
		myMesh.GetBoneTransforms(Transforms); // update bone transforms from kinect
		mySkin.Enable();
		for (uint i = 0; i < Transforms.size(); i++) {
			mySkin.SetBoneTransform(i, Transforms[i]); // send transforms to vertex shader
		}
	}
	else {
		if (print) cout << "Mesh is not successfully loaded. Cannot transform bones." << endl;
	}
	//if (!print) cout.clear();
}
void KeyPressed(unsigned char key, int x, int y)
{
	int mod = glutGetModifiers();
	if (key == 27) exit(0);
	switch (key) {
	case 'a': renderAxes = !renderAxes; break;
	case 'c': renderCloud = !renderCloud; break;
	case 'o': renderObject = !renderObject; break;
	case 's': renderSkeleton = !renderSkeleton; break;
	case 'w': 
		mySensor.m_InvertedSides = !mySensor.m_InvertedSides;
		cout << "InvertedSides: " << mySensor.m_InvertedSides << endl;
		mySensor.SwapSides();
		//mySensor.GetKinectData();
		Transform(true);
		break;
	case 'e':
		//myMesh.ToggleSkinning(); // does not work properly
		myMesh.m_Skinned = !myMesh.m_Skinned;
		cout << "Skin " << (myMesh.m_Skinned ? "ON" : "OFF") << endl;
		mySkin.Enable();
		mySkin.SetSkinningSwitch(myMesh.m_Skinned);
		Transform(false);
		break;
	case 'q': if (mod & GLUT_ACTIVE_ALT) NextJoint(-1); else NextJoint(0); break;
	case 'Q': NextJoint(1); break;
	case 'm':
		if (mod & GLUT_ACTIVE_ALT) myMesh.NextModel(-1); else myMesh.NextModel(0);
		Transform(true);
		break;
	case 'M': myMesh.NextModel(1); Transform(true); break;
	case 'j':
		if (mod & GLUT_ACTIVE_ALT) myMesh.NextJoint(-1); else myMesh.NextJoint(0); break;
	case 'J':
		myMesh.NextJoint(1); break;
	case 'r':
		break;
	case 'R':
		break;
	case 'n':
		if (mod & GLUT_ACTIVE_ALT) myMesh.NextBoneTransformInfo(-1); else myMesh.NextBoneTransformInfo(0); break;
	case 'N': myMesh.NextBoneTransformInfo(1); break;
	case 'i': if (mod & GLUT_ACTIVE_ALT) NextInfoBlock(-1); else NextInfoBlock(0); break;
	case 'I': NextInfoBlock(1); break;
	case 'b':
		myMesh.m_BindPose = !myMesh.m_BindPose;
		cout << (myMesh.m_BindPose ? "Bind pose on" : "Bind pose off") << endl;
		Transform(true);
		break;
	//case 'w': myMesh.Adjust
	case 'z': mySkin.Enable(); mySkin.GetBoneLocations(); break; // useless function
	case 'g': 
		cout << "Getting kinect data" << endl;
		mySensor.GetKinectData();
		break;
	case 't': Transform(true); break;
	case 'p': play = !play; cout << (play ? "Play on" : "Play off") << endl; break;
	case '1': myMesh.FlipParameter(1); Transform(true); break;
	case '2': myMesh.FlipParameter(2); Transform(true); break;
	case '3': myMesh.FlipParameter(3); Transform(true); break;
	case '4': myMesh.FlipParameter(4); Transform(true); break;
	case '5': myMesh.FlipParameter(5); Transform(true); break;
	case '6': myMesh.FlipParameter(6); Transform(true); break;
	case '7': myMesh.FlipParameter(7); Transform(true); break;
	case '8': myMesh.FlipParameter(8); Transform(true); break;
	case '9': myMesh.FlipParameter(9); Transform(true); break;
	case '0': myMesh.FlipParameter(0); Transform(true); break;
	default:
		if (myCam.OnKeyboardNum(key, true)) {
			myPipe.SetCamera(myCam.GetPos(), myCam.GetTarget(), myCam.GetUp());
			mySkin.Enable();
			mySkin.SetEyeWorldPos(myCam.GetPos());
			mySkin.SetWVP(myPipe.GetWVPTrans());
			myTech.Enable();
			myTech.SetDefault(myPipe.GetVPTrans());
		}
	}
	glutPostRedisplay();
}

void Setup()
{
	mySensor.Init();
	mySensor.PrintJointHierarchy();
	myMesh.NextModel(0);
	mySensor.GetKinectData(); // to successfully acquire frame init sensor before mesh and load mesh before getting data
	mySensor.GetKinectData();
	// setup pipeline		
	myPipe.Scale(1.0f, 1.0f, 1.0f);
	myPipe.Rotate(0.0f, 0.0f, 0.0f);
	myPipe.WorldPos(0, 0, 2);
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
	//myCam.SetCam();
	myPipe.SetCamera(myCam.GetPos(), myCam.GetTarget(), myCam.GetUp());
	myCam.PrintInfo();
	persProjInfo.FOV = 60.0f;
	persProjInfo.Height = HEIGHT;
	persProjInfo.Width = WIDTH;
	persProjInfo.zNear = 0.1f;
	persProjInfo.zFar = 1000.0f;
	myPipe.SetPerspectiveProj(persProjInfo);

	// setup shaders	
	if (!myTech.InitDefault()) cout << "Could not initialize default shaders" << endl;
	myTech.Enable();
	myTech.SetDefault(myPipe.GetWVPTrans());
	
	if (!mySkin.Init()) cout << "Could not initialize skinning shaders" << endl;
	mySkin.Enable();
	mySkin.SetColorTextureUnit(0);
	directionalLight.Color = Vector3f(1.0f, 1.0f, 1.0f);
	directionalLight.AmbientIntensity = 0.7f;
	directionalLight.DiffuseIntensity = 0.9f;
	directionalLight.Direction = Vector3f(0.0f, -1.0, 0.0);
	mySkin.SetDirectionalLight(directionalLight);
	mySkin.SetMatSpecularIntensity(0.0f);
	mySkin.SetMatSpecularPower(0);
	mySkin.SetWVP(myPipe.GetWVPTrans());
	Transform(true);
}

void Draw()
{
	myCam.OnRender(false);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//*
	mySkin.Enable();
	mySkin.SetWVP(myPipe.GetWVPTrans());
	if (play) {
		mySensor.GetKinectData();
		Transform(false);
	}
	if (renderObject) myMesh.Render();
	
	myTech.Enable();
	myTech.SetDefault(myPipe.GetWVPTrans());
	//if (renderAxes)		drawAxes();
	myTech.SetDefault(myPipe.GetVPTrans());
	if (renderAxes)		DrawAxes();
	//if (renderAxes)		drawCameraVectors();
	if (renderQuats)	DrawQuats();
	if (renderSkeleton) DrawSkeleton(JointType_SpineBase);
	if (renderCloud)	mySensor.DrawKinectData();
	//*/
	glutSwapBuffers();
}

int main(int argc, char* argv[]) {
	if (!InitGLUT(argc, argv)) printf("InitGLUT failed\n");	
	SetupOGL();
	Setup();
	
    // Main loop
    Execute();
	
    return 0;
}
