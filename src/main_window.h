#pragma once
#include "skinned_mesh.h"
#include "skinning_technique.h"
#include "pipeline.h"
#include "camera.h"
#include "math_3d.h"
#include "sensor.h"
#include "util.h"
#include <QtWidgets\QOpenGLWidget>

class MainWindow : public QOpenGLWidget, protected OPENGL_FUNCTIONS
{
	Q_OBJECT
public:
	MainWindow(QWidget *parent = Q_NULLPTR);
	
protected:
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();

	QOpenGLContext *m_context;

private:
	Camera* m_Cam; // width, height?
	KSensor* m_Sensor;
	SkinnedMesh* m_Mesh; //Mesh(mySensor)
	Technique* m_Tech;
	SkinningTechnique* m_Skin;
	Pipeline* m_Pipe;

	bool m_renderCloud = false;
	bool m_renderSkeleton = true;
	bool m_renderAxes = true;
	bool m_renderCameraVectors = false;
	bool m_renderObject = true;
	bool m_renderJoint = true;
	bool m_play = false;

	#define NUM_INFO_BLOCKS 3
	uint activeJoint = JointType_SpineBase;
	
	uint activeInfo = 0;
	void NextInfoBlock(int step);
	void Transform(bool print);
	void DrawAxes();
	void DrawAxes(Vector3f center, Vector3f vx, Vector3f vy, Vector3f vz, float length);
	void DrawTestAxes();
	void SetupOpenGL();
	void MySetup();
};

