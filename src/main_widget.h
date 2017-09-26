#pragma once
#include "skinned_mesh.h"
#include "skinning_technique.h"
#include "pipeline.h"
#include "camera.h"
#include "math_3d.h"
#include "sensor.h"
#include "util.h"
#include <QtWidgets\QOpenGLWidget>

class MainWidget : public QOpenGLWidget, protected OPENGL_FUNCTIONS
{
	Q_OBJECT
public:
	MainWidget(QWidget *parent = Q_NULLPTR);
	bool renderAxes() const;
	bool renderModel() const;
	
public slots:
	void setRenderAxes(bool state);
	void setRenderModel(bool state);
	
protected:
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();
	void keyPressEvent(QKeyEvent *event);
	
	
	QOpenGLContext *m_context;

private:
	Camera* m_Cam; // width, height?
	KSensor* m_Sensor;
	SkinnedMesh* m_Mesh; //Mesh(mySensor)
	Technique* m_Tech;
	SkinningTechnique* m_Skin;
	Pipeline* m_Pipe;

	bool m_renderAxes = true;
	bool m_renderModel = true;
	bool m_renderSkeleton = false;
	bool m_renderActiveJoint = false;
	bool m_renderCloud = false;
	bool m_renderCameraVectors = false;	
	
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

