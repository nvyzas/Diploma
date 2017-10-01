#pragma once

#include "math_3d.h"
#include "util.h"
#include <QtWidgets\QOpenGLWidget>
// Forward declarations
class Camera;
class KSensor;
class SkinnedMesh;
class Technique;
class SkinningTechnique;
class Pipeline;

class MainWidget : public QOpenGLWidget, protected OPENGL_FUNCTIONS
{
	Q_OBJECT
public:
	MainWidget(QWidget *parent = Q_NULLPTR);
	bool renderAxes() const;
	bool renderModel() const;
	QStringList ModelBoneList() const;
	bool boneVisibility(const QString& boneName) const;
	bool modelSkinning() const;
	
public slots:
	void setRenderAxes(bool state);
	void setRenderModel(bool state);
	void setBoneVisibility(const QString& boneName, bool state);
	void setModelSkinning(bool state);
	void setModel(const QString& model);
	
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
	bool m_modelSkinning = true;

	#define NUM_INFO_BLOCKS 3
	uint activeJoint = 0;
	
	uint activeInfo = 0;
	void NextInfoBlock(int step);
	void Transform(bool print);
	void DrawAxes();
	void DrawAxes(Vector3f center, Vector3f vx, Vector3f vy, Vector3f vz, float length);
	void DrawTestAxes();
	void SetupOpenGL();
	void MySetup();
};

