#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

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
	void keyPressEvent(QKeyEvent *event);
	
public slots:
	void setRenderAxes(bool state);
	void setRenderModel(bool state);
	void flipBonesVisibility();
	void setBoneVisibility(const QString& boneName, bool state);
	void setModelSkinning(bool state);
	void setModelName(const QString& model);
	
protected:
	void initializeGL();
	void paintGL();

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
	QString m_modelName;

	#define NUM_INFO_BLOCKS 3
	uint activeJoint = 0;
	
	uint activeInfo = 0;
	void NextInfoBlock(int step);
	void Transform(bool print);
	void DrawAxes();
	void DrawAxes(Vector3f center, Vector3f vx, Vector3f vy, Vector3f vz, float length);
	void DrawTestAxes();
	void MySetup();
};

#endif /* MAIN_WIDGET_H */
