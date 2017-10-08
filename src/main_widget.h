#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

// Project
class Camera;
class KSensor;
class SkinnedMesh;
class Technique;
class SkinningTechnique;
class Pipeline;
#include "math_3d.h"
#include "util.h"

// Assimp
struct aiScene;

// Qt
#include <QtWidgets\QOpenGLWidget>
QT_FORWARD_DECLARE_CLASS(QOpenGLTexture);

// Standard C/C++
//#include <vector>

class MainWidget : public QOpenGLWidget, protected OPENGL_FUNCTIONS
{
	Q_OBJECT
public:
	MainWidget(QWidget *parent = Q_NULLPTR);
	~MainWidget();
	bool renderAxes() const;
	bool renderModel() const;
	QStringList ModelBoneList() const;
	QString boneTransformInfo(const QString& boneName) const;
	bool boneVisibility(const QString& boneName) const;
	bool modelSkinning() const;
	
public slots:
	void setRenderAxes(bool state);
	void setRenderModel(bool state);
	void flipBonesVisibility();
	void setBoneVisibility(const QString& boneName, bool state);
	void setBoneRotation(const QString& boneName, float xRot, float yRot, float zRot);
	void setModelSkinning(bool state);
	void setModelName(const QString& model);
	
protected:
	void initializeGL();
	void paintGL();
	void keyPressEvent(QKeyEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void mouseReleaseEvent(QMouseEvent *event) override;

private:
	Camera* m_Cam; 
	KSensor* m_Sensor;
	SkinnedMesh* m_Mesh;
	Technique* m_Tech;
	SkinningTechnique* m_Skin;
	Pipeline* m_Pipe;
	
	QPoint m_lastPos;

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
	bool loadToGPU(const string& basename);
	void unloadFromGPU();
	enum VB_TYPES {
		INDEX_BUFFER,
		POS_VB,
		NORMAL_VB,
		TEXCOORD_VB,
		BONE_VB,
		NUM_VBs
	};
	GLuint m_VAO;
	GLuint m_Buffers[NUM_VBs];
	vector<QOpenGLTexture*> m_textures;
	void drawSkinnedMesh();
	bool m_successfullyLoaded = false;
};

#endif /* MAIN_WIDGET_H */
