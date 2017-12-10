#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

// Project
class KSensor;
class Camera;
class SkinnedMesh;
class Technique;
class SkinningTechnique;
class Pipeline;
#include "math_3d.h"
#include "opensim_model.h"
#include "util.h"

// Qt
#include <QtCore\QTimer>
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

	// skinned mesh related
	SkinnedMesh *skinnedMesh();
	SkinningTechnique *skinningTechnique();
	bool modelSkinning() const;
	QStringList modelBoneList() const;	
	void Transform(bool print);
	bool renderModel() const;
	void setKSensor(KSensor &ksensor);

public slots:
	void setRenderAxes(bool state);
	void setRenderModel(bool state);
	void setModelName(const QString &modelName);
	void setModelSkinning(bool state);

	void changeMode();
	void updateIndirect();
	
protected:
	void initializeGL();
	void paintGL();
	void keyPressEvent(QKeyEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;

private:
	enum class Mode
	{
		CAPTURE,
		PLAYBACK
	};
	QTimer m_timer;

	Mode m_modeOfOperation;
	const uint m_captureInterval = 10; // milliseconds
	uint m_playbackInterval;

	KSensor* m_ksensor;

	SkinnedMesh* m_skinnedMesh;
	Camera* m_Cam;
	Technique* m_Tech;
	SkinningTechnique* m_Skin;
	Pipeline* m_Pipe;
	OpenSimModel m_osm;
	
	QPoint m_lastPos;

	bool m_renderAxes = true;
	bool m_renderSkeleton = true;
	bool m_renderCameraVectors = false;	
	bool m_play = true;	

	#define NUM_INFO_BLOCKS 3
	uint activeJoint = 0;
	
	uint activeInfo = 0;
	void NextInfoBlock(int step);
	void DrawAxes();
	void DrawAxes(Vector3f center, Vector3f vx, Vector3f vy, Vector3f vz, float length);
	void DrawTestAxes();
	void MySetup();

	// Skinned mesh variables
	bool m_renderModel = true;
	bool m_modelSkinning = true;
	QString m_modelName;
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
