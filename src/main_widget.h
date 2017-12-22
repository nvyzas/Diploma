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
#include <QtGui\QOpenGLShaderProgram>
QT_FORWARD_DECLARE_CLASS(QOpenGLTexture);

// Standard C/C++
//#include <vector>

class MainWidget : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
	Q_OBJECT
public:
	MainWidget(QWidget *parent = Q_NULLPTR);
	~MainWidget();
	

	// get functions
	SkinnedMesh* skinnedMesh();
	SkinningTechnique* skinningTechnique();
	Technique* technique();
	bool renderAxes() const;
	bool renderModel() const;
	bool modelSkinning() const;
	QStringList modelBoneList() const;

	void Transform(bool print);

public slots:
	void setRenderAxes(bool state);
	void setRenderModel(bool state);
	void setModelName(const QString& modelName);
	void setModelSkinning(bool state);
	void setBoneAxes(const QString& boneName);
	void setKSensor(KSensor& ksensor);
	void changeMode();
	void updateIndirect();
	void setActiveBone(const QString& modelName);

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
	QOpenGLShaderProgram m_linesProgram;
	
	QPoint m_lastPos;

	bool m_renderAxes = false;
	bool m_renderSkeleton = true;
	bool m_renderSkinnedMesh = true;

	bool m_play = true;	

	#define NUM_INFO_BLOCKS 3
	
	uint activeInfo = 0;
	void NextInfoBlock(int step);
	void MySetup();

	// Skinned mesh variables
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

	uint m_activeBone = 0;
};

#endif /* MAIN_WIDGET_H */
