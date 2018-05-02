#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

// Project
class KSensor;
class Camera;
class Technique;
class SkinningTechnique;
class Pipeline;
#include "opensim_model.h"
#include "util.h"
#include "skinned_mesh.h"

// Kinect
#include <Kinect.h>

// Assimp
#include <assimp\Importer.hpp>      
#include <assimp\scene.h>			
#include <assimp\postprocess.h>     

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
	

	// #todo #? make private and create get/set functions
	const uint m_captureInterval = 10; // in milliseconds
	uint m_playbackInterval; // in milliseconds
	uint m_fpsCount = 0; // for counting fps
	
	float m_kneeAngle;
	float m_barAngle;
	QVector3D m_barSpeed;
	double m_time;

	void transformSkinnedMesh(bool print);

	// get functions
	SkinnedMesh* skinnedMesh();
	SkinningTechnique* skinningTechnique();
	Technique* technique();
	KSensor* ksensor();
	bool renderAxes() const;
	bool renderSkinnedMesh() const;
	bool renderKinectSkeleton() const;
	bool modelSkinning() const;
	QStringList modelBoneList() const;
	QString mode() const;

public slots:
	void setRenderAxes(bool state);
	void setRenderModel(bool state);
	void setRenderSkeleton(bool state);
	void setModelName(const QString& modelName);
	void setModelSkinning(bool state);
	void setBoneAxes(const QString& boneName);
	void changeMode();
	void updateIndirect(); // #todo replace it with something else
	void setActiveFrame(uint index);
	void setActiveBone(const QString& modelName);

protected:
	void initializeGL();
	void paintGL();
	void calculateFPS();
	void keyPressEvent(QKeyEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;

private:
	KSensor* m_ksensor;
	SkinnedMesh* m_skinnedMesh;
	Camera* m_camera;
	Technique* m_technique;
	SkinningTechnique* m_skinningTechnique;
	Pipeline* m_pipeline;
	OpenSimModel m_osm;

	enum class Mode
	{
		CAPTURE,
		PLAYBACK
	};
	Mode m_mode;
	
	QPoint m_lastMousePosition;
	bool m_play = true;	
	QTimer m_timer;
	bool m_shouldUpdate = false;
	void setup();

	uint m_activeBone = 0;
	uint m_activeFrame = 0;

	// Skinned mesh
	QString m_skinnedMeshModelName;
	QVector3D m_skinnedMeshOffset;
	bool m_skinningEnabled = true;
	bool m_renderSkinnedMesh = true;
	void loadSkinnedMesh();
	void unloadSkinnedMesh();
	enum VB_TYPES {
		INDEX_BUFFER,
		POS_VB,
		NORMAL_VB,
		TEXCOORD_VB,
		BONE_VB,
		NUM_VBs
	};
	GLuint m_skinnedMeshVBOs[NUM_VBs];
	GLuint m_skinnedMeshVAO;
	vector<QOpenGLTexture*> m_skinnedMeshTextures;
	void drawSkinnedMesh();
	bool m_defaultPose = false;

	// Skinned mesh joint dots
#define NUM_BONES 31
	bool m_renderSkinnedMeshJoints = true;
	GLuint m_skinnedMeshJointsVAO;
	GLuint m_skinnedMeshJointsVBO;
	void loadSkinnedMeshJoints();
	float m_skinnedMeshJoints[2 * 3 * NUM_BONES];
	void drawSkinnedMeshJoints();

	// kinect skeleton
	bool m_renderKinectSkeleton = true;
	QVector3D m_kinectSkeletonOffset;
	GLuint m_kinectSkeletonJointsVAO;
	GLuint m_kinectSkeletonJointsVBO;
	void loadKinectSkeletonJoints();
	float m_kinectSkeletonJoints[2 * 3 * JointType_Count]; // 2 attributes x 3 components x JointType_Count joints
	void drawKinectSkeletonJoints();

	// arrow
	GLuint m_arrowVAO;
	void loadArrow();
	void drawArrow();

	// axes
	bool m_renderAxes = true;
	GLuint m_axesVAO;
	void loadAxes();
	void drawAxes();

	// cube
	GLuint m_cubeVAO;
	GLuint m_cubeVBO;
	float m_cubeColors[3 * 8];
	void loadCube(float r);
	void drawCube();

	// shaders for plane drawing
	QOpenGLShaderProgram* m_shaderProgram;
	int m_mvpLocation;
	int m_specificLocation;

	// plane
#define PLANE_VERTICES 4
	QOpenGLTexture* m_planeTexture;
	GLuint m_planeVAO;
	void loadPlane();
	void drawPlane();

	// barbell
	QVector<QOpenGLTexture*> m_barbellTextures;
	QVector<MeshEntry> m_barbellMeshEntries;
	GLuint m_barbellVAO;
	void loadBarbell();
	void drawBarbell();
	const aiScene* m_barbellScene;
	QVector<QMatrix4x4> globals;
	void traverseBarbellSceneNodes(aiNode* node);

	// shaders for material lighting
	QOpenGLShaderProgram* m_lighting;
	uint m_modelViewLocation;
	uint m_projectionLocation;
};

#endif /* MAIN_WIDGET_H */
