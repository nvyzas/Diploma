#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

// Project
class KSensor;
class Camera;
class Technique;
class SkinningTechnique;
class Pipeline;
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

struct Material
{
	QVector3D diffuseColor;
	QVector3D specularColor;
	QVector3D ambientColor;

	Material()
	{
	}

	Material(QVector3D& difColor, QVector3D& specColor, QVector3D& ambColor)
	{
		diffuseColor = difColor;
		specularColor = specColor;
		ambientColor = ambColor;
	}
};

class MainWidget : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
	Q_OBJECT
public:
	MainWidget(QWidget *parent = Q_NULLPTR);
	~MainWidget();
	
	// #todo make get/set functions
	const uint m_captureInterval = 10; // in milliseconds
	uint m_playbackInterval; // in milliseconds
	uint m_fpsCount = 0; // for counting fps
	
	float m_kneeAngle;
	float m_barAngle;
	QVector3D m_barSpeed;

	QVector<KFrame>* m_activeAthleteMotion;
	QVector<KFrame>* m_activeTrainerMotion;

	uint m_activeFrameIndex = 0;

	KFrame m_activeAthleteFrame;
	KFrame m_activeTrainerFrame;

	double m_activeFrameTimestamp;

	uint m_activeBoneId = 0;
	uint m_activeJointId = 0;

	// get functions
	SkinnedMesh* skinnedMesh();
	SkinningTechnique* skinningTechnique();
	Technique* technique();
	KSensor* ksensor();
	bool modelSkinning() const;
	QStringList modelBoneList() const;
	QStringList motionTypeList() const;

	enum class Mode
	{
		CAPTURE,
		PLAYBACK
	};
	enum class Person
	{
		ATHLETE,
		TRAINER,
		BOTH
	};
	enum class MotionType
	{
		RAW,
		INTERPOLATED,
		FILTERED,
		ADJUSTED,
		RESIZED
	};

	bool captureEnabled() const;

	bool athleteEnabled() const;
	bool trainerEnabled() const;

	int activeMotionType() const;

	int activeMotionProgress() const;

	bool isPaused() const;


	bool axesDrawing() const;
	bool skinnedMeshDrawing() const;
	bool kinectSkeletonDrawing() const;
	bool floorDrawing() const;
	bool barbellDrawing() const;
	bool tipsDrawing() const;

public slots:
	void setCaptureEnabled(bool state);

	void setAthleteEnabled(bool state);
	void setTrainerEnabled(bool state);

	void setActiveMotionType(int motionType);
	void setActiveMotionProgress(int progressPercent);

	void setIsPaused(bool state);

	void setAxesDrawing(bool state);
	void setSkinnedMeshDrawing(bool state);
	void setKinectSkeletonDrawing(bool state);
	void setFloorDrawing(bool state);
	void setBarbellDrawing(bool state);
	void setTipsDrawing(bool state);

	void setModelName(const QString& modelName);
	void setModelSkinning(bool state);

	void setActiveBone(const QString& modelName);

	void intervalPassed();

signals:
	void frameChanged(int progressPercent);

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

	QStringList m_motionTypeList = { "Raw", "Interpolated", "Filtered", "Adjusted", "Resized" };


	Mode m_activeMode;

	bool m_athleteEnabled;
	bool m_trainerEnabled;

	int m_activeMotionType;

	// render
	bool m_axesDrawing = true;
	bool m_skinnedMeshDrawing = true;
	bool m_kinectSkeletonDrawing = true;
	bool m_barbellDrawing = true;
	bool m_floorDrawing = true;
	bool m_tipsDrawing = true;

	bool m_drawSkinnedMeshJoints = true;



	QPoint m_lastMousePosition;
	bool m_isPaused = true;	
	QTimer m_timer;
	bool m_shouldUpdate = false;
	void setup();

	// Skinned mesh
	QString m_skinnedMeshModelName;
	QVector3D m_skinnedMeshOffset;
	bool m_skinningEnabled = true;
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
	bool m_defaultPose = true;

	// Skinned mesh joint dots
#define NUM_BONES 53
	GLuint m_skinnedMeshJointsVAO;
	GLuint m_skinnedMeshJointsVBO;
	float m_skinnedMeshJoints[2 * 3 * NUM_BONES];
	void loadSkinnedMeshJoints();
	void drawSkinnedMeshJoints();

	// kinect skeleton
	QVector3D m_athleteSkeletonOffset = QVector3D(0, 0, 0);
	QVector3D m_trainerSkeletonOffset = QVector3D(0, 0, 0);
	GLuint m_skeletonVAO;
	GLuint m_skeletonVBO;
	float m_skeleton[2 * 3 * JointType_Count]; // 2 attributes x 3 components x JointType_Count joints
	void loadSkeleton();
	void drawSkeleton(const array<KJoint, JointType_Count>& joints);

	// arrow
	GLuint m_arrowVAO;
	void loadArrow();
	void drawArrow();

	// axes
	GLuint m_axesVAO;
	void loadAxes();
	void drawAxes();

	// cube
	GLuint m_cubeVAO;
	GLuint m_cubeVBO;
	float m_cubeColors[3 * 8];
	void loadCube(float r);
	void drawCube(const KJoint& joint);

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
	QVector<Material> m_barbellMaterials;
	void loadBarbell();
	void drawBarbell();
	const aiScene* m_barbellScene;
	QVector<QMatrix4x4> globals;
	void traverseBarbellSceneNodes(aiNode* node);

	// shaders for material lighting
	QOpenGLShaderProgram* m_lighting;
	uint m_modelViewLocation;
	uint m_projectionLocation;
	uint m_diffuseLocation;
	uint m_specularLocation;
	uint m_ambientLocation;
};

#endif /* MAIN_WIDGET_H */
