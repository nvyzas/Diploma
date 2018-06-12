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
	
	float m_playbackInterval = 0.033333; 
	uint m_fpsCount = 0;
	
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
	QStringList skeletonJointList() const;
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
		RESCALED
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
	QVector3D activeJointVelocity() const;
	float activeJointAngle() const;
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

	void setModelSkinning(bool state);

	void setActiveBone(const QString& boneName);
	void setActiveJointId(int jointId);

	void intervalPassed();

	void setGeneralOffset(int percent);

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
	SkinnedMesh* m_athlete;
	SkinnedMesh* m_trainer;
	Camera* m_camera;
	Technique* m_technique;
	SkinningTechnique* m_skinningTechnique;
	Pipeline* m_pipeline;

	QStringList m_motionTypeList = { "Raw", "Interpolated", "Filtered", "Adjusted", "Resized" };

	QVector3D m_generalOffset;
	Mode m_activeMode;

	bool m_athleteEnabled;
	bool m_trainerEnabled;

	int m_activeMotionType;

	// render
	bool m_axesDrawing = false;
	bool m_skinnedMeshDrawing = true;
	bool m_kinectSkeletonDrawing = true;
	bool m_barbellDrawing = true;
	bool m_floorDrawing = true;
	bool m_tipsDrawing = true;

	bool m_kinectSkeletonJointsDrawing = true;
	bool m_SkinnedMeshJointsDrawing = true;

	QPoint m_lastMousePosition;
	bool m_isPaused = true;	
	QTimer m_timer;
	bool m_shouldUpdate = false;
	void setup();

	// skinned mesh
	enum VB_TYPES {
		INDEX_BUFFER,
		POS_VB,
		NORMAL_VB,
		TEXCOORD_VB,
		BONE_VB,
		NUM_VBs
	};
	bool m_skinningEnabled = true;
	bool m_defaultPose = true;
	// athlete
	vector<QOpenGLTexture*> m_athleteTextures;
	GLuint m_athleteVBOs[NUM_VBs];
	GLuint m_athleteVAO;
	void loadAthlete();
	void unloadAthlete();
	void drawAthlete();
	// trainer
	vector<QOpenGLTexture*> m_trainerTextures;
	GLuint m_trainerVBOs[NUM_VBs];
	GLuint m_trainerVAO;
	void loadTrainer();
	void unloadTrainer();
	void drawTrainer();

	// Skinned mesh joint dots
#define NUM_BONES 52
	GLuint m_skinnedMeshJointsVAO;
	GLuint m_skinnedMeshJointsVBO;
	float m_skinnedMeshJoints[2 * 3 * NUM_BONES];
	void loadSkinnedMeshJoints();
	void drawSkinnedMeshJoints(const SkinnedMesh* sm);

	// kinect skeleton
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

	// Meshes
	void traverseSceneNodes(aiNode* node);

	// bar (athlete's barbell)
	QVector<QOpenGLTexture*> m_barTextures;
	QVector<MeshEntry> m_barMeshEntries;
	QVector<Material> m_barMaterials;
	GLuint m_barVAO;
	void loadBar();
	void drawBar();
	const aiScene* m_barScene;

	// barbell (trainer's barbell)
	QVector<QOpenGLTexture*> m_barbellTextures;
	QVector<MeshEntry> m_barbellMeshEntries;
	QVector<Material> m_barbellMaterials;
	GLuint m_barbellVAO;
	void loadBarbell();
	void drawBarbell();
	const aiScene* m_barbellScene;

	// pointer
	QVector<QOpenGLTexture*> m_pointerTextures;
	QVector<MeshEntry> m_pointerMeshEntries;
	QVector<Material> m_pointerMaterials;
	GLuint m_pointerVAO;
	void loadPointer();
	void drawPointer();
	const aiScene* m_pointerScene;



	// shaders for material lighting
	QOpenGLShaderProgram* m_lighting;
	uint m_modelViewLocation;
	uint m_projectionLocation;
	uint m_diffuseLocation;
	uint m_specularLocation;
	uint m_ambientLocation;
};

#endif /* MAIN_WIDGET_H */
