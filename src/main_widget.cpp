// Own
#include "main_widget.h"

// Project
#include "skinned_mesh.h"
#include "skinning_technique.h"
#include "pipeline.h"
#include "camera.h"
#include "kskeleton.h"

// Assimp
#include <assimp\Importer.hpp>      
#include <assimp\scene.h>			 
#include <assimp\postprocess.h>     

// Qt
#include <QtCore\QDebug>
#include <QtGui\QKeyEvent>
#include <QtGui\QOpenGLTexture>

// Standard C/C++
#include <cassert>
#include <iomanip>

MainWidget::MainWidget(QWidget *parent)
	: 
	QOpenGLWidget(parent),
	m_ksensor(new KSensor()),
	m_athlete(new SkinnedMesh()),
	m_trainer(new SkinnedMesh()),
	m_camera(new Camera()),
	m_pipeline(new Pipeline())
{
	cout << "MainWidget class constructor start." << endl;

	// Init skinned mesh
	m_athlete->loadFromFile("athlete.dae");
	m_athlete->initKBoneMap();
	m_trainer->loadFromFile("trainer.dae");
	m_trainer->initKBoneMap();
	
	// Setup timer
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(intervalPassed()));
	m_timer.setTimerType(Qt::PreciseTimer);
	m_timer.start();

	// Setup mode
	setCaptureEnabled(false);
	setAthleteEnabled(true);
	setTrainerEnabled(true);
	setActiveMotionType(4);

	// #todo: Setup camera
	m_camera->printInfo();

	// Setup pipeline
	PerspectiveProjectionInfo perspectiveProjectionInfo;
	perspectiveProjectionInfo.fieldOfView = 60.f;
	perspectiveProjectionInfo.aspectRatio = m_camera->windowWidth() / m_camera->windowHeight();
	perspectiveProjectionInfo.nearPlane = 0.1f;
	perspectiveProjectionInfo.farPlane = 1000.f;
	m_pipeline->setPersProjInfo(perspectiveProjectionInfo);
	m_pipeline->setCamera(m_camera->GetPos(), m_camera->GetTarget(), m_camera->GetUp());

	cout << "MainWidget class constructor end.\n" << endl;
}
MainWidget::~MainWidget()
{
	// delete class members
	delete m_ksensor;
	delete m_athlete;
	delete m_trainer;
	delete m_camera;
	delete m_pipeline;

	// Release OpenGL resources
	makeCurrent();

	// delete skinned mesh
	unloadAthlete();
	unloadTrainer();

	// delete shaders
	delete m_technique;
	delete m_skinningTechnique;
	delete m_shaderProgram;
	delete m_lighting;

	// delete VAOs
	glDeleteVertexArrays(1, &m_axesVAO);
	glDeleteVertexArrays(1, &m_arrowVAO);
	glDeleteVertexArrays(1, &m_cubeVAO);
	glDeleteVertexArrays(1, &m_skeletonVAO);
	glDeleteVertexArrays(1, &m_skinnedMeshJointsVAO);
	glDeleteVertexArrays(1, &m_planeVAO);
	glDeleteVertexArrays(1, &m_barVAO);
	glDeleteVertexArrays(1, &m_barbellVAO);

	// delete VBOs
	glDeleteBuffers(1, &m_cubeVBO);
	glDeleteBuffers(1, &m_skeletonVBO);
	glDeleteBuffers(1, &m_skinnedMeshJointsVBO);

	// delete textures
	delete m_planeTexture;

	doneCurrent();
}
// This virtual function is called once before the first call to paintGL() or resizeGL().
// Instantiate and initialize context dependent classes in it.
void MainWidget::initializeGL()
{
	cout << "MainWidget initializeGL start." << endl;

	qDebug() << "Obtained format:" << format();
	initializeOpenGLFunctions();

	glEnable(GL_TEXTURE_2D);
	glClearColor(0.0f, 0.0f, 0.0f, 1.f);
	glClearDepth(1.0f);
	glEnable(GL_POINT_SMOOTH);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	glPointSize(3.f);

	m_athlete->printInfo();

	// Init technique
	m_technique = new Technique();
	m_technique->initDefault();
	m_technique->enable();
	m_technique->setMVP(m_pipeline->getWVPtrans());
	m_technique->setSpecific(QMatrix4x4());

	// Init skinning technique
	m_skinningTechnique = new SkinningTechnique();
	m_skinningTechnique->Init();
	m_skinningTechnique->enable();
	m_skinningTechnique->SetColorTextureUnit(0);
	DirectionalLight directionalLight;
	directionalLight.Color = QVector3D(1.f, 1.f, 1.f);
	directionalLight.AmbientIntensity = 0.7f;
	directionalLight.DiffuseIntensity = 0.9f;
	directionalLight.Direction = QVector3D(1.f, -1.f, 0.f);
	m_skinningTechnique->setDirectionalLight(directionalLight);
	m_skinningTechnique->setMatSpecularIntensity(0.0f);
	m_skinningTechnique->setMatSpecularPower(0);
	m_skinningTechnique->setSkinning(true);
	for (uint i = 0; i < m_athlete->numBones(); i++) {
		m_skinningTechnique->setBoneVisibility(i, m_athlete->boneVisibility(i));
	}
	
	// Init plane shaders
	QOpenGLShader planeVS(QOpenGLShader::Vertex);
	QOpenGLShader planeFS(QOpenGLShader::Fragment);
	planeVS.compileSourceFile("shaders/plane.vs");
	planeFS.compileSourceFile("shaders/plane.fs");

	cout << "Initializing plane shaders" << endl;
	m_shaderProgram = new QOpenGLShaderProgram(context());
	if (!m_shaderProgram->addShader(&planeVS)) cout << "Could not add plane vertex shader." << endl;
	if (!m_shaderProgram->addShader(&planeFS)) cout << "Could not add plane fragment shader." << endl;
	if (!m_shaderProgram->link()) cout              << "Could not link plane shaders." << endl;
	if (!m_shaderProgram->bind()) cout              << "Could not bind plane shaders." << endl;
	cout << "Program id: " << m_shaderProgram->programId() << endl;
	cout << "Is supported by system? " << m_shaderProgram->hasOpenGLShaderPrograms() << endl;
	m_mvpLocation = m_shaderProgram->uniformLocation("gMVP");
	m_specificLocation = m_shaderProgram->uniformLocation("gSpecific");
	cout << "Locations:" << endl;
	cout << m_mvpLocation << " ";
	cout << m_specificLocation << endl;
	float skinnedMeshFeet;
	skinnedMeshFeet = m_athlete->boneEndPosition(m_athlete->findBoneId("foot_l")).y();
	skinnedMeshFeet += m_athlete->boneEndPosition(m_athlete->findBoneId("foot_r")).y();
	skinnedMeshFeet /= 2;
	QMatrix4x4 S = fromScaling(QVector3D(2.f, 1.f, 2.f));
	QMatrix4x4 R = fromRotation(QQuaternion::fromEulerAngles(QVector3D(0.f, 45.f, 0.f)));
	QMatrix4x4 T = fromTranslation(QVector3D(0, skinnedMeshFeet, 0));
	m_shaderProgram->setUniformValue(m_specificLocation, T * R * S);

	// Init lighting shaders
	QOpenGLShader lightingVS(QOpenGLShader::Vertex);
	QOpenGLShader lightingFS(QOpenGLShader::Fragment);
	lightingVS.compileSourceFile("shaders/lighting.vert");
	lightingFS.compileSourceFile("shaders/lighting.frag");

	cout << "Initializing lighting shaders" << endl;
	m_lighting = new QOpenGLShaderProgram(context());
	if (!m_lighting->addShader(&lightingVS)) cout << "Could not add lighting vertex shader." << endl;
	if (!m_lighting->addShader(&lightingFS)) cout << "Could not add lighting fragment shader." << endl;
	if (!m_lighting->link()) cout << "Could not link lighting shaders." << endl;
	if (!m_lighting->bind()) cout << "Could not bind lighting shaders." << endl;
	cout << "Program id: " << m_lighting->programId() << endl;
	cout << "Is supported by system? " << m_lighting->hasOpenGLShaderPrograms() << endl;
	m_modelViewLocation = m_lighting->uniformLocation("modelView");
	m_projectionLocation = m_lighting->uniformLocation("projection");
	m_diffuseLocation = m_lighting->uniformLocation("diffuseAlbedo");
	m_specularLocation = m_lighting->uniformLocation("specularAlbedo");
	m_ambientLocation = m_lighting->uniformLocation("ambient");

	cout << "Locations: ";
	cout << m_modelViewLocation << " ";
	cout << m_projectionLocation << endl;

	loadAthlete(); 
	loadTrainer();
	loadAxes();
	loadArrow();
	loadSkeleton();
	loadSkinnedMeshJoints();
	loadCube(0.02);
	loadPlane();
	loadBar();
	loadBarbell();
	loadPointer();

	cout << "MainWidget initializeGL end." << endl;
}
void MainWidget::paintGL()
{
	glClearColor(0.f, 0.f, 0.f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	KFrame* activeFrame = (m_athleteEnabled ? &m_activeAthleteFrame : &m_activeTrainerFrame);
	if (m_activeMode == Mode::CAPTURE) {
		m_ksensor->getBodyFrame(*activeFrame);
	}
	else if (m_activeMode == Mode::PLAYBACK){
		if (m_activeFrameIndex < m_activeAthleteMotion->size()) {
			m_activeAthleteFrame = m_activeAthleteMotion->operator[](m_activeFrameIndex);
		}
		if (m_activeFrameIndex < m_activeTrainerMotion->size()) {
			m_activeTrainerFrame = m_activeTrainerMotion->operator[](m_activeFrameIndex);
		}
	}
	else {
		cout << "Error: Mode=" << (int)m_activeMode << endl;
		return;
	}
	m_activeFrameTimestamp = activeFrame->timestamp;

	// calculate skinned mesh bone transforms (used by barbell as well)
	m_athlete->calculateBoneTransforms(
		m_athlete->m_pScene->mRootNode,
		QMatrix4x4(),
		m_activeAthleteFrame.joints);
	m_trainer->calculateBoneTransforms(
		m_trainer->m_pScene->mRootNode,
		QMatrix4x4(),
		m_activeTrainerFrame.joints);

	// draw humans
	if (m_skinnedMeshDrawing) {

		// athlete
		if (m_athleteEnabled && m_athlete->m_successfullyLoaded) {
			m_pipeline->setWorldScale(QVector3D(1.f, 1.f, 1.f));
			m_pipeline->setWorldOrientation(QQuaternion());
			m_pipeline->setWorldPosition(
				m_activeAthleteFrame.joints[JointType_SpineBase].position-
				m_ksensor->skeleton()->m_athleteFeetOffset+
				m_generalOffset
			);

			// skinned mesh
			m_skinningTechnique->enable();
			m_skinningTechnique->setWVP(m_pipeline->getWVPtrans());
			for (uint i = 0; i < m_athlete->numBones(); i++) {
				m_skinningTechnique->setBoneTransform(i, m_athlete->boneInfo(i).combined);
			}
			drawAthlete();

			// skinned mesh joints
			m_technique->enable();
			if (m_SkinnedMeshJointsDrawing) {
				m_technique->setSpecific(QMatrix4x4());
				m_technique->setMVP(m_pipeline->getWVPtrans());
				drawSkinnedMeshJoints(m_athlete);
			}

			// skinned mesh bone axes
			m_technique->enable();
			if (m_axesDrawing) {
				m_technique->setSpecific(m_athlete->boneGlobal(m_activeBoneId));
				m_technique->setMVP(m_pipeline->getWVPtrans());
				drawAxes();
			}
		}

		// trainer
		if (m_trainerEnabled) {
			m_pipeline->setWorldScale(QVector3D(1.f, 1.f, 1.f));
			m_pipeline->setWorldOrientation(QQuaternion());
			m_pipeline->setWorldPosition(
				m_activeTrainerFrame.joints[JointType_SpineBase].position -
				m_ksensor->skeleton()->m_trainerFeetOffset -
				m_generalOffset
			);

			// skinned mesh
			m_skinningTechnique->enable();
			for (uint i = 0; i < m_trainer->numBones(); i++) {
				m_skinningTechnique->setBoneTransform(i, m_trainer->boneInfo(i).combined);
			}
			m_skinningTechnique->setWVP(m_pipeline->getWVPtrans());
			drawTrainer();

			// skinned mesh joints
			m_technique->enable();
			if (m_SkinnedMeshJointsDrawing) {
				m_technique->setSpecific(QMatrix4x4());
				m_technique->setMVP(m_pipeline->getWVPtrans());
				drawSkinnedMeshJoints(m_trainer);
			}

			// skinned mesh bone axes
			m_technique->enable();
			if (m_axesDrawing) {
				m_technique->setSpecific(m_trainer->boneGlobal(m_activeBoneId));
				m_technique->setMVP(m_pipeline->getWVPtrans());
				drawAxes();
			}
		}
	}

	// draw barbells
	if (m_barbellDrawing) {

		// bind lighting shaders
		m_lighting->bind();

		// athlete
		if (m_athleteEnabled) {
			QVector3D skinnedMeshLeftHand = m_athlete->boneEndPosition(m_athlete->findBoneId("thumb_01_l"));
			QVector3D skinnedMeshRightHand = m_athlete->boneEndPosition(m_athlete->findBoneId("thumb_01_r"));
			QVector3D barbellDirection = skinnedMeshRightHand - skinnedMeshLeftHand;
			QVector3D barbellPosition = (skinnedMeshLeftHand + skinnedMeshRightHand) / 2.f;
			m_pipeline->setWorldScale(0.75, 0.75, 1.0);
			m_pipeline->setWorldOrientation(QQuaternion::rotationTo(QVector3D(0.f, 0.f, 1.f), barbellDirection));
			m_pipeline->setWorldPosition(
				m_activeAthleteFrame.joints[JointType_SpineBase].position -
				m_ksensor->skeleton()->m_athleteFeetOffset+
				m_generalOffset+
				barbellPosition
			);
			m_lighting->setUniformValue(m_modelViewLocation, m_pipeline->GetWVTrans() );
			m_lighting->setUniformValue(m_projectionLocation, m_pipeline->GetProjTrans());
			drawBar();
		}

		// trainer
		if (m_trainerEnabled) {
			QVector3D skinnedMeshLeftHand = m_trainer->boneEndPosition(m_trainer->findBoneId("thumb_01_l"));
			QVector3D skinnedMeshRightHand = m_trainer->boneEndPosition(m_trainer->findBoneId("thumb_01_r"));
			QVector3D barbellDirection = skinnedMeshRightHand - skinnedMeshLeftHand;
			QVector3D barbellPosition = (skinnedMeshLeftHand + skinnedMeshRightHand) / 2.f;
			m_pipeline->setWorldScale(1.f, 0.75f, 0.75f);
			m_pipeline->setWorldOrientation(QQuaternion::rotationTo(QVector3D(1.f, 0.f, 0.f), barbellDirection));
			m_pipeline->setWorldPosition(
				m_activeTrainerFrame.joints[JointType_SpineBase].position -
				m_ksensor->skeleton()->m_trainerFeetOffset-
				m_generalOffset+
				barbellPosition
			);
			m_lighting->setUniformValue(m_modelViewLocation, m_pipeline->GetWVTrans());
			m_lighting->setUniformValue(m_projectionLocation, m_pipeline->GetProjTrans());
			drawBarbell();
		}
	}

	if (m_tipsDrawing) {

		// bind lighting shaders
		m_lighting->bind();
		m_lighting->setUniformValue(m_projectionLocation, m_pipeline->GetProjTrans());
		for (uint i = 0; i < m_comparisonJoints.size(); i++) {
			QQuaternion athleteAlign = QQuaternion::fromDirection(
				-QVector3D::crossProduct(m_ksensor->skeleton()->m_athleteInitialBarbellDirection, QVector3D(0, 1, 0)),
				QVector3D(0, 1, 0));
			QQuaternion trainerAlign = QQuaternion::fromDirection(
				-QVector3D::crossProduct(m_ksensor->skeleton()->m_athleteInitialBarbellDirection, QVector3D(0, 1, 0)),
				QVector3D(0, 1, 0));
			const QVector3D& athleteJoint =
				athleteAlign *(m_activeAthleteFrame.joints[m_comparisonJoints[i]].position
				- m_ksensor->skeleton()->m_athleteFeetOffset);
			const QVector3D& trainerJoint =
				trainerAlign * (m_activeTrainerFrame.joints[m_comparisonJoints[i]].position
				- m_ksensor->skeleton()->m_trainerFeetOffset);
			float scaleFactor = athleteJoint.distanceToPoint(trainerJoint);
			if (scaleFactor < 0.1) continue;
			m_pipeline->setWorldScale(scaleFactor / 3, scaleFactor, scaleFactor / 3);
			m_pipeline->setWorldOrientation(QQuaternion::rotationTo(QVector3D(0.f, 1.f, 0.f), trainerJoint - athleteJoint));
			m_pipeline->setWorldPosition(athleteJoint + m_generalOffset);

			m_lighting->setUniformValue(m_modelViewLocation, m_pipeline->GetWVTrans());
			drawPointer();
		}
	}

	// draw kinect skeletons
	if (m_kinectSkeletonDrawing) {
		m_technique->enable();
		// athlete
		if (m_athleteEnabled) {
			QQuaternion q = QQuaternion::fromDirection(
				-QVector3D::crossProduct(m_ksensor->skeleton()->m_athleteInitialBarbellDirection, QVector3D(0, 1, 0)),
				QVector3D(0, 1, 0));
			cout << toStringEulerAngles(q).toStdString() << endl;
			QMatrix4x4 athleteSkeletonAlignX(fromRotation(q));
			QMatrix4x4 athleteSkeletonOffset(fromTranslation(-m_ksensor->skeleton()->m_athleteFeetOffset));
			QMatrix4x4 athleteSkeletonTransform(athleteSkeletonAlignX*athleteSkeletonOffset);

			m_pipeline->setWorldScale(QVector3D(1.f, 1.f, 1.f));
			m_pipeline->setWorldOrientation(QQuaternion());
			m_pipeline->setWorldPosition(m_generalOffset);

			// skeleton bones
			m_technique->setMVP(m_pipeline->getWVPtrans() * athleteSkeletonTransform);
			m_technique->setSpecific(QMatrix4x4());
			drawSkeleton(m_activeAthleteFrame.joints, true);

			// skeleton joints
			if (m_kinectSkeletonJointsDrawing) {
				m_technique->setMVP(m_pipeline->getWVPtrans() * athleteSkeletonTransform);
				for (uint i = 0; i < JointType_Count; i++) {
					m_technique->setSpecific(fromTranslation(m_activeAthleteFrame.joints[i].position));
					drawCube(m_activeAthleteFrame.joints[i], true);
				}
			}

			// joint axes
			if (m_axesDrawing) {
				m_technique->setMVP(m_pipeline->getWVPtrans() * athleteSkeletonTransform);
				QMatrix4x4 T(fromTranslation(m_activeAthleteFrame.joints[m_activeJointId].position));
				QMatrix4x4 R(fromRotation(m_activeAthleteFrame.joints[m_activeJointId].orientation));
				QMatrix4x4 S(fromScaling(0.5f, 0.5f, 0.5f));
				m_technique->setSpecific(T * R * S);
				drawAxes();
			}
		}

		// trainer
		if (m_trainerEnabled) {
			QQuaternion q = QQuaternion::fromDirection(
				-QVector3D::crossProduct(m_ksensor->skeleton()->m_trainerInitialBarbellDirection, QVector3D(0, 1, 0)),
				QVector3D(0, 1, 0));
			QMatrix4x4 trainerSkeletonAlignX = fromRotation(q);
			cout << toStringEulerAngles(q).toStdString() << endl;
			QMatrix4x4 trainerSkeletonOffset = fromTranslation(-m_ksensor->skeleton()->m_trainerFeetOffset);
			QMatrix4x4 trainerSkeletonTransform(trainerSkeletonAlignX*trainerSkeletonOffset);

			m_pipeline->setWorldScale(QVector3D(1.f, 1.f, 1.f));
			m_pipeline->setWorldOrientation(QQuaternion());
			m_pipeline->setWorldPosition(-m_generalOffset);

			// skeleton bones
			m_technique->setMVP(m_pipeline->getWVPtrans() * trainerSkeletonTransform);
			m_technique->setSpecific(QMatrix4x4());
			drawSkeleton(m_activeTrainerFrame.joints, false);

			// skeleton joints
			if (m_kinectSkeletonJointsDrawing) {
				m_technique->setMVP(m_pipeline->getWVPtrans() * trainerSkeletonTransform);
				for (uint i = 0; i < JointType_Count; i++) {
					m_technique->setSpecific(fromTranslation(m_activeTrainerFrame.joints[i].position));
					drawCube(m_activeTrainerFrame.joints[i], false);
				}
			}

			// joint axes
			if (m_axesDrawing) {
				m_technique->setMVP(m_pipeline->getWVPtrans() * trainerSkeletonTransform);
				QMatrix4x4 S(fromScaling(0.5f, 0.5f, 0.5f));
				QMatrix4x4 R(fromRotation(m_activeTrainerFrame.joints[m_activeJointId].orientation));
				QMatrix4x4 T(fromTranslation(m_activeTrainerFrame.joints[m_activeJointId].position));
				m_technique->setSpecific(T * R * S);
				drawAxes();
			}
		}
	}

	// draw axes at origin VP origin
	m_technique->enable();
	if (m_axesDrawing) {
		m_technique->setSpecific(QMatrix4x4());
		m_technique->setMVP(m_pipeline->getVPtrans()); // only VP transformation! 
		drawAxes();
	}

	// enable plane shaders
	m_shaderProgram->bind();
	
	// draw plane
	if (m_floorDrawing) {
		m_pipeline->setWorldScale(2.f, 2.f, 2.f);
		m_pipeline->setWorldOrientation(QQuaternion());
		m_pipeline->setWorldPosition(0.f, -0.05f, 0.f);
		m_shaderProgram->setUniformValue(m_mvpLocation, m_pipeline->getWVPtrans());
		drawPlane();
	}

	// calculate barbell direction
	QVector3D leftHand;
	QVector3D rightHand;
	if (m_athleteEnabled) {
		leftHand = m_activeAthleteFrame.joints[JointType_HandLeft].position;
		rightHand = m_activeAthleteFrame.joints[JointType_HandRight].position;
	}
	else {
		leftHand = m_activeTrainerFrame.joints[JointType_HandLeft].position;
		rightHand = m_activeTrainerFrame.joints[JointType_HandRight].position;
	}
	QVector3D barDirection = rightHand - leftHand;

	// calculate bar horizontal angle
	m_barAngle = ToDegrees(atan2(barDirection.y(), sqrt(pow(barDirection.x(), 2) + pow(barDirection.z(), 2))));
	
	// calculate knee angle
	QVector3D hipRight   = (m_activeAthleteFrame).joints[JointType_HipRight].position;
	QVector3D kneeRight  = (m_activeAthleteFrame).joints[JointType_KneeRight].position;
	QVector3D ankleRight = (m_activeAthleteFrame).joints[JointType_AnkleRight].position;
	QVector3D kneeToHip  = (hipRight - kneeRight).normalized();
	QVector3D kneeToAnkle = (ankleRight - kneeRight).normalized();
	m_kneeAngle = ToDegrees(acos(QVector3D::dotProduct(kneeToHip, kneeToAnkle)));
	
	if (!m_isPaused && m_shouldUpdate && m_activeMode == Mode::PLAYBACK) {
		if (++m_activeFrameIndex > m_ksensor->skeleton()->m_bigMotionSize)  m_activeFrameIndex = 0;
		emit frameChanged(activeMotionProgress());
		calculateFPS();
		m_shouldUpdate = false;
	}
}
void MainWidget::calculateFPS()
{
	static clock_t ticksThisTime;
	static clock_t ticksLastTime = clock();
	static uint framesPassed = 0;

	ticksThisTime = clock();
	clock_t ticksPassed = ticksThisTime - ticksLastTime;
	double millisecondsPassed = ticksToMilliseconds(ticksPassed);

	framesPassed++;
	if (millisecondsPassed > 1000.) {
		m_fpsCount = framesPassed / (millisecondsPassed / 1000.);
		framesPassed = 0;
		ticksLastTime = ticksThisTime;
	}
}
void MainWidget::keyPressEvent(QKeyEvent *event)
{
	int key = event->key();
	switch (key) {
	case Qt::Key_Down:
	case Qt::Key_Up:
	case Qt::Key_Left:
	case Qt::Key_Right:
		m_camera->onKeyboardArrow(key, false);
		m_pipeline->setCamera(m_camera->GetPos(), m_camera->GetTarget(), m_camera->GetUp());
		m_skinningTechnique->enable();
		m_skinningTechnique->SetEyeWorldPos(m_camera->GetPos());
		m_skinningTechnique->setWVP(m_pipeline->getWVPtrans());
		m_technique->enable();
		m_technique->setMVP(m_pipeline->getVPtrans());
		break;
	case Qt::Key_0:
	case Qt::Key_1:
	case Qt::Key_2:
	case Qt::Key_3:
	case Qt::Key_4:
	case Qt::Key_5:
	case Qt::Key_6:
	case Qt::Key_7:
	case Qt::Key_8:
	case Qt::Key_9:
		m_athlete->flipParameter(key - Qt::Key_0);
		m_trainer->flipParameter(key - Qt::Key_0);
		break;
	case Qt::Key_C:
		m_ksensor->skeleton()->calculateJointOrientations(*m_activeAthleteMotion);
		m_ksensor->skeleton()->calculateJointOrientations(*m_activeTrainerMotion);
		break;
	case Qt::Key_D:
		m_defaultPose = !m_defaultPose;
		cout << "Default pause " << (m_defaultPose ? "ON" : "OFF") << endl;
		break;
	case Qt::Key_I:
		if (m_athleteEnabled) {
			m_ksensor->skeleton()->m_athletePhases = m_ksensor->skeleton()->identifyPhases(
				m_ksensor->skeleton()->m_athleteAdjustedMotion);
		}
		if (m_trainerEnabled) {
			m_ksensor->skeleton()->m_trainerPhases = m_ksensor->skeleton()->identifyPhases(
				m_ksensor->skeleton()->m_trainerAdjustedMotion);
		}
		break;
	case Qt::Key_L:
		if (m_athleteEnabled) {
			cout << "Athlete limbs" << endl;
			m_ksensor->skeleton()->calculateLimbLengths(*m_activeAthleteMotion);
			m_ksensor->skeleton()->printLimbLengths();
		}
		if (m_trainerEnabled) {
			cout << "Trainer limbs" << endl;
			m_ksensor->skeleton()->calculateLimbLengths(*m_activeTrainerMotion);
			m_ksensor->skeleton()->printLimbLengths();
		}
		break;
	case Qt::Key_M:
		m_playbackInterval /= 1.2f; 
		cout << "Playback interval: " << m_playbackInterval << endl;
		m_timer.setInterval(m_playbackInterval);
		break;
	case Qt::Key_N:
		m_playbackInterval *= 1.2f; 
		cout << "Playback interval: " << m_playbackInterval << endl;
		m_timer.setInterval(m_playbackInterval);
		break;
	case Qt::Key_P:
		if (m_isPaused) {
			m_isPaused = false;
			cout << "Playback Resumed" << endl;
			m_timer.start();
		}
		else {
			m_isPaused = true;
			cout << "Playback paused" << endl;
			m_timer.stop();
		}
		break;
	case Qt::Key_Q:
		m_ksensor->skeleton()->processSpecific();
		break;
	case Qt::Key_R:
		if (m_activeMode==Mode::CAPTURE) m_ksensor->skeleton()->record(m_trainerEnabled);
		else cout << "Record does not work in this mode." << endl;
		break;
	case Qt::Key_S:
		m_ksensor->skeleton()->saveFrameSequences();
		break;
	case Qt::Key_T:
		m_ksensor->skeleton()->exportToTRC();
		break;
	case Qt::Key_X:
		if (!m_trainerEnabled) {
			for (uint i = 0; i < NUM_PHASES; i++) {
				if (m_ksensor->skeleton()->m_athletePhases[i] > m_activeFrameIndex) {
					m_activeFrameIndex = m_ksensor->skeleton()->m_athletePhases[i];
					emit frameChanged(activeMotionProgress());
					return;
				}
			}
			m_activeFrameIndex = m_activeAthleteMotion->size() - 1;
		}
		else {
			for (uint i = 0; i < NUM_PHASES; i++) {
				if (m_ksensor->skeleton()->m_trainerPhases[i] > m_activeFrameIndex) {
					m_activeFrameIndex = m_ksensor->skeleton()->m_trainerPhases[i];
					emit frameChanged(activeMotionProgress());
					return;
				}
			}
			m_activeFrameIndex = m_activeTrainerMotion->size() - 1;
		}
		emit frameChanged(activeMotionProgress());
		break;
	case Qt::Key_Z:
		if (!m_trainerEnabled) {
			for (uint i = 0; i < NUM_PHASES; i++) {
				if (m_ksensor->skeleton()->m_athletePhases[NUM_PHASES - i -1] < m_activeFrameIndex) {
					m_activeFrameIndex = m_ksensor->skeleton()->m_athletePhases[NUM_PHASES - i - 1];
					emit frameChanged(activeMotionProgress());
					return;
				}
			}
			m_activeFrameIndex = 0;
		}
		else {
			for (uint i = 0; i < NUM_PHASES; i++) {
				if (m_ksensor->skeleton()->m_trainerPhases[NUM_PHASES - i - 1] < m_activeFrameIndex) {
					m_activeFrameIndex = m_ksensor->skeleton()->m_trainerPhases[NUM_PHASES - i - 1];
					emit frameChanged(activeMotionProgress());
					return;
				}
			}
			m_activeFrameIndex = 0;
		}
		emit frameChanged(activeMotionProgress());
		break;
	case Qt::Key_Escape:
		event->ignore();	// event passed to MainWidget's parent (MainWindow)
		break;
	default:
		cout << "MainWidget: This key does not do anything." << endl;
		break;
	}
	update();
}
void MainWidget::mousePressEvent(QMouseEvent *event)
{
	m_lastMousePosition = event->pos();
}
void MainWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - m_lastMousePosition.x();
	int dy = event->y() - m_lastMousePosition.y();

	if (event->buttons() & Qt::LeftButton){
		if (event->modifiers() & Qt::ControlModifier) {
			m_camera->rotateRight(-dx);
		}
		else if (event->modifiers() & Qt::ShiftModifier) {
			m_camera->rotateUp(-dy);
		}
		else {
			m_camera->rotateRight(-dx);
			m_camera->rotateUp(-dy);
		}
	}
	m_lastMousePosition = event->pos();

	m_pipeline->setCamera(m_camera->GetPos(), m_camera->GetTarget(), m_camera->GetUp());
	m_skinningTechnique->enable();
	m_skinningTechnique->SetEyeWorldPos(m_camera->GetPos());
	m_skinningTechnique->setWVP(m_pipeline->getWVPtrans());
	m_technique->enable();
	m_technique->setMVP(m_pipeline->getVPtrans());
	update();
}
void MainWidget::wheelEvent(QWheelEvent *event)
{
	QPoint degrees = event->angleDelta() / 8;
	//cout << degrees.x() << " " << degrees.y() << endl;
	m_camera->onMouseWheel(degrees.y(), false);

	m_pipeline->setCamera(m_camera->GetPos(), m_camera->GetTarget(), m_camera->GetUp());
	m_skinningTechnique->enable();
	m_skinningTechnique->SetEyeWorldPos(m_camera->GetPos());
	m_skinningTechnique->setWVP(m_pipeline->getWVPtrans());
	m_technique->enable();
	m_technique->setMVP(m_pipeline->getVPtrans());
	update();
}
KSensor* MainWidget::ksensor()
{
	return m_ksensor;
}
void MainWidget::setAxesDrawing(bool state)
{
	m_axesDrawing = state;		
	update();
}
void MainWidget::setSkinnedMeshDrawing(bool state)
{
	m_skinnedMeshDrawing = state;
	update();
}
void MainWidget::setKinectSkeletonDrawing(bool state)
{
	m_kinectSkeletonDrawing = state;
	update();
}
void MainWidget::setFloorDrawing(bool state)
{
	m_floorDrawing = state;
	update();
}
void MainWidget::setBarbellDrawing(bool state)
{
	m_barbellDrawing = state;
	update();
}
void MainWidget::setTipsDrawing(bool state)
{
	m_tipsDrawing = state;
	update();
}
bool MainWidget::axesDrawing() const
{
	return m_axesDrawing;
}
bool MainWidget::skinnedMeshDrawing() const
{
	return m_skinnedMeshDrawing;
}
bool MainWidget::kinectSkeletonDrawing() const
{
	return m_kinectSkeletonDrawing;
}
bool MainWidget::floorDrawing() const
{
	return m_floorDrawing;
}
bool MainWidget::barbellDrawing() const
{
	return m_barbellDrawing;
}
bool MainWidget::tipsDrawing() const
{
	return m_tipsDrawing;
}
QVector3D MainWidget::activeJointVelocity() const
{
	QVector3D currentPosition, nextPosition;
	double currentTime, nextTime;
	if (m_athleteEnabled) {
		currentPosition = 
			m_activeAthleteMotion->operator[](m_activeFrameIndex).joints[m_activeJointId].position;
		nextPosition =
			(m_activeFrameIndex == m_activeAthleteMotion->size() - 1) ?
			QVector3D(0.f, 0.f, 0.f) :
			m_activeAthleteMotion->operator[](m_activeFrameIndex + 1).joints[m_activeJointId].position;
		currentTime =
			m_activeAthleteMotion->operator[](m_activeFrameIndex).timestamp;
		nextTime =
			(m_activeFrameIndex == m_activeAthleteMotion->size() - 1) ?
			0 :
			m_activeAthleteMotion->operator[](m_activeFrameIndex + 1).timestamp;
	}
	else {
		currentPosition =
			m_activeTrainerMotion->operator[](m_activeFrameIndex).joints[m_activeJointId].position;
		nextPosition =
			(m_activeFrameIndex == m_activeTrainerMotion->size() - 1) ?
			QVector3D(0.f, 0.f, 0.f) :
			m_activeTrainerMotion->operator[](m_activeFrameIndex + 1).joints[m_activeJointId].position;
		currentTime =
			m_activeTrainerMotion->operator[](m_activeFrameIndex).timestamp;
		nextTime =
			(m_activeFrameIndex == m_activeTrainerMotion->size() - 1) ?
			0 :
			m_activeTrainerMotion->operator[](m_activeFrameIndex + 1).timestamp;
	}
	return (nextPosition - currentPosition) / (nextTime - currentTime);
}
float MainWidget::activeJointAngle() const
{
	if (m_activeJointId == JointType_SpineBase || m_ksensor->skeleton()->nodes()[m_activeJointId].childrenId.size() == 0) return 0.f;
	
	uint parentId = m_ksensor->skeleton()->nodes()[m_activeJointId].parentId;
	uint childId = m_ksensor->skeleton()->nodes()[m_activeJointId].childrenId[0];
	QVector3D thisToParent;
	QVector3D thisToChild;
	if (m_athleteEnabled) {
		thisToParent =
			m_activeAthleteFrame.joints[parentId].position-
			m_activeAthleteFrame.joints[m_activeJointId].position;
		thisToChild =
			m_activeAthleteFrame.joints[childId].position -
			m_activeAthleteFrame.joints[m_activeJointId].position;
	}
	else {
		thisToParent =
			m_activeTrainerFrame.joints[parentId].position -
			m_activeTrainerFrame.joints[m_activeJointId].position;
		thisToChild =
			m_activeTrainerFrame.joints[childId].position -
			m_activeTrainerFrame.joints[m_activeJointId].position;
	}
	double angle = ToDegrees(acos(QVector3D::dotProduct(thisToParent.normalized(), thisToChild.normalized())));
	return angle;
}
bool MainWidget::modelSkinning() const
{
	return m_skinningEnabled;
}
QStringList MainWidget::skeletonJointList() const
{
	QStringList qsl;
	for (int i = 0; i < JointType_Count; i++) qsl << m_ksensor->skeleton()->nodes()[i].name;
	return qsl;
}
QStringList MainWidget::modelBoneList() const
{
	QStringList qsl;
	for (const auto& it : m_athlete->boneMap()) qsl << QString(it.first.c_str());
	return qsl;
}
QStringList MainWidget::motionTypeList() const
{
	return m_motionTypeList;
}
bool MainWidget::captureEnabled() const
{
	if (m_activeMode == Mode::CAPTURE) return true;
	else return false;
}
void MainWidget::setCaptureEnabled(bool state)
{
	if (state == true) {
		cout << "Active mode: Capture" << endl;
		if (!m_ksensor->isPrepared()) {
			cout << "Kinect sensor is not prepared to capture!" << endl;
		}
		m_activeMode = Mode::CAPTURE;
		m_timer.setInterval(0);
	}
	else {
		cout << "Active mode: Playback" << endl;
		m_activeMode = Mode::PLAYBACK;
		m_timer.setInterval(m_playbackInterval * 1000);
	}
}
bool MainWidget::athleteEnabled() const
{
	return m_athleteEnabled;
}
bool MainWidget::trainerEnabled() const
{
	return m_trainerEnabled;
}
void MainWidget::setAthleteEnabled(bool state)
{
	m_athleteEnabled = state;
	if (m_athleteEnabled && m_trainerEnabled) m_generalOffset = QVector3D(1.5f, 0.f, 0.f);
	else m_generalOffset = QVector3D(0.f, 0.f, 0.f);
	m_ksensor->skeleton()->m_athleteRecording = state;
	cout << "Athlete " << ( m_athleteEnabled ? "enabled" : "disabled" ) << endl;
	update();
}
void MainWidget::setTrainerEnabled(bool state)
{
	m_trainerEnabled = state;
	if (m_athleteEnabled && m_trainerEnabled) m_generalOffset = QVector3D(1.5f, 0.f, 0.f);
	else m_generalOffset = QVector3D(0.f, 0.f, 0.f);
	m_ksensor->skeleton()->m_trainerRecording = state;
	cout << "Trainer " << (m_trainerEnabled ? "enabled" : "disabled") << endl;
	update();
}
int MainWidget::activeMotionType() const
{
	return m_activeMotionType;
}
void MainWidget::setActiveMotionType(int motionType)
{	
	m_activeMotionType = motionType;
	if (motionType == 0) {
		m_activeAthleteMotion = &m_ksensor->skeleton()->m_athleteRawMotion;
		m_activeTrainerMotion = &m_ksensor->skeleton()->m_trainerRawMotion;
	}
	else if (motionType == 1) {
		m_activeAthleteMotion = &m_ksensor->skeleton()->m_athleteInterpolatedMotion;
		m_activeTrainerMotion = &m_ksensor->skeleton()->m_trainerInterpolatedMotion;
	}
	else if (motionType == 2) {
		m_activeAthleteMotion = &m_ksensor->skeleton()->m_athleteFilteredMotion;
		m_activeTrainerMotion = &m_ksensor->skeleton()->m_trainerFilteredMotion;
	}
	else if (motionType == 3) {
		m_activeAthleteMotion = &m_ksensor->skeleton()->m_athleteAdjustedMotion;
		m_activeTrainerMotion = &m_ksensor->skeleton()->m_trainerAdjustedMotion;
	}
	else if (motionType == 4) {
		m_activeAthleteMotion = &m_ksensor->skeleton()->m_athleteRescaledMotion;
		m_activeTrainerMotion = &m_ksensor->skeleton()->m_trainerAdjustedMotion;
	}
	else {
		cout << "Error: Motion type = " << m_activeMotionType << endl;
		return;
	}
	cout << "Motion type: " << m_motionTypeList[m_activeMotionType].toStdString() << endl;
	update();
}
void MainWidget::setModelSkinning(bool state)
{
	m_skinningTechnique->enable();
	m_skinningTechnique->setSkinning(state);
	update();
}
SkinnedMesh* MainWidget::skinnedMesh()
{
	return m_athlete;
}
SkinningTechnique* MainWidget::skinningTechnique()
{
	return m_skinningTechnique;
}
Technique* MainWidget::technique()
{
	return m_technique;
}
bool MainWidget::isPaused() const
{
	return m_isPaused;
}
void MainWidget::setIsPaused(bool state)
{
	m_isPaused = state;
}
void MainWidget::intervalPassed()
{
	m_shouldUpdate = true;
	update();
}
void MainWidget::setGeneralOffset(int percent)
{
	m_generalOffset = QVector3D(percent / 50.f, 0.f, 0.f);
	update();
}
int MainWidget::activeMotionProgress() const
{
	int progressPercent = (int)((float)m_activeFrameIndex / m_ksensor->skeleton()->m_bigMotionSize * 100.f);
	return progressPercent;
}
void MainWidget::setActiveMotionProgress(int progressPercent)
{
	if (progressPercent > 100 || progressPercent < 0) {
		cout << "Progress percent out of bounds: " << progressPercent << endl;
		return;
	}
	int newFrameIndex = (int)(progressPercent / 100.f * m_ksensor->skeleton()->m_bigMotionSize);
	m_activeFrameIndex = newFrameIndex;
	if (newFrameIndex > m_ksensor->skeleton()->m_bigMotionSize || newFrameIndex < 0) {
		cout << "New frame index out of bounds: " << newFrameIndex << endl;
		return;
	}
	update();
}
void MainWidget::setActiveBone(const QString& boneName)
{
	m_activeBoneId = m_athlete->findBoneId(boneName);
	update();
}
void MainWidget::setActiveJointId(int jointId)
{
	m_activeJointId = jointId;
}
void MainWidget::unloadAthlete()
{
	for (uint i = 0; i < m_athleteTextures.size(); i++) {
		SAFE_DELETE(m_athleteTextures[i]);
	}

	if (m_athleteVBOs[0] != 0) {
		glDeleteBuffers(ARRAY_SIZE_IN_ELEMENTS(m_athleteVBOs), m_athleteVBOs);
	}

	if (m_athleteVAO != 0) {
		glDeleteVertexArrays(1, &m_athleteVAO);
		m_athleteVAO = 0;
	}
}
void MainWidget::loadAthlete()
{
	unloadAthlete();

	glGenVertexArrays(1, &m_athleteVAO);
	cout << "skinnedMeshVAO=" << m_athleteVAO << endl;
	glBindVertexArray(m_athleteVAO);

	glGenBuffers(ARRAY_SIZE_IN_ELEMENTS(m_athleteVBOs), m_athleteVBOs);
	for (uint i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_athleteVBOs); i++) {
		cout << "skinnedMeshVBO=" << m_athleteVBOs[i] << endl;
	}

#define POSITION_LOCATION    0
#define TEX_COORD_LOCATION   1
#define NORMAL_LOCATION      2
#define BONE_ID_LOCATION     3
#define BONE_WEIGHT_LOCATION 4

	const auto& positions = m_athlete->positions();
	const auto& texCoords = m_athlete->texCoords();
	const auto& normals = m_athlete->normals();
	const auto& vertexBoneData = m_athlete->vertexBoneData();
	const auto& indices = m_athlete->indices();

	glBindBuffer(GL_ARRAY_BUFFER, m_athleteVBOs[POS_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(positions[0]) * positions.size(), positions.constData(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(POSITION_LOCATION);
	glVertexAttribPointer(POSITION_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_athleteVBOs[TEXCOORD_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(texCoords[0]) * texCoords.size(), texCoords.constData(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(TEX_COORD_LOCATION);
	glVertexAttribPointer(TEX_COORD_LOCATION, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_athleteVBOs[NORMAL_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(normals[0]) * normals.size(), normals.constData(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(NORMAL_LOCATION);
	glVertexAttribPointer(NORMAL_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_athleteVBOs[BONE_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertexBoneData[0]) * vertexBoneData.size(), vertexBoneData.constData(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(BONE_ID_LOCATION);
	glVertexAttribIPointer(BONE_ID_LOCATION, 4, GL_INT, sizeof(VertexBoneData), (const GLvoid*)0);
	glEnableVertexAttribArray(BONE_WEIGHT_LOCATION);
	glVertexAttribPointer(BONE_WEIGHT_LOCATION, 4, GL_FLOAT, GL_FALSE, sizeof(VertexBoneData), (const GLvoid*)16);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_athleteVBOs[INDEX_BUFFER]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0]) * indices.size(), indices.constData(), GL_STATIC_DRAW);

	glBindVertexArray(0);

	for (int i = 0; i < m_athlete->images().size(); i++) {
		m_athleteTextures.push_back(new QOpenGLTexture(m_athlete->images()[i]));
	}

	if (GLNoError()) {
		cout << "Successfully loaded SkinnedMesh to GPU" << endl;
	}
	else {
		cout << "Error loading SkinnedMesh to GPU" << endl;
		GLPrintError();
	}
}
void MainWidget::drawAthlete()
{
	glBindVertexArray(m_athleteVAO);

	const auto& meshEntries = m_athlete->meshEntries();
	for (uint i = 0; i < meshEntries.size(); i++) {
		const uint materialIndex = meshEntries[i].materialIndex;
		assert(materialIndex < m_athleteTextures.size());
		m_athleteTextures[materialIndex]->bind();
		glDrawElementsBaseVertex(
			GL_TRIANGLES									,
			meshEntries[i].numIndices						,
			GL_UNSIGNED_INT									,
			(void*)(sizeof(uint) * meshEntries[i].baseIndex),
			meshEntries[i].baseVertex
		);
	}

	glBindVertexArray(0);
}
void MainWidget::loadTrainer()
{
	unloadTrainer();

	glGenVertexArrays(1, &m_trainerVAO);
	cout << "skinnedMeshVAO=" << m_trainerVAO << endl;
	glBindVertexArray(m_trainerVAO);

	glGenBuffers(ARRAY_SIZE_IN_ELEMENTS(m_trainerVBOs), m_trainerVBOs);
	for (uint i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_trainerVBOs); i++) {
		cout << "skinnedMeshVBO=" << m_trainerVBOs[i] << endl;
	}

#define POSITION_LOCATION    0
#define TEX_COORD_LOCATION   1
#define NORMAL_LOCATION      2
#define BONE_ID_LOCATION     3
#define BONE_WEIGHT_LOCATION 4

	const auto& positions = m_trainer->positions();
	const auto& texCoords = m_trainer->texCoords();
	const auto& normals = m_trainer->normals();
	const auto& vertexBoneData = m_trainer->vertexBoneData();
	const auto& indices = m_trainer->indices();

	glBindBuffer(GL_ARRAY_BUFFER, m_trainerVBOs[POS_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(positions[0]) * positions.size(), positions.constData(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(POSITION_LOCATION);
	glVertexAttribPointer(POSITION_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_trainerVBOs[TEXCOORD_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(texCoords[0]) * texCoords.size(), texCoords.constData(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(TEX_COORD_LOCATION);
	glVertexAttribPointer(TEX_COORD_LOCATION, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_trainerVBOs[NORMAL_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(normals[0]) * normals.size(), normals.constData(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(NORMAL_LOCATION);
	glVertexAttribPointer(NORMAL_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_trainerVBOs[BONE_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertexBoneData[0]) * vertexBoneData.size(), vertexBoneData.constData(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(BONE_ID_LOCATION);
	glVertexAttribIPointer(BONE_ID_LOCATION, 4, GL_INT, sizeof(VertexBoneData), (const GLvoid*)0);
	glEnableVertexAttribArray(BONE_WEIGHT_LOCATION);
	glVertexAttribPointer(BONE_WEIGHT_LOCATION, 4, GL_FLOAT, GL_FALSE, sizeof(VertexBoneData), (const GLvoid*)16);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_trainerVBOs[INDEX_BUFFER]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0]) * indices.size(), indices.constData(), GL_STATIC_DRAW);

	glBindVertexArray(0);

	for (int i = 0; i < m_trainer->images().size(); i++) {
		m_trainerTextures.push_back(new QOpenGLTexture(m_trainer->images()[i]));
	}

	if (GLNoError()) {
		cout << "Successfully loaded trainer to GPU" << endl;
	}
	else {
		cout << "Error loading trainer to GPU" << endl;
		GLPrintError();
	}
}
void MainWidget::unloadTrainer()
{
	for (uint i = 0; i < m_trainerTextures.size(); i++) {
		SAFE_DELETE(m_trainerTextures[i]);
	}

	if (m_trainerVBOs[0] != 0) {
		glDeleteBuffers(ARRAY_SIZE_IN_ELEMENTS(m_trainerVBOs), m_trainerVBOs);
	}

	if (m_trainerVAO != 0) {
		glDeleteVertexArrays(1, &m_trainerVAO);
		m_trainerVAO = 0;
	}
}
void MainWidget::drawTrainer()
{
	glBindVertexArray(m_trainerVAO);

	const auto& meshEntries = m_trainer->meshEntries();
	for (uint i = 0; i < meshEntries.size(); i++) {
		const uint materialIndex = meshEntries[i].materialIndex;
		assert(materialIndex < m_trainerTextures.size());
		m_trainerTextures[materialIndex]->bind();
		glDrawElementsBaseVertex(
			GL_TRIANGLES,
			meshEntries[i].numIndices,
			GL_UNSIGNED_INT,
			(void*)(sizeof(uint) * meshEntries[i].baseIndex),
			meshEntries[i].baseVertex
		);
	}

	glBindVertexArray(0);
}
void MainWidget::loadArrow()
{
	const float head = 0.1f;
	const float colorRed   = 255.f;
	const float colorBlue  = 255.f;
	const float colorGreen = 255.f;
	const GLfloat vertices[] =
	{
		0       , 0			, 0		   , // start
		colorRed, colorGreen, colorBlue,
		0       , 1			, 0		   , // end
		colorRed, colorGreen, colorBlue,
		head    , 1 - head  , 0		   , // +x
		colorRed, colorGreen, colorBlue,
		-head   , 1 - head  , 0		   , // -x
		colorRed, colorGreen, colorBlue,
		0       , 1 - head  , head	   , // +z
		colorRed, colorGreen, colorBlue,
		0       , 1 - head  , -head	   , // -z
		colorRed, colorGreen, colorBlue,
	};

	const GLushort indices[] =
	{
		0, 1,
		1, 2,
		1, 3,
		1, 4,
		1, 5,
	};

	glGenVertexArrays(1, &m_arrowVAO);
	cout << "arrowVAO=" << m_arrowVAO << endl;
	glBindVertexArray(m_arrowVAO);

	GLuint arrowIBO;
	glGenBuffers(1, &arrowIBO);
	cout << "arrowIBO=" << arrowIBO << endl;
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, arrowIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	GLuint arrowVBO;
	glGenBuffers(1, &arrowVBO);
	cout << "arrowVBO=" << arrowVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, arrowVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, BUFFER_OFFSET(sizeof(GLfloat) * 3));

	glBindVertexArray(0);
}
void MainWidget::drawArrow()
{
	glBindVertexArray(m_arrowVAO);

	glDrawElements(GL_LINES, 10, GL_UNSIGNED_SHORT, 0);

	glBindVertexArray(0);
}
void MainWidget::loadAxes()
{
	GLfloat vertices[] =
	{
		0.f  , 0.f, 0.f,  // origin position
		0.f  , 0.f, 0.f, // origin color
		1.f  , 0.f, 0.f, // x axis position
		255.f, 0.f  , 0.f, // x axis color
		0.f  , 1.f, 0.f, // y axis position
		0.f  , 255.f, 0.f, // y axis color
		0.f  , 0.f, 1.f, // z axis position
		0.f  , 0.f  , 255.f // z axis color
	};

	GLushort indices[] =
	{
		0, 1,
		0, 2,
		0, 3
	};

	glGenVertexArrays(1, &m_axesVAO);
	glBindVertexArray(m_axesVAO);
	cout << "axesVAO=" << m_axesVAO << endl;

	GLuint axesIBO;
	glGenBuffers(1, &axesIBO);
	cout << "axesIBO=" << axesIBO << endl;
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, axesIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	GLuint axesVBO;
	glGenBuffers(1, &axesVBO);
	cout << "axesVBO=" << axesVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, axesVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, BUFFER_OFFSET(sizeof(GLfloat) * 3));

	glBindVertexArray(0);
}
void MainWidget::drawAxes()
{
	glBindVertexArray(m_axesVAO);

	glDrawElements(GL_LINES, 6, GL_UNSIGNED_SHORT, 0);

	glBindVertexArray(0);
}
void MainWidget::loadSkeleton()
{
	GLushort indices[] =
	{
		// core (4 parts)
		JointType_SpineBase    , JointType_SpineMid,
		JointType_SpineMid     , JointType_SpineShoulder,
		JointType_SpineShoulder, JointType_Neck,
		JointType_Neck         , JointType_Head,
		// left side (10 parts)	   
		JointType_SpineShoulder, JointType_ShoulderLeft,
		JointType_ShoulderLeft , JointType_ElbowLeft,
		JointType_ElbowLeft    , JointType_WristLeft,
		JointType_WristLeft    , JointType_HandLeft,
		JointType_HandLeft     , JointType_ThumbLeft,
		JointType_HandLeft     , JointType_HandTipLeft,
		JointType_SpineBase    , JointType_HipLeft,
		JointType_HipLeft      , JointType_KneeLeft,
		JointType_KneeLeft     , JointType_AnkleLeft,
		JointType_AnkleLeft    , JointType_FootLeft,
		// Right side (10 parts) 
		JointType_SpineShoulder, JointType_ShoulderRight,
		JointType_ShoulderRight, JointType_ElbowRight,
		JointType_ElbowRight   , JointType_WristRight,
		JointType_WristRight   , JointType_HandRight,
		JointType_HandRight    , JointType_ThumbRight,
		JointType_HandRight    , JointType_HandTipRight,
		JointType_SpineBase    , JointType_HipRight,
		JointType_HipRight     , JointType_KneeRight,
		JointType_KneeRight    , JointType_AnkleRight,
		JointType_AnkleRight   , JointType_FootRight
	};

	glGenVertexArrays(1, &m_skeletonVAO);
	glBindVertexArray(m_skeletonVAO);
	cout << "kinectSkeletonJointsVAO=" << m_skeletonVAO << endl;

	GLuint kinectSkeletonJointsIBO;
	glGenBuffers(1, &kinectSkeletonJointsIBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, kinectSkeletonJointsIBO);
	cout << "kinectSkeletonJointsIBO=" << kinectSkeletonJointsIBO << endl;
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_skeletonVBO);
	cout << "kinectSkeletonJointsVBO=" << m_skeletonVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, m_skeletonVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * 3 * JointType_Count, NULL, GL_STREAM_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, BUFFER_OFFSET(sizeof(GLfloat) * 3));

	glBindVertexArray(0);
}
void MainWidget::drawSkeleton(const array<KJoint, JointType_Count>& joints, bool athlete)
{
	for (uint i = 0; i < JointType_Count; i++) {
		m_skeleton[6 * i    ] = joints[i].position.x();
		m_skeleton[6 * i + 1] = joints[i].position.y();
		m_skeleton[6 * i + 2] = joints[i].position.z();
		m_skeleton[6 * i + 3] = (athlete ? 255.f : 0.f);
		m_skeleton[6 * i + 4] = (!athlete ? 255.f : 0.f);
		m_skeleton[6 * i + 5] = 0.f;
	}

	glBindBuffer(GL_ARRAY_BUFFER, m_skeletonVBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_skeleton), m_skeleton);

	glBindVertexArray(m_skeletonVAO);

	glDrawElements(GL_LINES, 48, GL_UNSIGNED_SHORT, 0);

	glBindVertexArray(0);
}
void MainWidget::loadSkinnedMeshJoints()
{
	glGenVertexArrays(1, &m_skinnedMeshJointsVAO);
	glBindVertexArray(m_skinnedMeshJointsVAO);
	cout << "skinnedMeshJointsVAO=" << m_skinnedMeshJointsVAO << endl;

	glGenBuffers(1, &m_skinnedMeshJointsVBO);
	glBindBuffer(GL_ARRAY_BUFFER, m_skinnedMeshJointsVBO);
	cout << "skinnedMeshJointsVBO=" << m_skinnedMeshJointsVBO << endl;
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * 3 * NUM_BONES, NULL, GL_STREAM_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, BUFFER_OFFSET(sizeof(GLfloat) * 3));

	glBindVertexArray(0);
}
void MainWidget::drawSkinnedMeshJoints(const SkinnedMesh* sm)
{
	for (uint i = 0; i < NUM_BONES; i++) {
		m_skinnedMeshJoints[6 * i    ] = sm->boneEndPosition(i).x();
		m_skinnedMeshJoints[6 * i + 1] = sm->boneEndPosition(i).y();
		m_skinnedMeshJoints[6 * i + 2] = sm->boneEndPosition(i).z();
		m_skinnedMeshJoints[6 * i + 3] = 255.f;
		m_skinnedMeshJoints[6 * i + 4] = 255.f;
		m_skinnedMeshJoints[6 * i + 5] = 255.f;	 
	}


	glBindVertexArray(m_skinnedMeshJointsVAO);

	glBindBuffer(GL_ARRAY_BUFFER, m_skinnedMeshJointsVBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_skinnedMeshJoints), m_skinnedMeshJoints);
	glDrawArrays(GL_POINTS, 0, NUM_BONES*60);

	glBindVertexArray(0);
}
// r = edgeLength / 2
void MainWidget::loadCube(float r)
{
	GLfloat vertices[] =
	{
		   +r,  -r,  +r, // 0
		   +r,  +r,  +r, // 1
		   -r,  +r,  +r, // 2
		   -r,  -r,  +r, // 3
		   +r,  -r,  -r, // 4
		   +r,  +r,  -r, // 5
		   -r,  +r,  -r, // 6
		   -r,  -r,  -r, // 7
		  0.f, 0.f, 0.f,
		  0.f, 0.f, 0.f,
		  0.f, 0.f, 0.f,
		  0.f, 0.f, 0.f,
		  0.f, 0.f, 0.f,
		  0.f, 0.f, 0.f,
		  0.f, 0.f, 0.f,
		  0.f, 0.f, 0.f
	};

	GLushort indices[] =
	{
		0, 1, 2, 3,
		0, 4, 5, 1,
		4, 7, 6, 5,
		7, 3, 2, 6,
		2, 1, 5, 6,
		0, 4, 7, 3
	};

	glGenVertexArrays(1, &m_cubeVAO);
	glBindVertexArray(m_cubeVAO);
	cout << "cubeVAO=" << m_cubeVAO << endl;

	GLuint cubeIBO;
	glGenBuffers(1, &cubeIBO);
	cout << "cubeIBO=" << cubeIBO << endl;
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_cubeVBO);
	cout << "cubeVBO=" << m_cubeVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, m_cubeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 3, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 3, BUFFER_OFFSET(sizeof(vertices) / 2));

	glBindVertexArray(0);
}
void MainWidget::drawCube(const KJoint& joint, bool athlete)
{
	QVector3D color(0.f, 0.f, 0.f);
	if (joint.trackingState == TrackingState_Inferred) color.setZ(255.f);
	if (athlete) color.setX(255.f);
	else color.setY(255.f);

	for (uint j = 0; j < 8; j++) {
		m_cubeColors[3 * j + 0] = color.x();
		m_cubeColors[3 * j + 1] = color.y();
		m_cubeColors[3 * j + 2] = color.z();
	}

	glBindVertexArray(m_cubeVAO);

	glBindBuffer(GL_ARRAY_BUFFER, m_cubeVBO);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(m_cubeColors), sizeof(m_cubeColors), m_cubeColors);
	glDrawElements(GL_TRIANGLE_STRIP, JointType_Count, GL_UNSIGNED_SHORT, 0);

	glBindVertexArray(0);
}
void MainWidget::loadPlane()
{
	const GLfloat l = 1.f; // hypotenuse length
	GLfloat planePositions[PLANE_VERTICES * 3] = {
		  l, 0.f, 0.f,
		0.f, 0.f,  -l,
		 -l, 0.f, 0.f,
		0.f, 0.f,   l
	};

	GLfloat planeTexCoords[PLANE_VERTICES * 2] = {
		0.f, 0.f,
		0.f, 1.f,
		1.f, 1.f,
		1.f, 0.f
	};
	GLfloat planeNormals[PLANE_VERTICES * 3] = { 
		0, 255, 0,
		0, 255, 0, 
		0, 255, 0, 
		0, 255, 0 
	};

	glGenVertexArrays(1, &m_planeVAO);
	glBindVertexArray(m_planeVAO);
	cout << "planeVAO=" << m_planeVAO << endl;

	GLuint planeVBO;
	glGenBuffers(1, &planeVBO);
	cout << "planeVBO=" << planeVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(planePositions) + sizeof(planeTexCoords) + sizeof(planeNormals), NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER,											   0, sizeof(planePositions), planePositions);
	glBufferSubData(GL_ARRAY_BUFFER,						  sizeof(planePositions), sizeof(planeTexCoords), planeTexCoords);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(planePositions) + sizeof(planeTexCoords), sizeof(planeNormals)  , planeNormals);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(sizeof(planePositions)));
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(sizeof(planePositions) + sizeof(planeTexCoords)));
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);

	glBindVertexArray(0);

	// Init plane texture
	QImage image;
	if (!image.load("plane/wood.jpg")) cout << "Wood.jpg load failed" << endl;
	m_planeTexture = new QOpenGLTexture(image.mirrored());
	m_planeTexture->setMinificationFilter(QOpenGLTexture::Linear);
	m_planeTexture->setMagnificationFilter(QOpenGLTexture::Linear);
}
void MainWidget::drawPlane()
{
	glBindVertexArray(m_planeVAO);

	m_planeTexture->bind();
	glDrawArrays(GL_TRIANGLE_FAN, 0, PLANE_VERTICES);

	glBindVertexArray(0);
}
void MainWidget::loadBarbell()
{
	cout << "Loading barbell model" << endl;

	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile(
		"models/barbell blendered.obj",
		aiProcess_Triangulate
	);

	if (!scene) {
		cout << "Could not import the barbell model." << endl;
		return;
	}

	// Count vertices and indices and update mesh entries
	uint numVertices = 0;
	uint numIndices = 0;
	m_barbellMeshEntries.resize(scene->mNumMeshes);
	m_barbellMaterials.resize(scene->mNumMaterials);
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		m_barbellMeshEntries[i].materialIndex = scene->mMeshes[i]->mMaterialIndex;
		m_barbellMeshEntries[i].numIndices = scene->mMeshes[i]->mNumFaces * 3;
		m_barbellMeshEntries[i].baseVertex = numVertices;
		m_barbellMeshEntries[i].baseIndex = numIndices;
		numVertices += scene->mMeshes[i]->mNumVertices;
		numIndices += scene->mMeshes[i]->mNumFaces * 3;
		m_barbellMeshEntries[i].print();
	}
	cout << "Totals: NumVertices:" << numVertices << " NumIndices:" << numIndices << endl;

	// Reserve appropriate container space and push back vertex attributes
	QVector<QVector3D> positions;
	QVector<QVector2D> texCoords;
	QVector<QVector3D> normals;
	QVector<uint> indices;
	positions.reserve(numVertices);
	texCoords.reserve(numVertices);
	normals.reserve(numVertices);
	indices.reserve(numIndices);
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		// Push back vertices
		bool hasPositions = true;
		bool hasTexCoords = true;
		bool hasNormals = true;
		if (!scene->mMeshes[i]->HasPositions()) {
			cout << "Mesh " << i << " has no positions!" << endl;
			hasPositions = false;
		}
		if (!scene->mMeshes[i]->HasTextureCoords(0)) {
			cout << "Mesh " << i << " has no tex coords!" << endl;
			hasTexCoords = false;
		}
		if (!scene->mMeshes[i]->HasNormals()) {
			cout << "Mesh " << i << " has no normals!" << endl;
			hasNormals = false;
		}
		for (uint j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
			const aiVector3D* position = &(scene->mMeshes[i]->mVertices[j]);
			const aiVector3D* texCoord = &(scene->mMeshes[i]->mTextureCoords[0][j]);
			const aiVector3D* normal = &(scene->mMeshes[i]->mNormals[j]);
			if (hasPositions) positions.push_back(QVector3D(position->x, position->y, position->z));
			else positions.push_back(QVector3D(0.f, 0.f, 0.f));
			if (hasTexCoords) texCoords.push_back(QVector2D(texCoord->x, texCoord->y));
			else texCoords.push_back(QVector2D(0.f, 0.f));
			if (hasNormals) normals.push_back(QVector3D(normal->x, normal->y, normal->z));
			else normals.push_back(QVector3D(0.f, 1.f, 0.f));
		}

		// Push back indices
		for (uint j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
			const aiFace& face = scene->mMeshes[i]->mFaces[j];
			assert(face.mNumIndices == 3);
			indices.push_back(face.mIndices[0]);
			indices.push_back(face.mIndices[1]);
			indices.push_back(face.mIndices[2]);
		}
	}
	cout << endl;

	traverseSceneNodes(scene->mRootNode);

	cout << "Vector lengths:";
	cout << positions.length() << " ";
	cout << texCoords.length() << " ";
	cout << normals.length() << " ";
	cout << indices.length() << endl;

	cout << "Scene info:";
	cout << " Meshes:" << setw(3) << scene->mNumMeshes;
	cout << " Materials:" << setw(3) << scene->mNumMaterials;
	cout << " Textures:" << setw(3) << scene->mNumTextures;
	cout << " Lights:" << setw(3) << scene->mNumLights;
	cout << " Animations:" << setw(3) << scene->mNumAnimations;
	cout << endl;

	cout << "Meshes:" << endl;
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		cout << "Id:" << i;
		cout << " Name:" << setw(20) << scene->mMeshes[i]->mName.C_Str();
		cout << " Vertices:" << setw(6) << scene->mMeshes[i]->mNumVertices;
		cout << " Faces:" << setw(6) << scene->mMeshes[i]->mNumFaces;
		cout << " Bones:" << setw(6) << scene->mMeshes[i]->mNumBones;
		cout << " MaterialId:" << setw(6) << scene->mMeshes[i]->mMaterialIndex;
		cout << endl;
	}
	cout << endl;

	// Materials
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		// General properties
		const uint materialIndex = scene->mMeshes[i]->mMaterialIndex;
		const aiMaterial* material = scene->mMaterials[materialIndex];
		aiString name;
		int shadingMethod;
		int blendFunction;
		int twoSided;
		float opacity;
		float shininess;
		float shininessStrength;
		material->Get(AI_MATKEY_NAME, name);
		material->Get(AI_MATKEY_SHADING_MODEL, shadingMethod);
		material->Get(AI_MATKEY_BLEND_FUNC, blendFunction);
		material->Get(AI_MATKEY_TWOSIDED, twoSided);
		material->Get(AI_MATKEY_OPACITY, opacity);
		material->Get(AI_MATKEY_SHININESS, shininess);
		material->Get(AI_MATKEY_SHININESS_STRENGTH, shininessStrength);
		cout << "Material:" << i;
		cout << " Name:" << string(name.data);
		cout << " ShadingMethod:" << hex << shadingMethod << dec;
		cout << " BlendFunction:" << hex << blendFunction << dec;
		cout << " TwoSided:" << twoSided;
		cout << " Opacity:" << opacity;
		cout << " Shininess:" << shininess;
		cout << " ShininessStrength:" << shininessStrength;
		cout << endl;

		// Colors
		aiColor3D diffuseColor;
		aiColor3D specularColor;
		aiColor3D ambientColor;
		aiColor3D reflectiveColor;
		aiColor3D emissiveColor;
		aiColor3D transparentColor;
		material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor);
		material->Get(AI_MATKEY_COLOR_SPECULAR, specularColor);
		material->Get(AI_MATKEY_COLOR_AMBIENT, ambientColor);
		material->Get(AI_MATKEY_COLOR_REFLECTIVE, reflectiveColor);
		material->Get(AI_MATKEY_COLOR_EMISSIVE, emissiveColor);
		material->Get(AI_MATKEY_COLOR_TRANSPARENT, transparentColor);
		cout << "Colors:";
		cout << " Diffuse:" << diffuseColor.r << " " << diffuseColor.g << " " << diffuseColor.b;
		cout << " Specular:" << specularColor.r << " " << specularColor.g << " " << specularColor.b;
		cout << " Ambient:" << ambientColor.r << " " << ambientColor.g << " " << ambientColor.b;
		cout << " Reflective:" << reflectiveColor.r << " " << reflectiveColor.g << " " << reflectiveColor.b;
		cout << " Emissive:" << emissiveColor.r << " " << emissiveColor.g << " " << emissiveColor.b;
		cout << " Transparent:" << transparentColor.r << " " << transparentColor.g << " " << transparentColor.b;
		cout << endl;
		// Texture types
		cout << "Texture type-count:";
		cout << " Diffuse" << "-" << material->GetTextureCount(aiTextureType_DIFFUSE);
		cout << " Specular" << "-" << material->GetTextureCount(aiTextureType_SPECULAR);
		cout << " Ambient" << "-" << material->GetTextureCount(aiTextureType_AMBIENT);
		cout << " Reflective" << "-" << material->GetTextureCount(aiTextureType_REFLECTION);
		cout << " Emissive" << "-" << material->GetTextureCount(aiTextureType_EMISSIVE);
		cout << " Opacity" << "-" << material->GetTextureCount(aiTextureType_OPACITY);
		cout << " Shininess" << "-" << material->GetTextureCount(aiTextureType_SHININESS);
		cout << " Normals" << "-" << material->GetTextureCount(aiTextureType_NORMALS);
		cout << " Reflective" << "-" << material->GetTextureCount(aiTextureType_REFLECTION);
		cout << endl;

		m_barbellMaterials[materialIndex] = Material(
			QVector3D(diffuseColor.r, diffuseColor.g, diffuseColor.b),
			QVector3D(specularColor.r, specularColor.g, specularColor.b),
			QVector3D(ambientColor.r, ambientColor.g, ambientColor.b)
		);
	}

	glGenVertexArrays(1, &m_barbellVAO);
	glBindVertexArray(m_barbellVAO);
	cout << "barbellVAO=" << m_barbellVAO << endl;

	GLuint barbellIBO;
	glGenBuffers(1, &barbellIBO);
	cout << "barbellIBO=" << barbellIBO << endl;
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, barbellIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0])*indices.size(), indices.constData(), GL_STATIC_DRAW);

	GLsizeiptr sizeInBytes = 
		sizeof(positions[0])*positions.size() + 
		sizeof(texCoords[0])*texCoords.size() + 
		sizeof(normals[0])*normals.size();
	cout << "Total size in bytes:" << sizeInBytes << endl;
	GLuint barbellVBO;
	glGenBuffers(1, &barbellVBO);
	cout << "barbellVBO=" << barbellVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, barbellVBO);
	glBufferData(
		GL_ARRAY_BUFFER,
		sizeInBytes,
		NULL,
		GL_STATIC_DRAW
	);
	
	GLsizeiptr offset = 0;
	cout << "Offset size in bytes:" << offset << endl;

	glBufferSubData(GL_ARRAY_BUFFER, offset, sizeof(positions[0]) * positions.size(), positions.constData());
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(offset));
	glEnableVertexAttribArray(0);

	offset += sizeof(positions[0]) * positions.size();
	cout << "Offset size in bytes:" << offset << endl;

	glBufferSubData(GL_ARRAY_BUFFER, offset, sizeof(texCoords[0]) * texCoords.size(), texCoords.constData());
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(offset));
	glEnableVertexAttribArray(1);

	offset += sizeof(texCoords[0]) * texCoords.size();
	cout << "Offset size in bytes:" << offset << endl;

	glBufferSubData(GL_ARRAY_BUFFER, offset, sizeof(normals[0]) * normals.size(), normals.constData());
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(offset));
	glEnableVertexAttribArray(2);

	glBindVertexArray(0);
}
void MainWidget::drawBarbell()
{
	glBindVertexArray(m_barbellVAO);

	for (uint i = 0; i < m_barbellMeshEntries.size(); i++) {
		const uint materialIndex = m_barbellMeshEntries[i].materialIndex;
		m_lighting->setUniformValue(m_diffuseLocation, 4*m_barbellMaterials[materialIndex].diffuseColor);
		m_lighting->setUniformValue(m_specularLocation, 1*m_barbellMaterials[materialIndex].specularColor);
		m_lighting->setUniformValue(m_ambientLocation, 0.1*m_barbellMaterials[materialIndex].ambientColor);
		glDrawElementsBaseVertex(
			GL_TRIANGLES,
			m_barbellMeshEntries[i].numIndices,
			GL_UNSIGNED_INT,
			(void*)(sizeof(uint) * m_barbellMeshEntries[i].baseIndex),
			m_barbellMeshEntries[i].baseVertex
		);
	}

	glBindVertexArray(0);
}
void MainWidget::loadBar()
{
	cout << "Loading bar model" << endl;

	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile(
		"models/barbell empty blendered.obj",
		aiProcess_Triangulate
	);

	if (!scene) {
		cout << "Could not import the bar model." << endl;
		return;
	}

	// Count vertices and indices and update mesh entries
	uint numVertices = 0;
	uint numIndices = 0;
	m_barMeshEntries.resize(scene->mNumMeshes);
	m_barMaterials.resize(scene->mNumMaterials);
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		m_barMeshEntries[i].materialIndex = scene->mMeshes[i]->mMaterialIndex;
		m_barMeshEntries[i].numIndices = scene->mMeshes[i]->mNumFaces * 3;
		m_barMeshEntries[i].baseVertex = numVertices;
		m_barMeshEntries[i].baseIndex = numIndices;
		numVertices += scene->mMeshes[i]->mNumVertices;
		numIndices += scene->mMeshes[i]->mNumFaces * 3;
		m_barMeshEntries[i].print();
	}
	cout << "Totals: NumVertices:" << numVertices << " NumIndices:" << numIndices << endl;

	// Reserve appropriate container space and push back vertex attributes
	QVector<QVector3D> positions;
	QVector<QVector2D> texCoords;
	QVector<QVector3D> normals;
	QVector<uint> indices;
	positions.reserve(numVertices);
	texCoords.reserve(numVertices);
	normals.reserve(numVertices);
	indices.reserve(numIndices);
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		// Push back vertices
		bool hasPositions = true;
		bool hasTexCoords = true;
		bool hasNormals = true;
		if (!scene->mMeshes[i]->HasPositions()) {
			cout << "Mesh " << i << " has no positions!" << endl;
			hasPositions = false;
		}
		if (!scene->mMeshes[i]->HasTextureCoords(0)) {
			cout << "Mesh " << i << " has no tex coords!" << endl;
			hasTexCoords = false;
		}
		if (!scene->mMeshes[i]->HasNormals()) {
			cout << "Mesh " << i << " has no normals!" << endl;
			hasNormals = false;
		}
		for (uint j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
			const aiVector3D* position = &(scene->mMeshes[i]->mVertices[j]);
			const aiVector3D* texCoord = &(scene->mMeshes[i]->mTextureCoords[0][j]);
			const aiVector3D* normal = &(scene->mMeshes[i]->mNormals[j]);
			if (hasPositions) positions.push_back(QVector3D(position->x, position->y, position->z));
			else positions.push_back(QVector3D(0.f, 0.f, 0.f));
			if (hasTexCoords) texCoords.push_back(QVector2D(texCoord->x, texCoord->y));
			else texCoords.push_back(QVector2D(0.f, 0.f));
			if (hasNormals) normals.push_back(QVector3D(normal->x, normal->y, normal->z));
			else normals.push_back(QVector3D(0.f, 1.f, 0.f));
		}

		// Push back indices
		for (uint j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
			const aiFace& face = scene->mMeshes[i]->mFaces[j];
			assert(face.mNumIndices == 3);
			indices.push_back(face.mIndices[0]);
			indices.push_back(face.mIndices[1]);
			indices.push_back(face.mIndices[2]);
		}
	}
	cout << endl;

	traverseSceneNodes(scene->mRootNode);

	cout << "Vector lengths:";
	cout << positions.length() << " ";
	cout << texCoords.length() << " ";
	cout << normals.length() << " ";
	cout << indices.length() << endl;

	cout << "Scene info:";
	cout << " Meshes:" << setw(3) << scene->mNumMeshes;
	cout << " Materials:" << setw(3) << scene->mNumMaterials;
	cout << " Textures:" << setw(3) << scene->mNumTextures;
	cout << " Lights:" << setw(3) << scene->mNumLights;
	cout << " Animations:" << setw(3) << scene->mNumAnimations;
	cout << endl;

	cout << "Meshes:" << endl;
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		cout << "Id:" << i;
		cout << " Name:" << setw(20) << scene->mMeshes[i]->mName.C_Str();
		cout << " Vertices:" << setw(6) << scene->mMeshes[i]->mNumVertices;
		cout << " Faces:" << setw(6) << scene->mMeshes[i]->mNumFaces;
		cout << " Bones:" << setw(6) << scene->mMeshes[i]->mNumBones;
		cout << " MaterialId:" << setw(6) << scene->mMeshes[i]->mMaterialIndex;
		cout << endl;
	}
	cout << endl;

	// Materials
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		// General properties
		const uint materialIndex = scene->mMeshes[i]->mMaterialIndex;
		const aiMaterial* material = scene->mMaterials[materialIndex];
		aiString name;
		int shadingMethod;
		int blendFunction;
		int twoSided;
		float opacity;
		float shininess;
		float shininessStrength;
		material->Get(AI_MATKEY_NAME, name);
		material->Get(AI_MATKEY_SHADING_MODEL, shadingMethod);
		material->Get(AI_MATKEY_BLEND_FUNC, blendFunction);
		material->Get(AI_MATKEY_TWOSIDED, twoSided);
		material->Get(AI_MATKEY_OPACITY, opacity);
		material->Get(AI_MATKEY_SHININESS, shininess);
		material->Get(AI_MATKEY_SHININESS_STRENGTH, shininessStrength);
		cout << "Material:" << i;
		cout << " Name:" << string(name.data);
		cout << " ShadingMethod:" << hex << shadingMethod << dec;
		cout << " BlendFunction:" << hex << blendFunction << dec;
		cout << " TwoSided:" << twoSided;
		cout << " Opacity:" << opacity;
		cout << " Shininess:" << shininess;
		cout << " ShininessStrength:" << shininessStrength;
		cout << endl;

		// Colors
		aiColor3D diffuseColor;
		aiColor3D specularColor;
		aiColor3D ambientColor;
		aiColor3D reflectiveColor;
		aiColor3D emissiveColor;
		aiColor3D transparentColor;
		material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor);
		material->Get(AI_MATKEY_COLOR_SPECULAR, specularColor);
		material->Get(AI_MATKEY_COLOR_AMBIENT, ambientColor);
		material->Get(AI_MATKEY_COLOR_REFLECTIVE, reflectiveColor);
		material->Get(AI_MATKEY_COLOR_EMISSIVE, emissiveColor);
		material->Get(AI_MATKEY_COLOR_TRANSPARENT, transparentColor);
		cout << "Colors:";
		cout << " Diffuse:" << diffuseColor.r << " " << diffuseColor.g << " " << diffuseColor.b;
		cout << " Specular:" << specularColor.r << " " << specularColor.g << " " << specularColor.b;
		cout << " Ambient:" << ambientColor.r << " " << ambientColor.g << " " << ambientColor.b;
		cout << " Reflective:" << reflectiveColor.r << " " << reflectiveColor.g << " " << reflectiveColor.b;
		cout << " Emissive:" << emissiveColor.r << " " << emissiveColor.g << " " << emissiveColor.b;
		cout << " Transparent:" << transparentColor.r << " " << transparentColor.g << " " << transparentColor.b;
		cout << endl;
		// Texture types
		cout << "Texture type-count:";
		cout << " Diffuse" << "-" << material->GetTextureCount(aiTextureType_DIFFUSE);
		cout << " Specular" << "-" << material->GetTextureCount(aiTextureType_SPECULAR);
		cout << " Ambient" << "-" << material->GetTextureCount(aiTextureType_AMBIENT);
		cout << " Reflective" << "-" << material->GetTextureCount(aiTextureType_REFLECTION);
		cout << " Emissive" << "-" << material->GetTextureCount(aiTextureType_EMISSIVE);
		cout << " Opacity" << "-" << material->GetTextureCount(aiTextureType_OPACITY);
		cout << " Shininess" << "-" << material->GetTextureCount(aiTextureType_SHININESS);
		cout << " Normals" << "-" << material->GetTextureCount(aiTextureType_NORMALS);
		cout << " Reflective" << "-" << material->GetTextureCount(aiTextureType_REFLECTION);
		cout << endl;

		m_barMaterials[materialIndex] = Material(
			QVector3D(diffuseColor.r, diffuseColor.g, diffuseColor.b),
			QVector3D(specularColor.r, specularColor.g, specularColor.b),
			QVector3D(ambientColor.r, ambientColor.g, ambientColor.b)
		);
	}

	glGenVertexArrays(1, &m_barVAO);
	glBindVertexArray(m_barVAO);
	cout << "barVAO=" << m_barVAO << endl;

	GLuint barIBO;
	glGenBuffers(1, &barIBO);
	cout << "barIBO=" << barIBO << endl;
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, barIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0])*indices.size(), indices.constData(), GL_STATIC_DRAW);

	GLsizeiptr sizeInBytes =
		sizeof(positions[0])*positions.size() +
		sizeof(texCoords[0])*texCoords.size() +
		sizeof(normals[0])*normals.size();
	cout << "Total size in bytes:" << sizeInBytes << endl;
	GLuint barVBO;
	glGenBuffers(1, &barVBO);
	cout << "barVBO=" << barVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, barVBO);
	glBufferData(
		GL_ARRAY_BUFFER,
		sizeInBytes,
		NULL,
		GL_STATIC_DRAW
	);

	GLsizeiptr offset = 0;
	cout << "Offset size in bytes:" << offset << endl;

	glBufferSubData(GL_ARRAY_BUFFER, offset, sizeof(positions[0]) * positions.size(), positions.constData());
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(offset));
	glEnableVertexAttribArray(0);

	offset += sizeof(positions[0]) * positions.size();
	cout << "Offset size in bytes:" << offset << endl;

	glBufferSubData(GL_ARRAY_BUFFER, offset, sizeof(texCoords[0]) * texCoords.size(), texCoords.constData());
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(offset));
	glEnableVertexAttribArray(1);

	offset += sizeof(texCoords[0]) * texCoords.size();
	cout << "Offset size in bytes:" << offset << endl;

	glBufferSubData(GL_ARRAY_BUFFER, offset, sizeof(normals[0]) * normals.size(), normals.constData());
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(offset));
	glEnableVertexAttribArray(2);

	glBindVertexArray(0);
}
void MainWidget::drawBar()
{
	glBindVertexArray(m_barVAO);

	for (uint i = 0; i < m_barMeshEntries.size(); i++) {
		const uint materialIndex = m_barMeshEntries[i].materialIndex;
		m_lighting->setUniformValue(m_diffuseLocation, 4 * m_barMaterials[materialIndex].diffuseColor);
		m_lighting->setUniformValue(m_specularLocation, 1 * m_barMaterials[materialIndex].specularColor);
		m_lighting->setUniformValue(m_ambientLocation, 0.1*m_barMaterials[materialIndex].ambientColor);
		glDrawElementsBaseVertex(
			GL_TRIANGLES,
			m_barMeshEntries[i].numIndices,
			GL_UNSIGNED_INT,
			(void*)(sizeof(uint) * m_barMeshEntries[i].baseIndex),
			m_barMeshEntries[i].baseVertex
		);
	}

	glBindVertexArray(0);
}
void MainWidget::loadPointer()
{
	cout << "Loading pointer model" << endl;

	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile(
		"models/arrow blendered.obj",
		aiProcess_Triangulate
	);

	if (!scene) {
		cout << "Could not import the pointer model." << endl;
		return;
	}

	// Count vertices and indices and update mesh entries
	uint numVertices = 0;
	uint numIndices = 0;
	m_pointerMeshEntries.resize(scene->mNumMeshes);
	m_pointerMeshEntries.resize(scene->mNumMaterials);
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		m_pointerMeshEntries[i].materialIndex = scene->mMeshes[i]->mMaterialIndex;
		m_pointerMeshEntries[i].numIndices = scene->mMeshes[i]->mNumFaces * 3;
		m_pointerMeshEntries[i].baseVertex = numVertices;
		m_pointerMeshEntries[i].baseIndex = numIndices;
		numVertices += scene->mMeshes[i]->mNumVertices;
		numIndices += scene->mMeshes[i]->mNumFaces * 3;
		m_pointerMeshEntries[i].print();
	}
	cout << "Totals: NumVertices:" << numVertices << " NumIndices:" << numIndices << endl;

	// Reserve appropriate container space and push back vertex attributes
	QVector<QVector3D> positions;
	QVector<QVector2D> texCoords;
	QVector<QVector3D> normals;
	QVector<uint> indices;
	positions.reserve(numVertices);
	texCoords.reserve(numVertices);
	normals.reserve(numVertices);
	indices.reserve(numIndices);
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		// Push back vertices
		bool hasPositions = true;
		bool hasTexCoords = true;
		bool hasNormals = true;
		if (!scene->mMeshes[i]->HasPositions()) {
			cout << "Mesh " << i << " has no positions!" << endl;
			hasPositions = false;
		}
		if (!scene->mMeshes[i]->HasTextureCoords(0)) {
			cout << "Mesh " << i << " has no tex coords!" << endl;
			hasTexCoords = false;
		}
		if (!scene->mMeshes[i]->HasNormals()) {
			cout << "Mesh " << i << " has no normals!" << endl;
			hasNormals = false;
		}
		for (uint j = 0; j < scene->mMeshes[i]->mNumVertices; j++) {
			const aiVector3D* position = &(scene->mMeshes[i]->mVertices[j]);
			const aiVector3D* texCoord = &(scene->mMeshes[i]->mTextureCoords[0][j]);
			const aiVector3D* normal = &(scene->mMeshes[i]->mNormals[j]);
			if (hasPositions) positions.push_back(QVector3D(position->x, position->y, position->z));
			else positions.push_back(QVector3D(0.f, 0.f, 0.f));
			if (hasTexCoords) texCoords.push_back(QVector2D(texCoord->x, texCoord->y));
			else texCoords.push_back(QVector2D(0.f, 0.f));
			if (hasNormals) normals.push_back(QVector3D(normal->x, normal->y, normal->z));
			else normals.push_back(QVector3D(0.f, 1.f, 0.f));
		}

		// Push back indices
		for (uint j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
			const aiFace& face = scene->mMeshes[i]->mFaces[j];
			assert(face.mNumIndices == 3);
			indices.push_back(face.mIndices[0]);
			indices.push_back(face.mIndices[1]);
			indices.push_back(face.mIndices[2]);
		}
	}
	cout << endl;

	traverseSceneNodes(scene->mRootNode);

	cout << "Vector lengths:";
	cout << positions.length() << " ";
	cout << texCoords.length() << " ";
	cout << normals.length() << " ";
	cout << indices.length() << endl;

	cout << "Scene info:";
	cout << " Meshes:" << setw(3) << scene->mNumMeshes;
	cout << " Materials:" << setw(3) << scene->mNumMaterials;
	cout << " Textures:" << setw(3) << scene->mNumTextures;
	cout << " Lights:" << setw(3) << scene->mNumLights;
	cout << " Animations:" << setw(3) << scene->mNumAnimations;
	cout << endl;

	cout << "Meshes:" << endl;
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		cout << "Id:" << i;
		cout << " Name:" << setw(20) << scene->mMeshes[i]->mName.C_Str();
		cout << " Vertices:" << setw(6) << scene->mMeshes[i]->mNumVertices;
		cout << " Faces:" << setw(6) << scene->mMeshes[i]->mNumFaces;
		cout << " Bones:" << setw(6) << scene->mMeshes[i]->mNumBones;
		cout << " MaterialId:" << setw(6) << scene->mMeshes[i]->mMaterialIndex;
		cout << endl;
	}
	cout << endl;

	// Materials
	for (uint i = 0; i < scene->mNumMeshes; i++) {
		// General properties
		const uint materialIndex = scene->mMeshes[i]->mMaterialIndex;
		const aiMaterial* material = scene->mMaterials[materialIndex];
		aiString name;
		int shadingMethod;
		int blendFunction;
		int twoSided;
		float opacity;
		float shininess;
		float shininessStrength;
		material->Get(AI_MATKEY_NAME, name);
		material->Get(AI_MATKEY_SHADING_MODEL, shadingMethod);
		material->Get(AI_MATKEY_BLEND_FUNC, blendFunction);
		material->Get(AI_MATKEY_TWOSIDED, twoSided);
		material->Get(AI_MATKEY_OPACITY, opacity);
		material->Get(AI_MATKEY_SHININESS, shininess);
		material->Get(AI_MATKEY_SHININESS_STRENGTH, shininessStrength);
		cout << "Material:" << i;
		cout << " Name:" << string(name.data);
		cout << " ShadingMethod:" << hex << shadingMethod << dec;
		cout << " BlendFunction:" << hex << blendFunction << dec;
		cout << " TwoSided:" << twoSided;
		cout << " Opacity:" << opacity;
		cout << " Shininess:" << shininess;
		cout << " ShininessStrength:" << shininessStrength;
		cout << endl;

		// Colors
		aiColor3D diffuseColor;
		aiColor3D specularColor;
		aiColor3D ambientColor;
		aiColor3D reflectiveColor;
		aiColor3D emissiveColor;
		aiColor3D transparentColor;
		material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor);
		material->Get(AI_MATKEY_COLOR_SPECULAR, specularColor);
		material->Get(AI_MATKEY_COLOR_AMBIENT, ambientColor);
		material->Get(AI_MATKEY_COLOR_REFLECTIVE, reflectiveColor);
		material->Get(AI_MATKEY_COLOR_EMISSIVE, emissiveColor);
		material->Get(AI_MATKEY_COLOR_TRANSPARENT, transparentColor);
		cout << "Colors:";
		cout << " Diffuse:" << diffuseColor.r << " " << diffuseColor.g << " " << diffuseColor.b;
		cout << " Specular:" << specularColor.r << " " << specularColor.g << " " << specularColor.b;
		cout << " Ambient:" << ambientColor.r << " " << ambientColor.g << " " << ambientColor.b;
		cout << " Reflective:" << reflectiveColor.r << " " << reflectiveColor.g << " " << reflectiveColor.b;
		cout << " Emissive:" << emissiveColor.r << " " << emissiveColor.g << " " << emissiveColor.b;
		cout << " Transparent:" << transparentColor.r << " " << transparentColor.g << " " << transparentColor.b;
		cout << endl;
		// Texture types
		cout << "Texture type-count:";
		cout << " Diffuse" << "-" << material->GetTextureCount(aiTextureType_DIFFUSE);
		cout << " Specular" << "-" << material->GetTextureCount(aiTextureType_SPECULAR);
		cout << " Ambient" << "-" << material->GetTextureCount(aiTextureType_AMBIENT);
		cout << " Reflective" << "-" << material->GetTextureCount(aiTextureType_REFLECTION);
		cout << " Emissive" << "-" << material->GetTextureCount(aiTextureType_EMISSIVE);
		cout << " Opacity" << "-" << material->GetTextureCount(aiTextureType_OPACITY);
		cout << " Shininess" << "-" << material->GetTextureCount(aiTextureType_SHININESS);
		cout << " Normals" << "-" << material->GetTextureCount(aiTextureType_NORMALS);
		cout << " Reflective" << "-" << material->GetTextureCount(aiTextureType_REFLECTION);
		cout << endl;

		m_barbellMaterials[materialIndex] = Material(
			QVector3D(diffuseColor.r, diffuseColor.g, diffuseColor.b),
			QVector3D(specularColor.r, specularColor.g, specularColor.b),
			QVector3D(ambientColor.r, ambientColor.g, ambientColor.b)
		);
	}

	glGenVertexArrays(1, &m_pointerVAO);
	glBindVertexArray(m_pointerVAO);
	cout << "pointerVAO=" << m_pointerVAO << endl;

	GLuint pointerIBO;
	glGenBuffers(1, &pointerIBO);
	cout << "pointerIBO=" << pointerIBO << endl;
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pointerIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0])*indices.size(), indices.constData(), GL_STATIC_DRAW);

	GLsizeiptr sizeInBytes = 
		sizeof(positions[0])*positions.size() + 
		sizeof(texCoords[0])*texCoords.size() + 
		sizeof(normals[0])*normals.size();
	cout << "Total size in bytes:" << sizeInBytes << endl;
	GLuint pointerVBO;
	glGenBuffers(1, &pointerVBO);
	cout << "pointerVBO=" << pointerVBO << endl;
	glBindBuffer(GL_ARRAY_BUFFER, pointerVBO);
	glBufferData(
		GL_ARRAY_BUFFER,
		sizeInBytes,
		NULL,
		GL_STATIC_DRAW
	);

	GLsizeiptr offset = 0;
	cout << "Offset size in bytes:" << offset << endl;

	glBufferSubData(GL_ARRAY_BUFFER, offset, sizeof(positions[0]) * positions.size(), positions.constData());
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(offset));
	glEnableVertexAttribArray(0);

	offset += sizeof(positions[0]) * positions.size();
	cout << "Offset size in bytes:" << offset << endl;

	glBufferSubData(GL_ARRAY_BUFFER, offset, sizeof(texCoords[0]) * texCoords.size(), texCoords.constData());
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(offset));
	glEnableVertexAttribArray(1);

	offset += sizeof(texCoords[0]) * texCoords.size();
	cout << "Offset size in bytes:" << offset << endl;

	glBufferSubData(GL_ARRAY_BUFFER, offset, sizeof(normals[0]) * normals.size(), normals.constData());
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(offset));
	glEnableVertexAttribArray(2);

	glBindVertexArray(0);
}
void MainWidget::drawPointer()
{
	glBindVertexArray(m_pointerVAO);

	for (uint i = 0; i < m_pointerMeshEntries.size(); i++) {
		const uint materialIndex = m_pointerMeshEntries[i].materialIndex;
		m_lighting->setUniformValue(m_diffuseLocation, 4 * m_pointerMaterials[materialIndex].diffuseColor);
		m_lighting->setUniformValue(m_specularLocation, 1 * m_pointerMaterials[materialIndex].specularColor);
		m_lighting->setUniformValue(m_ambientLocation,  m_pointerMaterials[materialIndex].ambientColor);
		glDrawElementsBaseVertex(
			GL_TRIANGLES,
			m_pointerMeshEntries[i].numIndices,
			GL_UNSIGNED_INT,
			(void*)(sizeof(uint) * m_pointerMeshEntries[i].baseIndex),
			m_pointerMeshEntries[i].baseVertex
		);
	}

	glBindVertexArray(0);
}
void MainWidget::traverseSceneNodes(aiNode * node)
{
	cout << "NodeName=" << node->mName.data;
	cout << " NumMeshes=" << node->mNumMeshes << endl;
	cout << "Children=";
	for (uint i = 0; i < node->mNumChildren; i++) {
		cout << node->mChildren[i]->mName.data << " ";
	}
	cout << endl;
	const QMatrix4x4& local = toQMatrix(node->mTransformation);
	cout << toString(local).toStdString() << endl;
	for (uint i = 0; i < node->mNumChildren; i++) {
		traverseSceneNodes(node->mChildren[i]);
	}
}
