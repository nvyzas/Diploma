// Own
#include "main_window.h"

// Project
#include "skinned_mesh.h"
#include "skinning_technique.h"
#include "ui_main_window.h"

// Qt
#include <QtCore\QDir>
#include <QtCore\QTimer>
#include <QtWidgets\QAction>
#include <QtWidgets\QMenu>
#include <QtWidgets\QLCDNumber>
#include <QKeyEvent>
#include <QDesktopWidget>

// Standard C/C++
#include <iomanip>

MainWindow::MainWindow(QWidget *parent)
	: 
	QMainWindow(parent)
{
	cout << "MainWindow class constructor start." << endl;
	
	ui = new Ui::MainWindow;
	ui->setupUi(this);
	m_sensor = ui->openGLWidget->ksensor();
	setupObjects();
	setupConnections();
	//resize(QDesktopWidget().availableGeometry(this).size()*0.5);
	
	cout << "MainWindow class constructor end.\n" << endl;
}
MainWindow::~MainWindow()
{
	delete ui;
}
void MainWindow::loadActiveBoneInfo()
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	ui->checkBox_boneVisible->setChecked(ui->openGLWidget->skinnedMesh()->boneVisibility(boneName));
	loadActiveBoneRotationX();
	loadActiveBoneRotationY();
	loadActiveBoneRotationZ();
	ui->openGLWidget->setActiveBone(boneName);
}
void MainWindow::setActiveBoneVisible(bool state)
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	uint boneId = ui->openGLWidget->skinnedMesh()->findBoneId(boneName);
	ui->openGLWidget->skinnedMesh()->setBoneVisibility(boneName, state);
	ui->openGLWidget->skinningTechnique()->enable();
	ui->openGLWidget->skinningTechnique()->setBoneVisibility(boneId, state);
	ui->openGLWidget->update();
}
void MainWindow::setActiveBoneFocused(bool state)
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	uint boneId = ui->openGLWidget->skinnedMesh()->findBoneId(boneName);
	ui->openGLWidget->skinningTechnique()->enable();
	for (uint i = 0; i < ui->openGLWidget->skinnedMesh()->numBones(); i++) {
		if (i != boneId) {
			ui->openGLWidget->skinnedMesh()->setBoneVisibility(i, !state);
			ui->openGLWidget->skinningTechnique()->setBoneVisibility(i, !state);
		}
	}
	ui->openGLWidget->update();
}
void MainWindow::setActiveBoneRotationX(int value)
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	ui->openGLWidget->skinnedMesh()->setBoneRotationX(boneName, value);
	ui->openGLWidget->update();
	loadActiveBoneRotationX();
	ui->openGLWidget->update();
}
void MainWindow::setActiveBoneRotationY(int value)
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	ui->openGLWidget->skinnedMesh()->setBoneRotationY(boneName, value);
	ui->openGLWidget->update();
	loadActiveBoneRotationY();
	ui->openGLWidget->update();
}
void MainWindow::setActiveBoneRotationZ(int value)
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	ui->openGLWidget->skinnedMesh()->setBoneRotationZ(boneName, value);
	ui->openGLWidget->update();
	loadActiveBoneRotationZ();
	ui->openGLWidget->update();
}
void MainWindow::printActiveBoneTransforms() const
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	cout << ui->openGLWidget->skinnedMesh()->boneTransformInfo(boneName).toStdString() << endl;
}
void MainWindow::togglePlayback()
{
	ui->openGLWidget->setIsPaused(!ui->openGLWidget->isPaused());
	ui->pushButton_play->setText(ui->openGLWidget->isPaused() ? "Play" : "Pause");
}
void MainWindow::keyPressEvent(QKeyEvent *event)
{
	cout << "MainWindow saw this keyboard event" << endl;
	int key = event->key();
	switch (key) {
	case Qt::Key_Escape:
		close(); // implicitly makes the application quit by generating close event
		break;
	default:
		cout << "MainWindow: This key does not do anything." << endl;
		break;
	}	
}
void MainWindow::loadActiveBoneRotationX()
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	float rotX = ui->openGLWidget->skinnedMesh()->boneRotationX(boneName);
	ui->horizontalSlider_xRot->blockSignals(true);
	ui->spinBox_xRot->blockSignals(true);
	ui->horizontalSlider_xRot->setValue(rotX);
	ui->spinBox_xRot->setValue(rotX);
	ui->horizontalSlider_xRot->blockSignals(false);
	ui->spinBox_xRot->blockSignals(false);
}
void MainWindow::loadActiveBoneRotationY()
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	float rotY = ui->openGLWidget->skinnedMesh()->boneRotationY(boneName);
	ui->horizontalSlider_yRot->blockSignals(true);
	ui->spinBox_yRot->blockSignals(true);
	ui->horizontalSlider_yRot->setValue(rotY);
	ui->spinBox_yRot->setValue(rotY);
	ui->horizontalSlider_yRot->blockSignals(false);
	ui->spinBox_yRot->blockSignals(false);
}
void MainWindow::loadActiveBoneRotationZ()
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	float rotZ = ui->openGLWidget->skinnedMesh()->boneRotationZ(boneName);
	ui->horizontalSlider_zRot->blockSignals(true);
	ui->spinBox_zRot->blockSignals(true);
	ui->horizontalSlider_zRot->setValue(rotZ);
	ui->spinBox_zRot->setValue(rotZ);
	ui->horizontalSlider_zRot->blockSignals(false);
	ui->spinBox_zRot->blockSignals(false);
}
void MainWindow::setupObjects()
{
	cout << "MainWindow: Setting up objects" << endl;

	// Main Status
	ui->radioButton_capture->setChecked(ui->openGLWidget->captureIsEnabled());
	ui->radioButton_playback->setChecked(!ui->openGLWidget->captureIsEnabled());
	ui->checkBox_athlete->setChecked(ui->openGLWidget->athleteEnabled());
	ui->checkBox_trainer->setChecked(ui->openGLWidget->trainerEnabled());
	ui->comboBox_motionType->addItems(ui->openGLWidget->motionTypeList());
	ui->comboBox_motionType->setCurrentIndex(ui->openGLWidget->activeMotionType());

	ui->checkBox_modelSkinning->setChecked(ui->openGLWidget->modelSkinning());
	ui->comboBox_activeBone->addItems(ui->openGLWidget->modelBoneList());
	loadActiveBoneInfo();

	ui->checkBox_axes->setChecked(ui->openGLWidget->axesDrawing());
	ui->checkBox_model->setChecked(ui->openGLWidget->skinnedMeshDrawing());
	ui->checkBox_skeleton->setChecked(ui->openGLWidget->kinectSkeletonDrawing());
	ui->checkBox_floor->setChecked(ui->openGLWidget->floorDrawing());
	ui->checkBox_barbell->setChecked(ui->openGLWidget->barbellDrawing());
	ui->checkBox_tips->setChecked(ui->openGLWidget->tipsDrawing());

	// Playback controls
	ui->pushButton_play->setText(ui->openGLWidget->isPaused() ? "Play" : "Pause");

	m_guiTimer = new QTimer(this);
	m_guiTimer->setTimerType(Qt::CoarseTimer);
	m_guiTimer->setInterval(100);
}
void MainWindow::setupConnections()
{
	cout << "MainWindow: Setting up connections" << endl;
	
	// Main Status
	// only need to connect one of the two radio buttons
	connect(ui->radioButton_capture, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(enableCaptureMode(bool)));
	connect(ui->checkBox_athlete, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(enableAthlete(bool)));
	connect(ui->checkBox_trainer, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(enableTrainer(bool)));
	connect(ui->comboBox_motionType, SIGNAL(currentIndexChanged(int)), ui->openGLWidget, SLOT(setActiveMotionType(int)));

	// Skinned mesh related
	// toggle skinning
	connect(ui->checkBox_modelSkinning          , SIGNAL(toggled(bool))               , ui->openGLWidget, SLOT(setModelSkinning(bool)));
	
	// Active bone related
	// bone selection
	connect(ui->comboBox_activeBone             , SIGNAL(currentIndexChanged(QString)), SLOT(loadActiveBoneInfo()));
	connect(ui->comboBox_activeBone             , SIGNAL(currentIndexChanged(QString)), ui->openGLWidget, SLOT(setActiveBone(QString)));
	// toggle visible
	connect(ui->checkBox_boneVisible            , SIGNAL(clicked(bool))               , SLOT(setActiveBoneVisible(bool)));
	// toggle focused
	connect(ui->checkBox_boneFocused            , SIGNAL(clicked(bool))               , SLOT(setActiveBoneFocused(bool)));
	// print transform info
	connect(ui->pushButton_info                 , SIGNAL(clicked())                   , SLOT(printActiveBoneTransforms()));
	// rotation control
	connect(ui->horizontalSlider_xRot           , SIGNAL(valueChanged(int))           , SLOT(setActiveBoneRotationX(int)));
	connect(ui->horizontalSlider_yRot           , SIGNAL(valueChanged(int))           , SLOT(setActiveBoneRotationY(int)));
	connect(ui->horizontalSlider_zRot           , SIGNAL(valueChanged(int))           , SLOT(setActiveBoneRotationZ(int)));
	connect(ui->spinBox_xRot                    , SIGNAL(valueChanged(int))           , SLOT(setActiveBoneRotationX(int)));
	connect(ui->spinBox_yRot                    , SIGNAL(valueChanged(int))           , SLOT(setActiveBoneRotationY(int)));
	connect(ui->spinBox_zRot                    , SIGNAL(valueChanged(int))           , SLOT(setActiveBoneRotationZ(int)));
	
	// render
	connect(ui->checkBox_axes, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setAxesDrawing(bool)));
	connect(ui->checkBox_model, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setSkinnedMeshDrawing(bool)));
	connect(ui->checkBox_skeleton, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setKinectSkeletonDrawing(bool)));
	connect(ui->checkBox_floor, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setFloorDrawing(bool)));
	connect(ui->checkBox_barbell, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setBarbellDrawing(bool)));
	connect(ui->checkBox_tips, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setTipsDrawing(bool)));

	// Play controls
	connect(ui->horizontalSlider_progressPercent, SIGNAL(valueChanged(int))           , ui->openGLWidget, SLOT(setActiveMotionProgress(int)));
	connect(ui->openGLWidget, SIGNAL(frameChanged(int)), ui->horizontalSlider_progressPercent, SLOT(setValue(int)));
	connect(ui->pushButton_play, SIGNAL(clicked()), this, SLOT(togglePlayback()));

	// gui timer
	connect(m_guiTimer, SIGNAL(timeout()), this, SLOT(updateInfo()));
	connect(m_guiTimer, SIGNAL(timeout()), this, SLOT(updateInfo()));

	m_guiTimer->start();
}
void MainWindow::updateInfo()
{
	ui->label_barAngle->setNum((double)ui->openGLWidget->m_barAngle);
	ui->label_fps->setNum((int)ui->openGLWidget->m_fpsCount);
	QString barSpeedX; 
	barSpeedX.setNum(ui->openGLWidget->m_barSpeed.x(), 'f', 3);
	QString barSpeedY;
	barSpeedY.setNum(ui->openGLWidget->m_barSpeed.y(), 'f', 3);
	QString barSpeedZ;
	barSpeedZ.setNum(ui->openGLWidget->m_barSpeed.z(), 'f', 3);

	ui->label_barSpeed->setText(barSpeedX+" "+barSpeedY+" "+barSpeedZ);

	ui->label_kneeAngle->setNum((double)ui->openGLWidget->m_kneeAngle);
	ui->label_time->setNum(ui->openGLWidget->m_activeFrameTimestamp);
}
