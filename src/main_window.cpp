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
	//ui->openGLWidget->update();
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
void MainWindow::keyPressEvent(QKeyEvent *event)
{
	cout << "MainWindow saw this keyboard event" << endl;
	int key = event->key();
	switch (key) {
	// do not put any other keys here!
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

	ui->checkBox_modelSkinning->setChecked(ui->openGLWidget->modelSkinning());

	ui->comboBox_activeBone->addItems(ui->openGLWidget->modelBoneList());
	loadActiveBoneInfo();

	ui->checkBox_axes->setChecked(ui->openGLWidget->renderAxes());
	ui->checkBox_model->setChecked(ui->openGLWidget->renderSkinnedMesh());
	ui->checkBox_skeleton->setChecked(ui->openGLWidget->renderKinectSkeleton());

	m_guiTimer = new QTimer(this);
	m_guiTimer->setTimerType(Qt::CoarseTimer);
	connect(m_guiTimer, SIGNAL(timeout()), this, SLOT(updateStatusBar()));
	
	m_guiTimer->setInterval(100);
	m_guiTimer->start();
}
void MainWindow::setActiveBoneAxes() {
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	ui->openGLWidget->setBoneAxes(boneName);
}
void MainWindow::setupConnections()
{
	cout << "MainWindow: Setting up connections" << endl;
	
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
	
	// drawing related
	// toggle draw axes
	connect(ui->checkBox_axes                   , SIGNAL(toggled(bool))               , ui->openGLWidget, SLOT(setRenderAxes(bool)));
	// toggle draw skineed mesh
	connect(ui->checkBox_model                  , SIGNAL(toggled(bool))               , ui->openGLWidget, SLOT(setRenderModel(bool)));
	// toggle draw kinect skeleton
	connect(ui->checkBox_skeleton               , SIGNAL(toggled(bool))               , ui->openGLWidget, SLOT(setRenderSkeleton(bool)));
	
	// Kinect
	connect(ui->horizontalSlider_progressPercent, SIGNAL(valueChanged(int))           , SLOT(setActiveKinectSkeletonFrame(int)));

}
void MainWindow::setActiveKinectSkeletonFrame(int progressPercent)
{
	uint pp = progressPercent; // #? progressPercent implicitly cast to uint?
	if (progressPercent > 100 || progressPercent < 0) {
		cout << "Progress percent out of bounds: " << progressPercent << endl;
	}
	else {
		ui->openGLWidget->setActiveFrame(progressPercent);
		ui->openGLWidget->update();
	}
}
void MainWindow::updateStatusBar()
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
