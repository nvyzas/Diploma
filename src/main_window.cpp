// Own
#include "main_window.h"

// Project
#include "skinned_mesh.h"
#include "skinning_technique.h"
#include "ui_main_window.h"

// Qt
#include <QtCore\QDir>
#include <QtWidgets\QAction>
#include <QtWidgets\QMenu>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
	ui = new Ui::MainWindow;
	ui->setupUi(this);
	setupObjects();
	setupConnections();
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
	ui->openGLWidget->Transform(false);
	ui->openGLWidget->update();
	loadActiveBoneRotationX();
}
void MainWindow::setActiveBoneRotationY(int value)
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	ui->openGLWidget->skinnedMesh()->setBoneRotationY(boneName, value);
	ui->openGLWidget->Transform(false);
	ui->openGLWidget->update();
	loadActiveBoneRotationY();
}
void MainWindow::setActiveBoneRotationZ(int value)
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	ui->openGLWidget->skinnedMesh()->setBoneRotationZ(boneName, value);
	ui->openGLWidget->Transform(false);
	ui->openGLWidget->update();
	loadActiveBoneRotationZ();
}
void MainWindow::printActiveBoneTransforms() const
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	cout << string(ui->openGLWidget->skinnedMesh()->boneTransformInfo(boneName).toUtf8());
}
void MainWindow::printActiveBoneRotations() const
{
	const QString &boneName = ui->comboBox_activeBone->currentText();
	if (boneName.isEmpty()) return;
	float xRot = ui->openGLWidget->skinnedMesh()->boneRotationX(boneName);
	float yRot = ui->openGLWidget->skinnedMesh()->boneRotationY(boneName);
	float zRot = ui->openGLWidget->skinnedMesh()->boneRotationZ(boneName);
	cout << setw(20) << string(boneName.toUtf8()) << " " 
							   << setw(5) << xRot << " " 
							   << setw(5) << yRot << " " 
		                       << setw(5) << zRot << endl;
}

//const QString &MainWindow::activeBone() const
//{
//	const QString &boneName = ui->comboBox_activeBone->currentText();
//	if (boneName.isEmpty()) {
//		cout << "Active bone is empty" << endl;
//		return;
//	}
//	else {
//		return boneName;
//	}
//}

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
	for (const auto& fi : QDir("models/", "*.dae").entryInfoList()) {
		ui->comboBox_activeModel->addItem(fi.baseName());
	}
	ui->openGLWidget->setModelName(ui->comboBox_activeModel->currentText());
	ui->checkBox_modelSkinning->setChecked(ui->openGLWidget->modelSkinning());

	ui->comboBox_activeBone->addItems(ui->openGLWidget->modelBoneList());
	loadActiveBoneInfo();

	ui->checkBox_axes->setChecked(ui->openGLWidget->renderModel());
	ui->checkBox_model->setChecked(ui->openGLWidget->renderAxes());

	QMenu *menuInfo = new QMenu(ui->pushButton_info);
	menuInfo->addAction("Bone Transforms", this, SLOT(printActiveBoneTransforms()));
	menuInfo->addAction("Bone Rotations", this, SLOT(printActiveBoneRotations()));
	ui->pushButton_info->setMenu(menuInfo);
}
void MainWindow::setupConnections()
{
	// active model
	connect(ui->comboBox_activeModel, SIGNAL(currentIndexChanged(QString)), ui->openGLWidget, SLOT(setModelName(QString)));
	// skinning on
	connect(ui->checkBox_modelSkinning, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setModelSkinning(bool)));
	// Active bone related
	// combo box
	connect(ui->comboBox_activeBone, SIGNAL(currentIndexChanged(QString)), SLOT(loadActiveBoneInfo()));
	// check box
	connect(ui->checkBox_boneVisible, SIGNAL(clicked(bool)), SLOT(setActiveBoneVisible(bool)));
	connect(ui->checkBox_boneFocused, SIGNAL(clicked(bool)), SLOT(setActiveBoneFocused(bool)));
	// sliders
	connect(ui->horizontalSlider_xRot, SIGNAL(valueChanged(int)), SLOT(setActiveBoneRotationX(int)));
	connect(ui->horizontalSlider_yRot, SIGNAL(valueChanged(int)), SLOT(setActiveBoneRotationY(int)));
	connect(ui->horizontalSlider_zRot, SIGNAL(valueChanged(int)), SLOT(setActiveBoneRotationZ(int)));
	// spin boxes
	connect(ui->spinBox_xRot, SIGNAL(valueChanged(int)), SLOT(setActiveBoneRotationX(int)));
	connect(ui->spinBox_yRot, SIGNAL(valueChanged(int)), SLOT(setActiveBoneRotationY(int)));
	connect(ui->spinBox_zRot, SIGNAL(valueChanged(int)), SLOT(setActiveBoneRotationZ(int)));
	// render axes
	connect(ui->checkBox_axes, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setRenderAxes(bool)));
	// render model
	connect(ui->checkBox_model, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setRenderModel(bool)));
}