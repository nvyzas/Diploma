// Own
#include "main_window.h"

// Project
#include "ui_main_window.h"

// Qt
#include "QtCore\QDir"

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
	ui->checkBox_boneVisible->setChecked(ui->openGLWidget->boneVisibility(ui->comboBox_activeBone->currentText()));
}
void MainWindow::setActiveBoneVisibility(bool state)
{
	ui->openGLWidget->setBoneVisibility(ui->comboBox_activeBone->currentText(), state);
}

void MainWindow::setupObjects()
{
	QDir modelDir("models/", "*.dae");
	for (const auto& fi : modelDir.entryInfoList()) {
		ui->comboBox_activeModel->addItem(fi.baseName());
	}
	//ui->openGLWidget->setModel(ui->comboBox_activeModel->currentText());

	ui->comboBox_activeBone->addItems(ui->openGLWidget->ModelBoneList());
	loadActiveBoneInfo();
	
	ui->checkBox_axes->setChecked(ui->openGLWidget->renderModel());
	ui->checkBox_model->setChecked(ui->openGLWidget->renderAxes());
	ui->checkBox_modelSkinning->setChecked(ui->openGLWidget->modelSkinning());
	
}
void MainWindow::setupConnections()
{
	connect(ui->comboBox_activeModel, SIGNAL(currentIndexChanged(QString)), ui->openGLWidget, SLOT(setModel(QString)));
	connect(ui->comboBox_activeBone, SIGNAL(currentIndexChanged(QString)), SLOT(loadActiveBoneInfo()));
	connect(ui->checkBox_boneVisible, SIGNAL(toggled(bool)), SLOT(setActiveBoneVisibility(bool)));
	connect(ui->spinBox_xRot, SIGNAL(valueChanged(int)), ui->horizontalSlider_xRot, SLOT(setValue(int)));
	connect(ui->horizontalSlider_xRot, SIGNAL(valueChanged(int)), ui->spinBox_xRot, SLOT(setValue(int)));
	connect(ui->spinBox_yRot, SIGNAL(valueChanged(int)), ui->horizontalSlider_yRot, SLOT(setValue(int)));
	connect(ui->horizontalSlider_yRot, SIGNAL(valueChanged(int)), ui->spinBox_yRot, SLOT(setValue(int)));
	connect(ui->spinBox_zRot, SIGNAL(valueChanged(int)), ui->horizontalSlider_zRot, SLOT(setValue(int)));
	connect(ui->horizontalSlider_zRot, SIGNAL(valueChanged(int)), ui->spinBox_zRot, SLOT(setValue(int)));
	connect(ui->checkBox_axes, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setRenderAxes(bool)));
	connect(ui->checkBox_model, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setRenderModel(bool)));
	connect(ui->checkBox_modelSkinning, SIGNAL(toggled(bool)),ui->openGLWidget, SLOT(setModelSkinning(bool)));
}
