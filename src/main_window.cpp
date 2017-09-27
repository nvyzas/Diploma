#include "main_window.h"
#include "ui_main_window.h"
#include <QtCore\QStringList>
//#include <QtWidgets\QMenu>

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	setupObjects();
	setupConnections();
}
MainWindow::~MainWindow()
{
	delete ui;
}
void MainWindow::setupObjects()
{
	ui->checkBox_model->setChecked(ui->openGLWidget->renderAxes());
	ui->checkBox_axes->setChecked(ui->openGLWidget->renderModel());
	ui->comboBox_activeBone->addItems(ui->openGLWidget->ModelBoneList());
}
void MainWindow::setupConnections()
{
	connect(ui->horizontalSlider_xRot, SIGNAL(valueChanged(int)), ui->spinBox_xRot, SLOT(setValue(int)));
	connect(ui->horizontalSlider_yRot, SIGNAL(valueChanged(int)), ui->spinBox_yRot, SLOT(setValue(int)));
	connect(ui->horizontalSlider_zRot, SIGNAL(valueChanged(int)), ui->spinBox_zRot, SLOT(setValue(int)));
	connect(ui->spinBox_xRot, SIGNAL(valueChanged(int)), ui->horizontalSlider_xRot, SLOT(setValue(int)));
	connect(ui->spinBox_yRot, SIGNAL(valueChanged(int)), ui->horizontalSlider_yRot, SLOT(setValue(int)));
	connect(ui->spinBox_zRot, SIGNAL(valueChanged(int)), ui->horizontalSlider_zRot, SLOT(setValue(int)));
	connect(ui->checkBox_axes, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setRenderAxes(bool)));
	connect(ui->checkBox_model, SIGNAL(toggled(bool)), ui->openGLWidget, SLOT(setRenderModel(bool)));
	
}
void MainWindow::keyPressEvent(QKeyEvent *event)
{
	if (event->key() == Qt::Key_Left)
	{
		//ui->openGLWidget->keyPressEvent()
	}
}
