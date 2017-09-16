#include "main_window.h"

MainWindow::MainWindow()
{
}
MainWindow::MainWindow(QWidget *parent) : QOpenGLWidget(parent)
{
}
void MainWindow::initializeGL()
{
	/*
	GLenum res = glewInit();
	if (res != GLEW_OK) {
		printf("Error: '%s'\n", glewGetErrorString(res));
	}
	//*/
	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	initializeOpenGLFunctions();
	f->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
}
void MainWindow::paintGL()
{
	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	f->glClear(GL_COLOR_BUFFER_BIT);

}
void MainWindow::resizeGL(int w, int h)
{

}