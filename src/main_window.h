#pragma once
#include <QtWidgets\QOpenGLWidget>
#include <QtGui\QOpenGLFunctions>
#include <QtGui\QOpenGLFunctions_4_5_Compatibility>

class MainWindow : public QOpenGLWidget, QOpenGLFunctions
{
public:
	MainWindow::MainWindow();
	MainWindow::MainWindow(QWidget *parent);

	//void initialize() override;
	//void render() override;
	
protected:
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();

	QOpenGLContext *m_context;
private:
	GLuint m_posAttr;
	GLuint m_colAttr;
	GLuint m_matrixUniform;

	//QOpenGLShaderProgram *m_program;
	int m_frame;
};

