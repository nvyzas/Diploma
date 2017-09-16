#pragma once
#include "main_window.h"
#include <iostream>
#include <iomanip>
#include <QtWidgets/QOpenGLWidget>
#include <QtWidgets/QApplication>

int main(int argc, char* argv[]) {
	QApplication myApplication(argc, argv);
	//*
	QSurfaceFormat format;
	format.setDepthBufferSize(24);
	format.setStencilBufferSize(8);
	format.setVersion(3, 3);
	format.setProfile(QSurfaceFormat::CompatibilityProfile);
	QSurfaceFormat::setDefaultFormat(format);
	//*/
	MainWindow myMainWindow;
	myMainWindow.show();
	system("PAUSE");
	//SetupOGL();
	//Setup();
	
    // Main loop
    //Execute();
	
    return 0;
}
