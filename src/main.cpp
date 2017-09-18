#pragma once
#include "main_window.h"
#include <iostream>
#include <iomanip>
#include <QtWidgets/QOpenGLWidget>
#include <QtWidgets/QApplication>

int main(int argc, char* argv[]) {
	QApplication myApplication(argc, argv); // 
	cout << "hup" << endl;
	//*
	QSurfaceFormat format;
	format.setVersion(4, 5);
	format.setProfile(QSurfaceFormat::CompatibilityProfile);
	format.setRenderableType(QSurfaceFormat::OpenGL);
	cout << "Surface format:";
	cout << " Version:" << format.version().first << "."  << format.version().second;
	cout << " Profile:" << format.profile();
	cout << " Renderable type:" << format.renderableType() << endl;
	QSurfaceFormat::setDefaultFormat(format);	
	//*/
	MainWindow myMainWindow;
	cout << "two" << endl;
	myMainWindow.show();
	system("PAUSE");
	
    return 0;
}
