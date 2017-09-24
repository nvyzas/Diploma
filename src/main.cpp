#pragma once
#include "main_window.h"
#include <iostream>
#include <iomanip>
#include <QtWidgets/QOpenGLWidget>
#include <QtWidgets/QApplication>

using namespace std;

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);

	QSurfaceFormat format;
	format.setVersion(3, 3);
	format.setProfile(QSurfaceFormat::CompatibilityProfile);
	format.setRenderableType(QSurfaceFormat::OpenGL);
	format.setOptions(QSurfaceFormat::DebugContext | QSurfaceFormat::DeprecatedFunctions);
	QSurfaceFormat::setDefaultFormat(format);

	MainWindow mainWindow;
	mainWindow.show();

	return app.exec();
}
