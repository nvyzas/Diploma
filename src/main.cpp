#pragma once
#include "main_widget.h"
#include <iostream>
#include <iomanip>
#include <QtWidgets/QOpenGLWidget>
#include <QtWidgets/QApplication>

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);

	QSurfaceFormat format;
	format.setVersion(3, 3);
	format.setProfile(QSurfaceFormat::CompatibilityProfile);
	format.setRenderableType(QSurfaceFormat::OpenGL);
	//format.setOption(QSurfaceFormat::DeprecatedFunctions);
	//format.setOption(QSurfaceFormat::DebugContext);
	format.setOptions(QSurfaceFormat::DebugContext | QSurfaceFormat::DeprecatedFunctions);
	QSurfaceFormat::setDefaultFormat(format);

	cout << "Surface format:";
	cout << " Version:" << format.version().first << "." << format.version().second;
	cout << " Profile:" << format.profile();
	cout << " Renderable type:" << format.renderableType();
	cout << " Option deprecated funcs:" << format.testOption(QSurfaceFormat::DeprecatedFunctions);
	cout << " Option debug context:" << format.testOption(QSurfaceFormat::DebugContext);
	cout << " Options:" << format.options() << endl;

	MainWidget widget;
	widget.show();

	return app.exec();
}
