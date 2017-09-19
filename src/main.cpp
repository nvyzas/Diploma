#pragma once
#include "main_window.h"
#include <iostream>
#include <iomanip>
#include <QtWidgets/QOpenGLWidget>
#include <QtWidgets/QApplication>

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);

	QSurfaceFormat format;
	format.setVersion(4, 5);
	format.setProfile(QSurfaceFormat::CompatibilityProfile);
	format.setRenderableType(QSurfaceFormat::OpenGL);
	QSurfaceFormat::setDefaultFormat(format);

	cout << "Surface format:";
	cout << " Version:" << format.version().first << "." << format.version().second;
	cout << " Profile:" << format.profile();
	cout << " Renderable type:" << format.renderableType() << endl;

	MainWindow widget;
	widget.show();

	return app.exec();
}
