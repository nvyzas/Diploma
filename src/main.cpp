// Project
#include "main_window.h"

// Qt
#include <QtCore\QDebug>
#include <QtGui\QSurfaceFormat>
#include <QtWidgets\QApplication>

int main(int argc, char* argv[])
{
	QSurfaceFormat format;
	format.setVersion(3, 3);
	format.setProfile(QSurfaceFormat::CompatibilityProfile);
	QSurfaceFormat::setDefaultFormat(format);
	qDebug() << "Requested format:" << format;

	QApplication app(argc, argv);
	MainWindow mainWindow;
	mainWindow.show();
	return app.exec();
}
