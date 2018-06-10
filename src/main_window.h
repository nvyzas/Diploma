#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

// Project
namespace Ui {
	class MainWindow;
}
#include "ksensor.h"

// Qt
#include <QtWidgets\QMainWindow>
class QTimer;
class QLabel;
class QLCDNumber;

class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	MainWindow(QWidget *parent = Q_NULLPTR);
	~MainWindow();

private slots:
	void loadActiveBoneInfo();
	void setActiveBoneVisible(bool state);
	void setActiveBoneFocused(bool state);
	void setActiveBoneRotationX(int value);
	void setActiveBoneRotationY(int value);
	void setActiveBoneRotationZ(int value);
	void printActiveBoneTransforms() const;

	void togglePlayback();
	
	void updateInfo();
	void updateActiveFrameInfo();
protected:
	void keyPressEvent(QKeyEvent *event) override;

private:
	void loadActiveBoneRotationX();
	void loadActiveBoneRotationY();
	void loadActiveBoneRotationZ();
	void setupObjects();
	void setupConnections();
	Ui::MainWindow *ui;

	// Kinect
	KSensor* m_sensor;	
	QTimer* m_guiTimer;

	// Status Bar
	QLabel *statusLabel;
	QLCDNumber *fps;
	QLCDNumber *barAngle;
};

#endif /* MAIN_WINDOW_H */
