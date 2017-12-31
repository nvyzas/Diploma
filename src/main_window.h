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
	void setActiveBoneAxes();
	void setActiveBoneVisible(bool state);
	void setActiveBoneFocused(bool state);
	void setActiveBoneRotationX(int value);
	void setActiveBoneRotationY(int value);
	void setActiveBoneRotationZ(int value);
	void printActiveBoneTransforms() const;
	void printActiveBoneRotations() const;

	// Kinect
	void setActiveKinectSkeletonFrame(int progressPercent);
	void updateStatusBar();

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
	KSensor m_ksensor;	
	QTimer* m_guiTimer;

	// Status Bar
	QLabel *statusLabel;
	QLCDNumber *fps;
};

#endif /* MAIN_WINDOW_H */
