#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

// Project
namespace Ui {
	class MainWindow;
}
#include "sensor.h"

// Qt
#include <QtWidgets\QMainWindow>
class QTimer;

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
	void printActiveBoneRotations() const;
	void updateKinect();
	void recordKinectData();

protected:
	void keyPressEvent(QKeyEvent *event) override;

private:
	const QString &activeBone() const;
	void loadActiveBoneRotationX();
	void loadActiveBoneRotationY();
	void loadActiveBoneRotationZ();
	void setupObjects();
	void setupConnections();
	Ui::MainWindow *ui;

	// Kinect variables
	KSensor m_Sensor;	
	QTimer *m_timer;
	bool m_renderSkeleton = false;
	bool m_renderActiveJoint = false;
};

#endif /* MAIN_WINDOW_H */
