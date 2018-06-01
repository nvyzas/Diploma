#ifndef KSENSOR_H
#define KSENSOR_H

// Project
#include "util.h"
#include "kskeleton.h"

// Kinect
#include <Kinect.h>

// Qt
#include <QtCore/QFile>

// Standard C/C++
#include <string>
#include <vector>

class KSensor
{
public:
	KSensor();
	~KSensor();
	bool init();
	bool prepare();
	bool getBodyFrame();

	double calculateFPS();

	// Start/Stop frame recording in KSkeleton class
	void record();

	KSkeleton *skeleton();
private:
	IKinectSensor *m_sensor = nullptr;
	IBodyFrameSource *m_source = nullptr;
	IBodyFrameReader *m_reader = nullptr;

	QFile m_captureLog;
	QTextStream m_forCaptureLog;

	KSkeleton m_skeleton;
};
#endif /* SENSOR_H */
