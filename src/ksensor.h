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
	void processBodyFrameData(IBody** bodies, double timestamp);
	void calculateFPS();

	// Start/Stop frame recording in KSkeleton class
	void record();

	KSkeleton *skeleton();
private:
	IKinectSensor *m_sensor = nullptr;
	IBodyFrameSource *m_source = nullptr;
	IBodyFrameReader *m_reader = nullptr;

	QFile m_captureLog;
	QTextStream m_forCaptureLog;

	// Frame counter and timestamps
	bool m_lastAttemptFailed = false;
	uint m_consecutiveFails = 0;

	// FPS counter
	double m_fps = 0.;

	KSkeleton m_skeleton;
};
#endif /* SENSOR_H */
