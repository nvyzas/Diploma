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
	bool open();
	KSkeleton *skeleton();
	bool getBodyFrame();
	void processBodyFrameData(IBody** bodies, double timestamp);
	void calculateFPS();

	// Start/Stop frame recording in KSkeleton class
	void record();

private:
	KJoint m_Joints[JointType_Count];
	IKinectSensor *m_sensor = nullptr;
	IBodyFrameSource *m_source = nullptr;
	IBodyFrameReader *m_reader = nullptr;

	QFile m_captureLog;
	QTextStream m_forCaptureLog;

	// Frame counter and timestamps
	bool m_lastAttemptFailed = false;
	uint m_consecutiveFails = 0;

	double m_relativeTimeOffset;
	double m_lastRelativeTime;
	double m_relativeTime;

	// FPS counter
	double m_fps = 0.;

	KSkeleton m_skeleton;
};
#endif /* SENSOR_H */
