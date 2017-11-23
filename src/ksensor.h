#ifndef KSENSOR_H
#define KSENSOR_H

// Project
#include "math_3d.h"
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
	bool connect();
	KSkeleton *skeleton();
	bool getBodyData();
	void processBodyFrameData(IBody** bodies);
	void calculateFPS();

	// Recording
	void record();

	// Playback
	void setSkeletonActiveFrame(uint progressPercent);

private:
	KJoint m_Joints[JointType_Count];
	IKinectSensor *m_sensor = nullptr;
	IBodyFrameSource *m_source = nullptr;
	IBodyFrameReader *m_reader = nullptr;

	// Recording
	QFile m_captureLog;
	QTextStream m_forLog;
	bool m_isRecording = false;

	// Frame counter and timestamps
	uint m_acceptedFrames = 0;
	double m_totalSeconds = 0;
	clock_t m_frameBegin = 0;
	clock_t m_frameEnd = 0;

	// FPS counter
	clock_t m_currentTime = 0;
	clock_t m_previousTime = 0;
	int m_fps = 0;
	int m_frameCount = 0;

	KSkeleton m_skeleton;
	KJoint m_leftFootStance;
	KJoint m_rightFootStance;
};
#endif /* SENSOR_H */
