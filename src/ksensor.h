#ifndef KSENSOR_H
#define KSENSOR_H

// Project
#include "math_3d.h"
#include "util.h"
#include "kskeleton.h"

// Kinect
#include <Kinect.h>

// Qt
class QFile;

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
	KSkeleton *kskeleton();
	bool getBodyData();
	void processBodyFrameData(IBody** bodies);
	void calculateFPS();
	void addMarkerData();
	bool createTRC();
	void resetRecordVars();
	bool m_isRecording = false;
private:

	KJoint m_Joints[JointType_Count];
	IKinectSensor *m_sensor = nullptr;
	IBodyFrameSource *m_source = nullptr;
	IBodyFrameReader *m_reader = nullptr;

	QFile *m_trcFile;
	QString m_markerData;
	uint m_numMarkers;

	QFile *m_log;

	// variables used to calculate non-discarded frame quantity and timestamps
	uint m_acceptedFrames = 0;
	double m_totalSeconds = 0;
	clock_t m_frameBegin = 0, m_frameEnd = 0;

	// variables used to calculate non-discarded fps
	clock_t m_currentTime = 0, m_previousTime = 0; 
	int m_fps = 0, m_frameCount = 0;

	KSkeleton m_skeleton;
	KJoint m_leftFootStance;
	KJoint m_rightFootStance;
};
#endif /* SENSOR_H */
