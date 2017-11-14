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

class KSensor : protected OPENGL_FUNCTIONS
{
public:
	KSensor();
	~KSensor();
	bool init();
	bool connect();
	bool getBodyData();
	void processBodyFrameData(INT64 timestamp, int bodyCount, IBody** bodies);
	void calculateFPS();
	void addMarkerData();
	bool createTRC();
	const KJoint* getKJoints() const;
	void SwapSides();
	void PrintInfo() const;
	void PrintJointHierarchy() const;
	void PrintJointData() const;
	void DrawActiveJoint();
	void DrawSkeleton(uint id);
	void resetRecordVars();
	void setFootStance();

	bool m_isRecording = false;
	bool m_InvertedSides;
	bool m_GotFrame = false;
private:
	//KSkeleton m_skeleton;
	void initJoints();
	void NextJoint(int step);

	KJoint m_Joints[JointType_Count];
	IKinectSensor *m_sensor = nullptr;
	IBodyFrameSource *m_source = nullptr;
	IBodyFrameReader *m_reader = nullptr;

	uint m_ActiveJoint = JointType_SpineBase;

	QFile *m_trcFile;
	QString m_markerData;
	uint m_numMarkers;

	// used to calculate frame timestamps
	uint m_totalFramesCount = 0;
	double m_totalTime = 0;
	clock_t m_frameBegin = 0, m_frameEnd = 0;

	// used to calculate calculate fps
	clock_t m_currentTime = 0, m_previousTime = 0; 
	int m_fps, m_frameCount;

	KSkeleton m_skeleton;
	KJoint m_leftFootStance;
	KJoint m_rightFootStance;
};
#endif /* SENSOR_H */
