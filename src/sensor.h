#ifndef SENSOR_H
#define SENSOR_H

// Project
#include "math_3d.h"
#include "skeleton.h"
#include "util.h"

// Kinect
#include <Kinect.h>

// Qt
class QFile;

// Standard C/C++
#include <string>
#include <vector>

#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424
#define COLOR_WIDTH 1920
#define COLOR_HEIGHT 1080
#define INVALID_JOINT_ID 123

using namespace std;

struct KJoint
{
	string name;
	Vector3f Position;
	QQuaternion Orientation;
	uint id;
	uint idOpposite; // id of corresponding opposite side joint (eg. left->right)
	uint parent;
	vector<uint> children;
	bool toBeTracked;
	
	KJoint()
	{
	}
	KJoint(string _name, uint _parent, uint _idOpposite, bool _toBeTracked)
	{
		name = _name;
		parent = _parent;
		idOpposite = _idOpposite;
		toBeTracked = _toBeTracked;
	}

};

class KSensor : protected OPENGL_FUNCTIONS
{
public:
	KSensor();
	~KSensor();
	bool init();
	bool connect();
	bool update();
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

	Skeleton m_skeleton;
	KJoint m_leftFootStance;
	KJoint m_rightFootStance;
};
#endif /* SENSOR_H */
