#ifndef KSKELETON_H
#define KSKELETON_H

// Project
#include "math_3d.h"
#include "util.h"

// Kinect
#include <Kinect.h>

// Qt
class QFile;

// Standard C/C++
#include <vector>
#include <array>

#define INVALID_JOINT_ID -1
#define NUM_MARKERS JointType_Count

struct KNode
{
	QString name;
	uint parentId;
	bool marked;
	vector<uint> childrenId;

	KNode()
		:name("aJoint"),
		parentId(INVALID_JOINT_ID),
		marked(true)
	{
	}

	KNode(QString _name, uint  _parentId, bool _marked = true)
		:name(_name),
		parentId(_parentId),
		marked(_marked)
	{
	}

};

struct KJoint
{
	uint id; // probably not needed
	Vector3f position;
	uint trackingState;  // #! uint defined in qglobal.h
	QQuaternion orientation;

	KJoint()
		:id(INVALID_JOINT_ID),
		position(0.f,0.f,0.f),
		orientation(1.f, 0.f, 0.f, 0.f),
		trackingState(0) // #? initial value = 0
	{
	}

	string getTrackingState() const
	{
		if (trackingState == TrackingState_Tracked) return "Tracked";
		else if (trackingState == TrackingState_Inferred) return "Inferred";
		else return "Not tracked";
	}

	void print() const
	{
		cout << "position: " << setw(25) << position.ToString() << getTrackingState() <<  endl;
	}

	void printOrientation() const
	{
		cout << "orientation: " << printQuaternion1(orientation) << printQuaternion2(orientation) << printQuaternion3(orientation) << endl;
	}
};

QDataStream& operator<<(QDataStream& out, const KJoint& jt);
QDataStream& operator>>(QDataStream& in, const KJoint& jt);

struct KFrame
{
	uint serial; // serial number
	double timestamp;
	array<KJoint, NUM_MARKERS> joints;

	// Interpolates this frame between frames "next" and "previous" at time "interpolationTime"
	void interpolate(const KFrame& previous, const KFrame& next, double interpolationTime)
	{
		cout << "Interpolating between frame " << previous.serial << " and " << next.serial << " at time " << interpolationTime << endl;
		double percentDistance = (interpolationTime - previous.timestamp) / (next.timestamp - previous.timestamp);
		/*if (next.timestamp < previous.timestamp) cout << "next frame time < previous frame time !" << endl;
		if (interpolationTime > next.timestamp) cout << "interpolation time > next frame time " << interpolationTime << " > " << next.timestamp << endl;
		if (interpolationTime < previous.timestamp) cout << "interpolation time < previous frame time " << interpolationTime << " < " << previous.timestamp << endl;*/

		for (uint i = 0; i < NUM_MARKERS; i++) {
			joints[i].position.x = previous.joints[i].position.x + percentDistance * (next.joints[i].position.x - previous.joints[i].position.x);
			joints[i].position.y = previous.joints[i].position.y + percentDistance * (next.joints[i].position.y - previous.joints[i].position.y);
			joints[i].position.z = previous.joints[i].position.z + percentDistance * (next.joints[i].position.z - previous.joints[i].position.z);
		}
		timestamp = interpolationTime;
		serial = next.serial;
	}
};

class KSkeleton: protected OPENGL_FUNCTIONS
{
public:
	KSkeleton();
	void addFrame(const Joint *joints, const JointOrientation *orientations, const double &time);
	void drawActiveJoint();
	void drawSkeleton(uint id);
	void initJoints();
	bool initOGL();
	void nextJoint(int step);
	void printInfo() const;
	void printJointHierarchy() const;
	void printJoints() const;
	void printSequence() const;
	bool createTRC();
	bool readTRC();
	void setActiveFrame(uint progressPercent);
	void saveToBinary() const;
	void loadFromBinary();
	void resetRecordVariables();

	bool m_recordingOn = false;
	bool m_filteringOn = true;
private:
	array<KNode, NUM_MARKERS> m_nodes; // these define the kinect skeleton hierarchy
	array<KJoint, NUM_MARKERS> m_joints;
	vector<KFrame> m_sequence;
	vector<KFrame> m_interpolatedSequence;
	vector<KFrame> m_filteredInterpolatedSequence;
	vector<KFrame> m_filteredSequence;

	uint m_activeFrame;

	// trc file
	QFile *m_trcFile;
	QString m_markerData;
	
	uint m_activeJoint = JointType_SpineBase;

	// Savitzky-Golay filter
	const double m_timeStep = 0.03;
	static const int m_numPoints = 5;
	// Cubic, Symmetric, 5 Points
	float m_sgCoefficients[m_numPoints] = {-3, 12, 17, 12, -3 };
	void sgFilter();
};

#endif
