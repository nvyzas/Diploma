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

#define INVALID_JOINT_ID 123
#define NUM_MARKERS JointType_Count

struct KJoint
{
	string name;
	Vector3f position;
	uint trackingState;
	QQuaternion orientation;
	uint parentId;
	vector<uint> childrenId;

	KJoint()
	{
	}

	KJoint(string _name, uint  _parent)
	{
		name = _name;
		parentId = _parent;
	}

	string getTrackingState() const
	{
		if (trackingState == TrackingState_Tracked) return "Tracked";
		else if (trackingState == TrackingState_Inferred) return "Inferred";
		else return "Not tracked";
	}

	void print() const
	{
		cout << setw(15) << name << "p: " << setw(25) << position.ToString() << getTrackingState() <<  endl;
	}

	void printOrientation() const
	{
		cout << setw(15) << name << "q: " << printQuaternion1(orientation) << printQuaternion2(orientation) << printQuaternion3(orientation) << endl;
	}

	
};
QDataStream& operator<<(QDataStream& out, const KJoint& jt)
{
	out << jt.position << jt.trackingState;
	return out;
}
QDataStream& operator>>(QDataStream& in, const KJoint& jt)
{
	in << jt.position << jt.trackingState;
	return in;
}

struct KFrame
{
	array<KJoint, NUM_MARKERS> joints;
	double timestamp;

	// Interpolates this frame between frames "next" and "previous" at time "interpolationTime"
	void interpolate(const KFrame& previous, const KFrame& next, double interpolationTime)
	{
		timestamp = interpolationTime;

		double percentDistance = (interpolationTime - previous.timestamp) / (next.timestamp - previous.timestamp);
		if (next.timestamp < previous.timestamp) cout << "next frame time < previous frame time !" << endl;
		if (interpolationTime > next.timestamp) cout << "interpolation time > next frame time " << interpolationTime << " > " << next.timestamp << endl;
		if (interpolationTime < previous.timestamp) cout << "interpolation time < previous frame time " << interpolationTime << " < " << previous.timestamp << endl;

		for (uint i = 0; i < NUM_MARKERS; i++) {
			joints[i].position.x = previous.joints[i].position.x + percentDistance * (next.joints[i].position.x - previous.joints[i].position.x);
			joints[i].position.y = previous.joints[i].position.y + percentDistance * (next.joints[i].position.y - previous.joints[i].position.y);
			joints[i].position.z = previous.joints[i].position.z + percentDistance * (next.joints[i].position.z - previous.joints[i].position.z);
		}
	}

	void operator<<
};

class KSkeleton: protected OPENGL_FUNCTIONS
{
public:
	KSkeleton();
	void addFrame(const Joint *joints, const JointOrientation *orientations, const double &time, bool record);
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

private:
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
#define NUM_POINTS 5
	bool m_enableFiltering = true;
	const double m_timeStep = 1. / 30.;
	// Cubic, Symmetric, 5 Points
	float m_sgCoefficients[NUM_POINTS] = {-3, 12, 17, 12, -3 };
	void sgFilter();
};

#endif
