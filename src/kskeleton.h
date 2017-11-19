#ifndef KSKELETON_H
#define KSKELETON_H

// Project
#include "math_3d.h"
#include "util.h"

// Kinect
#include <Kinect.h>

// Standard C/C++
#include <vector>
#include <array>

#define INVALID_JOINT_ID 123

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
	uint trackingState;

	KJoint() {}

	KJoint(std::string _name, uint  _parent, uint  _idOpposite, bool _toBeTracked)
	{
		name = _name;
		parent = _parent;
		idOpposite = _idOpposite;
		toBeTracked = _toBeTracked;
	}

	void print()
	{
		cout << setw(15) << name << "p: " << setw(25) << Position.ToString() << " q: " << printQuaternion1(Orientation) << printQuaternion2(Orientation) << printQuaternion3(Orientation) << endl;
	}
};

class KSkeleton: protected OPENGL_FUNCTIONS
{
public:
	KSkeleton();
	void addFrame(const Joint *joints, const JointOrientation *orientations, const double &timestamp, bool record);
	void drawActiveJoint();
	void drawSkeleton(uint id);
	const KJoint* getKJoints() const;
	void initJoints();
	bool initOGL();
	void nextJoint(int step);
	void printInfo() const;
	void printJointHierarchy() const;
	void printJoints() const;
	void printSequence() const;

private:
	KJoint m_joints[JointType_Count];
	vector<KJoint*> m_sequence;

	array<KJoint, JointType_Count> m_jts;
	vector<array<KJoint, JointType_Count>> m_seq;

	vector<double> m_timestamps;

	uint m_activeJoint = JointType_SpineBase;
};

#endif
