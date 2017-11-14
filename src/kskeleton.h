#ifndef KSKELETON_H
#define KSKELETON_H

// Project
#include "math_3d.h"
//#include "util.h"

// Standard C/C++
#include <vector>

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

	KJoint()
	{
	}
	KJoint(std::string _name, uint  _parent, uint  _idOpposite, bool _toBeTracked)
	{
		name = _name;
		parent = _parent;
		idOpposite = _idOpposite;
		toBeTracked = _toBeTracked;
	}

};

class KSkeleton
{
public:
	KSkeleton();
	void addFrame(const KJoint &data, const double &timestamp);
	void draw();

private:
	vector<KJoint> sequence;
};

#endif
