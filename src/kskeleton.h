#ifndef KSKELETON_H
#define KSKELETON_H

// Project
#include "math_3d.h"
#include "util.h"

// Kinect
#include <Kinect.h>

// Qt
//#include <QtCore\QVector>
class QFile;

// Standard C/C++
//#include <vector>
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
	QVector3D position;
	uint trackingState;  // #! uint defined in qglobal.h
	QQuaternion orientation;

	KJoint()
		:id(INVALID_JOINT_ID),
		position(0.f,0.f,0.f),
		orientation(1.f, 0.f, 0.f, 0.f),
		trackingState(0) // #? initial value = 0
	{
	}

	QString getTrackingState() const
	{
		if (trackingState == TrackingState_Tracked) return "Tracked";
		else if (trackingState == TrackingState_Inferred) return "Inferred";
		else return "Not tracked";
	}

	void print() const
	{
		qDebug() << "position: " << position << getTrackingState();
	}

	void printOrientation() const
	{
		qDebug() << "orientation: " << toString(orientation) << toStringEulerAngles(orientation) << toStringAxisAngle(orientation);
	}
};

QDataStream& operator<<(QDataStream& out, const KJoint& joint);
QDataStream& operator>>(QDataStream& in, const KJoint& joint);

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
			joints[i].position.setX(previous.joints[i].position.x() + percentDistance * (next.joints[i].position.x() - previous.joints[i].position.x()));
			joints[i].position.setY(previous.joints[i].position.y() + percentDistance * (next.joints[i].position.y() - previous.joints[i].position.y()));
			joints[i].position.setZ(previous.joints[i].position.z() + percentDistance * (next.joints[i].position.z() - previous.joints[i].position.z()));
		}
		timestamp = interpolationTime;
		serial = next.serial;
	}
};

QDataStream& operator<<(QDataStream& out, const KFrame& frame);
QDataStream& operator>>(QDataStream& in, const KFrame& frame);

class KSkeleton: protected OPENGL_FUNCTIONS
{
public:
	KSkeleton();
	void addFrame(const Joint *joints, const JointOrientation *orientations, const double &time);
	void drawActiveJoint();
	void drawSkeleton(uint id);
	void initJointHierarchy();
	bool initOGL();
	void nextJoint(int step);
	void printInfo() const;
	void printJointHierarchy() const;
	void printJoints() const;
	void printSequence() const;
	bool createTRC();
	bool readTRC();

	uint activeFrame() const;
	void setActiveFrame(uint index);
	void setActiveFrame(float progressPercent);
	void nextActiveFrame();
	double timeStep() const;
	void setTimestep(double timestep);
	void saveToBinary() const;
	void loadFromBinary();
	void resetRecordVariables();

	bool m_playOn = false;
	bool m_recordingOn = false;
	bool m_filteringOn = true;
private:
	array<KNode, NUM_MARKERS> m_nodes; // these define the kinect skeleton hierarchy
	array<KJoint, NUM_MARKERS> m_joints;
	QVector<KFrame> m_sequence;
	QVector<KFrame> m_interpolatedSequence;
	QVector<KFrame> m_filteredInterpolatedSequence;
	QVector<KFrame> m_filteredSequence;

	uint m_activeFrame;

	// trc file
	QFile *m_trcFile;
	QString m_markerData;
	
	uint m_activeJoint = JointType_SpineBase;

	// Savitzky-Golay filter
	double m_timeStep = 0.0333345;
	static const int m_numPoints = 5;
	// Cubic, Symmetric, 5 Points
	float m_sgCoefficients[m_numPoints] = {-3, 12, 17, 12, -3 };
	void sgFilter();
};

#endif
