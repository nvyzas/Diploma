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
	uint id; // #! maybe not needed
	QVector3D position;
	uint trackingState;  
	QQuaternion orientation;

	KJoint()
		:id(INVALID_JOINT_ID),
		position(0.f, 0.f, 0.f),
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

};

QDataStream& operator<<(QDataStream& out, const KJoint& joint);
QDataStream& operator>>(QDataStream& in, const KJoint& joint);

struct KFrame
{
	uint serial; // serial number #! maybe not needed
	double timestamp;
	array<KJoint, NUM_MARKERS> joints;

	// Interpolates this frame between frames "next" and "previous" at time "interpolationTime"
	void interpolate(const KFrame& previous, const KFrame& next, double interpolationTime)
	{
		if (next.timestamp < previous.timestamp) cout << " Error: next.timestamp < previous.timestamp" << endl;
		
		double percentDistance = (interpolationTime - previous.timestamp) / (next.timestamp - previous.timestamp);
		cout << (interpolationTime < next.timestamp ? " Interpolation " : " Extrapolation ");
		cout << previous.serial << "-" << next.serial << " " << previous.timestamp << "-" << next.timestamp;
		cout << " @" << interpolationTime <<"->" << percentDistance*100 << "% ";
		
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

class KSkeleton: protected QOpenGLFunctions_3_3_Core
{
public:
	KSkeleton();
	void addFrame(const Joint *joints, const JointOrientation *orientations, const double &time);
	void initJointHierarchy();
	bool initOGL();
	void printInfo() const;
	void printJointHierarchy() const;
	void printJoints() const;
	void printSequence() const;
	// files
	bool saveToTrc();
	void saveToBinary() const;
	void loadFromBinary();

	uint activeFrame() const;
	double timeStep() const;
	void setTimeStep(double timestep);
	void clearSequences();

	bool m_playOn = false;
	bool m_recordingOn = false;

	bool m_playbackInterpolated = true;
	bool m_playbackFiltered = true;
	void drawSkeleton();
	void printJointBufferData();

	void loadSkeletonData();
	uint sequenceSize();
	void setActiveJoints(uint frameIndex);
	array<KJoint, NUM_MARKERS>& joints();

private:
	array<KNode, NUM_MARKERS> m_nodes; // these define the kinect skeleton hierarchy
	array<KJoint, NUM_MARKERS> m_joints;
	QVector<KFrame> m_sequence;
	QVector<KFrame> m_interpolatedSequence;
	QVector<KFrame> m_filteredInterpolatedSequence;
	QVector<KFrame> m_filteredSequence;

	uint m_activeFrame = 0;

	// trc file
	QFile *m_trcFile;
	QString m_markerData;
	
	uint m_activeJoint = JointType_SpineBase;

	// Savitzky-Golay filter
	double m_timeStep;

	// Cubic, Symmetric, 1st element = 1/commonFactor
	const array<float, 6> m_sgCoefficients5 = { -3, 12, 17, 12, -3, 35 };
	const array<float, 8> m_sgCoefficients7 = { -2, 3, 6, 7, 6, 3, -2, 21 };
	const array<float, 10> m_sgCoefficients9 = { -21, 14, 39, 54, 59, 54, 39, 14, -21, 231 };
	const array<float, 26> m_sgCoefficients25 = { -253, -138, -33, 62, 147, 222, 287, 343, 387, 422, 447, 462, 467, 462, 447, 422, 387, 343, 278, 222, 147, 62, -33, -138, -253, 5175 };
	
	float m_jointBufferData[2 * 3 * JointType_Count]; // 2 attributes x 3 components x JointType_Count joints
	
	GLuint m_skeletonVAO;
	GLuint m_skeletonIBO;
	GLuint m_skeletonVBO;
};

#endif
