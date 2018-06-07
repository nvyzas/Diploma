#ifndef KSKELETON_H
#define KSKELETON_H

// Project
#include "util.h"

// Kinect
#include <Kinect.h>

// Qt
#include <QtCore/QFile>

// Standard C/C++
#include <array>
#include <iomanip>
#include <iostream>

#define INVALID_JOINT_ID -1
#define NUM_LIMBS 23

// Represents a node of the skeletal hierarchy
struct KNode
{
	QString name;
	uint parentId;
	vector<uint> childrenId;

	KNode()
		:
		name("aJoint"),
		parentId(INVALID_JOINT_ID)
	{
	}

	KNode(QString _name, uint  _parentId, bool _marked = true)
		:
		name(_name),
		parentId(_parentId)
	{
	}
};

// Represents the segment between 2 joints.
struct KLimb
{
	QString name;
	uint start;		// starting joint id
	uint end;		// ending joint id 
	uint sibling;	// sibling limb's id

	float minLength = FLT_MAX, maxLength = FLT_MIN, averageLength = 0, desiredLength = 0;
	int serialMin = -1, serialMax = -1;
	static float gapAverage;
	float siblingsLengthAverage = 0;

	KLimb()
		:
		name("aLimb"),
		start(INVALID_JOINT_ID),
		end(INVALID_JOINT_ID),
		sibling(INVALID_JOINT_ID)
	{
	}

	KLimb(uint _start, uint _end, uint _sibling = INVALID_JOINT_ID)
		:
		start(_start),
		end(_end),
		sibling(_sibling)
	{
	}
};

// Represents a joint's skeletal data
struct KJoint
{
	QVector3D position;
	uint trackingState;  
	QQuaternion orientation;

	KJoint()
		:
		position(0.f, 0.f, 0.f),
		orientation(1.f, 0.f, 0.f, 0.f),
		trackingState(TrackingState_NotTracked)
	{
	}

};

QDataStream& operator<<(QDataStream& out, const KJoint& joint);
QDataStream& operator>>(QDataStream& in, const KJoint& joint);

struct KFrame
{
	int serial;
	double timestamp;
	array<KJoint, JointType_Count> joints;

	// Interpolation between frames "previous" and "next" at time "interpolationTime"
	void interpolateJoints(const KFrame& previous, const KFrame& next, double interpolationTime)
	{
		double percentDistance = (interpolationTime - previous.timestamp) / (next.timestamp - previous.timestamp);
		
		if (interpolationTime == previous.timestamp) cout << setw(16) << "Coincidence-";
		else if (interpolationTime == next.timestamp) cout << setw(16) << "Coincidence+";
		else if (interpolationTime < previous.timestamp) cout << setw(16) << "Extrapolation-";
		else if (interpolationTime > next.timestamp) cout << setw(16) << "Extrapolation+";
		else cout << setw(16) << "Interpolation";
		
		cout << " " << setw(4) << previous.serial << " " << setw(4) << next.serial << " @" << setw(4) << serial;
		cout << " " << setw(10) << previous.timestamp << " " << setw(10) << next.timestamp << " @" << setw(10) << interpolationTime;
		cout << "->" << setw(10) << percentDistance * 100 << " %";
		cout << " Interval=" << next.timestamp - previous.timestamp;
		if (next.timestamp <= previous.timestamp) cout << " Error: next.timestamp <= previous.timestamp";
		cout << endl;
		
		for (uint i = 0; i < JointType_Count; i++) {
			joints[i].position.setX(previous.joints[i].position.x() + percentDistance * (next.joints[i].position.x() - previous.joints[i].position.x()));
			joints[i].position.setY(previous.joints[i].position.y() + percentDistance * (next.joints[i].position.y() - previous.joints[i].position.y()));
			joints[i].position.setZ(previous.joints[i].position.z() + percentDistance * (next.joints[i].position.z() - previous.joints[i].position.z()));
		}
	}

	QString toString() const
	{
		QString s;
		QTextStream qts(&s);
		qts << "Serial=" << qSetFieldWidth(4) << serial << " Timestamp=" << qSetFieldWidth(10) << timestamp << flush;
		return s;
	}
};

QDataStream& operator<<(QDataStream& out, const KFrame& frame);
QDataStream& operator>>(QDataStream& in, const KFrame& frame);

class KSkeleton
{
public:
	KSkeleton();
	~KSkeleton();
	KFrame addFrame(const Joint* joints, const JointOrientation* orientations, const double& time);

	void processRecording(bool trainerRecording);

	void printInfo() const;
	void printJointHierarchy() const;
	void printActiveJoints() const;
	void printLimbLengths() const;
	void printMotionsToLog();

	// files
	bool exportToTRC();


	void saveFrameSequences();
	void loadFrameSequences();

	bool m_isRecording = false;
	bool m_isFinalizing = false;

	array<KJoint, JointType_Count>& activeJoints();
	void setActiveJoints(array<KJoint, JointType_Count>& joints);
	
	const double m_interpolationInterval = 0.0333333; // calculated from trainer's motion

	const array<KLimb, NUM_LIMBS>& limbs() const;
	const array<KNode, JointType_Count>& nodes() const;
	void calculateLimbLengths(const QVector<KFrame>& sequence);

	void record(bool trainerRecording);

	// Motions
	QVector<KFrame> m_athleteRawMotion;
	QVector<KFrame> m_athleteInterpolatedMotion;
	QVector<KFrame> m_athleteFilteredMotion;
	QVector<KFrame> m_athleteAdjustedMotion;
	QVector<KFrame> m_athleteResizedMotion;

	QVector<KFrame> m_trainerRawMotion;
	QVector<KFrame> m_trainerInterpolatedMotion;
	QVector<KFrame> m_trainerFilteredMotion;
	QVector<KFrame> m_trainerAdjustedMotion;
	QVector<KFrame> m_trainerResizedMotion;

	bool m_trainerRecording;
	QVector<KFrame> m_recordedMotion;
	int m_bigMotionSize = 0;
private:
	array<KNode, JointType_Count> m_nodes; // these define the kinect skeleton hierarchy
	array<KJoint, JointType_Count> m_activeJoints;

	QFile m_sequenceLog;
	QTextStream m_sequenceLogData;

	// trc file
	QFile *m_trcFile;
	
	// Savitzky-Golay filter of cubic order with symmetric coefficients
	static const int m_framesDelayed = 12;
	const array<float, 2*m_framesDelayed+2> m_sgCoefficients = { -253, -138, -33, 62, 147, 222, 287, 343, 387, 422, 447, 462, 467, 462, 447, 422, 387, 343, 278, 222, 147, 62, -33, -138, -253, 1 / 5175.f };
	array<KFrame, m_framesDelayed> m_firstRawFrames;
	uint m_firstFrameIndex = 0;

	array<KLimb, NUM_LIMBS> m_limbs;
	void initJoints();
	void initLimbs();
	QVector<KFrame> interpolateMotion(const QVector<KFrame>& motion);
	QVector<KFrame> resizeMotion(const QVector<KFrame>& motion);
	QVector<KFrame> filterMotion(const QVector<KFrame>& motion);
	void cropMotion(QVector<KFrame>& motion);
	QVector<KFrame> adjustMotion(const QVector<KFrame>& motion);
	void adjustLimbLength(KFrame& kframe, uint jointId, const QVector3D& direction, float factor); // recursively adjust joints
	
	QVector3D m_leftFootOffset;
	QVector3D m_rightFootOffset;
	QVector3D m_leftFootPosition;
	QVector3D m_rightFootPosition;
};

#endif
