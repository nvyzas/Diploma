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
#define NUM_LIMBS JointType_Count+10

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

// Represents the segment between 2 joints.
struct KLimb
{
	QString name;
	uint start;		// starting joint id
	uint end;		// ending joint id 
	uint sibling;	// sibling's ending joint id

	float lengthMin = FLT_MAX, lengthMax = FLT_MIN, lengthAverage = 0;
	int serialMin = -1, serialMax = -1;
	static float gapAverage;
	float siblingsLengthAverage = 0;
	bool needsAdjustment = false;

	KLimb()
		:
		name("aLimb"),
		start(INVALID_JOINT_ID),
		end(INVALID_JOINT_ID),
		sibling(INVALID_JOINT_ID)
	{
	}

	KLimb(QString _name, uint _start, uint _end, uint _sibling = INVALID_JOINT_ID)
		:
		name(_name),
		start(_start),
		end(_end),
		sibling(_sibling)
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
	int serial; // serial number #! maybe not needed
	double timestamp;
	array<KJoint, JointType_Count> joints;

	// Interpolates this frame between frames "next" and "previous" at time "interpolationTime"
	void interpolateJoints(const KFrame& previous, const KFrame& next, double interpolationTime)
	{
		double percentDistance = (interpolationTime - previous.timestamp) / (next.timestamp - previous.timestamp);
		
		if (interpolationTime == previous.timestamp) cout << setw(16) << "Coincidence-";
		else if (interpolationTime == next.timestamp) cout << setw(16) << "Coincidence+";
		else if (interpolationTime < previous.timestamp) cout << setw(16) << "Extrapolation-";
		else if (interpolationTime > next.timestamp) cout << setw(16) << "Extrapolation+";
		else cout << setw(16) << "Interpolation";
		
		cout << " " << setw(4) << previous.serial << " " << setw(4) << next.serial;
		cout << " " << setw(10) << previous.timestamp << " " << setw(10) << next.timestamp;
		cout << " @" << setw(10) << interpolationTime << "->" << setw(10) << percentDistance * 100 << " %";
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
	void addFrame(const Joint* joints, const JointOrientation* orientations, const double& time);
	void processFrames();
	void initJointHierarchy();
	void printInfo() const;
	void printJointHierarchy() const;
	void printActiveJoints() const;
	void printSequenceToLog(const QVector<KFrame>& seq);

	// files
	bool saveToTrc();
	void saveFrameSequences();
	void loadFrameSequences();

	void clearSequences();

	bool m_playbackOn = false;
	bool m_recordingOn = false;
	bool m_finalizingOn = false;

	uint activeFrame() const;

	array<KJoint, JointType_Count>& activeJoints();
	void setActiveJoints(uint frameIndex);

	bool m_playbackFiltered = true;
	QVector<KFrame>* m_activeSequence;
	void nextActiveSequence();

	double m_interpolationInterval;
	double m_recordingDuration;
	void calculateLimbLengths(const QVector<KFrame>& sequence);
	void printLimbLengths() const;
	const array<KLimb, NUM_LIMBS>& limbs() const;

private:
	array<KNode, JointType_Count> m_nodes; // these define the kinect skeleton hierarchy
	array<KJoint, JointType_Count> m_activeJoints;
	QVector<KFrame> m_rawSequence;
	QVector<KFrame> m_interpolatedSequence;
	QVector<KFrame> m_filteredSequence;
	QVector<KFrame> m_adjustedSequence;
	QFile m_sequencesLog;
	QTextStream m_forSequencesLog;

	uint m_activeFrame = 0;

	// trc file
	QFile *m_trcFile;
	
	uint m_activeJoint = JointType_SpineBase;

	// Savitzky-Golay filter

	// Coefficients (Symmetric), Cubic order, 1st element = 1/commonFactor #? should make them static?
	static const uint m_framesDelayed = 12;
	const array<float, 2*m_framesDelayed+2> m_sgCoefficients25 = { -253, -138, -33, 62, 147, 222, 287, 343, 387, 422, 447, 462, 467, 462, 447, 422, 387, 343, 278, 222, 147, 62, -33, -138, -253, 5175 };
	array<KFrame, m_framesDelayed> m_firstRawFrames;
	array<KFrame, m_framesDelayed> m_lastRawFrames;
	uint m_firstFrameIndex = 0;

	array<KLimb, NUM_LIMBS> m_limbs;
	void initLimbs();
	void interpolateFrames();
	void filterFrames();
	void adjustFrames();
	void adjustLimbLength(uint frameLocation, uint jointId, const QVector3D& direction, float factor); // recursively adjust joints



};

#endif
