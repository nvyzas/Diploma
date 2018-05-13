// Own
#include "kskeleton.h"

// Qt
#include <QtCore/QFile>

// Standard C/C++
#include <iomanip>

QDataStream& operator<<(QDataStream& out, const KJoint& joint)
{
	out << joint.position << joint.orientation << joint.trackingState;
	return out;
}
QDataStream& operator>>(QDataStream& in, KJoint& joint)
{
	in >> joint.position >> joint.orientation >> joint.trackingState;
	return in;
}
QDataStream& operator<<(QDataStream& out, const KFrame& frame)
{
	out << frame.serial << frame.timestamp;
	for (uint i = 0; i < JointType_Count; i++) {
		out << frame.joints[i];
	}
	return out;
}
QDataStream& operator>>(QDataStream& in, KFrame& frame)
{
	in >> frame.serial >> frame.timestamp;
	for (uint i = 0; i < JointType_Count; i++) {
		in >> frame.joints[i];
	}
	return in;
}

KSkeleton::KSkeleton() 
{
	cout << "KSkeleton constructor start." << endl;
	
	m_sequencesLog.setFileName("sequences_log.txt");
	if (!m_sequencesLog.open(QIODevice::WriteOnly | QIODevice::Text)) {
		cout << "Could not open sequences log file." << endl;
		return;
	}
	m_forSequencesLog.setFieldAlignment(QTextStream::AlignLeft);
	m_forSequencesLog.setRealNumberPrecision(6);
	m_forSequencesLog.setDevice(&m_sequencesLog);

	initJoints();
	loadFrameSequences();
	initLimbs();
	calculateLimbLengths(m_adjustedSequence);
	printLimbLengths();
	m_activeSequence = &m_rawSequence;
	m_interpolationInterval = m_activeSequence->at(1).timestamp - m_activeSequence->at(0).timestamp;
	cout << "Frame interval in active sequence: " << m_interpolationInterval << endl;

	cout << "KSkeleton constructor end.\n" << endl;
}
KSkeleton::~KSkeleton()
{
	m_sequencesLog.close();
}
void KSkeleton::addFrame(const Joint* joints, const JointOrientation* orientations, const double& time)
{
	static uint addedFrames = 0;
	addedFrames++;

	for (uint i = 0; i < JointType_Count; i++) {
		const Joint& jt = joints[i];
		const JointOrientation& or = orientations[i];
		m_activeJoints[i].position = QVector3D(jt.Position.X, jt.Position.Y, jt.Position.Z);
		m_activeJoints[i].orientation = QQuaternion(or.Orientation.w, or.Orientation.x, or.Orientation.y, or.Orientation.z);
		m_activeJoints[i].trackingState = jt.TrackingState;
	}

	KFrame kframe;
	kframe.timestamp = time;
	kframe.joints = m_activeJoints;
	kframe.serial = addedFrames; // serial = size - 1

	if (m_recordingOn) {
		m_rawSequence.push_back(kframe);
	} 
	else if (m_finalizingOn) {
		static uint counter = 0;
		if (counter < m_framesDelayed) {
			counter++;
			uint index = (m_firstFrameIndex - counter + m_framesDelayed) % m_framesDelayed;

			m_rawSequence.push_back(kframe);
			cout << "counter=" << counter << " ";

			m_rawSequence.push_front(m_firstRawFrames[index]);
			cout << "index=" << index << " ";
		}
		else {
			cout << "Recording finished." << endl;
			m_finalizingOn = false;
			processFrames();
			counter = 0;
			addedFrames = 0;
		}
	}
	else {
		if (addedFrames <= m_framesDelayed) {
			m_firstRawFrames[addedFrames - 1] = kframe;
		}
		else {
			m_firstRawFrames[m_firstFrameIndex] = kframe;
			m_firstFrameIndex = addedFrames % m_framesDelayed;
		}
	}
}
void KSkeleton::initJoints()
{
	// Set parents
	// core
	m_nodes[JointType_SpineBase]     = KNode("SpineBase", INVALID_JOINT_ID);
	m_nodes[JointType_SpineMid]      = KNode("SpineMid", JointType_SpineBase);
	m_nodes[JointType_SpineShoulder] = KNode("SpineShoulder", JointType_SpineMid);
	m_nodes[JointType_Neck]          = KNode("Neck", JointType_SpineShoulder);
	m_nodes[JointType_Head]          = KNode("Head", JointType_Neck);
	// left side																				
	m_nodes[JointType_HipLeft]       = KNode("HipLeft", JointType_SpineBase);
	m_nodes[JointType_KneeLeft]      = KNode("KneeLeft", JointType_HipLeft);
	m_nodes[JointType_AnkleLeft]     = KNode("AnkleLeft", JointType_KneeLeft);
	m_nodes[JointType_FootLeft]      = KNode("FootLeft", JointType_AnkleLeft);
	m_nodes[JointType_ShoulderLeft]  = KNode("ShoulderLeft", JointType_SpineShoulder);
	m_nodes[JointType_ElbowLeft]     = KNode("ElbowLeft", JointType_ShoulderLeft);
	m_nodes[JointType_WristLeft]     = KNode("WristLeft", JointType_ElbowLeft);
	m_nodes[JointType_HandLeft]      = KNode("HandLeft", JointType_WristLeft);
	m_nodes[JointType_ThumbLeft]     = KNode("ThumbLeft", JointType_HandLeft);
	m_nodes[JointType_HandTipLeft]   = KNode("HandTipLeft", JointType_HandLeft);
	// right side																				
	m_nodes[JointType_HipRight]      = KNode("HipRight", JointType_SpineBase);
	m_nodes[JointType_KneeRight]     = KNode("KneeRight", JointType_HipRight);
	m_nodes[JointType_AnkleRight]    = KNode("AnkleRight", JointType_KneeRight);
	m_nodes[JointType_FootRight]     = KNode("FootRight", JointType_AnkleRight);
	m_nodes[JointType_ShoulderRight] = KNode("ShoulderRight", JointType_SpineShoulder);
	m_nodes[JointType_ElbowRight]    = KNode("ElbowRight", JointType_ShoulderRight);
	m_nodes[JointType_WristRight]    = KNode("WristRight", JointType_ElbowRight);
	m_nodes[JointType_HandRight]     = KNode("HandRight", JointType_WristRight);
	m_nodes[JointType_ThumbRight]    = KNode("ThumbRight", JointType_HandRight);
	m_nodes[JointType_HandTipRight]  = KNode("HandTipRight", JointType_HandRight);

	// Set children
	for (uint i = 0; i < JointType_Count; i++) {
		uint p = m_nodes[i].parentId;
		if (p != INVALID_JOINT_ID) {
			(m_nodes[p].childrenId).push_back(i);
		}
	}
}
// Frame sequence must be initialized before calling this
void KSkeleton::initLimbs()
{
	// Core
	m_limbs[0 ] = KLimb(JointType_SpineBase    , JointType_SpineShoulder    );
	// Legs 
	m_limbs[1 ] = KLimb(JointType_SpineBase    , JointType_HipLeft      ,  2);
	m_limbs[2 ] = KLimb(JointType_SpineBase    , JointType_HipRight     ,  1);
	m_limbs[3 ] = KLimb(JointType_HipLeft      , JointType_KneeLeft     ,  4);
	m_limbs[4 ] = KLimb(JointType_HipRight     , JointType_KneeRight    ,  3);
	m_limbs[5 ] = KLimb(JointType_KneeLeft     , JointType_AnkleLeft    ,  6);
	m_limbs[6 ] = KLimb(JointType_KneeRight    , JointType_AnkleRight   ,  5);
	m_limbs[7 ] = KLimb(JointType_AnkleLeft    , JointType_FootLeft     ,  8);
	m_limbs[8 ] = KLimb(JointType_AnkleRight   , JointType_FootRight    ,  7);
	// Arms																 			  
	m_limbs[9 ] = KLimb(JointType_SpineBase    , JointType_ShoulderLeft	    ); // helper limb, must be before limb 11
	m_limbs[10] = KLimb(JointType_SpineBase    , JointType_ShoulderRight    ); // helper limb, must be before limb 12
	m_limbs[11] = KLimb(JointType_SpineShoulder, JointType_ShoulderLeft , 12);
	m_limbs[12] = KLimb(JointType_SpineShoulder, JointType_ShoulderRight, 11);
	m_limbs[13] = KLimb(JointType_ShoulderLeft , JointType_ElbowLeft    , 14);
	m_limbs[14] = KLimb(JointType_ShoulderRight, JointType_ElbowRight   , 13);
	m_limbs[15] = KLimb(JointType_ElbowLeft    , JointType_WristLeft    , 16);
	m_limbs[16] = KLimb(JointType_ElbowRight   , JointType_WristRight   , 15);
	m_limbs[17] = KLimb(JointType_WristLeft    , JointType_HandLeft     , 18);
	m_limbs[18] = KLimb(JointType_WristRight   , JointType_HandRight    , 17);
	m_limbs[19] = KLimb(JointType_WristLeft    , JointType_HandTipLeft  , 20);
	m_limbs[20] = KLimb(JointType_WristRight   , JointType_HandTipRight , 19);
	m_limbs[21] = KLimb(JointType_HandLeft     , JointType_ThumbLeft    , 22);
	m_limbs[22] = KLimb(JointType_HandRight    , JointType_ThumbRight   , 21);
	
	calculateLimbLengths(m_filteredSequence);

	for (uint l = 0; l < m_limbs.size(); l++) {
		m_limbs[l].name = m_nodes[m_limbs[l].start].name + "->" + m_nodes[m_limbs[l].end].name;
		m_limbs[l].desiredLength = (
			m_limbs[l].sibling == INVALID_JOINT_ID ?
			m_limbs[l].averageLength :
			(m_limbs[l].averageLength + m_limbs[m_limbs[l].sibling].averageLength) / 2.f
			);
	}
}
void KSkeleton::interpolateFrames()
{
	if (m_rawSequence.empty()) {
		cout << "No recorded frames!" << endl;
		return;
	}
	if (!m_interpolatedSequence.empty()) {
		cout << "Interpolated frames vector is not empty!" << endl;
		return;
	}

	cout << "Interpolating recorded frames." << endl;
	m_recordingDuration = (m_rawSequence.end() - m_framesDelayed - 1)->timestamp - (m_rawSequence.begin() + m_framesDelayed)->timestamp;
	m_interpolationInterval = m_recordingDuration / (m_rawSequence.size() - 2 * m_framesDelayed - 1);
	cout << "Recording duration: " << m_recordingDuration << endl;
	cout << "Interpolation interval: " << m_interpolationInterval << endl;
	cout << "First frame index: " << m_firstFrameIndex << endl;
	static uint index = 1;
	for (uint i = 0; i < m_rawSequence.size(); i++) {
		KFrame interpolatedFrame;
		int interpolatedSerial = i - m_framesDelayed;
		double interpolatedTime = m_interpolationInterval * interpolatedSerial;
		double interpolationTime = m_rawSequence[m_framesDelayed].timestamp + interpolatedTime;
		interpolatedFrame.serial = interpolatedSerial;
		interpolatedFrame.timestamp = interpolatedTime;
		while (index < m_rawSequence.size() && m_rawSequence[index].timestamp <= interpolationTime) {
			index++;
		}
		if (index > m_rawSequence.size() - 2) index = m_rawSequence.size() - 2;
		interpolatedFrame.interpolateJoints(m_rawSequence[--index], m_rawSequence[index + 1], interpolationTime);
		m_interpolatedSequence.push_back(interpolatedFrame);
	}
	index = 0;
}
void KSkeleton::filterFrames()
{
	if (m_interpolatedSequence.empty()) {
		cout << "No interpolated frames!" << endl;
		return;
	}
	if (!m_filteredSequence.empty()) {
		cout << "Filtered frames vector is not empty!" << endl;
		return;
	}

	cout << "Filtering recorded frames." << endl;
	uint np = m_sgCoefficients25.size() - 1; // number of points used
	for (uint i = m_framesDelayed; i < m_interpolatedSequence.size() - m_framesDelayed; i++) {
		KFrame filteredFrame;
		for (uint j = 0; j < JointType_Count; j++) {
			filteredFrame.joints[j].position = QVector3D(0.f, 0.f, 0.f);
			for (uint k = 0; k < 2 * m_framesDelayed + 1; k++) {
				filteredFrame.joints[j].position += m_interpolatedSequence[i + k - np / 2].joints[j].position *	m_sgCoefficients25[k] / m_sgCoefficients25.back();
				//if (j == 0) cout << (i + k - np / 2 == i ? "F:" : "") << i + k - np / 2 << ",";
			}
		}
		//cout << endl;
		filteredFrame.serial = m_interpolatedSequence[i].serial;
		filteredFrame.timestamp = m_interpolatedSequence[i].timestamp;
		m_filteredSequence.push_back(filteredFrame);
	}

	// Remove first and last m_framesDelayed elements from raw and interpolated sequences
	cout << "Raw sequence size before crop:" << m_rawSequence.size() << endl;
	for (uint i = 0; i < m_framesDelayed; i++) {
		m_rawSequence.removeFirst();
		m_rawSequence.removeLast();
		m_interpolatedSequence.removeFirst();
		m_interpolatedSequence.removeLast();
	}
	cout << "Raw sequence size after crop:" << m_rawSequence.size() << endl;
	cout << "Filtered sequence size:" << m_filteredSequence.size() << endl;
}
void KSkeleton::adjustFrames()
{
	if (m_filteredSequence.empty()) {
		cout << "No filtered frames! Returning." << endl;
		return;
	}
	if (!m_adjustedSequence.empty()) {
		cout << "Adjusted frames vector is not empty!" << endl;
		//return;
	}

	cout << "Adjusting filtered frames" << endl;
	cout << "Before adjustments:" << endl;
	calculateLimbLengths(m_filteredSequence);
	printLimbLengths();

	m_adjustedSequence = m_filteredSequence;
	for (uint i = 0; i < m_adjustedSequence.size(); i++) {
		m_adjustedSequence[i].joints[JointType_AnkleLeft].position =
			m_adjustedSequence[0].joints[JointType_AnkleLeft].position;
		m_adjustedSequence[i].joints[JointType_AnkleRight].position =
			m_adjustedSequence[0].joints[JointType_AnkleRight].position;
		m_leftFootOffset = QVector3D();
		m_rightFootOffset = QVector3D();
		for (uint l = 0; l < m_limbs.size(); l++) {
			KLimb& limb = m_limbs[l];
			const QVector3D& startPosition = m_adjustedSequence[i].joints[limb.start].position;
			const QVector3D& endPosition = m_adjustedSequence[i].joints[limb.end].position;
			QVector3D direction = endPosition - startPosition;
			float currentLength = startPosition.distanceToPoint(endPosition);
			float desiredLength = limb.desiredLength;
			if (l == 9 || l == 10) {
				QVector3D spine =
					m_adjustedSequence[i].joints[JointType_SpineShoulder].position -
					m_adjustedSequence[i].joints[JointType_SpineBase].position;
				QVector3D shoulder = (l == 9) ?
					m_adjustedSequence[i].joints[JointType_ShoulderLeft].position -
					m_adjustedSequence[i].joints[JointType_SpineShoulder].position :
					m_adjustedSequence[i].joints[JointType_ShoulderRight].position -
					m_adjustedSequence[i].joints[JointType_SpineShoulder].position;
				float angle = acos(QVector3D::dotProduct(spine.normalized(), direction.normalized()));
				float angleB = PI - (PI / 2.f - angle);
				float angleA = acos(QVector3D::dotProduct(-shoulder.normalized(), -direction.normalized()));
				float a = shoulder.length() * sin(angleA) / sin(angleB);
				desiredLength = sqrt(spine.lengthSquared() + pow(a, 2.f));
				if (i == 0) {
					cout << "Angle=" << ToDegrees(angle);
					cout << " AngleB=" << ToDegrees(angleB);
					cout << " AngleA=" << ToDegrees(angleA);
					cout << " CustomDesiredLength: " << desiredLength << endl;
				}
			}
			float adjustmentFactor = desiredLength / currentLength;
			if (i == 0) {
				cout << "Limb=" << limb.name.toStdString();
				cout << " DesiredLength=" << limb.desiredLength;
				cout << " CurrentLength=" << currentLength;
				cout << " CurrentFactor=" << adjustmentFactor << endl;
			}
			adjustLimbLength(i, limb.end, direction, adjustmentFactor);
		}

		// Point thumbs towards opposite hand
		QVector3D handRightToLeft =
			m_adjustedSequence[i].joints[JointType_HandLeft].position -
			m_adjustedSequence[i].joints[JointType_HandRight].position;

		m_adjustedSequence[i].joints[JointType_ThumbLeft].position =
			m_adjustedSequence[i].joints[JointType_HandLeft].position -
			handRightToLeft.normalized()*m_limbs[21].desiredLength;
		m_adjustedSequence[i].joints[JointType_ThumbRight].position =
			m_adjustedSequence[i].joints[JointType_HandRight].position +
			handRightToLeft.normalized()*m_limbs[22].desiredLength;

		// Move all joints towards the ground
		if (i == 0) {
			cout << " LeftFootOffset=" << toStringCartesian(m_leftFootOffset).toStdString();
			cout << " RightFootOffset=" << toStringCartesian(m_rightFootOffset).toStdString();
			cout << "\n" << endl;
		}
		for (uint j = 0; j < JointType_Count; j++) {
			m_adjustedSequence[i].joints[j].position.setY(
				m_adjustedSequence[i].joints[j].position.y() -
				(m_leftFootOffset.y() + m_rightFootOffset.y()) / 2.f
			);
		}

		// immobilize feet
		m_leftFootPosition = m_adjustedSequence[0].joints[JointType_AnkleLeft].position;
		m_rightFootPosition = m_adjustedSequence[0].joints[JointType_AnkleRight].position;

		m_adjustedSequence[i].joints[JointType_AnkleLeft].position = m_leftFootPosition;
		m_adjustedSequence[i].joints[JointType_AnkleRight].position = m_rightFootPosition;

		//*
		m_adjustedSequence[i].joints[JointType_FootLeft].position.setX(m_leftFootPosition.x());
		m_adjustedSequence[i].joints[JointType_FootRight].position.setX(m_rightFootPosition.x());
		
		m_adjustedSequence[i].joints[JointType_FootLeft].position.setY(
			m_adjustedSequence[0].joints[JointType_AnkleLeft].position.y()
		);
		m_adjustedSequence[i].joints[JointType_FootRight].position.setY(
			m_adjustedSequence[0].joints[JointType_AnkleRight].position.y()
		);
		QVector3D leftFootDirection =
			m_adjustedSequence[0].joints[JointType_FootLeft].position -
			m_adjustedSequence[0].joints[JointType_AnkleLeft].position;
		QVector3D rightFootDirection =
			m_adjustedSequence[0].joints[JointType_FootRight].position -
			m_adjustedSequence[0].joints[JointType_AnkleRight].position;
		m_adjustedSequence[i].joints[JointType_FootLeft].position =
			m_adjustedSequence[i].joints[JointType_AnkleLeft].position +
			leftFootDirection.normalized() * m_limbs[7].desiredLength;
		m_adjustedSequence[i].joints[JointType_FootRight].position =
			m_adjustedSequence[i].joints[JointType_AnkleRight].position +
			rightFootDirection.normalized() * m_limbs[8].desiredLength;
		//*/
	}

	calculateLimbLengths(m_adjustedSequence);
	cout << "After adjustments: " << endl;
	printLimbLengths();
}
void KSkeleton::adjustLimbLength(uint frameIndex, uint jointId, const QVector3D& direction, float factor)
{
	QVector3D& end = m_adjustedSequence[frameIndex].joints[jointId].position;
	QVector3D deltaEnd = -(1 - factor) * direction;
	end = end + deltaEnd;
	if (jointId == JointType_FootLeft) m_leftFootOffset += deltaEnd;
	if (jointId == JointType_FootRight) m_rightFootOffset += deltaEnd;
	if (frameIndex == 0) cout << m_nodes[jointId].name.toStdString() << " ";
	for (uint i = 0; i < m_nodes[jointId].childrenId.size(); i++) {
		uint childId = m_nodes[jointId].childrenId[i];
		adjustLimbLength(frameIndex, childId, direction, factor);
	}
}
float KLimb::gapAverage = 0.f;
void KSkeleton::calculateLimbLengths(const QVector<KFrame>& sequence)
{
	if (m_limbs.empty()) {
		cout << "Limbs array is empty! Returning." << endl;
		return;
	}
	if (sequence.empty()) {
		cout << "Frame sequence is empty! Returning." << endl;
		return;
	}

	KLimb::gapAverage = 0.f;
	for (uint l = 0; l < m_limbs.size(); l++) {
		if (m_limbs[l].end == INVALID_JOINT_ID) continue;
		m_limbs[l].maxLength = FLT_MIN;
		m_limbs[l].minLength = FLT_MAX;
		m_limbs[l].averageLength = 0;
		for (uint i = 0; i < sequence.size(); i++) {
			const QVector3D& startPosition = sequence[i].joints[m_limbs[l].start].position;
			const QVector3D& endPosition = sequence[i].joints[m_limbs[l].end].position;
			float length = startPosition.distanceToPoint(endPosition);
			if (length > m_limbs[l].maxLength) {
				m_limbs[l].maxLength = length;
				m_limbs[l].serialMax = sequence[i].serial;
			}
			if (length < m_limbs[l].minLength) {
				m_limbs[l].minLength = length;
				m_limbs[l].serialMin = sequence[i].serial;
			}
			m_limbs[l].averageLength += length;
		}
		m_limbs[l].averageLength /= sequence.size();
		KLimb::gapAverage += m_limbs[l].maxLength - m_limbs[l].minLength;
	}
	KLimb::gapAverage /= (m_limbs.size() - 1);

	for (uint l = 0; l < m_limbs.size(); l++) {
		m_limbs[l].desiredLength = (
			m_limbs[l].sibling == INVALID_JOINT_ID ?
			m_limbs[l].averageLength :
			(m_limbs[l].averageLength + m_limbs[m_limbs[l].sibling].averageLength) / 2.f
			);
	}
}
const array<KNode, JointType_Count>& KSkeleton::nodes() const
{
	return m_nodes;
}
void KSkeleton::printLimbLengths() const
{
	cout << "Limb lengths: " << endl;
	for (uint l = 0; l < m_limbs.size(); l++) {
		if (m_limbs[l].end == INVALID_JOINT_ID) continue;
		cout << setw(40) << left << m_limbs[l].name.toStdString() << " ";
		cout << "Min=" << setw(10) << m_limbs[l].minLength << " (" << setw(5) << m_limbs[l].serialMin;
		cout << ") Max=" << setw(10) << m_limbs[l].maxLength << " (" << setw(5) << m_limbs[l].serialMax;
		cout << ") Avg=" << setw(10) << m_limbs[l].averageLength << " ";
		cout << " Des=" << setw(10) << m_limbs[l].desiredLength << " ";
		cout << "Gap=" << setw(10) << m_limbs[l].maxLength - m_limbs[l].minLength << endl;
	}
	cout << "GapAverage=" << KLimb::gapAverage << endl;
}
void KSkeleton::printInfo() const
{
	cout << endl;
	cout << "Joint hierarchy:" << endl;
	printJointHierarchy();
	cout << "Joint data:" << endl;
	printActiveJoints();
	cout << endl;
}
void KSkeleton::printJointHierarchy() const
{
	for (uint i = 0; i < JointType_Count; i++) {
		uint p = m_nodes[i].parentId;
		cout << "Joint " << setw(2) << left << i << ": " << setw(20) << left << m_nodes[i].name.toStdString();
		cout << " Parent: " << setw(20) << left << (p == INVALID_JOINT_ID ? "NONE" : m_nodes[p].name.toStdString());
		cout << " Children (" << m_nodes[i].childrenId.size() << "): ";
		for (uint j = 0; j < m_nodes[i].childrenId.size(); j++) {
			uint c = m_nodes[i].childrenId[j];
			cout << m_nodes[c].name.toStdString() << " ";
		}
		cout << endl;
	}
}
void KSkeleton::printActiveJoints() const
{
	for (uint i = 0; i < JointType_Count; i++) {
		const KJoint &jt = m_activeJoints[i];
		cout << setw(15) << m_nodes[i].name.toStdString() << ": v:";
		cout << setw(15) << toStringCartesian(m_activeJoints[i].position).toStdString() << " q:";
		cout << setw(15) << toString(m_activeJoints[i].orientation).toStdString() << " ";
		cout << setw(15) << toStringEulerAngles(m_activeJoints[i].orientation).toStdString() << " ";
		cout << setw(15) << toStringAxisAngle(m_activeJoints[i].orientation).toStdString() << " ";
		cout << m_activeJoints[i].getTrackingState().toStdString();
		cout << endl;
	}
	cout << endl;
}
void KSkeleton::nextActiveSequence()
{
	if (m_activeSequence == &m_rawSequence) {
		m_activeSequence = &m_interpolatedSequence;
		cout << "Active frame sequence: Interpolated" << endl;
	}
	else if (m_activeSequence == &m_interpolatedSequence) {
		m_activeSequence = &m_filteredSequence;
		cout << "Active frame sequence: Filtered" << endl;
	} 
	else if (m_activeSequence == &m_filteredSequence) {
		m_activeSequence = &m_adjustedSequence;
		cout << "Active frame sequence: Adjusted" << endl;
	}
	else if (m_activeSequence == &m_adjustedSequence) {
		m_activeSequence = &m_rawSequence;
		cout << "Active frame sequence: Raw" << endl;
	}
	else {
		cout << "Unknown active frame sequence." << endl;
	}
}
void KSkeleton::setActiveJoints(uint frameIndex)
{
	m_activeJoints = m_activeSequence->at(frameIndex).joints;
}
double KSkeleton::timestamp(uint frameIndex) const
{
	return m_activeSequence->at(frameIndex).timestamp;
}
array<KJoint, JointType_Count>& KSkeleton::activeJoints()
{
	return m_activeJoints;
}
// saves filtered frame sequence to trc
bool KSkeleton::saveToTrc()
{
	QString fileName("joint_positions.trc");
	QFile qf(fileName);
	if (!qf.open(QIODevice::WriteOnly | QIODevice::Text)) {
		cout << "Could not create " << fileName.toStdString() << endl;
		return false;
	}

	QVector<KFrame>& framesToWrite = m_adjustedSequence;

	QTextStream out(&qf);
	// Line 1
	out << "PathFileType\t";
	out << "4\t";
	out << "(X / Y / Z)\t";
	out << "joint_positions.trc\n";
	// Line 2
	out << "DataRate\t";
	out << "CameraRate\t";
	out << "NumFrames\t";
	out << "NumMarkers\t";
	out << "Units\t";
	out << "OrigDataRate\t";
	out << "OrigDataStartFrame\t";
	out << "OrigNumFrames\n";
	// Line 3
	out << "30\t";
	out << "30\t";
	out << framesToWrite.size() << "\t";
	out << (JointType_Count + 1) << "\t";
	out << "mm\t";
	out << "30\t";
	out << "1\t";
	out << framesToWrite.size() << "\n";
	// Line 4
	out << "Frame#\t";
	out << "Time";
	for (int i = 0; i < JointType_Count; i++) {
		out << "\t" << m_nodes[i].name << "\t\t";
	}
	out << "\t" << "HipsMid" << "\t\t";
	out << "\n";
	// Line 5
	out << "\t";
	for (int i = 0; i < JointType_Count; i++) {
		out << "\t" << "X" << (i + 1) << "\t" << "Y" << (i + 1) << "\t" << "Z" << (i + 1);
	}
	out << "\t" << "X" << (JointType_Count + 1) << "\t" << "Y" << (JointType_Count + 1) << "\t" << "Z" << (JointType_Count + 1);

	out << "\n";
	// Lines 6+
	out.setFieldWidth(12);
	for (uint i = 0; i < framesToWrite.size(); i++) {
		out << "\n" << i << "\t" << framesToWrite[i].timestamp;
		for (int j = 0; j < JointType_Count; j++) {
			out << "\t" << framesToWrite[i].joints[j].position.x()*1000.f;
			out << "\t" << framesToWrite[i].joints[j].position.y()*1000.f;
			out << "\t" << framesToWrite[i].joints[j].position.z()*1000.f;
		}
		out << "\t" << (framesToWrite[i].joints[JointType_HipLeft].position.x()*1000.f + framesToWrite[i].joints[JointType_HipRight].position.x()*1000.f) / 2.f;
		out << "\t" << (framesToWrite[i].joints[JointType_HipLeft].position.y()*1000.f + framesToWrite[i].joints[JointType_HipRight].position.y()*1000.f) / 2.f;
		out << "\t" << (framesToWrite[i].joints[JointType_HipLeft].position.z()*1000.f + framesToWrite[i].joints[JointType_HipRight].position.z()*1000.f) / 2.f;
	}
	out << flush;
	qf.close();

	cout << "Successfully created " << fileName.toStdString() << endl;
	return true;
}
void KSkeleton::saveFrameSequences()
{
	QFile qf("sequences.txt");
	if (!qf.open(QIODevice::WriteOnly)) {
		cout << "Cannot write to sequences.txt binary file." << endl;
		return;
	}
	else {
		cout << "Saving to sequences.txt binary file" << endl;
	}
	QDataStream out(&qf);
	//out.setVersion(QDataStream::Qt_5_9);
	out << m_rawSequence;
	out << m_interpolatedSequence;
	out << m_filteredSequence;
	out << m_adjustedSequence;
	qf.close();

	//*
	m_forSequencesLog << "Saved sequences:" << endl;
	m_forSequencesLog << "Raw: " << endl;
	printSequenceToLog(m_rawSequence);
	m_forSequencesLog << "Interpolated: " << endl;
	printSequenceToLog(m_interpolatedSequence);
	m_forSequencesLog << "Filtered: " << endl;
	printSequenceToLog(m_filteredSequence);
	m_forSequencesLog << "Adjusted: " << endl;
	printSequenceToLog(m_adjustedSequence);
	//*/
}
void KSkeleton::loadFrameSequences()
{
	QFile qf("sequences.txt");
	if (!qf.open(QIODevice::ReadOnly)) {
		cout << "Cannot read from sequences.txt binary file." << endl;
		return;
	}
	else {
		cout << "Loading from sequences.txt binary file." << endl;
	}
	QDataStream in(&qf);
	//in.setVersion(QDataStream::Qt_5_9);
	//clearSequences(); // #? Is this needed
	in >> m_rawSequence;
	in >> m_interpolatedSequence;
	in >> m_filteredSequence;
	in >> m_adjustedSequence;
	qf.close();

	//*
	m_forSequencesLog    << "Loaded sequences:" << endl;
	m_forSequencesLog    << "Raw: "             << endl;
	printSequenceToLog(m_rawSequence);
	m_forSequencesLog    << "Interpolated:      " << endl;
	printSequenceToLog(m_interpolatedSequence);
	m_forSequencesLog    << "Filtered:          " << endl;
	printSequenceToLog(m_filteredSequence);
	m_forSequencesLog    << "Adjusted:          " << endl;
	printSequenceToLog(m_adjustedSequence);
	//*/
}
void KSkeleton::printSequenceToLog(const QVector<KFrame>& seq)
{
	m_forSequencesLog << "Size=" << seq.size() << " Duration=" << seq.back().timestamp - seq.front().timestamp << endl;
	for (uint i = 0; i < seq.size(); i++) {
		m_forSequencesLog << qSetFieldWidth(4) << i << reset << ": " << seq[i].toString() << endl;
	}
}
void KSkeleton::clearSequences()
{
	// #todo: clear may be expensive so add if for each clear
	m_rawSequence.clear();
	m_interpolatedSequence.clear();
	m_filteredSequence.clear();
	m_adjustedSequence.clear();
}
const array<KLimb, NUM_LIMBS>& KSkeleton::limbs() const
{
	return m_limbs;
}
void KSkeleton::processFrames()
{
	interpolateFrames();
	filterFrames();
	adjustFrames();
}
