// Own
#include "kskeleton.h"

// Qt
#include <QtCore/QFile>

// Standard C/C++
#include <iomanip>

QDataStream& operator<<(QDataStream& out, const KJoint& joint)
{
	out << joint.position << joint.trackingState;
	return out;
}
QDataStream& operator>>(QDataStream& in, KJoint& joint)
{
	in >> joint.position >> joint.trackingState;
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
	initJointHierarchy();
	initLimbs();
	loadFrameSequences();
	m_activeSequence = &m_adjustedFrames;
	m_interpolationInterval = m_interpolatedFrames[1].timestamp - m_interpolatedFrames[0].timestamp;
	cout << "KSkeleton constructor end." << endl;
}
void KSkeleton::addFrame(const Joint* joints, const JointOrientation* orientations, const double& time)
{
	static uint addedFrames = 0;
	addedFrames++;

	for (uint i = 0; i < JointType_Count; i++) {
		const Joint& jt = joints[i];
		const JointOrientation& or = orientations[i];
		m_activeJoints[i].position = QVector3D(jt.Position.X, jt.Position.Y, jt.Position.Z);
		m_activeJoints[i].trackingState = jt.TrackingState;
		m_activeJoints[i].orientation = QQuaternion(or.Orientation.w, or.Orientation.x, or.Orientation.y, or.Orientation.z);
	}

	KFrame kframe;
	kframe.timestamp = time;
	kframe.joints = m_activeJoints;
	kframe.serial = addedFrames; // serial = size - 1

	if (m_recordingOn) {
		m_rawFrames.push_back(kframe);
		kframe.print();
	} 
	else if (m_finalizingOn) {
		static uint counter = 0;
		if (counter < m_framesDelayed) {
			counter++;
			uint index = (m_firstFrameIndex - counter + m_framesDelayed) % m_framesDelayed;

			m_rawFrames.push_back(kframe);
			cout << "counter=" << counter << " ";
			kframe.print();

			m_rawFrames.push_front(m_firstRawFrames[index]);
			cout << "index=" << index << " ";
			m_firstRawFrames[index].print();
		}
		else {
			cout << "Recording finished." << endl;
			m_finalizingOn = false;
			processFrames();
			saveFrameSequences();
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
		//cout << "First frame index = " << m_firstFrameIndex << endl;
	}
}
void KSkeleton::interpolateFrames()
{
	if (m_rawFrames.empty()) {
		cout << "No recorded frames!" << endl;
		return;
	}
	if (!m_interpolatedFrames.empty()) {
		cout << "Interpolated frames vector is not empty!" << endl;
		return;
	}

	cout << "Interpolating recorded frames." << endl;
	m_recordingDuration = m_rawFrames.back().timestamp - m_rawFrames.front().timestamp;
	m_interpolationInterval = m_recordingDuration / (m_rawFrames.size() - 1);
	cout << "Recording duration: " << m_recordingDuration << endl;
	cout << "Interpolation interval: " << m_interpolationInterval << endl;
	cout << "First frame index: " << m_firstFrameIndex << endl;
	static uint index = 0;
	for (uint i = 0; i < m_rawFrames.size() - 1; i++) {
		KFrame interpolatedFrame;
		int interpolatedSerial = i - m_framesDelayed;
		double interpolatedTime = m_interpolationInterval * interpolatedSerial;
		double interpolationTime = m_rawFrames[m_framesDelayed].timestamp + interpolatedTime;
		interpolatedFrame.serial = interpolatedSerial;
		interpolatedFrame.timestamp = interpolatedTime;
		while (m_rawFrames[index].timestamp < interpolationTime) {
			index++;
		}
		uint firstIndex;
		if (index == 0) {
			firstIndex = 0;
		}
		else if (index < m_rawFrames.size()) {
			firstIndex = index - 1;
		}
		else {
			firstIndex = m_rawFrames.size() - 2;
		}
		//uint firstIndex = (index >= m_rawFrames.size() - 1 ? m_rawFrames.size() - 2 : index-1);
		interpolatedFrame.interpolateJoints(m_rawFrames[firstIndex], m_rawFrames[firstIndex+1], interpolationTime);
		m_interpolatedFrames.push_back(interpolatedFrame);
	}
	index = 0;
}
void KSkeleton::filterFrames()
{
	if (m_interpolatedFrames.empty()) {
		cout << "No interpolated frames!" << endl;
		return;
	}
	if (!m_filteredFrames.empty()) {
		cout << "Filtered frames vector not empty!" << endl;
		return;
	}
	
	cout << "Filtering recorded frames." << endl;
	uint np = m_sgCoefficients25.size() - 1; // number of points used
	for (uint i = m_framesDelayed; i < m_interpolatedFrames.size()- m_framesDelayed; i++) {
		KFrame filteredFrame;
		for (uint j = 0; j < JointType_Count; j++) {
			filteredFrame.joints[j].position = QVector3D(0.f, 0.f, 0.f);
			for (uint k = 0; k < 2 * m_framesDelayed + 1; k++) {
				filteredFrame.joints[j].position += m_interpolatedFrames[i + k - np / 2].joints[j].position *	m_sgCoefficients25[k] / m_sgCoefficients25.back();
				//if (j == 0) cout << (i + k - np / 2 == i ? "F:" : "") << i + k - np / 2 << ",";
			}
		}
		//cout << endl;
		filteredFrame.serial = m_interpolatedFrames[i].serial;
		filteredFrame.timestamp = m_interpolatedFrames[i].timestamp;
		m_filteredFrames.push_back(filteredFrame);
	}
}
void KSkeleton::initJointHierarchy()
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
void KSkeleton::printInfo() const
{
	cout << endl;
	cout << "Joint hierarchy:" << endl;
	printJointHierarchy();
	cout << "Joint data:" << endl;
	printJoints();
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
void KSkeleton::printJoints() const
{
	for (uint i = 0; i < JointType_Count; i++) {
		const KJoint &jt = m_activeJoints[i];
		cout << setw(15) << m_nodes[i].name.toStdString() << ": ";
		cout << setw(15) << m_activeJoints[i].position << " ";
		cout << m_activeJoints[i].getTrackingState().toStdString();
		cout << endl;
	}
	cout << endl;
}
uint KSkeleton::activeFrame() const
{
	return m_activeFrame;
}
void KSkeleton::setActiveJoints(uint frameIndex)
{
	m_activeJoints = m_activeSequence->at(frameIndex).joints;
}
void KSkeleton::nextActiveSequence()
{
	if (m_activeSequence == &m_interpolatedFrames) {
		m_activeSequence = &m_filteredFrames;
		cout << "Active frame sequence: Filtered" << endl;
	} else if (m_activeSequence == &m_filteredFrames) {
		m_activeSequence = &m_adjustedFrames;
		cout << "Active frame sequence: Adjusted" << endl;
	}
	else if (m_activeSequence == &m_adjustedFrames) {
		m_activeSequence = &m_interpolatedFrames;
		cout << "Active frame sequence: Interpolated" << endl;
	}
	else {
		cout << "Unknown active frame sequence." << endl;
	}
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

	QVector<KFrame>& framesToWrite = *m_activeSequence;

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
void KSkeleton::saveFrameSequences() const
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
	out << m_rawFrames;
	out << m_interpolatedFrames;
	out << m_filteredFrames;
	out << m_adjustedFrames;
	qf.close();

	//*
	cout << "Raw sequence: " << endl;
	printSequenceInfo(m_rawFrames);
	cout << "Interpolated sequence: " << endl;
	printSequenceInfo(m_interpolatedFrames);
	cout << "Filtered sequence: " << endl;
	printSequenceInfo(m_filteredFrames);
	cout << "Adjusted sequence: " << endl;
	printSequenceInfo(m_adjustedFrames);
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
	in >> m_rawFrames;
	in >> m_interpolatedFrames;
	in >> m_filteredFrames;
	in >> m_adjustedFrames;
	qf.close();

	//*
	cout << "Raw sequence: " << endl;
	printSequenceInfo(m_rawFrames);
	cout << "Interpolated sequence: " << endl;
	printSequenceInfo(m_interpolatedFrames);
	cout << "Filtered sequence: " << endl;
	printSequenceInfo(m_filteredFrames);
	cout << "Adjusted sequence: " << endl;
	printSequenceInfo(m_adjustedFrames);
	//*/
}
void KSkeleton::clearSequences()
{
	// #todo: clear may be expensive so add if for each clear
	m_rawFrames.clear();
	m_interpolatedFrames.clear();
	m_filteredFrames.clear();
	m_adjustedFrames.clear();
	m_activeFrame = 0;
}
void KSkeleton::printSequenceInfo(const QVector<KFrame>& seq) const
{
	cout << "Size=" << seq.size() << " Duration=" << seq.back().timestamp - seq.front().timestamp << endl;
	//for (uint i = 0; i < seq.size(); i++) seq[i].print();
}
void KSkeleton::initLimbs()
{
	//for (uint i = 0; i < JointType_Count; i++) {
	//	uint p = m_nodes[i].parentId;
	//	if (p != INVALID_JOINT_ID) {
	//		m_limbs.push_back(KLimb(m_nodes[p].name + "->" + m_nodes[i].name, p, i));
	//	}
	//}

	// Legs
	m_limbs[JointType_HipLeft] = KLimb("SpineBase->HipLeft", JointType_SpineBase, JointType_HipLeft, JointType_HipRight);
	m_limbs[JointType_HipRight] = KLimb("SpineBase->HipRight", JointType_SpineBase, JointType_HipRight, JointType_HipLeft);
	m_limbs[JointType_KneeLeft] = KLimb("HipLeft->KneeLeft", JointType_HipLeft, JointType_KneeLeft, JointType_KneeRight);
	m_limbs[JointType_KneeRight] = KLimb("HipRight->KneeRight", JointType_HipRight, JointType_KneeRight, JointType_KneeLeft);
	m_limbs[JointType_AnkleLeft] = KLimb("KneeLeft->AnkleLeft", JointType_KneeLeft, JointType_AnkleLeft, JointType_AnkleRight);
	m_limbs[JointType_AnkleRight] = KLimb("KneeRight->AnkleRight", JointType_KneeRight, JointType_AnkleRight, JointType_AnkleLeft);
	m_limbs[JointType_FootLeft] = KLimb("AnkleLeft->FootLeft", JointType_AnkleLeft, JointType_FootLeft, JointType_FootRight);
	m_limbs[JointType_FootRight] = KLimb("AnkleRight->FootRight", JointType_AnkleRight, JointType_FootRight, JointType_FootLeft);

	// Arms
	m_limbs[JointType_ShoulderRight] = KLimb("SpineShoulder->ShoulderRight", JointType_SpineShoulder, JointType_ShoulderRight, JointType_ShoulderLeft);
	m_limbs[JointType_ShoulderLeft] = KLimb("SpineShoulder->ShoulderLeft", JointType_SpineShoulder, JointType_ShoulderLeft, JointType_ShoulderRight);
	m_limbs[JointType_ElbowLeft] = KLimb("ShoulderLeft->ElbowLeft", JointType_ShoulderLeft, JointType_ElbowLeft, JointType_ElbowRight);
	m_limbs[JointType_ElbowRight] = KLimb("ShoulderRight->ElbowRight", JointType_ShoulderRight, JointType_ElbowRight, JointType_ElbowLeft);
	m_limbs[JointType_WristLeft] = KLimb("ElbowLeft->WristLeft", JointType_ElbowLeft, JointType_WristLeft, JointType_WristRight);
	m_limbs[JointType_WristRight] = KLimb("ElbowRight->WristRight", JointType_ElbowRight, JointType_WristRight, JointType_WristLeft);
	m_limbs[JointType_HandLeft] = KLimb("WristLeft->HandLeft", JointType_WristLeft, JointType_HandLeft, JointType_HandRight);
	m_limbs[JointType_HandRight] = KLimb("WristRight->HandRight", JointType_WristRight, JointType_HandRight, JointType_HandLeft);

	m_limbs[0] = KLimb("SpineBase->SpineShoulder", JointType_SpineBase, JointType_SpineShoulder);
	//m_limbs[JointType_Count + 1] = KLimb("ShoulderLeft->ShoulderRight", JointType_ShoulderLeft, JointType_ShoulderRight);
	m_limbs[JointType_Count + 2] = KLimb("WristRight->HandTipRight", JointType_WristRight, JointType_HandTipRight, JointType_Count + 3);
	m_limbs[JointType_Count + 3] = KLimb("WristLeft->HandTipLeft", JointType_WristLeft, JointType_HandTipLeft, JointType_Count + 2);

}

float KLimb::gapAverage = 0.f;
void KSkeleton::calculateLimbLengths(const QVector<KFrame>& sequence)
{
	if (m_limbs.empty()) {
		cout << "Limbs array is empty!" << endl;
		return;
	}
	if (sequence.empty()) {
		cout << "Frames sequence is empty!" << endl;
		return;
	}

	KLimb::gapAverage = 0.f;
	for (uint l = 0; l < m_limbs.size(); l++) {
		if (m_limbs[l].end == INVALID_JOINT_ID) continue;
		m_limbs[l].lengthMax = FLT_MIN;
		m_limbs[l].lengthMin = FLT_MAX;
		m_limbs[l].lengthAverage = 0;
		for (uint i = 0; i < sequence.size(); i++) {
			const QVector3D& start = sequence[i].joints[m_limbs[l].start].position;
			const QVector3D& end = sequence[i].joints[m_limbs[l].end].position;
			float length = start.distanceToPoint(end);
			if (length > m_limbs[l].lengthMax) {
				m_limbs[l].lengthMax = length;
				m_limbs[l].serialMax = sequence[i].serial;
			}
			if (length < m_limbs[l].lengthMin) {
				m_limbs[l].lengthMin = length;
				m_limbs[l].serialMin = sequence[i].serial;
			}
			m_limbs[l].lengthAverage += length;
		}
		m_limbs[l].lengthAverage /= sequence.size();
		KLimb::gapAverage += m_limbs[l].lengthMax - m_limbs[l].lengthMin;
	}
	KLimb::gapAverage /= (m_limbs.size()-1);
}
void KSkeleton::printLimbLengths() const
{
	cout << "Limb lengths: " << endl;
	for (uint l = 0; l < m_limbs.size(); l++) {
		if (m_limbs[l].end == INVALID_JOINT_ID) continue;
		cout << setw(40) << left << m_limbs[l].name.toStdString() << " ";
		cout << "Min=" << setw(10) << m_limbs[l].lengthMin << " (" << setw(5) << m_limbs[l].serialMin;
		cout << ") Max=" << setw(10) << m_limbs[l].lengthMax << " (" << setw(5) << m_limbs[l].serialMax;
		cout << ") Avg=" << setw(10) << m_limbs[l].lengthAverage << " ";
		cout << "Gap=" << setw(10) << m_limbs[l].lengthMax - m_limbs[l].lengthMin << endl;
	}
	cout << "GapAverage=" << KLimb::gapAverage << endl;
}
void KSkeleton::adjustFrames()
{
	if (m_filteredFrames.empty()) {
		cout << "No filtered frames!" << endl;
		return;
	}
	if (!m_adjustedFrames.empty()) {
		cout << "Frames already adjusted!" << endl;
		//return;
	}

	m_adjustedFrames = m_filteredFrames;
	for (uint l = 0; l < m_limbs.size(); l++) {
		if (m_limbs[l].end == INVALID_JOINT_ID) continue;
		calculateLimbLengths(m_adjustedFrames);
		printLimbLengths();
		for (uint i = 0; i < m_adjustedFrames.size(); i++) {
			KLimb& limb = m_limbs[l];
			QVector3D& start = m_adjustedFrames[i].joints[limb.start].position;
			QVector3D& end = m_adjustedFrames[i].joints[limb.end].position;
			QVector3D startToEnd = end - start;
			float length = start.distanceToPoint(end);
			float average = limb.lengthAverage;
			if (limb.sibling != INVALID_JOINT_ID) {
				if (limb.siblingsLengthAverage != 0) {
					average = limb.siblingsLengthAverage;
				}
				else {
					average = (limb.lengthAverage + m_limbs[limb.sibling].lengthAverage) / 2.f;
					m_limbs[limb.sibling].siblingsLengthAverage = average;
				}
			}
			float adjustmentFactor =  average / length;
			if (i == 0) {
				cout << "Limb=" << limb.name.toStdString();
				cout << " Average=" << average;
				cout << " CurrentLength=" << length;
				cout << " CurrentFactor=" << adjustmentFactor << endl;
			}
			adjustLimbLength(i, limb.end, startToEnd, adjustmentFactor);
			if (i == 0) cout << "\n" << endl;

			//if (limb.sibling != INVALID_JOINT_ID){
			//KLimb& siblingLimb = m_limbs[m_limbs[l].sibling];
			//QVector3D& siblingStart = m_adjustedFrames[i].joints[siblingLimb.start].position;
			//QVector3D& siblingEnd = m_adjustedFrames[i].joints[siblingLimb.end].position;
			//QVector3D siblingStartToEnd = siblingEnd - siblingStart;
			//adjustLimbLength(i, siblingLimb.end, siblingStartToEnd, adjustmentFactor);
			//if (i == 0) cout << "\n" << endl;
		}
	}

	calculateLimbLengths(m_adjustedFrames);
	cout << "After adjustments: " << endl;
	printLimbLengths();
}
void KSkeleton::adjustLimbLength(uint frameIndex, uint jointId, const QVector3D& direction, float factor)
{
	QVector3D& end = m_adjustedFrames[frameIndex].joints[jointId].position;
	m_adjustedFrames[frameIndex].joints[jointId].position = end - (1-factor) * direction;
	if (frameIndex == 0) {
		cout << m_nodes[jointId].name.toStdString() << " ";
	}
	for (uint i = 0; i < m_nodes[jointId].childrenId.size(); i++) {
		uint childId = m_nodes[jointId].childrenId[i];
		adjustLimbLength(frameIndex, childId, direction, factor);
	}
}
void KSkeleton::processFrames()
{
	interpolateFrames();
	filterFrames();
	adjustFrames();
}
