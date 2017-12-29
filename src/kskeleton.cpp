// Own
#include "kskeleton.h"

// Project
#include "math_3d.h"
#include "util.h"

// Qt
#include <QtCore/QFile>

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
	for (uint i = 0; i < NUM_MARKERS; i++) {
		out << frame.joints[i];
	}
	return out;
}
QDataStream& operator>>(QDataStream& in, KFrame& frame)
{
	in >> frame.serial >> frame.timestamp;
	for (uint i = 0; i < NUM_MARKERS; i++) {
		in >> frame.joints[i];
	}
	return in;
}

KSkeleton::KSkeleton()
{
	cout << "KSkeleton constructor start." << endl;
	initJointHierarchy();
	loadFromBinary();
	cout << "KSkeleton constructor end." << endl;
}

void KSkeleton::addFrame(const Joint *joints, const JointOrientation *orientations, const double &time)
{
	for (uint i = 0; i < JointType_Count; i++) {
		const Joint& jt = joints[i];
		const JointOrientation& or = orientations[i];
		m_joints[i].position = QVector3D(jt.Position.X, jt.Position.Y, jt.Position.Z);
		m_joints[i].trackingState = jt.TrackingState;
		m_joints[i].orientation = QQuaternion(or.Orientation.w, or.Orientation.x, or.Orientation.y, or.Orientation.z);
	}

	if (m_recordingOn) {
		KFrame kframe;
		kframe.timestamp = time;
		kframe.joints = m_joints;
		kframe.serial = m_sequence.size(); // serial = size - 1
		m_sequence.push_back(kframe);
		cout << "currentIndex=" << kframe.serial;

		// Interpolation
		double interpolationTime = m_timeStep * kframe.serial;
		if (interpolationTime == time) {
			cout << " interpolationTime = frameTime = " << time;
		}
		else if (interpolationTime > time) {
			cout << " Difference=" << interpolationTime - time;
			kframe.interpolate(m_sequence.end()[-2], m_sequence.back(), interpolationTime);
		}
		else {
			cout << " Difference=" << interpolationTime - time;
			kframe.interpolate(m_sequence.end()[-2], m_sequence.back(), interpolationTime);
		}
		m_interpolatedSequence.push_back(kframe);

		// Filtering
		uint np = m_sgCoefficients25.size() - 1; // number of points used
		if (kframe.serial < np - 1) {
			cout << " Not enough frames to start filtering." << endl;
		}
		else {
			uint fi = kframe.serial - np / 2; // index of frame to be filtered
			for (uint i = 0; i < NUM_MARKERS; i++) {
				kframe.joints[i].position = QVector3D(0.f, 0.f, 0.f);
				for (uint j = 0; j < np; j++) {
					kframe.joints[i].position += m_interpolatedSequence[fi + j - np / 2].joints[i].position *	m_sgCoefficients25[j] / m_sgCoefficients25.back();
					if (i == 0) cout << (fi + j - np / 2 == fi ? "F:" : "") << fi + j - np / 2 << ",";
				}
			}
			cout << endl;
		}
		m_filteredInterpolatedSequence.push_back(kframe);
	}
}
bool KSkeleton::writeTRC()
{
	QFile qf("joint_positions.trc");
	if (!qf.open(QIODevice::WriteOnly | QIODevice::Text)) {
		cout << "Could not create .trc file" << endl;
		return false;
	}

	QVector<KFrame> sequenceToWrite = m_filteredInterpolatedSequence;

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
	out << sequenceToWrite.size() << "\t";
	out << NUM_MARKERS << "\t";
	out << "mm\t";
	out << "30\t";
	out << "1\t";
	out << sequenceToWrite.size() << "\n";
	// Line 4
	out << "Frame#\t";
	out << "Time";
	for (int i = 0; i < NUM_MARKERS; i++) {
		out << "\t" << m_nodes[i].name << "\t\t";
	}
	out << "\n";
	// Line 5
	out << "\t";
	for (int i = 0; i < NUM_MARKERS; i++) {
		out << "\t" << "X" << (i + 1) << "\t" << "Y" << (i + 1) << "\t" << "Z" << (i + 1);
	}
	out << "\n";
	// Lines 6+
	out.setFieldWidth(12);
	for (uint i = 0; i < sequenceToWrite.size(); i++) {
		out << "\n" << i << "\t" << sequenceToWrite[i].timestamp;
		for (int j = 0; j < NUM_MARKERS; j++) {
			out << "\t" << sequenceToWrite[i].joints[j].position.x()*1000.f;
			out << "\t" << sequenceToWrite[i].joints[j].position.y()*1000.f;
			out << "\t" << sequenceToWrite[i].joints[j].position.z()*1000.f;
		}
	}
	out << flush;
	qf.close();

	cout << "Successfully created .trc file." << endl;
	return true;
}
bool KSkeleton::readTRC()
{
	QFile file("joint_positions.trc");
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		cout << "Could not read joint_positions.trc file" << endl;
		return false;
	}
	while (!file.atEnd()) {
		QByteArray line = file.readLine();
		// process data here
	}
	file.close();
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
	for (uint i = 0; i < NUM_MARKERS; i++) {
		uint p = m_nodes[i].parentId;
		if (p != INVALID_JOINT_ID) {
			(m_nodes[p].childrenId).push_back(i);
		}
	}
}

void KSkeleton::nextJoint(int step)
{
	m_activeJoint = Mod(m_activeJoint, JointType_Count, step);
	const KJoint &j = m_joints[m_activeJoint];
	qDebug() << qSetFieldWidth(15) << m_nodes[m_activeJoint].name << ": " << qSetFieldWidth(10) << m_joints[m_activeJoint].position << m_joints[m_activeJoint].getTrackingState();
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
		const KJoint &jt = m_joints[i];
		cout << setw(15) << m_nodes[i].name.toStdString() << ": ";
		cout << setw(15) << m_joints[i].position << " ";
		cout << m_joints[i].getTrackingState().toStdString();
		cout << endl;
	}
	cout << endl;
}

void KSkeleton::printSequence() const
{
	if (m_sequence.size() == 0) {
		cout << "Sequence is empty. Returning." << endl;
		return;
	}
	cout << "Number of frames in sequence: " << m_sequence.size() << endl;
	for (uint i = 0; i < 1; i++) {
		cout << "Frame=" << i << " Timestamp=" << m_sequence[i].timestamp << endl;
		for (uint j = 0; j < JointType_Count; j++) {
			const KJoint &jt = m_sequence[i].joints[j];
			cout << setw(15) << m_nodes[j].name.toStdString() << ": " << flush;
			qDebug() << qSetFieldWidth(15) << m_nodes[j].name << ": " << qSetFieldWidth(10) << m_joints[j].position << m_joints[j].getTrackingState();
		}
	}
}

double KSkeleton::timeStep() const
{
	return m_timeStep;
}
void KSkeleton::setTimestep(double timestep)
{
	m_timeStep = timestep;
}
uint KSkeleton::activeFrame() const
{
	return m_activeFrame;
}
void KSkeleton::nextActiveFrame()
{
	if (m_playbackFiltered && m_playbackInterpolated) {
		if (++m_activeFrame >= m_filteredInterpolatedSequence.size()) m_activeFrame = 0;
	}
	else if (m_playbackInterpolated) {
		if (++m_activeFrame >= m_interpolatedSequence.size()) m_activeFrame = 0;
	}
	else if (m_playbackFiltered) {
		if (++m_activeFrame >= m_interpolatedSequence.size()) m_activeFrame = 0;
	}
	else {
		if (++m_activeFrame > m_sequence.size()) m_activeFrame = 0;
	}
}
void KSkeleton::getActiveFrame()
{
	if (m_playbackFiltered && m_playbackInterpolated) {
		cout << "Playing back filteredInterpolated: " << m_activeFrame << "/" << m_filteredInterpolatedSequence.size() << endl;
		m_joints = m_filteredInterpolatedSequence[m_activeFrame].joints;
	}
	else if (m_playbackInterpolated) {
		cout << "Playing back interpolated " << m_activeFrame << "/" << m_interpolatedSequence.size() << endl;
		m_joints = m_interpolatedSequence[m_activeFrame].joints;
	}
	else if (m_playbackFiltered) {
		cout << "Playing back filtered " << m_activeFrame << "/" << m_filteredSequence.size() << endl;
		m_joints = m_filteredSequence[m_activeFrame].joints;
	}
	else {
		cout << "Playing back raw " << m_activeFrame << "/" << m_sequence.size() << endl;
		m_joints = m_sequence[m_activeFrame].joints;
	}
}

void KSkeleton::setActiveFrame(uint index)
{
	if (m_interpolatedSequence.size() == 0) {
		cout << "Interpolated sequence is empty." << endl;
	}
	else if (index<0 || index>=m_interpolatedSequence.size()) {
		cout << "Out of bounds" << endl;
	}
	else {
		m_activeFrame = index;
		cout << "Active frame index = " << m_activeFrame << endl;
		m_joints = m_sequence[m_activeFrame].joints;
	}
}
void KSkeleton::setActiveFrame(float progressPercent)
{
	if (m_sequence.size() == 0) {
		cout << "No sequence recorded." << endl;
	}
	else {
		m_activeFrame = m_sequence.size() * progressPercent / 100.f;
		cout << "Active frame = " << m_activeFrame << endl;
		m_joints = m_sequence[m_activeFrame].joints;
	}
}

void KSkeleton::saveToBinary() const
{
	QFile qf("sequences.txt");
	if (!qf.open(QIODevice::WriteOnly)) {
		cout << "Cannot write to sequences.txt file." << endl;
		return;
	}
	else {
		cout << "Saving to binary sequences.txt" << endl;
	}
	QDataStream out(&qf);
	//out.setVersion(QDataStream::Qt_5_9);
	out << m_sequence;
	out << m_interpolatedSequence;
	out << m_filteredInterpolatedSequence;
	out << m_filteredSequence;
	qf.close();
	cout << "Size of saved sequence: " << m_sequence.size() << endl;
	if (m_sequence.size() != 0) cout << "Duration: " << m_sequence.back().timestamp << endl;
	cout << "Size of saved interpolated sequence: " << m_interpolatedSequence.size() << endl;
	if (m_interpolatedSequence.size() != 0) cout << "Duration: " << m_interpolatedSequence.back().timestamp << endl;
	cout << "Size of saved filtered interpolated sequence: " << m_filteredInterpolatedSequence.size() << endl;
	if (m_filteredInterpolatedSequence.size() != 0) cout << "Duration: " << m_filteredInterpolatedSequence.back().timestamp << endl;
	cout << "Size of saved filtered sequence: " << m_filteredSequence.size() << endl;
	if (m_filteredSequence.size() != 0) cout << "Duration: " << m_filteredSequence.back().timestamp << endl;
}

void KSkeleton::loadFromBinary()
{
	QFile qf("sequences.txt");
	if (!qf.open(QIODevice::ReadOnly)) {
		cout << "Cannot read from sequences.txt file." << endl;
		return;
	}
	else {
		cout << "Loading from binary sequences.txt" << endl;
	}
	QDataStream in(&qf);
	//in.setVersion(QDataStream::Qt_5_9);
	clearSequences();
	in >> m_sequence;
	in >> m_interpolatedSequence;
	in >> m_filteredInterpolatedSequence;
	in >> m_filteredSequence;
	qf.close();

	cout << "Size of loaded sequence: " << m_sequence.size() << endl;
	if (m_sequence.size() != 0) cout << "Duration: " << m_sequence.back().timestamp << endl;
	cout << "Size of loaded interpolated sequence: " << m_interpolatedSequence.size() << endl;
	if (m_interpolatedSequence.size() != 0) cout << "Duration: " << m_interpolatedSequence.back().timestamp << endl;
	cout << "Size of loaded filtered interpolated sequence: " << m_filteredInterpolatedSequence.size() << endl;
	if (m_filteredInterpolatedSequence.size() != 0) cout << "Duration: " << m_filteredInterpolatedSequence.back().timestamp << endl;
	cout << "Size of loaded filtered sequence: " << m_filteredSequence.size() << endl;
	if (m_filteredSequence.size() != 0) cout << "Duration: " << m_filteredSequence.back().timestamp << endl;
}
void KSkeleton::clearSequences()
{
	// #todo: clear may be expensive so add if for each clear
	m_sequence.clear();
	m_interpolatedSequence.clear();
	m_filteredInterpolatedSequence.clear();
	m_filteredSequence.clear();
	m_activeFrame = 0;
}
bool KSkeleton::initOGL() {
	bool success = initializeOpenGLFunctions();

	GLushort indices[] =
	{
		// core (4 parts)
		JointType_SpineBase    , JointType_SpineMid,
		JointType_SpineMid     , JointType_SpineShoulder,
		JointType_SpineShoulder, JointType_Neck,
		JointType_Neck         , JointType_Head,
		// left side (10 parts)	   
		JointType_SpineShoulder, JointType_ShoulderLeft,
		JointType_ShoulderLeft , JointType_ElbowLeft,
		JointType_ElbowLeft    , JointType_WristLeft,
		JointType_WristLeft    , JointType_HandLeft,
		JointType_HandLeft     , JointType_ThumbLeft,
		JointType_HandLeft     , JointType_HandTipLeft,
		JointType_SpineBase    , JointType_HipLeft,
		JointType_HipLeft      , JointType_KneeLeft,
		JointType_KneeLeft     , JointType_AnkleLeft,
		JointType_AnkleLeft    , JointType_FootLeft,
		// Right side (10 parts) 
		JointType_SpineShoulder, JointType_ShoulderRight,
		JointType_ShoulderRight, JointType_ElbowRight,
		JointType_ElbowRight   , JointType_WristRight,
		JointType_WristRight   , JointType_HandRight,
		JointType_HandRight    , JointType_ThumbRight,
		JointType_HandRight    , JointType_HandTipRight,
		JointType_SpineBase    , JointType_HipRight,
		JointType_HipRight     , JointType_KneeRight,
		JointType_KneeRight    , JointType_AnkleRight,
		JointType_AnkleRight   , JointType_FootRight
	};

	glGenVertexArrays(1, &m_skeletonVAO);
	glBindVertexArray(m_skeletonVAO);
	
	glGenBuffers(1, &m_skeletonIBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_skeletonIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_skeletonVBO);
	glBindBuffer(GL_ARRAY_BUFFER, m_skeletonVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * 3 * JointType_Count, NULL, GL_STREAM_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, BUFFER_OFFSET(sizeof(GLfloat) * 3));

	glBindVertexArray(0);

	return success;
}
void KSkeleton::printJointBufferData()
{
	cout << "Joint Buffer Data:" << endl;
	for (uint i = 0; i < JointType_Count; i++) {
		cout << setw(15) << m_nodes[i].name.toStdString() << ": ";
		cout << setw(15) << m_jointBufferData[6 * i    ]  << " ";
		cout << setw(15) << m_jointBufferData[6 * i + 1]  << " ";
		cout << setw(15) << m_jointBufferData[6 * i + 2]  << " ";
		cout << setw(15) << m_jointBufferData[6 * i + 3]  << " ";
		cout << setw(15) << m_jointBufferData[6 * i + 4]  << " ";
		cout << setw(15) << m_jointBufferData[6 * i + 5]  << " ";
		cout << endl;
	}
}
void KSkeleton::loadSkeletonData()
{
	glBindBuffer(GL_ARRAY_BUFFER, m_skeletonVBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_jointBufferData), m_jointBufferData);
}
void KSkeleton::drawSkeleton()
{
	for (uint i = 0; i < JointType_Count; i++) {
		m_jointBufferData[6 * i]     = m_joints[i].position.x();
		m_jointBufferData[6 * i + 1] = m_joints[i].position.y();
		m_jointBufferData[6 * i + 2] = m_joints[i].position.z();
		m_jointBufferData[6 * i + 3] = (m_joints[i].trackingState == TrackingState_NotTracked ? 255.f : 0.f);
		m_jointBufferData[6 * i + 4] = (m_joints[i].trackingState == TrackingState_Tracked ? 255.f : 0.f);
		m_jointBufferData[6 * i + 5] = (m_joints[i].trackingState == TrackingState_Inferred ? 255.f : 0.f);
	}

	loadSkeletonData();

	glBindVertexArray(m_skeletonVAO);
	glDrawElements(GL_LINES, 48, GL_UNSIGNED_SHORT, 0);
	glBindVertexArray(0);
}
