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
	initJointHierarchy();
	printJointHierarchy();
	loadFromBinary();
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
			cout << " interpolationTime = frameTime = " << time << endl;
		}
		else if (interpolationTime > time) {
			/*cout << " interpolationTime > frameTime " << interpolationTime << " > " << time;
			cout << " Difference = " << interpolationTime - time << endl;*/
			kframe.interpolate(m_sequence.end()[-2], m_sequence.back(), interpolationTime);
		}
		else {
			/*cout << " interpolationTime < frameTime " << interpolationTime << " < " << time;
			cout << " Difference = " << interpolationTime - time << endl;*/
			kframe.interpolate(m_sequence.end()[-2], m_sequence.back(), interpolationTime);
		}
		m_interpolatedSequence.push_back(kframe);

		// Filtering
		if (kframe.serial < m_numPoints - 1) {			
			cout << " Not enough frames to start filtering." << endl;
		}
		else {
			uint filteredIndex = kframe.serial - m_numPoints / 2;
			//cout << "filteredIndex=" << filteredIndex;
			for (uint i = 0; i < NUM_MARKERS; i++) {
				for (uint j = 0; j < m_numPoints; j++) {
					kframe.joints[i].position +=
						m_interpolatedSequence[filteredIndex + j - m_numPoints / 2].joints[i].position *	m_sgCoefficients[j] * (1.f / 35.f);
				}
			}
		}
		//m_filteredInterpolatedSequence.push_back(kframe);
	}
}
bool KSkeleton::createTRC()
{
	QFile qf("joint_positions.trc");
	if (!qf.open(QIODevice::WriteOnly | QIODevice::Text)) {
		cout << "Could not create .trc file" << endl;
		return false;
	}
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
	out << m_sequence.size() << "\t";
	out << NUM_MARKERS << "\t";
	out << "mm\t";
	out << "30\t";
	out << "1\t";
	out << m_sequence.size() << "\n";
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
	for (uint i = 0; i < m_sequence.size(); i++) {
		out << "\n" << i << "\t" << m_sequence[i].timestamp;
		for (int j = 0; j < NUM_MARKERS; j++) {
			out << "\t" << m_sequence[i].joints[j].position.x()*1000.f;
			out << "\t" << m_sequence[i].joints[j].position.y()*1000.f;
			out << "\t" << m_sequence[i].joints[j].position.z()*1000.f;
		}
	}
	out << flush;
	qf.close();
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
void KSkeleton::drawSkeleton(uint parentId)
{
	const KJoint &parent = m_joints[parentId];
	const KNode& n = m_nodes[parentId];
	for (uint i = 0; i < n.childrenId.size(); i++) {
		uint childId = n.childrenId[i];
		const KJoint &child = m_joints[childId];
		glBegin(GL_LINES);
		glColor3f(0xFF, 0xFF, 0xFF);
		glVertex3f(parent.position.x(), parent.position.y(), parent.position.z());
		glVertex3f(child.position.x(), child.position.y(), child.position.z());
		glEnd();
		drawSkeleton(childId);
	}
}

void KSkeleton::drawActiveJoint()
{
	glBegin(GL_LINES);
	const QVector3D &p = m_joints[m_activeJoint].position;
	QVector3D qp(p.x(), p.y(), p.z());
	const QQuaternion &q = m_joints[m_activeJoint].orientation;
	QVector3D v;

	v = q.rotatedVector(QVector3D(1.f, 0.f, 0.f)) + qp;
	glColor3f(0xFF, 0xFF, 0);
	glVertex3f(p.x(), p.y(), p.z());
	glVertex3f(v.x(), v.y(), v.z());

	v = q.rotatedVector(QVector3D(0.f, 1.f, 0.f)) + qp;
	glColor3f(0, 0xFF, 0xFF);
	glVertex3f(p.x(), p.y(), p.z());
	glVertex3f(v.x(), v.y(), v.z());

	v = q.rotatedVector(QVector3D(0.f, 0.f, 1.f)) + qp;
	glColor3f(0xFF, 0, 0xFF);
	glVertex3f(p.x(), p.y(), p.z());
	glVertex3f(v.x(), v.y(), v.z());
	glEnd();
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

bool KSkeleton::initOGL() {
	return initializeOpenGLFunctions();
}

void KSkeleton::nextJoint(int step)
{
	m_activeJoint = Mod(m_activeJoint, JointType_Count, step);
	const KJoint &j = m_joints[m_activeJoint];
	j.print();
}

void KSkeleton::printInfo() const
{
	cout << endl;
	cout << "Joint hierarchy" << endl;
	printJointHierarchy();
	cout << "Joint data" << endl;
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
		jt.print();
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
			cout << setw(15) << (m_nodes[j].name).toStdString() << ": " << flush;
			jt.print();
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
		cout << "Both " << m_activeFrame << endl;
		m_joints = m_filteredInterpolatedSequence[m_activeFrame].joints;
	}
	else if (m_playbackInterpolated) {
		cout << "Just interpolated " << m_activeFrame << endl;
		m_joints = m_interpolatedSequence[m_activeFrame].joints;
	}
	else if (m_playbackFiltered) {
		cout << "Deinterpolated filtered " << m_activeFrame << endl;
		m_joints = m_filteredSequence[m_activeFrame].joints;
	}
	else {
		cout << "Raw " << m_activeFrame << endl;
		m_joints = m_sequence[m_activeFrame].joints;
	}
	//cout << "Active frame = " << m_activeFrame << endl;
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
}
