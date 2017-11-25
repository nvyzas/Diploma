// Own
#include "kskeleton.h"

// Project
#include "math_3d.h"
#include "util.h"

// Qt
#include <QtCore/QFile>

KSkeleton::KSkeleton()
{
	initJoints();
	printJointHierarchy();
}

void KSkeleton::addFrame(const Joint *joints, const JointOrientation *orientations, const double &time, bool record)
{
	for (uint i = 0; i < JointType_Count; i++) {
		const Joint &jt = joints[i];
		const JointOrientation & or = orientations[i];
		m_joints[i].position = Vector3f(jt.Position.X, jt.Position.Y, jt.Position.Z);
		m_joints[i].trackingState = jt.TrackingState;
		m_joints[i].orientation = QQuaternion(or .Orientation.w, or .Orientation.x, or .Orientation.y, or .Orientation.z);
	}

	if (record) {
		KFrame kframe;
		kframe.timestamp = time;
		kframe.joints = m_joints;
		m_sequence.push_back(kframe);
		if (m_enableFiltering) {
			uint currentIndex = m_sequence.size() - 1;
			double interpolationTime = m_timeStep * currentIndex;
			if (interpolationTime == time) {
				cout << "interpolation time = frame time = " << time << endl;
			}
			else if (interpolationTime > time) {
				cout << "interpolation time > frame time " << interpolationTime << " > " << time << endl;
			}
			else {
				kframe.interpolate(m_sequence.end()[-2], m_sequence.back(), interpolationTime);
			}
			m_interpolatedSequence.push_back(kframe);
		}
	}
}
void KSkeleton::sgFilter()
{
	uint currentIndex = m_sequence.size() - 1;
	if (currentIndex >= NUM_POINTS) {
		uint filteredIndex = currentIndex - NUM_POINTS / 2;
		KFrame filteredInterpolatedFrame;
		cout << "filteredIndex=" << filteredIndex;
		for (uint i = 0; i < 1; i++) {
			for (uint j = -NUM_POINTS; j < NUM_POINTS; j++) {
				filteredInterpolatedFrame.joints[i].position +=
					m_interpolatedSequence[filteredIndex + j].joints[i].position*m_sgCoefficients[j + NUM_POINTS / 2] * (1.f / 35.f);
				cout << " " << filteredIndex + j << " " << m_sgCoefficients[j + NUM_POINTS / 2];
			}
		}
		cout << endl;
	}
	else {
		cout << "Current index = " << currentIndex << "Not enough frames to start filtering." << endl;
	}

}
bool KSkeleton::createTRC()
{
	QFile file("joint_positions.trc");
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		cout << "Could not create .trc file" << endl;
		return false;
	}
	QTextStream out(m_trcFile);
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
			out << "\t" << m_sequence[i].joints[j].position.x*1000.;
			out << "\t" << m_sequence[i].joints[j].position.y*1000.;
			out << "\t" << m_sequence[i].joints[j].position.z*1000.;
		}
	}
	out << flush;
	file.close();
	return true;
}
bool KSkeleton::readTRC()
{
	QFile file("joint_positions.trc");
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		cout << "Could not read .trc file" << endl;
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
		glVertex3f(parent.position.x, parent.position.y, parent.position.z);
		glVertex3f(child.position.x, child.position.y, child.position.z);
		glEnd();
		drawSkeleton(childId);
	}
}

void KSkeleton::drawActiveJoint()
{
	glBegin(GL_LINES);
	const Vector3f &p = m_joints[m_activeJoint].position;
	QVector3D qp(p.x, p.y, p.z);
	const QQuaternion &q = m_joints[m_activeJoint].orientation;
	QVector3D v;

	v = q.rotatedVector(QVector3D(1.f, 0.f, 0.f)) + qp;
	glColor3f(0xFF, 0xFF, 0);
	glVertex3f(p.x, p.y, p.z);
	glVertex3f(v.x(), v.y(), v.z());

	v = q.rotatedVector(QVector3D(0.f, 1.f, 0.f)) + qp;
	glColor3f(0, 0xFF, 0xFF);
	glVertex3f(p.x, p.y, p.z);
	glVertex3f(v.x(), v.y(), v.z());

	v = q.rotatedVector(QVector3D(0.f, 0.f, 1.f)) + qp;
	glColor3f(0xFF, 0, 0xFF);
	glVertex3f(p.x, p.y, p.z);
	glVertex3f(v.x(), v.y(), v.z());
	glEnd();
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
	for (uint i = 0; i < 10; i++) {
		cout << "Frame=" << i << " Timestamp=" << m_sequence[i].timestamp << endl;
		for (uint j = 0; j < JointType_Count; j++) {
			const KJoint &jt = m_sequence[i].joints[j];
			jt.print();
		}
	}
}
void KSkeleton::setActiveFrame(uint progressPercent)
{
	if (m_sequence.size() == 0) {
		cout << "No sequence recorded." << endl;
	}
	else {
		m_activeFrame = m_sequence.size() * progressPercent / 100;
		cout << "Active frame = " << m_activeFrame << endl;
		m_joints = m_sequence[m_activeFrame].joints;
	}
}

void KSkeleton::saveToBinary() const
{
	QFile qf("sequences.txt");
	if (!qf.open(QIODevice::ReadOnly | QIODevice::Text)) {
		cout << "Count not create sequences.txt file" << endl;
		return;
	}
	QDataStream qds(&qf);
	for (uint i = 0; i < m_sequence.size(); i++) {
		qds << m_sequence[i].joints << m_sequence[i].timestamp;
	}
}
