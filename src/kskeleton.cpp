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
		m_joints[i].orientation = QQuaternion(or.Orientation.w, or.Orientation.x, or.Orientation.y, or.Orientation.z);
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
		out << "\t" << QString::fromStdString(m_joints[i].name) << "\t\t";
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
void KSkeleton::drawSkeleton(uint id)
{
	KJoint j = m_joints[id];
	for (uint i = 0; i < j.childrenId.size(); i++) {
		uint c = j.childrenId[i];
		const KJoint &cj = m_joints[c];
		glBegin(GL_LINES);
		glColor3f(0xFF, 0xFF, 0xFF);
		glVertex3f(j.position.x, j.position.y, j.position.z);
		glVertex3f(cj.position.x, cj.position.y, cj.position.z);
		glEnd();
		drawSkeleton(c);
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
	m_joints[JointType_SpineBase] = KJoint("SpineBase", INVALID_JOINT_ID);
		m_joints[JointType_SpineMid] = KJoint("SpineMid", JointType_SpineBase);
	m_joints[JointType_SpineShoulder] = KJoint("SpineShoulder", JointType_SpineMid);
	m_joints[JointType_Neck] = KJoint("Neck", JointType_SpineShoulder);
	m_joints[JointType_Head] = KJoint("Head", JointType_Neck);
	// left side																								
	m_joints[JointType_HipLeft] = KJoint("HipLeft", JointType_SpineBase);
	m_joints[JointType_KneeLeft] = KJoint("KneeLeft", JointType_HipLeft);
	m_joints[JointType_AnkleLeft] = KJoint("AnkleLeft", JointType_KneeLeft);
	m_joints[JointType_FootLeft] = KJoint("FootLeft", JointType_AnkleLeft);
	m_joints[JointType_ShoulderLeft] = KJoint("ShoulderLeft", JointType_SpineShoulder);
	m_joints[JointType_ElbowLeft] = KJoint("ElbowLeft", JointType_ShoulderLeft);
	m_joints[JointType_WristLeft] = KJoint("WristLeft", JointType_ElbowLeft);
	m_joints[JointType_HandLeft] = KJoint("HandLeft", JointType_WristLeft);
	m_joints[JointType_ThumbLeft] = KJoint("ThumbLeft", JointType_HandLeft);
	m_joints[JointType_HandTipLeft] = KJoint("HandTipLeft", JointType_HandLeft);
	// right side																									
	m_joints[JointType_HipRight] = KJoint("HipRight", JointType_SpineBase);
	m_joints[JointType_KneeRight] = KJoint("KneeRight", JointType_HipRight);
	m_joints[JointType_AnkleRight] = KJoint("AnkleRight", JointType_KneeRight);
	m_joints[JointType_FootRight] = KJoint("FootRight", JointType_AnkleRight);
	m_joints[JointType_ShoulderRight] = KJoint("ShoulderRight", JointType_SpineShoulder);
	m_joints[JointType_ElbowRight] = KJoint("ElbowRight", JointType_ShoulderRight);
	m_joints[JointType_WristRight] = KJoint("WristRight", JointType_ElbowRight);
	m_joints[JointType_HandRight] = KJoint("HandRight", JointType_WristRight);
	m_joints[JointType_ThumbRight] = KJoint("ThumbRight", JointType_HandRight);
	m_joints[JointType_HandTipRight] = KJoint("HandTipRight", JointType_HandRight);

	// Set children
	for (uint i = 0; i < JointType_Count; i++) {
		m_joints[i].position = Vector3f(0.f, 0.f, 0.f);
		m_joints[i].orientation = QQuaternion(1.f, 0.f, 0.f, 0.f);
		uint p = m_joints[i].parentId;
		if (p != INVALID_JOINT_ID) (m_joints[p].childrenId).push_back(i);
	}
}

bool KSkeleton::initOGL(){
	return initializeOpenGLFunctions();
}

void KSkeleton::nextJoint(int step)
{
	m_activeJoint = Mod(m_activeJoint, JointType_Count, step);
	const KJoint &j = m_joints[m_activeJoint];
	cout << left << setw(2) << m_activeJoint << ": " << setw(15) << j.name << " p=" << left << setw(25) << j.position.ToString() << " q=" << printQuaternion1(j.orientation) << printQuaternion2(j.orientation) << printQuaternion3(j.orientation) << endl;
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
		uint p = m_joints[i].parentId;
		cout << "Joint " << setw(2) << left << i << ": " << setw(20) << left << m_joints[i].name;
		cout << " Parent: " << setw(20) << left << (p == INVALID_JOINT_ID ? "NONE" : m_joints[p].name);
		cout << " Children (" << m_joints[i].childrenId.size() << "): ";
		for (uint j = 0; j < m_joints[i].childrenId.size(); j++) {
			uint c = m_joints[i].childrenId[j];
			cout << m_joints[c].name << " ";
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
		for (uint j = 0; j < JointType_Count; j++){
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
