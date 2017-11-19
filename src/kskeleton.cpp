// Own
#include "kskeleton.h"
#include "util.h"

KSkeleton::KSkeleton()
{
	initJoints();
	printJointHierarchy();
}

void KSkeleton::addFrame(const Joint *joints, const JointOrientation *orientations, const double &timestamp, bool record)
{
	for (uint i = 0; i < JointType_Count; i++) {
		const Joint &jt = joints[i];
		const TrackingState &ts = jt.TrackingState;
		/*if (ts == TrackingState_Inferred) {
			cout << m_joints[i].name << " is inferred." << endl;
		}
		else if (ts == TrackingState_NotTracked) {
			cout << m_joints[i].name << " is NOT tracked." << endl;
		}*/
		const JointOrientation & or = orientations[i];
		//cout << "JointType: " << setw(2) << jt.JointType << "\tJointOrientationType: " << setw(2) << or.JointType << endl;
		
		m_joints[i].Position = Vector3f(jt.Position.X, jt.Position.Y, jt.Position.Z);
		m_joints[i].Orientation = QQuaternion(or .Orientation.w, or .Orientation.x, or .Orientation.y, or .Orientation.z);
		m_joints[i].trackingState = ts;
		//cout << left << setw(2) << i << ": " << left << setw(15) << j.name << " p=" << left << setw(25) << j.Position.ToString() << " q=" << j.Orientation.ToString() << j.Orientation.ToEulerAnglesString() << j.Orientation.ToAxisAngleString() << endl;
		
		if (true) {
			for (uint i = 0; i < JointType_Count; i++) {
				m_jts[i] = m_joints[i];
			}
			m_timestamps.push_back(timestamp);
			m_seq.push_back(m_jts);
		}
		/*if (record) {
			for (int i = 0; i < JointType_Count;i++){
				m_seq.front() = (KJoint*)malloc(JointType_Count * sizeof(KJoint));
		}*/
	}
}
void KSkeleton::drawSkeleton(uint id)
{
	KJoint j = m_joints[id];
	for (uint i = 0; i < j.children.size(); i++) {
		uint c = j.children[i];
		const KJoint &cj = m_joints[c];
		glBegin(GL_LINES);
		glColor3f(0xFF, 0xFF, 0xFF);
		glVertex3f(j.Position.x, j.Position.y, j.Position.z);
		glVertex3f(cj.Position.x, cj.Position.y, cj.Position.z);
		glEnd();
		drawSkeleton(c);
	}
}

void KSkeleton::drawActiveJoint()
{
	glBegin(GL_LINES);
	const Vector3f &p = m_joints[m_activeJoint].Position;
	QVector3D qp(p.x, p.y, p.z);
	const QQuaternion &q = m_joints[m_activeJoint].Orientation;
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

const KJoint* KSkeleton::getKJoints() const
{
	return m_joints;
}

void KSkeleton::initJoints()
{
	// Set parents
	// core
	m_joints[JointType_SpineBase] = KJoint("SpineBase", INVALID_JOINT_ID, JointType_SpineBase, true); // root
	m_joints[JointType_SpineMid] = KJoint("SpineMid", JointType_SpineBase, JointType_SpineMid, true);
	m_joints[JointType_SpineShoulder] = KJoint("SpineShoulder", JointType_SpineMid, JointType_SpineShoulder, true);
	m_joints[JointType_Neck] = KJoint("Neck", JointType_SpineShoulder, JointType_Neck, true);
	m_joints[JointType_Head] = KJoint("Head", JointType_Neck, JointType_Head, true);
	// left side																								
	m_joints[JointType_HipLeft] = KJoint("HipLeft", JointType_SpineBase, JointType_HipRight, true);
	m_joints[JointType_KneeLeft] = KJoint("KneeLeft", JointType_HipLeft, JointType_KneeRight, true);
	m_joints[JointType_AnkleLeft] = KJoint("AnkleLeft", JointType_KneeLeft, JointType_AnkleRight, true);
	m_joints[JointType_FootLeft] = KJoint("FootLeft", JointType_AnkleLeft, JointType_FootRight, true);
	m_joints[JointType_ShoulderLeft] = KJoint("ShoulderLeft", JointType_SpineShoulder, JointType_ShoulderRight, true);
	m_joints[JointType_ElbowLeft] = KJoint("ElbowLeft", JointType_ShoulderLeft, JointType_ElbowRight, true);
	m_joints[JointType_WristLeft] = KJoint("WristLeft", JointType_ElbowLeft, JointType_WristRight, true);
	m_joints[JointType_HandLeft] = KJoint("HandLeft", JointType_WristLeft, JointType_HandRight, true);
	m_joints[JointType_ThumbLeft] = KJoint("ThumbLeft", JointType_HandLeft, JointType_ThumbRight, true);
	m_joints[JointType_HandTipLeft] = KJoint("HandTipLeft", JointType_HandLeft, JointType_HandTipRight, true);
	// right side																									
	m_joints[JointType_HipRight] = KJoint("HipRight", JointType_SpineBase, JointType_HipLeft, true);
	m_joints[JointType_KneeRight] = KJoint("KneeRight", JointType_HipRight, JointType_KneeLeft, true);
	m_joints[JointType_AnkleRight] = KJoint("AnkleRight", JointType_KneeRight, JointType_AnkleLeft, true);
	m_joints[JointType_FootRight] = KJoint("FootRight", JointType_AnkleRight, JointType_FootLeft, true);
	m_joints[JointType_ShoulderRight] = KJoint("ShoulderRight", JointType_SpineShoulder, JointType_ShoulderLeft, true);
	m_joints[JointType_ElbowRight] = KJoint("ElbowRight", JointType_ShoulderRight, JointType_ElbowLeft, true);
	m_joints[JointType_WristRight] = KJoint("WristRight", JointType_ElbowRight, JointType_WristLeft, true);
	m_joints[JointType_HandRight] = KJoint("HandRight", JointType_WristRight, JointType_HandLeft, true);
	m_joints[JointType_ThumbRight] = KJoint("ThumbRight", JointType_HandRight, JointType_ThumbLeft, true);
	m_joints[JointType_HandTipRight] = KJoint("HandTipRight", JointType_HandRight, JointType_HandTipLeft, true);

	// Set children
	for (uint i = 0; i < JointType_Count; i++) {
		m_joints[i].Position = Vector3f(0.f, 0.f, 0.f);
		m_joints[i].Orientation = QQuaternion(1.f, 0.f, 0.f, 0.f);
		uint p = m_joints[i].parent;
		if (p != INVALID_JOINT_ID) (m_joints[p].children).push_back(i);
	}
}

bool KSkeleton::initOGL(){
	return initializeOpenGLFunctions();
}

void KSkeleton::nextJoint(int step)
{
	m_activeJoint = Mod(m_activeJoint, JointType_Count, step);
	const KJoint &j = m_joints[m_activeJoint];
	cout << left << setw(2) << m_activeJoint << ": " << setw(15) << j.name << " p=" << left << setw(25) << j.Position.ToString() << " q=" << printQuaternion1(j.Orientation) << printQuaternion2(j.Orientation) << printQuaternion3(j.Orientation) << endl;
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
		uint p = m_joints[i].parent;
		cout << "Joint " << setw(2) << left << i << ": " << setw(20) << left << m_joints[i].name;
		cout << " Parent: " << setw(20) << left << (p == INVALID_JOINT_ID ? "NONE" : m_joints[p].name);
		cout << " Children (" << m_joints[i].children.size() << "): ";
		for (uint j = 0; j < m_joints[i].children.size(); j++) {
			uint c = m_joints[i].children[j];
			cout << m_joints[c].name << " ";
		}
		cout << endl;
	}
}

void KSkeleton::printJoints() const
{
	for (uint i = 0; i < JointType_Count; i++) {
		const KJoint &jt = m_joints[i];
		cout << setw(15) << jt.name << "p: " << setw(25) << jt.Position.ToString() << " q: " << printQuaternion1(jt.Orientation) << printQuaternion2(jt.Orientation) << printQuaternion3(jt.Orientation) << endl;
	}
	cout << endl;
}

void KSkeleton::printSequence() const
{
	cout << "Number of frames in sequence: " << m_seq.size() << endl;
	for (uint i = 0; i < 10; i++) {
		cout << "Sequence=" << i << " Seconds=" << m_timestamps[i] << endl;
		for (uint j = 0; j < JointType_Count; j++){
			const KJoint &jt = m_seq[i][j];
			cout << setw(15) << jt.name << "p: " << setw(25) << jt.Position.ToString() << " q: " << printQuaternion1(jt.Orientation) << printQuaternion2(jt.Orientation) << printQuaternion3(jt.Orientation) << endl;
		}
	}
}
