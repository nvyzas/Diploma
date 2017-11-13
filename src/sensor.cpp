// Own
#include "sensor.h"

// Project
#include "util.h"

// Qt
#include <QtCore/QFile>

// Windows
#include <Windows.h>
//#include <Ole2.h>
//#include <algorithm>

// Standard C/C++
#include <iomanip>
#include <iostream>
#include <vector>

template <class T> void safeRelease(T **ppT)
{
	if (*ppT)
	{
		(*ppT)->Release();
		*ppT = NULL;
	}
}

KSensor::KSensor()
{
	init(); 
	connect();
	//initializeOpenGLFunctions();
	initJoints();
	PrintJointHierarchy();
	m_GotFrame = false;
}
KSensor::~KSensor()
{
	safeRelease(&m_reader);
	m_sensor->Close();
}
bool KSensor::init() {
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_sensor);
	if (FAILED(hr)) {
		cout << "Could not get kinect sensor. hr = " <<  hr << endl;
		return false;
	}
	if (!m_sensor) {
		cout << "m_sensor = NULL" << endl;
		return false;
	}
	hr = m_sensor->Open();
	if (FAILED(hr)) {
		cout << hr << "Could not open sensor. hr = " << hr << endl;
		return false;
	}

	return true;	
}
bool KSensor::connect()
{
	if (m_sensor == NULL) {
		cout << "m_sensor = NULL" << endl;
		return false;
	}
	HRESULT hr;
	BOOLEAN isOpen = false;
	hr = m_sensor->get_IsOpen(&isOpen);
	if (SUCCEEDED(hr)) {
		if (!isOpen) cout << "Sensor is not open." << endl;
	}
	else {
		cout << "Could not specify if sensor is open. hr = " << hr << endl;
	}

	hr = m_sensor->get_BodyFrameSource(&m_source);
	if (FAILED(hr)) {
		cout << hr << "Could not get frame source. hr = " << hr << endl;
		return false;
	}
	hr = m_source->OpenReader(&m_reader);
	if (FAILED(hr)) {
		cout << hr << "Could not open reader.  hr = " << hr << endl;
		return false;
	}

	// Must open reader first for source to be active
	BOOLEAN isActive = false;
	hr = m_source->get_IsActive(&isActive);
	if (SUCCEEDED(hr)) {
		if (!isActive) cout << "Source is not active." << endl;
	}else {
		cout << "Could not specify if source is active. hr = " << hr << endl;
	}

	safeRelease(&m_source);

	return true;
}
bool KSensor::update()
{
	if (!m_sensor) {
		cout << "m_sensor = NULL" << endl;
		return false;
	}
	/*if (!m_source) {
		cout << "m_source = NULL" << endl;
		return false;
	}*/
	if (!m_reader) {
		cout << "m_reader = NULL" << endl;
		return false;
	}
	HRESULT hr;
	BOOLEAN isOpen = false;
	hr = m_sensor->get_IsOpen(&isOpen);
	if (SUCCEEDED(hr)) {
		if (!isOpen) cout << "Sensor is not open." << endl;
	}
	else {
		cout << "Could not specify if sensor is open. hr = " << hr << endl;
	}
	/*BOOLEAN isActive = false;
	hr = m_source->get_IsActive(&isActive);
	if (SUCCEEDED(hr)) {
		if (!isActive) cout << "Source is not active." << endl;
	}
	else {
		cout << "Could not specify if source is active. hr = " << hr << endl;
	}*/

	IBodyFrame* frame = NULL;
	hr = m_reader->AcquireLatestFrame(&frame);
	if (FAILED(hr)) {
		cout << "Failed to acquire latest frame. hr = " << hr << endl;
		return false;
	} else {
		INT64 nTime = 0;
		hr = frame->get_RelativeTime(&nTime);

		IBody* ppBodies[BODY_COUNT] = { 0 };

		if (FAILED(hr)) {
			cout << "Could not get relative time. hr = " << hr << endl;
		} else {
			hr = frame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		}

		if (FAILED(hr)) {
			cout << "Could not get and refresh body data. hr = " << hr << endl;
		} else	{
			processBodyFrameData(nTime, BODY_COUNT, ppBodies);
		}

		for (int i = 0; i < _countof(ppBodies); ++i) {
			safeRelease(&ppBodies[i]);
		}
		safeRelease(&frame);
	}
	// maybe release frame here
	return true;
}
void KSensor::processBodyFrameData(INT64 timestamp, int bodyCount, IBody** bodies)
{
	bool discardFrame = true;
	BOOLEAN isTracked;
	Joint joints[JointType_Count];
	JointOrientation orients[JointType_Count];
	for (int i = 0; i < BODY_COUNT; i++) {
		bodies[i]->get_IsTracked(&isTracked);
		if (isTracked) {
			cout << "Found a body " << i << endl;
			bodies[i]->GetJoints(JointType_Count, joints);
			bodies[i]->GetJointOrientations(JointType_Count, orients);
			discardFrame = false;
		}
	}
	for (uint i = 0; i < JointType_Count; i++) {
		const Joint &jt = joints[i];
		if (jt.TrackingState == TrackingState_Inferred) {
			cout << m_Joints[i].name << " tracking state is inferred." << endl;
		}
		const JointOrientation &or = orients[i];
		//cout << "JointType: " << setw(2) << jt.JointType << "\tJointOrientationType: " << setw(2) << or.JointType << endl;
		if (m_InvertedSides) {
			uint j = m_Joints[i].idOpposite;
			m_Joints[j].Position = Vector3f(jt.Position.X, jt.Position.Y, jt.Position.Z);
			m_Joints[j].Orientation = QQuaternion(or.Orientation.w, or.Orientation.x, or.Orientation.y, or.Orientation.z);
		} else {
			m_Joints[i].Position = Vector3f(jt.Position.X, jt.Position.Y, jt.Position.Z);
			m_Joints[i].Orientation = QQuaternion(or.Orientation.w, or.Orientation.x, or.Orientation.y, or.Orientation.z);
		}
		//cout << left << setw(2) << i << ": " << left << setw(15) << j.name << " p=" << left << setw(25) << j.Position.ToString() << " q=" << j.Orientation.ToString() << j.Orientation.ToEulerAnglesString() << j.Orientation.ToAxisAngleString() << endl;
	}
	
	if (!discardFrame) {
		m_totalFramesCount++; 
		if (m_frameBegin != 0) m_frameEnd = clock();
		m_totalTime += double(m_frameEnd - m_frameBegin) / CLOCKS_PER_SEC;
		//m_skeleton.addFrame();
		addMarkerData();
		calculateFPS();
		m_frameBegin = clock();
	}
	else {
		cout << "Frame dropped" << endl;
	}

}
void KSensor::calculateFPS() {
	//  Increase frame count
	m_frameCount++;

	m_currentTime = clock();

	//  Calculate time passed
	int timeInterval = m_currentTime - m_previousTime;

	if (timeInterval > 1000) {
		//  calculate the number of frames per second
		m_fps = m_frameCount / (timeInterval / 1000.f);

		//  Set time
		m_previousTime = m_currentTime;

		//  Reset frame count
		m_frameCount = 0;
	}

	cout << "FPS = " << m_fps << endl;
}
void KSensor::addMarkerData()
{
	cout << "Adding frame to marker data string." << endl;
	QTextStream qts(&m_markerData);
	qts.setFieldWidth(12); // width for minus sign + 4 important digits + comma + 6 decimals
	qts << QString("\n") << m_totalFramesCount << QString("\t") << m_totalTime;
	for (int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_Joints); i++) {
		qts << "\t" << m_Joints[i].Position.x*1000.;
		qts << "\t" << m_Joints[i].Position.y*1000.;
		qts << "\t" << m_Joints[i].Position.z*1000.;
	}
}
bool KSensor::createTRC()
{
	m_trcFile = new QFile("joint_positions.trc");
	if (!m_trcFile->open(QIODevice::WriteOnly | QIODevice::Text)) return false;
	QTextStream out(m_trcFile);
	out << "PathFileType\t4\t(X / Y / Z)\tjoint_positions.trc\n";
	out << "DataRate\t" << "CameraRate\t" << "NumFrames\t" << "NumMarkers\t" << "Units\t" << "OrigDataRate\t" << "OrigDataStartFrame\t" << "OrigNumFrames\n";
	out << "30\t" << "30\t" << m_totalFramesCount << "\t" << m_numMarkers << "\t" << "mm\t" << "30\t" << "1\t" << m_totalFramesCount << "\n";
	out << "Frame#  \tTime";
	for (int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_Joints); i++) {
		if (m_Joints[i].toBeTracked) out << "\t" << QString::fromStdString(m_Joints[i].name) << "\t\t";
	}
	out << "\n\t";
	for (int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_Joints); i++) {
		if (m_Joints[i].toBeTracked) out << "\t" << "X" << (i + 1) << "\t" << "Y" << (i + 1) << "\t" << "Z" << (i + 1);
	}
	out << "\n";
	out << m_markerData;
	m_trcFile->close();
	return true;
}
void KSensor::resetRecordVars()
{
	// used to calculate calculate frame timestamps
	uint m_totalFramesCount = 0;
	double m_totalTime = 0;
	clock_t m_frameBegin = 0, m_frameEnd = 0;

	// used to calculate calculate fps
	clock_t m_currentTime = 0, m_previousTime = 0;
	int m_fps, m_frameCount;
}
const KJoint* KSensor::getKJoints() const
{
	return m_Joints;
}
void KSensor::SwapSides()
{
	// Positions: left side <-> right
	swap(m_Joints[JointType_HipLeft].Position, m_Joints[JointType_HipRight].Position);
	swap(m_Joints[JointType_KneeLeft].Position, m_Joints[JointType_KneeRight].Position);
	swap(m_Joints[JointType_AnkleLeft].Position, m_Joints[JointType_AnkleRight].Position);
	swap(m_Joints[JointType_FootLeft].Position, m_Joints[JointType_FootRight].Position);
	swap(m_Joints[JointType_ShoulderLeft].Position, m_Joints[JointType_ShoulderRight].Position);
	swap(m_Joints[JointType_ElbowLeft].Position, m_Joints[JointType_ElbowRight].Position);
	swap(m_Joints[JointType_WristLeft].Position, m_Joints[JointType_WristRight].Position);
	swap(m_Joints[JointType_HandLeft].Position, m_Joints[JointType_HandRight].Position);
	swap(m_Joints[JointType_ThumbLeft].Position, m_Joints[JointType_ThumbRight].Position);
	swap(m_Joints[JointType_HandTipLeft].Position, m_Joints[JointType_HandTipRight].Position);
	// Orientations: left side <-> right
	swap(m_Joints[JointType_HipLeft].Orientation, m_Joints[JointType_HipRight].Orientation);
	swap(m_Joints[JointType_KneeLeft].Orientation, m_Joints[JointType_KneeRight].Orientation);
	swap(m_Joints[JointType_AnkleLeft].Orientation, m_Joints[JointType_AnkleRight].Orientation);
	swap(m_Joints[JointType_FootLeft].Orientation, m_Joints[JointType_FootRight].Orientation);
	swap(m_Joints[JointType_ShoulderLeft].Orientation, m_Joints[JointType_ShoulderRight].Orientation);
	swap(m_Joints[JointType_ElbowLeft].Orientation, m_Joints[JointType_ElbowRight].Orientation);
	swap(m_Joints[JointType_WristLeft].Orientation, m_Joints[JointType_WristRight].Orientation);
	swap(m_Joints[JointType_HandLeft].Orientation, m_Joints[JointType_HandRight].Orientation);
	swap(m_Joints[JointType_ThumbLeft].Orientation, m_Joints[JointType_ThumbRight].Orientation);
	swap(m_Joints[JointType_HandTipLeft].Orientation, m_Joints[JointType_HandTipRight].Orientation);
}

void KSensor::PrintInfo() const
{
	cout << endl;
	cout << "Kinect joint data" << endl;
	PrintJointData();
	cout << "Kinect skeleton" << endl;
	PrintJointHierarchy();
	cout << endl;
}
void KSensor::PrintJointData() const
{
	for (uint i = 0; i < JointType_Count; i++) {
		const KJoint &j = m_Joints[i];
		cout << setw(15) << j.name << "p: " << setw(25) << j.Position.ToString() << " q: " << printQuaternion1(j.Orientation) << printQuaternion2(j.Orientation) << printQuaternion3(j.Orientation) << endl;
	}
	cout << endl;
}
void KSensor::PrintJointHierarchy() const
{
	for (uint i = 0; i < JointType_Count; i++) {
		uint p = m_Joints[i].parent;
		cout << "Joint " << setw(2) << left << i << ": " << setw(20) << left << m_Joints[i].name;
		cout << " Parent: " << setw(20) << left << (p == INVALID_JOINT_ID ? "NONE" : m_Joints[p].name);
		cout << " Children (" << m_Joints[i].children.size() << "): ";
		for (uint j = 0; j < m_Joints[i].children.size(); j++) {
			uint c = m_Joints[i].children[j];
			cout << m_Joints[c].name << " ";
		}
		cout << endl;
	}
}
void KSensor::DrawSkeleton(uint id)
{
	KJoint j = m_Joints[id];
	for (uint i = 0; i < j.children.size(); i++) {
		uint c = j.children[i];
		const KJoint &cj = m_Joints[c];
		glBegin(GL_LINES);
		glColor3f(0xFF, 0xFF, 0xFF);
		glVertex3f(j.Position.x, j.Position.y, j.Position.z);
		glVertex3f(cj.Position.x, cj.Position.y, cj.Position.z);
		glEnd();
		DrawSkeleton(c);
	}
}
void KSensor::DrawActiveJoint()
{
	glBegin(GL_LINES);
	const Vector3f &p = m_Joints[m_ActiveJoint].Position;
	QVector3D qp(p.x, p.y, p.z);
	const QQuaternion &q = m_Joints[m_ActiveJoint].Orientation;
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
void KSensor::initJoints() 
{
	// Set parents
	// core
	m_Joints[JointType_SpineBase]		= KJoint("SpineBase"	, INVALID_JOINT_ID		 , JointType_SpineBase		, true); // root
	m_Joints[JointType_SpineMid]		= KJoint("SpineMid"		, JointType_SpineBase	 , JointType_SpineMid		, true);
	m_Joints[JointType_SpineShoulder]   = KJoint("SpineShoulder", JointType_SpineMid	 , JointType_SpineShoulder  , true);
	m_Joints[JointType_Neck]			= KJoint("Neck"			, JointType_SpineShoulder, JointType_Neck			, true);
	m_Joints[JointType_Head]			= KJoint("Head"			, JointType_Neck		 , JointType_Head			, true);
	// left side																								
	m_Joints[JointType_HipLeft]		    = KJoint("HipLeft"	    , JointType_SpineBase	 , JointType_HipRight		, true);
	m_Joints[JointType_KneeLeft]		= KJoint("KneeLeft"		, JointType_HipLeft		 , JointType_KneeRight		, true);
	m_Joints[JointType_AnkleLeft]		= KJoint("AnkleLeft"	, JointType_KneeLeft	 , JointType_AnkleRight		, true);
	m_Joints[JointType_FootLeft]		= KJoint("FootLeft"		, JointType_AnkleLeft	 , JointType_FootRight		, true);
	m_Joints[JointType_ShoulderLeft]	= KJoint("ShoulderLeft" , JointType_SpineShoulder, JointType_ShoulderRight	, true);
	m_Joints[JointType_ElbowLeft]		= KJoint("ElbowLeft"	, JointType_ShoulderLeft , JointType_ElbowRight		, true);
	m_Joints[JointType_WristLeft]		= KJoint("WristLeft"	, JointType_ElbowLeft    , JointType_WristRight		, true);
	m_Joints[JointType_HandLeft]		= KJoint("HandLeft"		, JointType_WristLeft    , JointType_HandRight		, true);
	m_Joints[JointType_ThumbLeft]		= KJoint("ThumbLeft"	, JointType_HandLeft     , JointType_ThumbRight		, true);
	m_Joints[JointType_HandTipLeft]		= KJoint("HandTipLeft"	, JointType_HandLeft     , JointType_HandTipRight	, true);
	// right side																									
	m_Joints[JointType_HipRight]		= KJoint("HipRight"		, JointType_SpineBase	 , JointType_HipLeft		, true);
	m_Joints[JointType_KneeRight]		= KJoint("KneeRight"	, JointType_HipRight	 , JointType_KneeLeft		, true);
	m_Joints[JointType_AnkleRight]		= KJoint("AnkleRight"	, JointType_KneeRight	 , JointType_AnkleLeft		, true);
	m_Joints[JointType_FootRight]		= KJoint("FootRight"	, JointType_AnkleRight	 , JointType_FootLeft		, true);
	m_Joints[JointType_ShoulderRight]	= KJoint("ShoulderRight", JointType_SpineShoulder, JointType_ShoulderLeft	, true);
	m_Joints[JointType_ElbowRight]		= KJoint("ElbowRight"	, JointType_ShoulderRight, JointType_ElbowLeft		, true);
	m_Joints[JointType_WristRight]		= KJoint("WristRight"	, JointType_ElbowRight	 , JointType_WristLeft		, true);
	m_Joints[JointType_HandRight]		= KJoint("HandRight"	, JointType_WristRight	 , JointType_HandLeft		, true);
	m_Joints[JointType_ThumbRight]		= KJoint("ThumbRight"	, JointType_HandRight	 , JointType_ThumbLeft		, true);
	m_Joints[JointType_HandTipRight]	= KJoint("HandTipRight"	, JointType_HandRight	 , JointType_HandTipLeft	, true);

	// Set children
	for (uint i = 0; i < JointType_Count; i++) {
		m_Joints[i].Position = Vector3f(0.f, 0.f, 0.f);
		m_Joints[i].Orientation = QQuaternion(1.f, 0.f, 0.f, 0.f);
		uint p = m_Joints[i].parent;
		if (p != INVALID_JOINT_ID) (m_Joints[p].children).push_back(i);
	}
}
void KSensor::NextJoint(int step)
{
	m_ActiveJoint = Mod(m_ActiveJoint, JointType_Count, step);
	const KJoint &j = m_Joints[m_ActiveJoint];
	cout << left << setw(2) << m_ActiveJoint << ": " << setw(15) << j.name << " p=" << left << setw(25) << j.Position.ToString() << " q=" << printQuaternion1(j.Orientation) << printQuaternion2(j.Orientation) << printQuaternion3(j.Orientation) << endl;
}
void KSensor::setFootStance()
{

}