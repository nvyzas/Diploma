// Own
#include "ksensor.h"

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
bool KSensor::getBodyData()
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
		INT64 relTime = 0;
		hr = frame->get_RelativeTime(&relTime);

		IBody* bodies[BODY_COUNT] = { 0 };
		if (FAILED(hr)) {
			cout << "Could not get relative time. hr = " << hr << endl;
		} else {
			cout << "Relative time = " << relTime << endl;
			hr = frame->GetAndRefreshBodyData(_countof(bodies), bodies);
		}

		if (FAILED(hr)) {
			cout << "Could not get and refresh body data. hr = " << hr << endl;
		} else	{
			processBodyFrameData(relTime, BODY_COUNT, bodies);
		}

		for (int i = 0; i < _countof(bodies); ++i) {
			safeRelease(&bodies[i]);
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
	JointOrientation orientations[JointType_Count];
	for (int i = 0; i < BODY_COUNT; i++) {
		bodies[i]->get_IsTracked(&isTracked);
		if (isTracked) {
			cout << "Found a body " << i << endl;
			bodies[i]->GetJoints(JointType_Count, joints);
			bodies[i]->GetJointOrientations(JointType_Count, orientations);
			discardFrame = false;
		}
	}

	if (!discardFrame) {
		m_totalFramesCount++; 
		if (m_frameBegin != 0) m_frameEnd = clock();
		m_totalTime += double(m_frameEnd - m_frameBegin) / CLOCKS_PER_SEC;
		m_skeleton.addFrame(joints, orientations, m_totalTime);
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
	m_totalFramesCount = 0;
	m_totalTime = 0;
	m_frameBegin = 0, m_frameEnd = 0;

	// used to calculate calculate fps
	m_currentTime = 0, m_previousTime = 0;
	m_fps = 0, m_frameCount = 0;
}
KSkeleton* KSensor::kskeleton()
{
	return &m_skeleton;
}