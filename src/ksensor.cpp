// Own
#include "ksensor.h"

// Project
#include "util.h"

// Windows
#include <Windows.h>

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

	m_captureLog.setFileName("capture_log.txt");
	if (!m_captureLog.open(QIODevice::WriteOnly | QIODevice::Text)) {
		cout << "Could not open capture log file." << endl;
		return;
	}
	m_forLog.setDevice(&m_captureLog);
}
KSensor::~KSensor()
{
	safeRelease(&m_reader);
	safeRelease(&m_source);
	m_sensor->Close();
	safeRelease(&m_sensor);
	m_captureLog.close();
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

	/*
	safeRelease(&m_source);
	//*/

	return true;
}
bool KSensor::getBodyData()
{
	// Checks 1
	if (!m_sensor) {
		cout << "m_sensor = NULL" << endl;
		return false;
	}
	//*
	if (!m_source) {
		cout << "m_source = NULL" << endl;
		return false;
	}
	//*/
	if (!m_reader) {
		cout << "m_reader = NULL" << endl;
		return false;
	}

	// Checks 2
	HRESULT hr;
	BOOLEAN isOpen = false;
	hr = m_sensor->get_IsOpen(&isOpen);
	if (SUCCEEDED(hr)) {
		if (!isOpen) {
			cout << "Sensor is not open." << endl;
			return false;
		}
	}
	else {
		cout << "Could not specify if sensor is open. hr = " << hr << endl;
	}
	//*
	BOOLEAN isActive = false;
	hr = m_source->get_IsActive(&isActive);
	if (SUCCEEDED(hr)) {
		if (!isActive) {
			cout << "Source is not active." << endl;
			return false;
		}
	}
	else {
		cout << "Could not specify if source is active. hr = " << hr << endl;
	}
	//*/
	BOOLEAN isPaused = false;
	hr = m_reader->get_IsPaused(&isPaused);
	if (SUCCEEDED(hr)) {
		if (isPaused) {
			cout << "Reader is paused." << endl;
			return false;
		}
	}
	else {
		cout << "Could not specify if reader is paused hr = " << hr << endl;
	}

	QString qs;
	QTextStream qts(&qs);
	// Get frame
	IBodyFrame* frame = NULL;
	hr = m_reader->AcquireLatestFrame(&frame);
	if (FAILED(hr)) {
		m_forLog << "Failed to acquire frame. hr = " << hr << endl;
		return false;
	} else {
		m_forLog << "Got frame. ";
		INT64 relTime = 0;
		hr = frame->get_RelativeTime(&relTime);

		IBody* bodies[BODY_COUNT] = { 0 };
		if (FAILED(hr)) {
			m_forLog << "Could not get relative time. hr = " << hr << endl;
		} else {
			m_forLog << "RelativeTime=" << relTime << flush;
			hr = frame->GetAndRefreshBodyData(_countof(bodies), bodies);
		}

		if (FAILED(hr)) {
			m_forLog << "Could not get and refresh body data. hr = " << hr << endl;
		} else {
			processBodyFrameData(bodies);
		}

		for (int i = 0; i < _countof(bodies); ++i) {
			safeRelease(&bodies[i]);
		}
		safeRelease(&frame);
	}

	// maybe release frame here
	return true;
}
void KSensor::processBodyFrameData(IBody** bodies)
{
	bool discardFrame = true;
	BOOLEAN isTracked;
	Joint joints[JointType_Count];
	JointOrientation orientations[JointType_Count];
	for (int i = 0; i < BODY_COUNT; i++) {
		bodies[i]->get_IsTracked(&isTracked);
		if (isTracked) {
			//m_forLog << " BodyIndex=" << i;
			bodies[i]->GetJoints(JointType_Count, joints);
			bodies[i]->GetJointOrientations(JointType_Count, orientations);
			discardFrame = false;
		}
	}

	if (!discardFrame) {
		m_acceptedFrames++;
		m_forLog << " Frame=" << m_acceptedFrames;

		if (m_frameBegin != 0) m_frameEnd = clock();
		m_totalSeconds += double(m_frameEnd - m_frameBegin) / CLOCKS_PER_SEC;
		m_forLog << " Time=" << m_totalSeconds;

		m_skeleton.addFrame(joints, orientations, m_totalSeconds, m_isRecording);
		calculateFPS();
		m_forLog << " FPS=" << m_fps << endl;
		m_frameBegin = clock();
	} else {
		m_forLog << " Frame dropped." << endl;
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
}
void KSensor::record()
{
	if (!m_isRecording) {
		// Reset record related variables
		m_acceptedFrames = 0;
		m_totalSeconds = 0;
		m_frameBegin = 0;
		m_frameEnd = 0;
		m_currentTime = 0;
		m_previousTime = 0;
		m_fps = 0;
		m_frameCount = 0;

		m_isRecording = true;
		cout << "Recording started." << endl;
	}
	else {
		m_isRecording = false;
		cout << "Recording stopped." << endl;
	}
}
KSkeleton* KSensor::skeleton()
{
	return &m_skeleton;
}
void KSensor::setSkeletonActiveFrame(uint progressPercent)
{
	m_skeleton.setActiveFrame(progressPercent);
}