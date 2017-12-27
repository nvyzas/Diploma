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
	m_forLog.setFieldAlignment(QTextStream::AlignLeft);
	m_forLog.setRealNumberPrecision(8);
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
	}
	else {
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

	// Get frame
	IBodyFrame* frame = NULL;
	hr = m_reader->AcquireLatestFrame(&frame);
	if (FAILED(hr)) {
		if (!m_lastAttemptFailed) m_forLog << "Failed to acquire frame." << endl;
		m_consecutiveFails++;
		if (m_consecutiveFails == 15 && m_skeleton.m_recordingOn) record();
		m_lastAttemptFailed = true;
		return false;
	} 
	else {
		m_acquiredFrames++;
		if (m_consecutiveFails > 1) m_forLog << "consecutiveFails=" << m_consecutiveFails << endl;
		m_consecutiveFails = 0;
		m_lastAttemptFailed = false;
		m_forLog << "Acquired frame " << qSetFieldWidth(4) << m_acquiredFrames;
		INT64 relativeTime;
		hr = frame->get_RelativeTime(&relativeTime);
		IBody* bodies[BODY_COUNT] = { 0 };
		if (FAILED(hr)) {
			m_forLog << " Could not get relative time. hr = " << hr << endl;
		} 
		else {
			m_totalRelativeTime = (double)relativeTime / 10000000.;
			m_forLog << " RelativeTime=" << qSetFieldWidth(10) << m_totalRelativeTime;
			hr = frame->GetAndRefreshBodyData(_countof(bodies), bodies);
		}

		if (FAILED(hr)) {
			m_forLog << " Could not get and refresh body data. hr = " << hr << endl;
		} 
		else {
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

	if (discardFrame) {
		m_forLog << " Frame discarded." << endl;
		if (m_skeleton.m_recordingOn) record(); // stop recording
	} 
	else {
		m_acceptedFrames++;
		m_forLog << " Frame accepted. " << qSetFieldWidth(4) << m_acceptedFrames;
		if (m_acceptedFrames == 1) {
			m_relativeTimeOffset = m_totalRelativeTime;
			m_lastRelativeTime = 0;
		}
		m_totalRelativeTime -= m_relativeTimeOffset;
		double interval = m_totalRelativeTime - m_lastRelativeTime;
		m_forLog << "  RelativeTime=" << qSetFieldWidth(10) << m_totalRelativeTime;
		m_forLog << "  Interval=" << qSetFieldWidth(10) << interval;
		if (m_acceptedFrames > 1) m_averageInterval = (m_averageInterval*(m_acceptedFrames - 2) + interval) / (m_acceptedFrames - 1);
		m_lastRelativeTime = m_totalRelativeTime;

		m_ticksNow = clock();
		if (m_acceptedFrames == 1) {
			m_ticksBefore = m_ticksNow;
		}

		double deltaTime = (double)(m_ticksNow - m_ticksBefore) / (double)CLOCKS_PER_SEC;
		m_totalTime += deltaTime;
		m_forLog << "  Time=" << qSetFieldWidth(5) << m_totalTime;
		m_forLog << "  deltaTime=" << qSetFieldWidth(5) << deltaTime;
		calculateFPS();
		m_skeleton.addFrame(joints, orientations, m_totalRelativeTime);
		m_forLog << "  FPS=" << m_fps << endl;
		m_ticksBefore = m_ticksNow;
	}
}
// must be called after m_acceptedFrames is incremented
void KSensor::calculateFPS() 
{
	static clock_t ticksThisTime;
	static clock_t ticksLastTime;
	static uint framesPassed;

	ticksThisTime = clock();
	// first time it is called or after resetting record variables
	if (m_acceptedFrames == 1) {
		ticksLastTime = ticksThisTime;
		framesPassed = 0;
	}

	framesPassed++;
	clock_t ticksPassed = ticksThisTime - ticksLastTime;
	double millisecondsPassed = ticksToMilliseconds(ticksPassed);

	if (millisecondsPassed > 1000.) {
		m_fps = framesPassed / (millisecondsPassed / 1000.);
		framesPassed = 0;
		ticksLastTime = ticksThisTime;
	}
}
void KSensor::record()
{
	if (!m_skeleton.m_recordingOn) {
		// Reset record related variables
		m_acceptedFrames = 0;
		m_totalTime = 0;
		m_averageInterval = 0;
		m_fps = 0;
		m_skeleton.clearSequences();

		m_skeleton.m_recordingOn = true;
		cout << "Recording started." << endl;
	}
	else {
		m_skeleton.m_recordingOn = false;
		cout << "Recording stopped." << endl;
		cout << "Average frame interval (milliseconds) = " << m_averageInterval << endl;
		m_skeleton.setTimestep(m_averageInterval);
		m_skeleton.writeTRC();
		m_skeleton.saveToBinary();
		m_skeleton.printSequence();
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