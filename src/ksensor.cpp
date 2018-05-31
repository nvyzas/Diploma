// Own
#include "ksensor.h"

// Windows
#include <Windows.h>

// Standard C/C++
#include <iomanip>
#include <iostream>
#include <vector>

template <class T> void safeRelease(T **ppT)
{
	if (*ppT) {
		(*ppT)->Release();
		*ppT = NULL;
	}
}

KSensor::KSensor()
{
	cout << "KSensor constructor start." << endl;
	
	init();
	prepare();

	m_captureLog.setFileName("capture_log.txt");
	if (!m_captureLog.open(QIODevice::WriteOnly | QIODevice::Text)) {
		cout << "Could not open capture log file." << endl;
		return;
	}
	m_forCaptureLog.setFieldAlignment(QTextStream::AlignLeft);
	m_forCaptureLog.setRealNumberPrecision(10);
	m_forCaptureLog.setDevice(&m_captureLog);

	cout << "KSensor constructor end.\n" << endl;
}
KSensor::~KSensor()
{
	m_sensor->Close();
	safeRelease(&m_sensor);
	safeRelease(&m_source);
	safeRelease(&m_reader);
	m_captureLog.close();
}
bool KSensor::init() {
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_sensor);
	if (FAILED(hr)) {
		cout << "Could not get kinect sensor. hr = " <<  hr << endl;
		return false;
	}

	return true;	
}
bool KSensor::prepare()
{
	HRESULT hr;

	// Safety check
	if (!m_sensor) {
		cout << "m_sensor = NULL." << endl;
		return false;
	}

	// Open sensor
	hr = m_sensor->Open();
	if (FAILED(hr)) {
		cout << hr << "Could not open sensor. hr = " << hr << endl;
		return false;
	}
	
	// Get source
	hr = m_sensor->get_BodyFrameSource(&m_source);
	if (FAILED(hr)) {
		cout << hr << "Could not get frame source. hr = " << hr << endl;
		return false;
	}

	// Open reader
	hr = m_source->OpenReader(&m_reader);
	if (FAILED(hr)) {
		cout << hr << "Could not open reader.  hr = " << hr << endl;
		return false;
	}

	return true;
}
bool KSensor::getBodyFrame()
{
	HRESULT hr;

	// Safety checks

	if (!m_sensor) {
		cout << "m_sensor = NULL" << endl;
		return false;
	}

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

	if (!m_source) {
		cout << "m_source = NULL" << endl;
		return false;
	}

	if (!m_reader) {
		cout << "m_reader = NULL" << endl;
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
		if (!m_lastAttemptFailed) m_forCaptureLog << "Failed to acquire frame. ";
		m_consecutiveFails++;
		if (m_consecutiveFails == 15 && m_skeleton.m_recordingOn) {
			cout << "\nStopping recording due to many consecutive fails." << endl;
			record();
		}
		m_lastAttemptFailed = true;
		return false;
	} 
	else {
		if (m_consecutiveFails > 0) {
			m_forCaptureLog << " ConsecutiveFails=" << m_consecutiveFails << endl;
			m_lastAttemptFailed = false;
			m_consecutiveFails = 0;
		}

		m_forCaptureLog << "Acquired frame. ";

		// get relative time
		INT64 relativeTime;
		hr = frame->get_RelativeTime(&relativeTime);
		if (FAILED(hr)) {
			m_forCaptureLog << "Could not get relative time. hr = " << hr << endl;
			return false;
		}

		// get body frame
		IBody* bodies[BODY_COUNT] = { 0 };
		hr = frame->GetAndRefreshBodyData(ARRAY_SIZE_IN_ELEMENTS(bodies), bodies);
		if (FAILED(hr)) {
			m_forCaptureLog << "Could not get and refresh body data. hr = " << hr << endl;
			return false;
		} 

		processBodyFrameData(bodies, (double)relativeTime / 10000000.);
		
		// release resources
		for (int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(bodies); ++i) {
			safeRelease(&bodies[i]);
		}
		safeRelease(&frame);
	}

	return true;
}
void KSensor::processBodyFrameData(IBody** bodies, double timestamp)
{
	bool discardFrame = true;
	BOOLEAN isTracked;
	Joint joints[JointType_Count];
	JointOrientation orientations[JointType_Count];
	for (int i = 0; i < BODY_COUNT; i++) {
		bodies[i]->get_IsTracked(&isTracked);
		if (isTracked) {
			bodies[i]->GetJoints(JointType_Count, joints);
			bodies[i]->GetJointOrientations(JointType_Count, orientations);
			discardFrame = false;
		}
	}

	if (discardFrame) {
		m_forCaptureLog << "Frame discarded." << endl;
		if (m_skeleton.m_recordingOn) {
			cout << "Discarded frame during recording." << endl;
			record(); // stop recording
		}
	} 
	else {
		m_forCaptureLog << (m_skeleton.m_recordingOn ? "Recorded" : "Captured")  << qSetFieldWidth(4);
		static double lastTimestamp = timestamp;
		double interval = timestamp - lastTimestamp;
		m_forCaptureLog << " RelativeTime=" << qSetFieldWidth(10) << timestamp;
		m_forCaptureLog << " Interval=" << qSetFieldWidth(10) << interval;
		lastTimestamp = timestamp;

		m_skeleton.addFrame(joints, orientations, timestamp);
		calculateFPS();
		m_forCaptureLog << " FPS=" << m_fps << endl;
	}
}
// must be called after m_acceptedFrames is incremented
void KSensor::calculateFPS() 
{
	static clock_t ticksThisTime;
	static clock_t ticksLastTime = clock();
	static uint framesPassed = 0;

	ticksThisTime = clock();
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
		cout << "Recording started." << endl;
		// Reset record related variables
		m_fps = 0;
		m_skeleton.clearSequences();
		m_skeleton.m_recordingOn = true;
	} else {
		cout << "Recording stopped." << endl;
		m_skeleton.m_recordingOn = false;
		m_skeleton.m_finalizingOn = true;
	}
}
KSkeleton* KSensor::skeleton()
{
	return &m_skeleton;
}