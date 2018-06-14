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

	m_sensorLog.setFileName("sensor.log");
	if (!m_sensorLog.open(QIODevice::WriteOnly | QIODevice::Text)) {
		cout << "Could not open capture log file." << endl;
		return;
	}
	m_sensorLogData.setFieldAlignment(QTextStream::AlignLeft);
	m_sensorLogData.setRealNumberPrecision(10);
	m_sensorLogData.setDevice(&m_sensorLog);

	cout << "KSensor constructor end.\n" << endl;
}
KSensor::~KSensor()
{
	m_sensor->Close();
	safeRelease(&m_sensor);
	safeRelease(&m_source);
	safeRelease(&m_reader);
	m_sensorLog.close();
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
bool KSensor::isPrepared()
{
	HRESULT hr;

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

	return true;
}
bool KSensor::getBodyFrame(KFrame& destination)
{
	HRESULT hr;

	// get frame
	static uint consecutiveFails = 0;
	IBodyFrame* frame = NULL;
	hr = m_reader->AcquireLatestFrame(&frame);
	if (FAILED(hr)) {
		consecutiveFails++;
		return false;
	}

	// get relative time
	INT64 relativeTime;
	hr = frame->get_RelativeTime(&relativeTime);
	if (FAILED(hr)) {
		m_sensorLogData << "Could not get relative time. hr = " << hr << endl;
		return false;
	}
	double timestamp = (double)relativeTime / 10000000.;

	// get bodies
	IBody* bodies[BODY_COUNT] = { 0 };
	hr = frame->GetAndRefreshBodyData(BODY_COUNT, bodies);
	if (FAILED(hr)) {
		m_sensorLogData << "Could not get and refresh body data. hr = " << hr << endl;
		return false;
	} 

	// get data from bodies
	bool discardFrame = false;
	uint personsTracked = 0;
	BOOLEAN bodyTracked;
	Joint joints[JointType_Count];
	JointOrientation orientations[JointType_Count];
	for (int i = 0; i < BODY_COUNT; i++) {
		bodies[i]->get_IsTracked(&bodyTracked);
		if (bodyTracked) {
			personsTracked++;
			bodies[i]->GetJoints(JointType_Count, joints);
			bodies[i]->GetJointOrientations(JointType_Count, orientations);
		}
	}

	// discard if persons tracked != 1
	if (personsTracked != 1) {
		discardFrame = true;
	}

	// discard if interval > 0.1
	static double lastTimestamp = timestamp;
	double interval = timestamp - lastTimestamp;
	lastTimestamp = timestamp;
	if (interval > 0.1) {
		discardFrame = true;
	}
		
	if (discardFrame) {
		m_sensorLogData << "Status=Discarded ";
		if (m_skeleton.m_isRecording) {
			cout << "Discarded frame during recording. Recording stopped." << endl;
			m_skeleton.m_isRecording = false; // stop recording
		}
	}
	else {
		destination = m_skeleton.addFrame(joints, orientations, timestamp);
		m_sensorLogData << (m_skeleton.m_isRecording ? "Status=Recorded  " : "Status=Captured  ") << qSetFieldWidth(4);
	}
	
	m_sensorLogData << " RelativeTime=" << qSetFieldWidth(10) << timestamp;
	m_sensorLogData << " Interval=" << qSetFieldWidth(10) << interval;
	m_sensorLogData << " FPS=" << calculateFPS();
	m_sensorLogData << " ConsecutiveFails=" << qSetFieldWidth(5) << consecutiveFails << endl;
	consecutiveFails = 0;

	// release resources
	for (int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(bodies); ++i) {
		safeRelease(&bodies[i]);
	}
	safeRelease(&frame);

	return true;
}
double KSensor::calculateFPS() 
{
	static clock_t ticksThisTime;
	static clock_t ticksLastTime = clock();
	static uint framesPassed = 0;
	static double fps = 0;

	ticksThisTime = clock();
	framesPassed++;
	clock_t ticksPassed = ticksThisTime - ticksLastTime;
	double millisecondsPassed = ticksToMilliseconds(ticksPassed);

	if (millisecondsPassed > 1000.) {
		fps = framesPassed / (millisecondsPassed / 1000.);
		framesPassed = 0;
		ticksLastTime = ticksThisTime;
	}
	return fps;
}
KSkeleton* KSensor::skeleton()
{
	return &m_skeleton;
}