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
	m_numFrames = 0;
}
KSensor::~KSensor()
{
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
		if (!isOpen) cout << "Sensor is not open. " << endl;
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

	safeRelease(&m_source);

	// Must open reader first for source to be active
	BOOLEAN isActive = false;
	hr = m_source->get_IsActive(&isActive);
	if (SUCCEEDED(hr)) {
		if (!isActive) cout << "Source is not active. " << endl;
	}else {
		cout << "Could not specify if source is active. hr = " << hr << endl;
	}

	return true;
}
void KSensor::update()
{
	if (!m_sensor) {
		cout << "m_sensor = NULL" << endl;
		return;
	}
	if (!m_source) {
		cout << "m_source = NULL" << endl;
		return;
	}
	if (!m_reader) {
		cout << "m_reader = NULL" << endl;
		return;
	}
	HRESULT hr;
	BOOLEAN isOpen = false;
	hr = m_sensor->get_IsOpen(&isOpen);
	if (SUCCEEDED(hr)) {
		if (!isOpen) cout << "Sensor is not open. " << endl;
	}
	else {
		cout << "Could not specify if sensor is open. hr = " << hr << endl;
	}
	BOOLEAN isActive = false;
	hr = m_source->get_IsActive(&isActive);
	if (SUCCEEDED(hr)) {
		if (!isActive) cout << "Source is not active. " << endl;
	}
	else {
		cout << "Could not specify if source is active. hr = " << hr << endl;
	}

	IBodyFrame* frame = NULL;
	hr = m_reader->AcquireLatestFrame(&frame);
	if (FAILED(hr)) {
		cout << "Failed to acquire latest frame. hr = " << hr << endl;
		return;
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
	}

	safeRelease(&frame);
	return;
}
void KSensor::processBodyFrameData(INT64 timestamp, int bodyCount, IBody** bodies)
{
	// Body tracking variables
	BOOLEAN tracked;
	Joint joints[JointType_Count];
	JointOrientation orients[JointType_Count];
	for (int i = 0; i < BODY_COUNT; i++) {
		bodies[i]->get_IsTracked(&tracked);
		if (tracked) {
			bodies[i]->GetJoints(JointType_Count, joints);
			bodies[i]->GetJointOrientations(JointType_Count, orients);
			break;
		}
	}
	for (uint i = 0; i < JointType_Count; i++) {
		const Joint &jt = joints[i];
		const JointOrientation & or = orients[i];
		//cout << "JointType: " << setw(2) << jt.JointType << "\tJointOrientationType: " << setw(2) << or.JointType << endl;
		if (m_InvertedSides) {
			uint j = m_Joints[i].idOpposite;
			m_Joints[j].Position = Vector3f(jt.Position.X, jt.Position.Y, jt.Position.Z);
			m_Joints[j].Orientation = QQuaternion(or .Orientation.w, or .Orientation.x, or .Orientation.y, or .Orientation.z);
		}
		else {
			m_Joints[i].Position = Vector3f(jt.Position.X, jt.Position.Y, jt.Position.Z);
			m_Joints[i].Orientation = QQuaternion(or .Orientation.w, or .Orientation.x, or .Orientation.y, or .Orientation.z);
		}
		//cout << left << setw(2) << i << ": " << left << setw(15) << j.name << " p=" << left << setw(25) << j.Position.ToString() << " q=" << j.Orientation.ToString() << j.Orientation.ToEulerAnglesString() << j.Orientation.ToAxisAngleString() << endl;
	}
	addMarkerData();
}
bool KSensor::addMarkerData()
{
	cout << "Adding frame to marker data string." << endl;
	m_numFrames++;
	QTextStream qts(&m_markerData);
	qts.setFieldWidth(12); // width for 4 important digits + 6 decimals + comma + minus sign
	qts << QString("\n") << m_numFrames << QString("\t") << (time ? *time : 0);
	for (int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_Joints); i++) {
		qts << "\t" << m_Joints[i].Position.x;
		qts << "\t" << m_Joints[i].Position.y;
		qts << "\t" << m_Joints[i].Position.z;
	}
	//m_markerData = qts.readAll();
	/*m_markerData.append(QString("\n") + m_numFrames + QString("\t") + (time ? *time : 0));
	for (int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_Joints); i++) {
	m_markerData.append("\t" + QString(m_Joints[i].Position.x);
	m_markerData.append("\t" + QString(m_Joints[i].Position.y);
	m_markerData.append("\t" + QString(m_Joints[i].Position.z);
	}*/

	return true;
}
bool KSensor::initOGLResources()
{
	if (m_sensor) {
		bool ret = initializeOpenGLFunctions();
		const int dataSize = DEPTH_WIDTH * DEPTH_HEIGHT * 3 * 4;
		glGenBuffers(1, &m_VBOid);
		glBindBuffer(GL_ARRAY_BUFFER, m_VBOid);
		glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);

		glGenBuffers(1, &m_CBOid);
		glBindBuffer(GL_ARRAY_BUFFER, m_CBOid);
		glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);
		return ret;
	}
	return false;
}
void KSensor::GetKinectData() {
	cout << "Getting kinect data" << endl;
	IMultiSourceFrame* frame = NULL;
	if (SUCCEEDED(m_Reader->AcquireLatestFrame(&frame))) {
		m_GotFrame = true;
		cout << "Got frame from reader" << endl;
		GLubyte* ptr;
		glBindBuffer(GL_ARRAY_BUFFER, m_VBOid);
		ptr = (GLubyte*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		if (ptr) {
			cout << "Getting depth data" << endl;
			GetDepthData(frame, ptr);
		}
		glUnmapBuffer(GL_ARRAY_BUFFER);
		glBindBuffer(GL_ARRAY_BUFFER, m_CBOid);
		ptr = (GLubyte*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		if (ptr) {
			cout << "Getting RGB data" << endl;
			GetRGBData(frame, ptr);
		}
		glUnmapBuffer(GL_ARRAY_BUFFER);
		cout << "Getting body data" << endl;
		GetBodyData(frame);
	}
	else {
		m_GotFrame = false;
		cout << "Could not get frame" << endl;		
	}
	if (frame) frame->Release();
}
void KSensor::GetDepthData(IMultiSourceFrame* frame, GLubyte* dest) {
	IDepthFrame* depthframe;
	IDepthFrameReference* frameref = NULL;
	frame->get_DepthFrameReference(&frameref);
	frameref->AcquireFrame(&depthframe);
	if (frameref) frameref->Release();

	if (!depthframe) {
		cout << "Could not get depth data" << endl;
		return;
	}

	// Get data from frame
	unsigned int sz;
	unsigned short* buf;
	depthframe->AccessUnderlyingBuffer(&sz, &buf);

	// Write vertex coordinates
	m_Mapper->MapDepthFrameToCameraSpace(DEPTH_WIDTH*DEPTH_HEIGHT, buf, DEPTH_WIDTH*DEPTH_HEIGHT, m_Depth2xyz);
	float* fdest = (float*)dest;
	for (int i = 0; i < sz; i++) {
		*fdest++ = m_Depth2xyz[i].X;
		*fdest++ = m_Depth2xyz[i].Y;
		*fdest++ = m_Depth2xyz[i].Z;
	}

	// Fill in depth2rgb map
	m_Mapper->MapDepthFrameToColorSpace(DEPTH_WIDTH*DEPTH_HEIGHT, buf, DEPTH_WIDTH*DEPTH_HEIGHT, m_Depth2RGB);
	if (depthframe) depthframe->Release();
}
void KSensor::GetRGBData(IMultiSourceFrame* frame, GLubyte* dest) {
	IColorFrame* colorframe;
	IColorFrameReference* frameref = NULL;
	frame->get_ColorFrameReference(&frameref);
	frameref->AcquireFrame(&colorframe);
	if (frameref) frameref->Release();

	if (!colorframe) {
		cout << "Could not get color data" << endl;
		return;
	}

	// Get data from frame
	colorframe->CopyConvertedFrameDataToArray(COLOR_WIDTH * COLOR_HEIGHT * 4, (BYTE*)m_RGBimage, ColorImageFormat_Rgba);

	// Write color array for vertices
	float* fdest = (float*)dest;
	for (int i = 0; i < DEPTH_WIDTH*DEPTH_HEIGHT; i++) {
		ColorSpacePoint p = m_Depth2RGB[i];
		// Check if color pixel coordinates are in bounds
		if (p.X < 0 || p.Y < 0 || p.X > COLOR_WIDTH || p.Y > COLOR_HEIGHT) {
			*fdest++ = 0;
			*fdest++ = 0;
			*fdest++ = 0;
		}
		else {
			int idx = (int)p.X + COLOR_WIDTH*(int)p.Y;
			*fdest++ = m_RGBimage[4 * idx + 0] / 255.;
			*fdest++ = m_RGBimage[4 * idx + 1] / 255.;
			*fdest++ = m_RGBimage[4 * idx + 2] / 255.;
		}
	}

	if (colorframe) colorframe->Release();
}
void KSensor::GetBodyData(IMultiSourceFrame* frame) {
	IBodyFrameReference* frameref = NULL;
	frame->get_BodyFrameReference(&frameref);
	IBodyFrame* bodyframe = NULL;
	frameref->AcquireFrame(&bodyframe);
	TIMESPAN *frameTime = NULL;
	frameref->get_RelativeTime(frameTime);
	if (frameref) frameref->Release();

	if (!frameTime) cout << "Could not get frame time" << endl;
	if (!bodyframe) {
		cout << "Could not get body data" << endl;
		return;
	}

	IBody* body[BODY_COUNT] = { 0 };
	if (FAILED(bodyframe->GetAndRefreshBodyData(BODY_COUNT, body))) cout << "Could not get and refresh body data" << endl;;

	// Body tracking variables
	BOOLEAN tracked;							// Whether we see a body
	Joint joints[JointType_Count];				
	JointOrientation orients[JointType_Count];
	for (int i = 0; i < BODY_COUNT; i++) {
		body[i]->get_IsTracked(&tracked);
		if (tracked) {
			body[i]->GetJoints(JointType_Count, joints);
			body[i]->GetJointOrientations(JointType_Count, orients);
			break;
		}
	}
	for (uint i = 0; i < JointType_Count; i++) {
		const Joint &jt = joints[i];
		const JointOrientation &or = orients[i];
		//cout << "JointType: " << setw(2) << jt.JointType << "\tJointOrientationType: " << setw(2) << or.JointType << endl;
		if (m_InvertedSides) {
			uint j = m_Joints[i].idOpposite;
			m_Joints[j].Position = Vector3f(jt.Position.X, jt.Position.Y, jt.Position.Z);
			m_Joints[j].Orientation = QQuaternion(or.Orientation.w, or.Orientation.x, or.Orientation.y, or.Orientation.z);
		}
		else {
			m_Joints[i].Position = Vector3f(jt.Position.X, jt.Position.Y, jt.Position.Z);
			m_Joints[i].Orientation = QQuaternion(or.Orientation.w, or.Orientation.x, or.Orientation.y, or.Orientation.z);
		}
		//cout << left << setw(2) << i << ": " << left << setw(15) << j.name << " p=" << left << setw(25) << j.Position.ToString() << " q=" << j.Orientation.ToString() << j.Orientation.ToEulerAnglesString() << j.Orientation.ToAxisAngleString() << endl;
	}
	if (bodyframe) {
		addMarkerData();
		bodyframe->Release();
	}
	//SetCorrectedQuaternions();
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
		cout << " Parent: " << setw(20) << left << m_Joints[p].name;
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
void KSensor::DrawCloud() {
	// Set up array buffers
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, m_VBOid);
	glVertexPointer(3, GL_FLOAT, 0, NULL);

	glBindBuffer(GL_ARRAY_BUFFER, m_CBOid);
	glColorPointer(3, GL_FLOAT, 0, NULL);

	glPointSize(1.f);
	glDrawArrays(GL_POINTS, 0, DEPTH_WIDTH*DEPTH_HEIGHT);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}
void KSensor::initJoints() 
{
	// Set parents
	// core
	m_Joints[JointType_SpineBase]		= KJoint("SpineBase"	, INVALID_JOINT_ID		 , JointType_SpineBase); // root
	m_Joints[JointType_SpineMid]		= KJoint("SpineMid"		, JointType_SpineBase	 , JointType_SpineMid);
	m_Joints[JointType_SpineShoulder]   = KJoint("SpineShoulder", JointType_SpineMid	 , JointType_SpineShoulder);
	m_Joints[JointType_Neck]			= KJoint("Neck"			, JointType_SpineShoulder, JointType_Neck);
	m_Joints[JointType_Head]			= KJoint("Head"			, JointType_Neck		 , JointType_Head);
	// left side
	m_Joints[JointType_HipLeft]		    = KJoint("HipLeft"	    , JointType_SpineBase	 , JointType_HipRight);
	m_Joints[JointType_KneeLeft]		= KJoint("KneeLeft"		, JointType_HipLeft		 , JointType_KneeRight);
	m_Joints[JointType_AnkleLeft]		= KJoint("AnkleLeft"	, JointType_KneeLeft	 , JointType_AnkleRight);
	m_Joints[JointType_FootLeft]		= KJoint("FootLeft"		, JointType_AnkleLeft	 , JointType_FootRight);
	m_Joints[JointType_ShoulderLeft]	= KJoint("ShoulderLeft" , JointType_SpineShoulder, JointType_ShoulderRight);
	m_Joints[JointType_ElbowLeft]		= KJoint("ElbowLeft"	, JointType_ShoulderLeft , JointType_ElbowRight);
	m_Joints[JointType_WristLeft]		= KJoint("WristLeft"	, JointType_ElbowLeft    , JointType_WristRight);
	m_Joints[JointType_HandLeft]		= KJoint("HandLeft"		, JointType_WristLeft    , JointType_HandRight);
	m_Joints[JointType_ThumbLeft]		= KJoint("ThumbLeft"	, JointType_HandLeft     , JointType_ThumbRight);
	m_Joints[JointType_HandTipLeft]		= KJoint("HandTipLeft"	, JointType_HandLeft     , JointType_HandTipRight);
	// right side
	m_Joints[JointType_HipRight]		= KJoint("HipRight"		, JointType_SpineBase	 , JointType_HipLeft);
	m_Joints[JointType_KneeRight]		= KJoint("KneeRight"	, JointType_HipRight	 , JointType_KneeLeft);
	m_Joints[JointType_AnkleRight]		= KJoint("AnkleRight"	, JointType_KneeRight	 , JointType_AnkleLeft);
	m_Joints[JointType_FootRight]		= KJoint("FootRight"	, JointType_AnkleRight	 , JointType_FootLeft);
	m_Joints[JointType_ShoulderRight]	= KJoint("ShoulderRight", JointType_SpineShoulder, JointType_ShoulderLeft);
	m_Joints[JointType_ElbowRight]		= KJoint("ElbowRight"	, JointType_ShoulderRight, JointType_ElbowLeft);
	m_Joints[JointType_WristRight]		= KJoint("WristRight"	, JointType_ElbowRight	 , JointType_WristLeft);
	m_Joints[JointType_HandRight]		= KJoint("HandRight"	, JointType_WristRight	 , JointType_HandLeft);
	m_Joints[JointType_ThumbRight]		= KJoint("ThumbRight"	, JointType_HandRight	 , JointType_ThumbLeft);
	m_Joints[JointType_HandTipRight]	= KJoint("HandTipRight"	, JointType_HandRight	 , JointType_HandTipLeft);

	// Set children
	for (uint i = 0; i < JointType_Count; i++) {
		m_Joints[i].Position = Vector3f(0.f, 0.f, 0.f);
		m_Joints[i].Orientation = QQuaternion(1.f, 0.f, 0.f, 0.f);
		uint p = m_Joints[i].parent;
		if (p != INVALID_JOINT_ID) (m_Joints[p].children).push_back(i);
	}
}

/*
void KSensor::processBodyFrame() {

	//calculate elapsed time from the previus frame
	m_frameEnd = clock();

	double elapsed_secs = double(m_frameEnd - m_frameBegin) / CLOCKS_PER_SEC;

	IBodyFrame *skeletonFrame = { 0 };

	HRESULT hr = sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);

	if (FAILED(hr)) {
		return;
	}

	// Smooth the skeleton joint positions
	if (FILTER_MODE == 1) {
		sensor->NuiTransformSmooth(&skeletonFrame, &defaultParams);
	}
	else if (FILTER_MODE == 2) {
		sensor->NuiTransformSmooth(&skeletonFrame, &somewhatLatentParams);
	}
	else if (FILTER_MODE == 3) {
		sensor->NuiTransformSmooth(&skeletonFrame, &verySmoothParams);
	}


	skeleton.setFloor(-skeletonFrame.vFloorClipPlane.w / skeletonFrame.vFloorClipPlane.y);
	//std::cout << skeletonFrame.vFloorClipPlane.w/skeletonFrame.vFloorClipPlane.y<<"\n";

	for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {

		NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

		NUI_SKELETON_BONE_ORIENTATION boneOrientations[NUI_SKELETON_POSITION_COUNT];
		NuiSkeletonCalculateBoneOrientations(&skeletonFrame.SkeletonData[i], boneOrientations);

		if (NUI_SKELETON_TRACKED == trackingState) {
			// We're tracking the skeleton, draw it
			SYSTEMTIME t;
			GetLocalTime(&t);

			skeleton.addFrame(skeletonFrame.SkeletonData[i], boneOrientations, elapsed_secs, t);
		}
		else if (NUI_SKELETON_POSITION_ONLY == trackingState) {
			// we've only received the center point of the skeleton, draw that
			///*D2D1_ELLIPSE ellipse = D2D1::Ellipse(
			//SkeletonToScreen(skeletonFrame.SkeletonData[i].Position, width, height),
			//g_JointThickness,
			//g_JointThickness
			//);

			//m_pRenderTarget->DrawEllipse(ellipse, m_pBrushJointTracked);
			SYSTEMTIME t;
			GetLocalTime(&t);
			skeleton.addFrame(skeletonFrame.SkeletonData[i], boneOrientations, elapsed_secs, t);
		}
	}

	calculateFPS();

	//for getting frame data time
	frameBegin = clock();
}

void Kinect::calculateFPS() {
	//  Increase frame count
	frameCount++;

	currentTime = clock();

	//  Calculate time passed
	int timeInterval = currentTime - previousTime;

	if (timeInterval > 1000) {
		//  calculate the number of frames per second
		fps = frameCount / (timeInterval / 1000.0f);

		//  Set time
		previousTime = currentTime;

		//  Reset frame count
		frameCount = 0;
	}
}
//*/

void KSensor::NextJoint(int step)
{
	m_ActiveJoint = Mod(m_ActiveJoint, JointType_Count, step);
	const KJoint &j = m_Joints[m_ActiveJoint];
	cout << left << setw(2) << m_ActiveJoint << ": " << setw(15) << j.name << " p=" << left << setw(25) << j.Position.ToString() << " q=" << printQuaternion1(j.Orientation) << printQuaternion2(j.Orientation) << printQuaternion3(j.Orientation) << endl;
}
bool KSensor::createTRC()
{
	m_trcFile = new QFile("joint_positions.trc");
	if (!m_trcFile->open(QIODevice::WriteOnly | QIODevice::Text)) return false;
	QTextStream out(m_trcFile);
	out << "PathFileType\t4\t(X / Y / Z)\tjoint_positions.trc\n";
	out << "DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames\n";
	out << "Frame#\tTime";
	for (int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_Joints); i++) {
		out << "\t" << QString::fromStdString(m_Joints[i].name) << "\t\t";
	}
	out << "\n\t";
	for (int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_Joints); i++) {
		out << "\t" << "X" << (i + 1) << "\t" << "Y" << (i + 1) << "\t" << "Z" << (i + 1);
	}
	out << "\n";
	out << m_markerData;
	m_trcFile->close();
	return true;
}