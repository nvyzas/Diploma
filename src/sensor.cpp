#include "sensor.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <Windows.h>
#include <Ole2.h>
#include <algorithm>

KSensor::KSensor()
{
	InitJoints();
	m_GotFrame = false;
	initializeOpenGLFunctions();
}
KSensor::~KSensor()
{
}
bool KSensor::Init() {
	//IKinectSensor* sensor;             
	if (FAILED(GetDefaultKinectSensor(&m_Sensor))) {
		cout << "Could not find kinect sensor" << endl;
		return false;
	}
	if (m_Sensor) {
		m_Sensor->get_CoordinateMapper(&m_Mapper);
		m_Sensor->Open();
		m_Sensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Depth | 
			FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Body, &m_Reader);	
		
		const int dataSize = DEPTH_WIDTH * DEPTH_HEIGHT * 3 * 4;
		glGenBuffers(1, &m_VBOid);
		glBindBuffer(GL_ARRAY_BUFFER, m_VBOid);
		glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);

		glGenBuffers(1, &m_CBOid);
		glBindBuffer(GL_ARRAY_BUFFER, m_CBOid);
		glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);
		return m_Reader;
	}
	else {
		cout << "Could not find kinect reader" << endl;
		return false;
	}
}
void KSensor::GetKinectData() {
	cout << "Getting kinect data" << endl;
	IMultiSourceFrame* frame = NULL;
	if (SUCCEEDED(m_Reader->AcquireLatestFrame(&frame))) {
		m_GotFrame = true;
		cout << "Got frame" << endl;
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
	
	IBodyFrame* bodyframe;
	IBodyFrameReference* frameref = NULL;
	frame->get_BodyFrameReference(&frameref);
	frameref->AcquireFrame(&bodyframe);
	if (frameref) frameref->Release();

	if (!bodyframe) {
		cout << "Could not get body data" << endl;
		return;
	}
	IBody* body[BODY_COUNT] = { 0 };
	bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);

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
			m_Joints[j].Orientation = Quaternion(or .Orientation.x, or .Orientation.y, or .Orientation.z, or .Orientation.w);
		}
		else {
			m_Joints[i].Position = Vector3f(jt.Position.X, jt.Position.Y, jt.Position.Z);
			m_Joints[i].Orientation = Quaternion(or .Orientation.x, or .Orientation.y, or .Orientation.z, or .Orientation.w);
		}
		const KJoint &j = m_Joints[i];
		//cout << left << setw(2) << i << ": " << left << setw(15) << j.name << " p=" << left << setw(25) << j.Position.ToString() << " q=" << j.Orientation.ToString() << j.Orientation.ToEulerAnglesString() << j.Orientation.ToAxisAngleString() << endl;
	}
	//SetCorrectedQuaternions();
	if (bodyframe) bodyframe->Release();
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
void KSensor::InitJoints() 
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
		m_Joints[i].Orientation = Quaternion(0.f, 0.f, 0.f, 1.f);
		uint p = m_Joints[i].parent;
		if (p != INVALID_JOINT_ID) (m_Joints[p].children).push_back(i);
	}
}
void KSensor::SwapSides()
{
	// Positions: left side <-> right
	swap(m_Joints[JointType_HipLeft].Position		 , m_Joints[JointType_HipRight].Position);
	swap(m_Joints[JointType_KneeLeft].Position		 , m_Joints[JointType_KneeRight].Position);
	swap(m_Joints[JointType_AnkleLeft].Position		 , m_Joints[JointType_AnkleRight].Position);
	swap(m_Joints[JointType_FootLeft].Position		 , m_Joints[JointType_FootRight].Position);
	swap(m_Joints[JointType_ShoulderLeft].Position 	 , m_Joints[JointType_ShoulderRight].Position);
	swap(m_Joints[JointType_ElbowLeft].Position		 , m_Joints[JointType_ElbowRight].Position);
	swap(m_Joints[JointType_WristLeft].Position		 , m_Joints[JointType_WristRight].Position);
	swap(m_Joints[JointType_HandLeft].Position		 , m_Joints[JointType_HandRight].Position);
	swap(m_Joints[JointType_ThumbLeft].Position		 , m_Joints[JointType_ThumbRight].Position);
	swap(m_Joints[JointType_HandTipLeft].Position	 , m_Joints[JointType_HandTipRight].Position);
	// Orientations: left side <-> right
	swap(m_Joints[JointType_HipLeft].Orientation	 , m_Joints[JointType_HipRight].Orientation);
	swap(m_Joints[JointType_KneeLeft].Orientation	 , m_Joints[JointType_KneeRight].Orientation);
	swap(m_Joints[JointType_AnkleLeft].Orientation	 , m_Joints[JointType_AnkleRight].Orientation);
	swap(m_Joints[JointType_FootLeft].Orientation	 , m_Joints[JointType_FootRight].Orientation);
	swap(m_Joints[JointType_ShoulderLeft].Orientation, m_Joints[JointType_ShoulderRight].Orientation);
	swap(m_Joints[JointType_ElbowLeft].Orientation	 , m_Joints[JointType_ElbowRight].Orientation);
	swap(m_Joints[JointType_WristLeft].Orientation	 , m_Joints[JointType_WristRight].Orientation);
	swap(m_Joints[JointType_HandLeft].Orientation	 , m_Joints[JointType_HandRight].Orientation);
	swap(m_Joints[JointType_ThumbLeft].Orientation	 , m_Joints[JointType_ThumbRight].Orientation);
	swap(m_Joints[JointType_HandTipLeft].Orientation , m_Joints[JointType_HandTipRight].Orientation);	
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
		cout << setw(15) << j.name << "p: " << setw(25) << j.Position.ToString() << " q: " << j.Orientation.ToString() << j.Orientation.ToEulerAnglesString() << j.Orientation.ToAxisAngleString() << endl;
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
	const Quaternion &q = m_Joints[m_ActiveJoint].Orientation;
	Vector3f v;

	v = q.RotateVector(Vector3f(1, 0, 0));
	v += p;
	glColor3f(0xFF, 0xFF, 0);
	glVertex3f(p.x, p.y, p.z);
	glVertex3f(v.x, v.y, v.z);

	v = q.RotateVector(Vector3f(0, 1, 0));
	v += p;
	glColor3f(0, 0xFF, 0xFF);
	glVertex3f(p.x, p.y, p.z);
	glVertex3f(v.x, v.y, v.z);

	v = q.RotateVector(Vector3f(0, 0, 1));
	v += p;
	glColor3f(0xFF, 0, 0xFF);
	glVertex3f(p.x, p.y, p.z);
	glVertex3f(v.x, v.y, v.z);
	glEnd();
}
void KSensor::NextJoint(int step)
{
	m_ActiveJoint = Mod(m_ActiveJoint, JointType_Count, step);
	const KJoint &j = m_Joints[m_ActiveJoint];
	cout << left << setw(2) << m_ActiveJoint << ": " << setw(15) << j.name << " p=" << left << setw(25) << j.Position.ToString() << " q=" << j.Orientation.ToString() << j.Orientation.ToEulerAnglesString() << j.Orientation.ToAxisAngleString() << endl;
}
