#pragma once
#include "math_3d.h"
#include "util.h"
#include <Kinect.h>
#include <string>
#include <vector>

#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424
#define COLOR_WIDTH 1920
#define COLOR_HEIGHT 1080
#define INVALID_JOINT_ID 123

using namespace std;
typedef unsigned int uint;

struct KJoint
{
	string name;
	Vector3f Position;
	Quaternion Orientation;
	Quaternion relOrientation; // relative orientation
	Quaternion corOrientation; // corrected orientation
	uint id;
	uint idOpposite; // id of corresponding opposite side joint (eg. left->right)
	uint parent;
	vector<uint> children;
	
	KJoint()
	{
	}
	KJoint(string _name, uint _parent, uint _idOpposite)
	{
		name = _name;
		parent = _parent;
		idOpposite = _idOpposite;
	}

};

class KSensor : protected OPENGL_FUNCTIONS
{
public:
	KSensor();
	~KSensor();
	bool Init();
	void GetKinectData();	
	void PrintInfo() const;
	void PrintJointHierarchy() const;
	void PrintJointData() const;
	bool m_InvertedSides;
	KJoint m_Joints[JointType_Count];
	bool m_GotFrame;
	void SwapSides();
	void DrawActiveJoint();
	void DrawSkeleton(uint id);
	void DrawCloud();

private:
	void InitJoints();	
	void GetDepthData(IMultiSourceFrame* frame, GLubyte* dest);
	void GetRGBData(IMultiSourceFrame* frame, GLubyte* dest);
	void GetBodyData(IMultiSourceFrame* frame);
	void NextJoint(int step);
	

	IKinectSensor* m_Sensor;
	IMultiSourceFrameReader* m_Reader;   // Kinect data source
	ICoordinateMapper* m_Mapper;         // Converts between depth, color, and 3d coordinates
	
	// Intermediate Buffers
	uint m_RGBimage[COLOR_WIDTH * COLOR_HEIGHT * 4];  // Stores RGB color image
	ColorSpacePoint m_Depth2RGB[DEPTH_WIDTH * DEPTH_HEIGHT];             // Maps depth pixels to rgb pixels
	CameraSpacePoint m_Depth2xyz[DEPTH_WIDTH * DEPTH_HEIGHT];			 // Maps depth pixels to 3d coordinates

	// OpenGL Buffers
	GLuint m_VBOid;
	GLuint m_CBOid;

	uint m_ActiveJoint = JointType_SpineBase;	
};

