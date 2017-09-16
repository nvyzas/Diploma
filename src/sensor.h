#pragma once
#include "math_3d.h"
#include <string>
#include <vector>
#include <Kinect.h>
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QOpenGLFunctions_4_5_Compatibility>

#define WIDTH 512
#define HEIGHT 424
#define COLORWIDTH 1920
#define COLORHEIGHT 1080
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

class KSensor : public QOpenGLFunctions_4_5_Compatibility
{
public:
	KSensor();
	~KSensor();
	bool Init();
	void GetKinectData();
	void DrawKinectData();
	void PrintInfo() const;
	void PrintJointHierarchy() const;
	void PrintJointData() const;
	bool m_InvertedSides;
	KJoint m_Joints[JointType_Count];
	bool m_GotFrame;
	void SwapSides();

private:
	void InitJoints();	
	void SetRelativeQuaternions(uint id);
	void SetCorrectedQuaternions();
	void GetDepthData(IMultiSourceFrame* frame, GLubyte* dest);
	void GetRGBData(IMultiSourceFrame* frame, GLubyte* dest);
	void GetBodyData(IMultiSourceFrame* frame);
	
	IKinectSensor* m_Sensor;
	IMultiSourceFrameReader* m_Reader;   // Kinect data source
	ICoordinateMapper* m_Mapper;         // Converts between depth, color, and 3d coordinates
	
	// Intermediate Buffers
	uint m_RGBimage[COLORWIDTH * COLORHEIGHT * 4];  // Stores RGB color image
	ColorSpacePoint m_Depth2RGB[WIDTH * HEIGHT];             // Maps depth pixels to rgb pixels
	CameraSpacePoint m_Depth2xyz[WIDTH * HEIGHT];			 // Maps depth pixels to 3d coordinates

	// OpenGL Buffers
	GLuint m_VBOid;
	GLuint m_CBOid;
};

