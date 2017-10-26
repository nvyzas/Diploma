#ifndef SENSOR_H
#define SENSOR_H

// Project
#include "math_3d.h"
#include "util.h"

// Kinect
#include <Kinect.h>

// Qt
class QFile;

// Standard C/C++
#include <string>
#include <vector>

#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424
#define COLOR_WIDTH 1920
#define COLOR_HEIGHT 1080
#define INVALID_JOINT_ID 123

using namespace std;

struct KJoint
{
	string name;
	Vector3f Position;
	QQuaternion Orientation;
	QQuaternion relOrientation; // relative orientation
	QQuaternion corOrientation; // corrected orientation
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
	void GetDepthData(IMultiSourceFrame* frame, GLubyte* dest);
	void GetRGBData(IMultiSourceFrame* frame, GLubyte* dest);
	void GetBodyData(IMultiSourceFrame* frame);
	const KJoint* getKJoints() const;
	void SwapSides();
	void PrintInfo() const;
	void PrintJointHierarchy() const;
	void PrintJointData() const;
	void DrawActiveJoint();
	void DrawSkeleton(uint id);
	void DrawCloud();
	bool createTRC();

	bool m_InvertedSides;
	bool m_GotFrame;
private:
	void initJoints();
	void NextJoint(int step);
	bool addMarkerData(TIMESPAN *time);

	KJoint m_Joints[JointType_Count];
	IKinectSensor* m_Sensor;
	IMultiSourceFrameReader* m_Reader;   // Kinect data source
	ICoordinateMapper* m_Mapper;         // Converts between depth, color, and 3d coordinates
	
	// Intermediate Buffers
	uint m_RGBimage[COLOR_WIDTH * COLOR_HEIGHT * 4];				     // Stores RGB color image
	ColorSpacePoint m_Depth2RGB[DEPTH_WIDTH * DEPTH_HEIGHT];             // Maps depth pixels to rgb pixels
	CameraSpacePoint m_Depth2xyz[DEPTH_WIDTH * DEPTH_HEIGHT];			 // Maps depth pixels to 3d coordinates

	// OpenGL Buffers
	GLuint m_VBOid;
	GLuint m_CBOid;

	uint m_ActiveJoint = JointType_SpineBase;
	QFile *m_trcFile;
	QString m_markerData;
	uint m_numFrames;
	uint m_numMarkers;
};
#endif /* SENSOR_H */
