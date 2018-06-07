#ifndef KSENSOR_H
#define KSENSOR_H

// Project
#include "util.h"
#include "kskeleton.h"

// Kinect
#include <Kinect.h>

// Qt
#include <QtCore/QFile>

// Standard C/C++
#include <string>
#include <vector>

class KSensor
{
public:
	KSensor();
	~KSensor();
	bool init();
	bool prepare();
	bool getBodyFrame(KFrame& destination);

	double calculateFPS();

	KSkeleton *skeleton();
private:
	IKinectSensor *m_sensor = nullptr;
	IBodyFrameSource *m_source = nullptr;
	IBodyFrameReader *m_reader = nullptr;

	QFile m_sensorLog;
	QTextStream m_sensorLogData;

	KSkeleton m_skeleton;
};
#endif /* SENSOR_H */
