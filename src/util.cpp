// Own
#include "util.h"

// Qt
#include <QtGui/QQuaternion>

// Standard C/C++
#include <iostream>
#include <fstream>
using namespace std;

bool ReadFile(const char* pFileName, string& outFile)
{
    ifstream f(pFileName);
    
    bool ret = false;
    
    if (f.is_open()) {
        string line;
        while (getline(f, line)) {
            outFile.append(line);
            outFile.append("\n");
        }
        
        f.close();
        
        ret = true;
    }
    else {
		cout << "Error reading file: " << *pFileName << endl;
    }
    
    return ret;
}
uint Mod(uint start, uint edge, int step)
{
	if (step >= 0) return (start + step) % edge;
	else {
		int n = start + step;
		if (n > 0) return n;
		else return edge + step;
	}
}
// x, y, z, w
QString toString(const QQuaternion &q)
{
	char buf[64];
	sprintf(buf, "(%+2.3f, %+.3f, %+.3f, %+.3f) ", q.x(), q.y(), q.z(), q.scalar());
	return QString(buf);
}
// xDegrees, yDegrees, zDegrees
QString toStringEulerAngles(const QQuaternion &q)
{
	char buf[64];
	const QVector3D &v = q.toEulerAngles();
	sprintf(buf, "(%+6.1f, %+6.1f, %+6.1f) ", v.x(), v.y(), v.z());
	return QString(buf);
}
// [xAxis, yAxis, zAxis], angleDegrees
QString toStringAxisAngle(const QQuaternion &q)
{
	char buf[64];
	float x, y, z, angle;
	q.getAxisAndAngle(&x, &y, &z, &angle);
	sprintf(buf, "([%+.3f, %+.3f, %+.3f], %+6.1f) ", x, y, z, angle);
	return QString(buf);
}
// x, y, z
QString toStringCartesian(const QVector3D &v)
{
	char buf[64];
	sprintf(buf, "(%+.3f, %+.3f, %+.3f)", v.x(), v.y(), v.z());
	return QString(buf);
}
// rho, theta, phi
QString toStringSpherical(const QVector3D &v)
{
	char buf[64];
	sprintf(buf, "(%+.3f, %+6.1f, %+6.1f)", v.x(), v.y(), v.z());
	return QString(buf);
}

// clamps angle between 0 and 360
float wrapAngle(float angle, float limit)
{
	return angle - limit * floor(angle / limit);
}

// units/(units/time) => time (seconds) * 1000 = milliseconds
double ticksToMilliseconds(clock_t ticks) {
	return (ticks / (double)CLOCKS_PER_SEC)*1000.;
}
