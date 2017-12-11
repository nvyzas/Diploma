// Own
#include "util.h"

// Qt
#include <QtGui/QQuaternion>
#include <QtCore/QTextStream>

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
QString toString(const Matrix4f &m)
{
	QString qs;
	QTextStream qts(&qs);
	qts << forcesign << forcepoint << fixed;
	int w = 10;
	for (int i = 0; i < 4; i++) {
		qts << qSetFieldWidth(w) << m.m[i][0] << " ";
		qts << qSetFieldWidth(w) << m.m[i][1] << " ";
		qts << qSetFieldWidth(w) << m.m[i][2] << " ";
		qts << qSetFieldWidth(w) << m.m[i][3] << " ";
		qts << "\n";
	}
	qts << flush;
	return qs;
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
	const QVector3D &v = q.toEulerAngles(); // z->x->y
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
//void matrixCopy(QMatrix4x4& qMat, aiMatrix4x4& aiMat)
//{
//	m[0][0] = aiMat.a1; m[0][1] = aiMat.a2; m[0][2] = aiMat.a3; m[0][3] = aiMat.a4;
//	m[1][0] = aiMat.b1; m[1][1] = aiMat.b2; m[1][2] = aiMat.b3; m[1][3] = aiMat.b4;
//	m[2][0] = aiMat.c1; m[2][1] = aiMat.c2; m[2][2] = aiMat.c3; m[2][3] = aiMat.c4;
//	m[3][0] = aiMat.d1; m[3][1] = aiMat.d2; m[3][2] = aiMat.d3; m[3][3] = aiMat.d4;
//}