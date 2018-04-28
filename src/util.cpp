// Own
#include "util.h"

// Qt
#include <QtGui/QQuaternion>
#include <QtCore/QTextStream>

// Standard C/C++
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;

bool readFile(const char* pFileName, string& outFile)
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
QString toString(const QMatrix4x4 &m)
{
	QString qs;
	QTextStream qts(&qs);
	qts << forcesign << forcepoint << fixed;
	int w = 10;
	for (int i = 0; i < 4; i++) {
		QVector4D row(m.row(i));
		qts << qSetFieldWidth(w) << row.x() << " ";
		qts << qSetFieldWidth(w) << row.y() << " ";
		qts << qSetFieldWidth(w) << row.z() << " ";
		qts << qSetFieldWidth(w) << row.w() << " ";
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
std::ostream& operator<<(std::ostream& out, const QVector3D& v)
{
	out << setw(15) << v.x() << " " << setw(15) << v.y() << " " << setw(15) << v.z() << " ";
	return out;
}
QMatrix4x4 toQMatrix(const aiMatrix4x4& aiMat)
{
	return QMatrix4x4(
		aiMat.a1, aiMat.a2, aiMat.a3, aiMat.a4,
		aiMat.b1, aiMat.b2, aiMat.b3, aiMat.b4,
		aiMat.c1, aiMat.c2, aiMat.c3, aiMat.c4,
		aiMat.d1, aiMat.d2, aiMat.d3, aiMat.d4
	);
}
QMatrix4x4 fromScaling(const QVector3D& v)
{
	QMatrix4x4 m;
	m.scale(v);
	return m;
}
QMatrix4x4 fromRotation(const QQuaternion& q)
{
	QMatrix4x4 m;
	m.rotate(q);
	return m;
}
QMatrix4x4 fromTranslation(const QVector3D& v)
{
	QMatrix4x4 m;
	m.translate(v);
	return m;
}
QQuaternion extractQuaternion(QMatrix4x4& m)
{
	float data[9] = { m(0, 0), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 1), m(2, 0), m(2, 1), m(2, 1) };
	QMatrix3x3 M(data);
	return QQuaternion::fromRotationMatrix(M);
}