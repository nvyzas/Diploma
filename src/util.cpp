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
string printQuaternion1(const QQuaternion &q)
{
	char buf[64];
	sprintf(buf, "(%+.2f, %+.2f, %+.2f, %+.2f) ", q.x(), q.y(), q.z(), q.scalar());
	return string(buf);
}
string printQuaternion2(const QQuaternion &q)
{
	char buf[64];
	const QVector3D &v = q.toEulerAngles();
	sprintf(buf, "(%+6.1f, %+6.1f, %+6.1f) ", v.x(), v.y(), v.z());
	return string(buf);
}
string printQuaternion3(const QQuaternion &q)
{
	char buf[64];
	float x, y, z, angle;
	q.getAxisAndAngle(&x, &y, &z, &angle);
	sprintf(buf, "([%+.2f, %+.2f, %+.2f], %+6.1f) ", x, y, z, angle);
	return string(buf);
}
