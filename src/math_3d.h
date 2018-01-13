#ifndef MATH_3D_H
#define	MATH_3D_H

// Assimp
#include <assimp\vector3.h>
#include <assimp\matrix3x3.h>
#include <assimp\matrix4x4.h>

// Qt
#include <QtGui\QQuaternion>

// Standard C/C++
#include <iostream>
#include <iomanip>

#define PI 3.141592653589
#define ToRadians(x) (float)(((x) * PI / 180.f))
#define ToDegrees(x) (float)(((x) * 180.f / PI))

using namespace std;

struct Vector3f
{
    float x;
    float y;
    float z;

    Vector3f() {}

    Vector3f(float _x, float _y, float _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }
    
    Vector3f(const float* pFloat)
    {
        x = pFloat[0];
        y = pFloat[0];
        z = pFloat[0];
    }
    
    Vector3f(float f)
    {
        x = y = z = f;
    }

    Vector3f& operator+=(const Vector3f& r)
    {
        x += r.x;
        y += r.y;
        z += r.z;

        return *this;
    }

    Vector3f& operator-=(const Vector3f& r)
    {
        x -= r.x;
        y -= r.y;
        z -= r.z;

        return *this;
    }
	
    Vector3f& operator*=(float f)
    {
        x *= f;
        y *= f;
        z *= f;

        return *this;
    }

    operator const float*() const
    {
        return &(x);
    }
    
	float DistanceFrom(const Vector3f& v) const;
    Vector3f Cross(const Vector3f& v) const;

    Vector3f& Normalize();

    void Print() const
    {
		printf("(%.3f, %.3f, %.3f)", x, y, z);
    }
	//friend std::ostream &operator<<(std::ostream &out, const Vector3f &v);
	
	/*
	string GetString() const
	{
		std::ostringstream oss;
		oss << "(" << x << ", " << y << ", " << z << ")";
		return oss.str();
	}
	//*/

	//*
	char* ToString() const
	{
		char* buffer = new char[30];
		sprintf(buffer, "(%+.3f, %+.3f, %+.3f)", x, y, z);
		return buffer;
	}
	//*/
	static const Vector3f Zero;
	static const Vector3f UnitX;
	static const Vector3f UnitY;
	static const Vector3f UnitZ;	
};


struct Vector4f
{
    float x;
    float y;
    float z;
    float w;

    Vector4f()
    {        
    }
    
    Vector4f(float _x, float _y, float _z, float _w)
    {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
    }

	Vector4f(QVector3D& v)
	{
		x = v.x();
		y = v.y();
		z = v.z();
		w = 1;
	}
    
    void Print(bool endl = true) const
    {
        printf("(%+.3f, %+.3f, %+.3f, %+.3f)", x, y, z, w);
        
        if (endl) {
            printf("\n");
        }
    }       
    
    Vector3f to3f() const
    {
        Vector3f v(x, y, z);
        return v;
    }
};



inline Vector3f operator+(const Vector3f& l, const Vector3f& r)
{
    Vector3f Ret(l.x + r.x,
                 l.y + r.y,
                 l.z + r.z);

    return Ret;
}

inline Vector3f operator-(const Vector3f& l, const Vector3f& r)
{
    Vector3f Ret(l.x - r.x,
                 l.y - r.y,
                 l.z - r.z);

    return Ret;
}

inline Vector3f operator*(const Vector3f& l, float f)
{
    Vector3f Ret(l.x * f,
                 l.y * f,
                 l.z * f);

    return Ret;
}


inline Vector4f operator/(const Vector4f& l, float f)
{
    Vector4f Ret(l.x / f,
                 l.y / f,
                 l.z / f,
                 l.w / f);
    
    return Ret;
}


struct PersProjInfo
{
	float fieldOfView;
	float aspectRatio;
	float nearPlane;
	float farPlane;
};

struct OrthoProjInfo
{
    float r;        // right
    float l;        // left
    float b;        // bottom
    float t;        // top
    float n;        // z near
    float f;        // z far
};
#endif	/* MATH_3D_H */

