#ifndef MATH_3D_H
#define	MATH_3D_H

// Assimp
#include <assimp/vector3.h>
#include <assimp/matrix3x3.h>
#include <assimp/matrix4x4.h>

// Standard C/C++
#include <iostream>
#include <iomanip>
using namespace std;

#define PI 3.141592653589
#define ToRadian(x) (float)(((x) * PI / 180.0f))
#define ToDegree(x) (float)(((x) * 180.0f / PI))

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

    void Rotate(float Angle, const Vector3f& Axis);

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
    float FOV;
    float Width; 
    float Height;
    float zNear;
    float zFar;
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

struct Quaternion
{
    float x, y, z, w;
	
	Quaternion();
    Quaternion(float _x, float _y, float _z, float _w);
	
    void Normalize();
	Quaternion Inverted() const;
    Quaternion Conjugate() const;  
	void FromAxisAngle(const Vector4f &v);
	Vector4f ToAxisAngle() const; 
    Vector3f ToEulerAngles() const;
	string ToString() const;
	string ToEulerAnglesString() const;
	string ToAxisAngleString() const;
	Vector3f RotateVector(const Vector3f &v) const;

	void Print(bool endl = false) const
	{
		printf("(%+.3f, %+.3f, %+.3f, %+.3f)", x, y, z, w);

		if (endl) {
			printf("\n");
		}
	}
 };


class Matrix4f
{
public:
	float m[4][4];

	Matrix4f()
	{
	}
	// constructor from Assimp 4x4 matrix
	Matrix4f(const aiMatrix4x4& AssimpMatrix)
	{
		m[0][0] = AssimpMatrix.a1; m[0][1] = AssimpMatrix.a2; m[0][2] = AssimpMatrix.a3; m[0][3] = AssimpMatrix.a4;
		m[1][0] = AssimpMatrix.b1; m[1][1] = AssimpMatrix.b2; m[1][2] = AssimpMatrix.b3; m[1][3] = AssimpMatrix.b4;
		m[2][0] = AssimpMatrix.c1; m[2][1] = AssimpMatrix.c2; m[2][2] = AssimpMatrix.c3; m[2][3] = AssimpMatrix.c4;
		m[3][0] = AssimpMatrix.d1; m[3][1] = AssimpMatrix.d2; m[3][2] = AssimpMatrix.d3; m[3][3] = AssimpMatrix.d4;
	}
	// constructor from Assimp 3x3 matrix
	Matrix4f(const aiMatrix3x3& AssimpMatrix)
	{
		m[0][0] = AssimpMatrix.a1; m[0][1] = AssimpMatrix.a2; m[0][2] = AssimpMatrix.a3; m[0][3] = 0.0f;
		m[1][0] = AssimpMatrix.b1; m[1][1] = AssimpMatrix.b2; m[1][2] = AssimpMatrix.b3; m[1][3] = 0.0f;
		m[2][0] = AssimpMatrix.c1; m[2][1] = AssimpMatrix.c2; m[2][2] = AssimpMatrix.c3; m[2][3] = 0.0f;
		m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
	}

	Matrix4f(float a00, float a01, float a02, float a03,
		float a10, float a11, float a12, float a13,
		float a20, float a21, float a22, float a23,
		float a30, float a31, float a32, float a33)
	{
		m[0][0] = a00; m[0][1] = a01; m[0][2] = a02; m[0][3] = a03;
		m[1][0] = a10; m[1][1] = a11; m[1][2] = a12; m[1][3] = a13;
		m[2][0] = a20; m[2][1] = a21; m[2][2] = a22; m[2][3] = a23;
		m[3][0] = a30; m[3][1] = a31; m[3][2] = a32; m[3][3] = a33;
	}
	// Rotation matrix (different result than InitRotateTransform function!)
	Matrix4f(const Quaternion &q, bool rotateFunction = true )
	{	
		if (rotateFunction) InitRotateTransform1(q); else InitRotateTransform2(q);
	}
    void SetZero()
    {
        memset(m, 0, sizeof(m));
    }
   
    Matrix4f Transpose() const
    {
        Matrix4f n;
        
        for (unsigned int i = 0 ; i < 4 ; i++) {
            for (unsigned int j = 0 ; j < 4 ; j++) {
                n.m[i][j] = m[j][i];
            }
        }
        
        return n;
    }

    inline void InitIdentity()
    {
        m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = 0.0f;
        m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f; m[1][3] = 0.0f;
        m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f; m[2][3] = 0.0f;
        m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
    }

    inline Matrix4f operator*(const Matrix4f& Right) const
    {
        Matrix4f Ret;

        for (unsigned int i = 0 ; i < 4 ; i++) {
            for (unsigned int j = 0 ; j < 4 ; j++) {
                Ret.m[i][j] = m[i][0] * Right.m[0][j] +
                              m[i][1] * Right.m[1][j] +
                              m[i][2] * Right.m[2][j] +
                              m[i][3] * Right.m[3][j];
            }
        }

        return Ret;
    }
    
    Vector4f operator*(const Vector4f& v) const
    {
        Vector4f r;
        
        r.x = m[0][0]* v.x + m[0][1]* v.y + m[0][2]* v.z + m[0][3]* v.w;
        r.y = m[1][0]* v.x + m[1][1]* v.y + m[1][2]* v.z + m[1][3]* v.w;
        r.z = m[2][0]* v.x + m[2][1]* v.y + m[2][2]* v.z + m[2][3]* v.w;
        r.w = m[3][0]* v.x + m[3][1]* v.y + m[3][2]* v.z + m[3][3]* v.w;
        
        return r;
    }
    
    operator const float*() const
    {
        return &(m[0][0]);
    }
    
    void Print() const
    {		
		cout << showpos << showpoint << fixed;
		int w = 10;
        for (int i = 0 ; i < 4 ; i++) {
			cout << setw(w) << m[i][0] << " " << setw(w) << m[i][1] << " " 
				<< setw(w) << m[i][2] << " " << setw(w) << m[i][3] << " " << endl;
        }
		cout << noshowpos;
    }

	friend ostream &operator<<(ostream &out, const Matrix4f &m);
    
    float Determinant() const;
    
    Matrix4f& Invert();
	Matrix4f GetInverse() const;
	Quaternion ExtractQuaternion1() const;
	Quaternion ExtractQuaternion2() const; //using aiQuat
	void InitRotateTransform1(const Quaternion& quat);
	void InitRotateTransform2(const Quaternion& quat); //using aiQuat
	void InitScaleTransform(float ScaleX, float ScaleY, float ScaleZ);
	void InitRotateTransform(float RotateX, float RotateY, float RotateZ);
	void InitTranslateTransform(float x, float y, float z);
    void InitCameraTransform(const Vector3f& Target, const Vector3f& Up);
    void InitPersProjTransform(const PersProjInfo& p);
    void InitOrthoProjTransform(const OrthoProjInfo& p);
	Matrix4f GetRotationPart() const;
	Matrix4f GetTranslationPart() const;
	

	static Matrix4f Identity();
	static Matrix4f Zero();
};

Quaternion operator*(const Quaternion& l, const Quaternion& r);

Quaternion operator*(const Quaternion& q, const Vector3f& v);

#endif	/* MATH_3D_H */

