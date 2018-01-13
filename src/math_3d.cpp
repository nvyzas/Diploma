// Own
#include "math_3d.h"

// Assimp
#include <assimp/quaternion.inl>

// Standard C/C++
#include <cassert>


const Vector3f Vector3f::Zero = Vector3f(0.0f, 0.0f, 0.0f);
const Vector3f Vector3f::UnitX = Vector3f(1.0f, 0.0f, 0.0f);
const Vector3f Vector3f::UnitY = Vector3f(1.0f, 0.0f, 0.0f);
const Vector3f Vector3f::UnitZ = Vector3f(1.0f, 0.0f, 0.0f);
Vector3f Vector3f::Cross(const Vector3f& v) const
{
    const float _x = y * v.z - z * v.y;
    const float _y = z * v.x - x * v.z;
    const float _z = x * v.y - y * v.x;

    return Vector3f(_x, _y, _z);
}
Vector3f& Vector3f::Normalize()
{
    const float Length = sqrtf(x * x + y * y + z * z);

    x /= Length;
    y /= Length;
    z /= Length;

    return *this;
}
float Vector3f::DistanceFrom(const Vector3f& v) const
{
	return sqrt(pow((double(this->x - v.x)), 2) + pow((double(this->y - v.y)), 2) + pow((double(this->z - v.z)), 2));
}
// #! does not flush
//ostream &operator<<(ostream &out, const Matrix4f &m)
//{
//	out << showpos << showpoint << fixed;
//	int w = 10;
//	for (int i = 0; i < 4; i++) {
//		out << setw(w) << m.m[i][0] << " "; 
//		out << setw(w) << m.m[i][1] << " ";
//		out	<< setw(w) << m.m[i][2] << " ";
//		out << setw(w) << m.m[i][3] << " ";
//	}
//	out << "\n";
//	out << noshowpos;
//	return out;
//}
