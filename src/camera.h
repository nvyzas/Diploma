#ifndef CAMERA_H
#define	CAMERA_H

#include "math_3d.h"
#include "util.h"

class Camera : protected OPENGL_FUNCTIONS
{
public:
    Camera();
	bool OnKeyboardNum(unsigned char Key, bool printInfo);
	bool onKeyboardArrow(int Key, bool printInfo);
	void PrintInfo();
	void Setup(const Vector3f& Pos, const Vector3f& Center, const Vector3f& Up);
	void SetSteps(float Step, float angleStep);

    const Vector3f& GetPos() const
    {
        return m_pos;
    }

    const Vector3f& GetTarget() const
    {
        return m_target;
    }

    const Vector3f& GetUp() const
    {
        return m_up;
    }
	const Vector3f& GetRight() const
	{
		return m_right;
	}
	const Vector3f& GetCenter() const
	{
		return m_center;
	}
	const int& GetWidth() const
	{
		return m_windowWidth;
	}
	const int& GetHeight() const
	{
		return m_windowHeight;
	}
	void SetCam(const Vector3f &Pos, const Vector3f &Target, const Vector3f &Up)
	{
		m_pos = Pos;
		m_target = Target;
		m_up = Up;
	}

	//Cartesian coordinates variables
	float m_x;
	float m_y;
	float m_z;
	//Spherical coordinates variables	
	float m_rho;
	float m_theta;
	float m_phi;
	Vector3f m_offset;
	Vector3f GetXYZ() const
	{
		return Vector3f(m_x, m_y, m_z);
	}
	Vector3f GetRTF() const
	{
		return Vector3f(m_rho, m_theta, m_phi);
	}
	void UpdateCamera();
	void DrawCameraVectors();

private:
    Vector3f m_pos;
    Vector3f m_target;
    Vector3f m_up;
	Vector3f m_right;
	
    int m_windowWidth;
    int m_windowHeight;

	float m_step;
	float m_angleStep;
	Vector3f m_center;
	void UpdateCartesian();
	void UpdateSpherical();	// not used atm
};

#endif	/* CAMERA_H */

