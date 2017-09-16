#ifndef CAMERA_H
#define	CAMERA_H

#include "math_3d.h"

class Camera
{
public:
    Camera(int WindowWidth, int WindowHeight);
	bool OnKeyboardNum(unsigned char Key, bool printInfo);
	bool OnKeyboardSpecial(int Key, bool printInfo);
    void OnMouse(int x, int y);
    void OnRender(bool printAngles);
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

	void SetCam(const Vector3f &Pos, const Vector3f &Target, const Vector3f &Up)
	{
		m_pos = Pos;
		m_target = Target;
		m_up = Up;
	}

	//Cartesian coordinates variables
	float m_X;
	float m_Y;
	float m_Z;
	//Spherical coordinates variables	
	float m_Rho;
	float m_Theta;
	float m_Phi;
	Vector3f m_Offset;
	Vector3f GetXYZ() const
	{
		return Vector3f(m_X, m_Y, m_Z);
	}
	Vector3f GetRTF() const
	{
		return Vector3f(m_Rho, m_Theta, m_Phi);
	}
	void UpdateCamera();

private:

    void Init();
    void Update();

    Vector3f m_pos;
    Vector3f m_target;
    Vector3f m_up;
	Vector3f m_right;
	
    int m_windowWidth;
    int m_windowHeight;

    float m_AngleH;
    float m_AngleV;
	
    bool m_OnUpperEdge;
    bool m_OnLowerEdge;
    bool m_OnLeftEdge;
    bool m_OnRightEdge;
    Vector2i m_mousePos;

	float m_Step;
	float m_angleStep;
	Vector3f m_center;
	void UpdateCartesian();
	void UpdateSpherical();	// float rho, float phi, float theta
};

#endif	/* CAMERA_H */

