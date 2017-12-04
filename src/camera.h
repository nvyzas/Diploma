#ifndef CAMERA_H
#define	CAMERA_H

// Project
#include "math_3d.h"
#include "util.h"

// Qt
#include "QtGui\QVector3D"

class Camera
{
public:
    Camera();
	void OnKeyboardNum(unsigned char key, bool print);
	void onKeyboardArrow(int key, bool print);
	void onMouseWheel(int degrees, bool print);
	void PrintInfo();
	void Setup(const QVector3D& Pos, const QVector3D& Center, const QVector3D& Up);

	void rotateRight(float mult);
	void rotateUp(float mult);

    const QVector3D& GetPos() const
    {
        return m_pos;
    }

    const QVector3D& GetTarget() const
    {
        return m_target;
    }

    const QVector3D& GetUp() const
    {
        return m_up;
    }
	const QVector3D& GetRight() const
	{
		return m_right;
	}
	const QVector3D& GetCenter() const
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
	void SetCam(const QVector3D &Pos, const QVector3D &Target, const QVector3D &Up)
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
	QVector3D m_offset;
	QVector3D GetXYZ() const
	{
		return QVector3D(m_x, m_y, m_z);
	}
	QVector3D GetRTF() const
	{
		return QVector3D(m_rho, m_theta, m_phi);
	}
	void UpdateCamera();
	//void DrawCameraVectors();

private:
	QVector3D m_pos;
	QVector3D m_target;
	QVector3D m_up;
	QVector3D m_right;
	
    int m_windowWidth;
    int m_windowHeight;

	QVector3D m_center;
	void UpdateCartesian();
	void UpdateSpherical();	// not used atm
};

#endif	/* CAMERA_H */

