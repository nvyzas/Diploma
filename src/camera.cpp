#include "camera.h"
#include <iostream>

using namespace std;

const static float STEP_SCALE = 1.0f;
const static float EDGE_STEP = 0.5f;
const static int MARGIN = 10;

Camera::Camera()
{
	//initializeOpenGLFunctions();
    m_windowWidth  = 512;
    m_windowHeight = 424;
	m_rho		   = 2.0f;
	m_theta		   = 90.0f;
	m_phi		   = 90.0f;
	UpdateCartesian();
	m_offset	   = Vector3f(0.0f, 0.0f, 2.0f); 
	m_center       = Vector3f(0.0f, 0.0f, 2.0f); // focus point of camera
    m_up           = Vector3f(0.0f, 1.0f, 0.0f); // direction of up vector
	UpdateCamera();
	m_step = 0.25;
	m_angleStep = 15;
}
// use it after setting XYZ, offset, center and up
void Camera::UpdateCamera() {
	m_pos = GetXYZ() + m_offset;
	m_target = m_center - m_pos;
	m_target.Normalize();
	//m_up = Vector3f(sin(90.0f-m_Phi)*cos(180.0f+m_Theta), cos(m_Phi), -sin(m_Phi)*sin(m_Theta));
	//m_up = (m_Phi == 180.0f || m_Phi == 360.0f) ? m_up*(-1) : m_up;
	m_up.Normalize();
	m_right = m_target.Cross(m_up);
	m_right.Normalize();
}
void Camera::UpdateSpherical()
{
	//m_Rho = sqrt(pow(m_X, 2) + pow(m_Y, 2) + pow(m_Z, 2));
	//m_Theta = atan2(-m_Z / m_X);
	//m_Phi;
}
void Camera::UpdateCartesian()
{
	float rho = m_rho;
	float theta = ToRadian(m_theta);
	float phi = ToRadian(m_phi);
	m_x = rho*sin(phi)*cos(theta);
	m_y = rho*cos(phi);
	m_z = -rho*sin(phi)*sin(theta);
}
bool Camera::OnKeyboardNum(unsigned char key, bool print)
{
	bool Ret = false;
	switch (key) {
	case '-':
		m_rho -= m_step;
		UpdateCartesian();
		UpdateCamera();
		PrintInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		Ret = true;
		break;
	case '+':
		m_rho += m_step;
		UpdateCartesian();
		UpdateCamera();
		PrintInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		Ret = true;
		break;
	case '[':
		m_angleStep = (m_angleStep > 5 ? m_angleStep-5 : m_angleStep);
		if (print) cout << "Camera: AngleStep(degrees)=" << m_angleStep << endl;
		Ret = true;
		break;
	case ']':
		m_angleStep = (m_angleStep < 355 ? m_angleStep + 5 : m_angleStep);
		if (print) cout << "Camera: AngleStep(degrees)=" << m_angleStep << endl;
		Ret = true;
		break;
	case ',':
		m_step = (m_step > 0.1 ? m_step - 0.1 : m_step);
		if (print) cout << "Camera: Step=" << m_step << endl;
		Ret = true;
		break;
	case '.':
		m_step += 0.1;
		if (print) cout << "Camera: Step=" << m_step << endl;
		Ret = true;
		break;
	default:
		break;
	}
	return Ret;
}
bool Camera::onKeyboardArrow(int Key, bool print)
{	
	switch (Key) {
	case Qt::Key_Up:
		m_phi -= m_angleStep;
		m_phi = (m_phi <= 0.0f) ? m_phi + 360.0f : m_phi;
		UpdateCartesian();
		UpdateCamera();
		PrintInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		return true;
	case Qt::Key_Down:
		m_phi += m_angleStep;
		m_phi = (m_phi > 360.0f) ? m_phi - 360.0f : m_phi;
		UpdateCartesian();
		UpdateCamera();
		PrintInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		return true;
	case Qt::Key_Left:
		m_theta += m_angleStep;
		m_theta = ((m_theta <= 360.0f) ? m_theta : m_theta - 360.0f);
		UpdateCartesian();
		UpdateCamera();
		PrintInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		return true;
	case Qt::Key_Right:
		m_theta -= m_angleStep;
		m_theta = (m_theta >= 0.0f) ? m_theta : m_theta + 360.0f;
		UpdateCartesian();
		UpdateCamera();
		PrintInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		return true;
	default:
		cout << "none of the arrows was pressed" << endl;
		return false;
	}
}
void Camera::Setup(const Vector3f& Pos, const Vector3f& Center, const Vector3f& Up)
{
	m_pos = Pos;
	m_center = Center;
	m_up = Up;
	m_up.Normalize();
	m_target = m_center - m_pos;
	m_target.Normalize();
}
void Camera::SetSteps(float Step, float angleStep)
{
	m_step = Step;
	m_angleStep = angleStep;
}
void Camera::PrintInfo()
{
	cout << "Camera:" << endl;
	cout << "RTF     =" << GetRTF().ToString() << " XYZ   =" << GetXYZ().ToString() << " Offset  =" << m_offset.ToString() << endl;
	cout << "Position=" << m_pos.ToString()    << " Center=" << m_center.ToString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
	cout << "Target  =" << m_target.ToString() << " Up    =" << m_up.ToString()     << " Right   =" << m_right.ToString() << endl;
	//cout << "Step=" << m_Step << " AngleStep(degrees)=" << m_angleStep << endl;
}
//void Camera::DrawCameraVectors()
//{
//	//*
//	glBegin(GL_LINES);
//	const Vector3f &c = GetCenter();
//	Vector3f v;
//	float f = 2;
//	v = c + (GetTarget()*f);
//	glColor3f(0, 0, 0xFF);
//	glVertex3f(c.x, c.y, c.z);
//	glVertex3f(v.x, v.y, v.z);
//
//	v = c + (GetUp()*f);
//	glColor3f(0, 0xFF, 0);
//	glVertex3f(c.x, c.y, c.z);
//	glVertex3f(v.x, v.y, v.z);
//
//	v = c + (GetRight()*f);
//	glColor3f(0xFF, 0, 0);
//	glVertex3f(c.x, c.y, c.z);
//	glVertex3f(v.x, v.y, v.z);
//	glEnd();
//	//*/
//}
