// Own
#include "camera.h"

// Standard C/C++
#include <iostream>

using namespace std;

static const float zoomStep = 0.25f;
static const float rotateStep = 15.f;

Camera::Camera()
{
	//initializeOpenGLFunctions();
    m_windowWidth  = 512;
    m_windowHeight = 424;
	m_rho		   = 2.0f;
	m_theta		   = 90.0f;
	m_phi		   = 90.0f;
	UpdateCartesian();
	m_offset	   = QVector3D(0.0f, 0.0f, 2.0f);
	m_center       = QVector3D(0.0f, 0.0f, 2.0f); // focus point of camera
    m_up           = QVector3D(0.0f, 1.0f, 0.0f); // direction of up vector
	UpdateCamera();
}
// use it after setting XYZ, offset, center and up
void Camera::UpdateCamera() {
	m_pos = GetXYZ() + m_offset;
	m_target = m_center - m_pos;
	m_target.normalize();
	//m_up = Vector3f(sin(90.0f-m_Phi)*cos(180.0f+m_Theta), cos(m_Phi), -sin(m_Phi)*sin(m_Theta));
	//m_up = (m_Phi == 180.0f || m_Phi == 360.0f) ? m_up*(-1) : m_up;
	m_up.normalize();
	m_right = QVector3D::crossProduct(m_target, m_up);
	m_right.normalize();
}
void Camera::UpdateSpherical()
{
	//m_Rho = sqrt(pow(m_X, 2) + pow(m_Y, 2) + pow(m_Z, 2));
	//m_Theta = atan2(-m_Z / m_X);
	//m_Phi;
}
void Camera::rotateRight(float angles)
{
	m_theta += angles;
	m_theta = wrapAngle(m_theta, 360.f);
	UpdateCartesian();
	UpdateCamera();
}
void Camera::rotateUp(float angles)
{
	float newPhi = m_phi + angles;
	if (newPhi >= 180.f || newPhi <= 0.f) return;
	m_phi = newPhi;
	UpdateCartesian();
	UpdateCamera();
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
//void Camera::OnKeyboardNum(int key, bool print)
//{
//	bool Ret = false;
//	switch (key) {
//	case '-':
//		m_rho -= m_step;
//		UpdateCartesian();
//		UpdateCamera();
//		printInfo();
//		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
//		Ret = true;
//		break;
//	case '+':
//		m_rho += m_step;
//		UpdateCartesian();
//		UpdateCamera();
//		printInfo();
//		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
//		Ret = true;
//		break;
//	case '[':
//		m_angleStep = (m_angleStep > 5 ? m_angleStep-5 : m_angleStep);
//		if (print) cout << "Camera: AngleStep(degrees)=" << m_angleStep << endl;
//		Ret = true;
//		break;
//	case ']':
//		m_angleStep = (m_angleStep < 355 ? m_angleStep + 5 : m_angleStep);
//		if (print) cout << "Camera: AngleStep(degrees)=" << m_angleStep << endl;
//		Ret = true;
//		break;
//	case ',':
//		m_step = (m_step > 0.1 ? m_step - 0.1 : m_step);
//		if (print) cout << "Camera: Step=" << m_step << endl;
//		Ret = true;
//		break;
//	case '.':
//		m_step += 0.1;
//		if (print) cout << "Camera: Step=" << m_step << endl;
//		Ret = true;
//		break;
//	default:
//		break;
//	}
//	return Ret;
//}
void Camera::onKeyboardArrow(int key, bool print)
{	
	switch (key) {
	case Qt::Key_Up:
		rotateUp(-rotateStep);
		if (print) printInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		return;
	case Qt::Key_Down:
		rotateUp(+rotateStep);
		if (print) printInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		return;
	case Qt::Key_Left:
		rotateRight(+rotateStep);
		if (print) printInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		return;
	case Qt::Key_Right:
		rotateRight(-rotateStep);
		if (print) printInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		return;
	default:
		cout << "none of the arrows was pressed" << endl;
		return;
	}
}
void Camera::onMouseWheel(int degrees, bool print)
{
	m_rho -= (float)degrees/30.f;
	UpdateCartesian();
	UpdateCamera();
	if (print) printInfo();
	//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
}
void Camera::Setup(const QVector3D& Pos, const QVector3D& Center, const QVector3D& Up)
{
	m_pos = Pos;
	m_center = Center;
	m_up = Up;
	m_up.normalize();
	m_target = m_center - m_pos;
	m_target.normalize();
}
void Camera::printInfo()
{
	qDebug() << "Camera:";
	qDebug() << "RTF      = " << toStringSpherical(GetRTF()) << " XYZ    = " << toStringCartesian(GetXYZ()) << " Offset   = " << toStringCartesian(m_offset);
	qDebug() << "Position = " << toStringCartesian(m_pos)    << " Center = " << toStringCartesian(m_center) << " Distance = " << m_pos.distanceToPoint(m_center);
	qDebug() << "Target   = " << toStringCartesian(m_target) << " Up     = " << toStringCartesian(m_up)     << " Right    = " << toStringCartesian(m_right);
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
