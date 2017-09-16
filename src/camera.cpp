#include "camera.h"
#include <iostream>
using namespace std;

const static float STEP_SCALE = 1.0f;
const static float EDGE_STEP = 0.5f;
const static int MARGIN = 10;

Camera::Camera(int WindowWidth, int WindowHeight)
{
    m_windowWidth  = WindowWidth;
    m_windowHeight = WindowHeight;
	m_Rho		   = 2.0f;
	m_Theta		   = 90.0f;
	m_Phi		   = 90.0f;
	UpdateCartesian();
	m_Offset	   = Vector3f(0.0f, 0.0f, 2.0f); 
	m_center       = Vector3f(0.0f, 0.0f, 2.0f); // focus point of camera
    m_up           = Vector3f(0.0f, 1.0f, 0.0f); // direction of up vector
	UpdateCamera();
	m_Step = 0.25;
	m_angleStep = 15;
    Init();
}
// use it after setting XYZ, offset, center and up
void Camera::UpdateCamera() {
	m_pos = GetXYZ() + m_Offset;
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
	float Rho = m_Rho;
	float Theta = ToRadian(m_Theta);
	float Phi = ToRadian(m_Phi);
	m_X = Rho*sin(Phi)*cos(Theta);
	m_Y = Rho*cos(Phi);
	m_Z = -Rho*sin(Phi)*sin(Theta);
}

bool Camera::OnKeyboardNum(unsigned char key, bool print)
{
	bool Ret = false;
	switch (key) {
	case '-':
		m_Rho -= m_Step;
		UpdateCartesian();
		UpdateCamera();
		PrintInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		Ret = true;
		break;
	case '+':
		m_Rho += m_Step;
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
		m_Step = (m_Step > 0.1 ? m_Step - 0.1 : m_Step);
		if (print) cout << "Camera: Step=" << m_Step << endl;
		Ret = true;
		break;
	case '.':
		m_Step += 0.1;
		if (print) cout << "Camera: Step=" << m_Step << endl;
		Ret = true;
		break;
	default:
		break;
	}
	return Ret;
}

bool Camera::OnKeyboardSpecial(int Key, bool print)
{
	bool Ret = false;
	
	switch (Key) {

	case 0: 
		m_Phi -= m_angleStep;
		m_Phi = (m_Phi <= 0.0f) ? m_Phi + 360.0f : m_Phi;
		UpdateCartesian();
		UpdateCamera();
		PrintInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		Ret = true;
		break;

	case 1:
		m_Phi += m_angleStep;
		m_Phi = (m_Phi > 360.0f) ? m_Phi - 360.0f : m_Phi;
		UpdateCartesian();
		UpdateCamera();
		PrintInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		Ret = true;
		break;

	case 2:
		m_Theta += m_angleStep;
		m_Theta = ((m_Theta <= 360.0f) ? m_Theta : m_Theta - 360.0f);
		UpdateCartesian();
		UpdateCamera();
		PrintInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		Ret = true;
		break;

	case 3:
		m_Theta -= m_angleStep;
		m_Theta = (m_Theta >= 0.0f) ? m_Theta : m_Theta + 360.0f;
		UpdateCartesian();
		UpdateCamera();
		PrintInfo();
		//if (print) cout << "Camera: Pos=" << m_pos.GetString() << " RTF=" << GetRTF().GetString() << " XYZ=" << GetXYZ().GetString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
		Ret = true;
		break;

	case 4:
		break;

	case 5:
		break;

	default:
		break;
	}
		return Ret;	
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
	m_Step = Step;
	m_angleStep = angleStep;
}


void Camera::OnMouse(int x, int y)
{
    const int DeltaX = x - m_mousePos.x;
    const int DeltaY = y - m_mousePos.y;

    m_mousePos.x = x;
    m_mousePos.y = y;

    m_AngleH += (float)DeltaX / 20.0f;
    m_AngleV += (float)DeltaY / 20.0f;

    if (DeltaX == 0) {
        if (x <= MARGIN) {
        //    m_AngleH -= 1.0f;
            m_OnLeftEdge = true;
        }
        else if (x >= (m_windowWidth - MARGIN)) {
        //    m_AngleH += 1.0f;
            m_OnRightEdge = true;
        }
    }
    else {
        m_OnLeftEdge = false;
        m_OnRightEdge = false;
    }

    if (DeltaY == 0) {
        if (y <= MARGIN) {
            m_OnUpperEdge = true;
        }
        else if (y >= (m_windowHeight - MARGIN)) {
            m_OnLowerEdge = true;
        }
    }
    else {
        m_OnUpperEdge = false;
        m_OnLowerEdge = false;
    }

    Update();
}

void Camera::Init()
{
	Vector3f HTarget(m_target.x, 0.0, m_target.z);
	HTarget.Normalize();

	if (HTarget.z >= 0.0f)
	{
		if (HTarget.x >= 0.0f)
		{
			m_AngleH = 360.0f - ToDegree(asin(HTarget.z));
		}
		else
		{
			m_AngleH = 180.0f + ToDegree(asin(HTarget.z));
		}
	}
	else
	{
		if (HTarget.x >= 0.0f)
		{
			m_AngleH = ToDegree(asin(-HTarget.z));
		}
		else
		{
			m_AngleH = 180.0f - ToDegree(asin(-HTarget.z));
		}
	}

	m_AngleV = -ToDegree(asin(m_target.y));

	m_OnUpperEdge = false;
	m_OnLowerEdge = false;
	m_OnLeftEdge = false;
	m_OnRightEdge = false;
	m_mousePos.x = m_windowWidth / 2;
	m_mousePos.y = m_windowHeight / 2;

	// glutWarpPointer(m_mousePos.x, m_mousePos.y);
}

void Camera::PrintInfo()
{
	cout << "Camera:" << endl;
	cout << "RTF=" << GetRTF().ToString() << " XYZ=" << GetXYZ().ToString() << " Offset=" << m_Offset.ToString() << endl;
	cout << "Position=" << m_pos.ToString() << " Center=" << m_center.ToString() << " Distance=" << m_pos.DistanceFrom(m_center) << endl;
	cout << "Target=" << m_target.ToString() << " Up=" << m_up.ToString() << " Right=" << m_right.ToString() << endl;
	//cout << "Step=" << m_Step << " AngleStep(degrees)=" << m_angleStep << endl;
}

void Camera::OnRender(bool printAngles)
{
    bool ShouldUpdate = false;

    if (m_OnLeftEdge) {
        m_AngleH -= EDGE_STEP;
		if (printAngles) cout << "Horizontal Angle = " << m_AngleH << endl;
        ShouldUpdate = true;
    }
    else if (m_OnRightEdge) {
        m_AngleH += EDGE_STEP;
		if (printAngles) cout << "Horizontal Angle = " << m_AngleH << endl;
        ShouldUpdate = true;
    }

    if (m_OnUpperEdge) {
        if (m_AngleV > -90.0f) {
            m_AngleV -= EDGE_STEP;
			if (printAngles) cout << "Vertical Angle = " << m_AngleV << endl;
            ShouldUpdate = true;
        }
    }
    else if (m_OnLowerEdge) {
        if (m_AngleV < 90.0f) {
           m_AngleV += EDGE_STEP;
		   if (printAngles) cout << "Vertical Angle = " << m_AngleV << endl;
           ShouldUpdate = true;
        }
    }

    if (ShouldUpdate) {
        Update();
    }
}

void Camera::Update()
{
    
	Vector3f View(1.0f, 0.0f, 0.0f);
    // Rotate the view vector by the horizontal angle around the vertical axis
	const Vector3f Vaxis(0.0f, 1.0f, 0.0f);
    View.Rotate(m_AngleH, Vaxis);
    View.Normalize();

    // Rotate the view vector by the vertical angle around the horizontal axis
    Vector3f Haxis = Vaxis.Cross(View);
    Haxis.Normalize();
    View.Rotate(m_AngleV, Haxis);
       
    m_target = View;
    m_target.Normalize();

    m_up = m_target.Cross(Haxis);
    m_up.Normalize();
}

