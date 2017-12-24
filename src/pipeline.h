#ifndef PIPELINE_H
#define	PIPELINE_H

#include "math_3d.h"
#include "camera.h"

struct Orientation
{
	QVector3D m_scale;
	QVector3D m_rotation;
	QVector3D m_pos;
    
    Orientation()
    {
        m_scale    = QVector3D(1.0f, 1.0f, 1.0f);
        m_rotation = QVector3D(0.0f, 0.0f, 0.0f);
        m_pos      = QVector3D(0.0f, 0.0f, 0.0f);
    }
};


class Pipeline
{
public:
    Pipeline()
    {
        m_worldScale      = QVector3D(1.0f, 1.0f, 1.0f);
        m_worldPosition   = QVector3D(0.0f, 0.0f, 0.0f);
        m_worldRotation   = QQuaternion::fromEulerAngles(0.0f, 0.0f, 0.0f);
    }

    void setWorldScale(float s)
    {
        setWorldScale(s, s, s);
    }
    
    void setWorldScale(const QVector3D& scale)
    {
        setWorldScale(scale.x(), scale.y(), scale.z());
    }
    
    void setWorldScale(float ScaleX, float ScaleY, float ScaleZ)
    {
        m_worldScale.setX(ScaleX);
        m_worldScale.setY(ScaleY);
        m_worldScale.setZ(ScaleZ);
    }

    void setWorldPosition(float x, float y, float z)
    {
        m_worldPosition.setX(x);
        m_worldPosition.setY(y);
        m_worldPosition.setZ(z);
    }
    
    void setWorldPosition(const QVector3D& Pos)
    {
        m_worldPosition = Pos;
    }

    void setWorldRotation(float RotateX, float RotateY, float RotateZ)
    {
        m_worldRotation.setX(RotateX);
        m_worldRotation.setY(RotateY);
        m_worldRotation.setZ(RotateZ);
    }
    void setWorldRotation(const QVector3D& r)
    {
        setWorldRotation(r.x(), r.y(), r.z());
    }
	void setWorldRotation(const QQuaternion& q)
	{
		m_worldRotation = q;
	}
    void SetPerspectiveProj(const PersProjInfo& p)
    {
        m_persProjInfo = p;
    }
    
    void SetOrthographicProj(const OrthoProjInfo& p)
    {
        m_orthoProjInfo = p;
    }    

    void SetCamera(const QVector3D& Pos, const QVector3D& Target, const QVector3D& Up)
    {
        m_camera.Pos = Pos;
        m_camera.Target = Target;
        m_camera.Up = Up;
    }
    
    void SetCamera(const Camera& camera)
    {
        SetCamera(camera.GetPos(), camera.GetTarget(), camera.GetUp());
    }
    
    void Orient(const Orientation& o)
    {
        m_worldScale      = o.m_scale;
        m_worldPosition   = o.m_pos;
        //m_worldRotation	  = o.m_rotation; #todo fix
    }

    const Matrix4f& GetWPTrans();
    const Matrix4f& GetWVTrans();
    const Matrix4f& GetVPTrans();
    const Matrix4f& GetWVPTrans();
    const Matrix4f& GetWVOrthoPTrans();
    const Matrix4f& GetWorldTrans();
    const Matrix4f& GetViewTrans();
    const Matrix4f& GetProjTrans();

private:
	QVector3D m_worldScale;
	QQuaternion m_worldRotation;
	QVector3D m_worldPosition;

    PersProjInfo m_persProjInfo;
    OrthoProjInfo m_orthoProjInfo;

    struct {
		QVector3D Pos;
		QVector3D Target;
		QVector3D Up;
    } m_camera;

    Matrix4f m_WVPtransformation;
    Matrix4f m_VPtransformation;
    Matrix4f m_WPtransformation;
    Matrix4f m_WVtransformation;
    Matrix4f m_Wtransformation;
    Matrix4f m_Vtransformation;
    Matrix4f m_ProjTransformation;
};


#endif	/* PIPELINE_H */

