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
        m_worldOrientation   = QQuaternion::fromEulerAngles(0.0f, 0.0f, 0.0f);
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
        m_worldOrientation.setX(RotateX);
        m_worldOrientation.setY(RotateY);
        m_worldOrientation.setZ(RotateZ);
    }
    void setWorldRotation(const QVector3D& r)
    {
        setWorldRotation(r.x(), r.y(), r.z());
    }
	void setWorldRotation(const QQuaternion& q)
	{
		m_worldOrientation = q;
	}
    void setPersProjInfo(const PersProjInfo& p)
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
        //m_worldOrientation	  = o.m_rotation; #todo fix
    }

    const QMatrix4x4& GetWVTrans();
    const QMatrix4x4& GetVPTrans();
    const QMatrix4x4& GetWVPTrans();
    const QMatrix4x4& GetWorldTrans();
    const QMatrix4x4& GetViewTrans();
    const QMatrix4x4& GetProjTrans();

private:
	QVector3D m_worldScale;
	QQuaternion m_worldOrientation;
	QVector3D m_worldPosition;

    PersProjInfo m_persProjInfo;
    OrthoProjInfo m_orthoProjInfo;

    struct {
		QVector3D Pos;
		QVector3D Target;
		QVector3D Up;
    } m_camera;

    QMatrix4x4 m_WVPtransformation;
    QMatrix4x4 m_VPtransformation;
    QMatrix4x4 m_WPtransformation;
    QMatrix4x4 m_WVtransformation;
    QMatrix4x4 m_Wtransformation;
    QMatrix4x4 m_Vtransformation;
    QMatrix4x4 m_ProjTransformation;
	QMatrix4x4 perspectiveProjection();
	QMatrix4x4 cameraRotation();
};


#endif	/* PIPELINE_H */

