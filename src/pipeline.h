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
        m_scale      = QVector3D(1.0f, 1.0f, 1.0f);
        m_worldPos   = QVector3D(0.0f, 0.0f, 0.0f);
        m_rotateInfo = QVector3D(0.0f, 0.0f, 0.0f);
    }

    void worldScale(float s)
    {
        worldScale(s, s, s);
    }
    
    void worldScale(const QVector3D& scale)
    {
        worldScale(scale.x(), scale.y(), scale.z());
    }
    
    void worldScale(float ScaleX, float ScaleY, float ScaleZ)
    {
        m_scale.setX(ScaleX);
        m_scale.setY(ScaleY);
        m_scale.setZ(ScaleZ);
    }

    void worldTranslate(float x, float y, float z)
    {
        m_worldPos.setX(x);
        m_worldPos.setY(y);
        m_worldPos.setZ(z);
    }
    
    void worldTranslate(const QVector3D& Pos)
    {
        m_worldPos = Pos;
    }

    void worldRotate(float RotateX, float RotateY, float RotateZ)
    {
        m_rotateInfo.setX(RotateX);
        m_rotateInfo.setY(RotateY);
        m_rotateInfo.setZ(RotateZ);
    }
    
    void worldRotate(const QVector3D& r)
    {
        worldRotate(r.x(), r.y(), r.z());
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
        m_scale      = o.m_scale;
        m_worldPos   = o.m_pos;
        m_rotateInfo = o.m_rotation;
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
	QVector3D m_scale;
	QVector3D m_worldPos;
	QVector3D m_rotateInfo;

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

