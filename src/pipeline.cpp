#include "pipeline.h"

const Matrix4f& Pipeline::GetProjTrans() 
{
    m_ProjTransformation.InitPersProjTransform(m_persProjInfo);
    return m_ProjTransformation;
}


const Matrix4f& Pipeline::GetVPTrans()
{
    GetViewTrans();
    GetProjTrans();
       
    m_VPtransformation = m_ProjTransformation * m_Vtransformation;
    return m_VPtransformation;
}

const Matrix4f& Pipeline::GetWorldTrans()
{
    Matrix4f ScaleTrans, RotateTrans, TranslationTrans;

    ScaleTrans.InitScaleTransform(m_worldScale.x(), m_worldScale.y(), m_worldScale.z());
	RotateTrans.InitRotateTransform2(m_worldRotation);
    TranslationTrans.InitTranslateTransform(m_worldPosition.x(), m_worldPosition.y(), m_worldPosition.z());

    m_Wtransformation = TranslationTrans * RotateTrans * ScaleTrans;
    return m_Wtransformation;
}

const Matrix4f& Pipeline::GetViewTrans()
{
    Matrix4f CameraTranslationTrans, CameraRotateTrans, CameraScalingTrans;

    CameraTranslationTrans.InitTranslateTransform(-m_camera.Pos.x(), -m_camera.Pos.y(), -m_camera.Pos.z());
    CameraRotateTrans.InitCameraTransform(m_camera.Target, m_camera.Up);
    m_Vtransformation =  CameraRotateTrans * CameraTranslationTrans;

    return m_Vtransformation;
}

const Matrix4f& Pipeline::GetWVPTrans()
{
    GetWorldTrans();
    GetVPTrans();

    m_WVPtransformation = m_VPtransformation * m_Wtransformation;
    return m_WVPtransformation;
}


const Matrix4f& Pipeline::GetWVOrthoPTrans()
{
    GetWorldTrans();
    GetViewTrans();

    Matrix4f P;
    P.InitOrthoProjTransform(m_orthoProjInfo);
    
    m_WVPtransformation = P * m_Vtransformation * m_Wtransformation;
    return m_WVPtransformation;
}


const Matrix4f& Pipeline::GetWVTrans()
{
	GetWorldTrans();
    GetViewTrans();
	
	m_WVtransformation = m_Vtransformation * m_Wtransformation;
	return m_WVtransformation;
}


const Matrix4f& Pipeline::GetWPTrans()
{
	Matrix4f PersProjTrans;

	GetWorldTrans();
	PersProjTrans.InitPersProjTransform(m_persProjInfo);

	m_WPtransformation = PersProjTrans * m_Wtransformation;
	return m_WPtransformation;
}