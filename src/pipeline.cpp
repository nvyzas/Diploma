// Own
#include "pipeline.h"

// Project
#include "util.h"

const QMatrix4x4& Pipeline::GetProjTrans()
{
	m_ProjTransformation = perspectiveProjection();
    return m_ProjTransformation;
}
const QMatrix4x4& Pipeline::getVPtrans()
{
    GetViewTrans();
    GetProjTrans();
       
    m_VPtransformation = m_ProjTransformation * m_Vtransformation;
    return m_VPtransformation;
}
const QMatrix4x4& Pipeline::GetWorldTrans()
{
	QMatrix4x4 scaling, rotation, translation;

    scaling = fromScaling(m_worldScale);
	rotation = fromRotation(m_worldOrientation);
    translation = fromTranslation(m_worldPosition);

    m_Wtransformation = translation * rotation * scaling;
    return m_Wtransformation;
}
// change from original code: multiplied m[0][0] by -1
QMatrix4x4 Pipeline::perspectiveProjection()
{
	const float ar = m_persProjInfo.aspectRatio;
	const float nearPlane = m_persProjInfo.nearPlane;
	const float farPlane = m_persProjInfo.farPlane;
	const float zRange = nearPlane - farPlane;
	const float tanHalfFOV = tanf(ToRadians(m_persProjInfo.fieldOfView / 2.f));

	QVector4D firstRow(-1.f / (tanHalfFOV * ar), 0.f, 0.f, 0.f);
	QVector4D secondRow(0.f, 1.f / tanHalfFOV, 0, 0);
	QVector4D thirdRow(0.f, 0.f, (-nearPlane - farPlane) / zRange, 2.f*farPlane*nearPlane / zRange);
	QVector4D fourthRow(0, 0, 1.f, 0);

	QMatrix4x4 ret;
	ret.setRow(0, firstRow);
	ret.setRow(1, secondRow);
	ret.setRow(2, thirdRow);
	ret.setRow(3, fourthRow);
	return ret;
}
QMatrix4x4 Pipeline::cameraRotation()
{
	QVector3D N = m_camera.Target;
	N.normalize();
	QVector3D U = m_camera.Up;
	U = QVector3D::crossProduct(U, N);
	U.normalize();
	QVector3D V = QVector3D::crossProduct(N, U);

	QVector4D firstRow(U.x(), U.y(), U.z(), 0);
	QVector4D secondRow(V.x(), V.y(), V.z(), 0);
	QVector4D thirdRow(N.x(), N.y(), N.z(), 0);
	QVector4D fourthRow(0, 0, 0, 1);

	QMatrix4x4 ret;
	ret.setRow(0, firstRow);
	ret.setRow(1, secondRow);
	ret.setRow(2, thirdRow);
	ret.setRow(3, fourthRow);
	return ret;
}
// #todo optimize
const QMatrix4x4& Pipeline::GetViewTrans()
{
	QMatrix4x4 translation = fromTranslation(-m_camera.Pos);
	QMatrix4x4 rotation = cameraRotation();
    m_Vtransformation = rotation * translation;

    return m_Vtransformation;
}

const QMatrix4x4& Pipeline::getWVPtrans()
{
    GetWorldTrans();
    getVPtrans();

    m_WVPtransformation = m_VPtransformation * m_Wtransformation;
    return m_WVPtransformation;
}
const QMatrix4x4& Pipeline::GetWVTrans()
{
	GetWorldTrans();
    GetViewTrans();
	
	m_WVtransformation = m_Vtransformation * m_Wtransformation;
	return m_WVtransformation;
}