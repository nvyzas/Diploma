#ifndef SKINNING_TECHNIQUE_H
#define	SKINNING_TECHNIQUE_H

// Project
#include "technique.h"

struct BaseLight
{
    QVector3D Color;
    float AmbientIntensity;
    float DiffuseIntensity;

    BaseLight()
    {
        Color = QVector3D(0.0f, 0.0f, 0.0f);
        AmbientIntensity = 0.0f;
        DiffuseIntensity = 0.0f;
    }
};
struct DirectionalLight : public BaseLight
{        
	QVector3D Direction;

    DirectionalLight()
    {
        Direction = QVector3D(0.0f, 0.0f, 0.0f);
    }
};
struct PointLight : public BaseLight
{
	QVector3D Position;

    struct
    {
        float Constant;
        float Linear;
        float Exp;
    } Attenuation;

    PointLight()
    {
        Position = QVector3D(0.0f, 0.0f, 0.0f);
        Attenuation.Constant = 1.0f;
        Attenuation.Linear = 0.0f;
        Attenuation.Exp = 0.0f;
    }
};
struct SpotLight : public PointLight
{
	QVector3D Direction;
    float Cutoff;

    SpotLight()
    {
        Direction = QVector3D(0.0f, 0.0f, 0.0f);
        Cutoff = 0.0f;
    }
};
class SkinningTechnique : public Technique {
public:

    static const uint MAX_POINT_LIGHTS = 2;
    static const uint MAX_SPOT_LIGHTS = 2;
    static const uint MAX_BONES = 100;

    virtual bool Init();

    void SetWVP(const Matrix4f& WVP);
    void SetWorldMatrix(const Matrix4f& WVP);
    void SetColorTextureUnit(uint TextureUnit);
    void SetDirectionalLight(const DirectionalLight& Light);
    void SetPointLights(uint NumLights, const PointLight* pLights);
    void SetSpotLights(uint NumLights, const SpotLight* pLights);
    void SetEyeWorldPos(const QVector3D& EyeWorldPos);
    void SetMatSpecularIntensity(float Intensity);
    void SetMatSpecularPower(float Power);
    void setBoneTransform(uint index, const QMatrix4x4& transform);
	void setSkinning(int value);
	void setBoneVisibility(uint Index, const bool& Visibility);

private:   
    GLuint m_WVPLocation;
    GLuint m_WorldMatrixLocation;
    GLuint m_colorTextureLocation;
    GLuint m_eyeWorldPosLocation;
    GLuint m_matSpecularIntensityLocation;
    GLuint m_matSpecularPowerLocation;
    GLuint m_numPointLightsLocation;
    GLuint m_numSpotLightsLocation;
	GLuint m_skinningOnLocation;
	

    struct {
        GLuint Color;
        GLuint AmbientIntensity;
        GLuint DiffuseIntensity;
        GLuint Direction;
    } m_dirLightLocation;

    struct {
        GLuint Color;
        GLuint AmbientIntensity;
        GLuint DiffuseIntensity;
        GLuint Position;
        struct {
            GLuint Constant;
            GLuint Linear;
            GLuint Exp;
        } Atten;
    } m_pointLightsLocation[MAX_POINT_LIGHTS];

    struct {
        GLuint Color;
        GLuint AmbientIntensity;
        GLuint DiffuseIntensity;
        GLuint Position;
        GLuint Direction;
        GLuint Cutoff;
        struct {
            GLuint Constant;
            GLuint Linear;
            GLuint Exp;
        } Atten;
    } m_spotLightsLocation[MAX_SPOT_LIGHTS];
    
    GLuint m_boneLocation[MAX_BONES];
	GLuint m_visibilityLocation[MAX_BONES];
};

#endif	/* SKINNING_TECHNIQUE_H */
