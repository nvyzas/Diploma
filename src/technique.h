#ifndef TECHNIQUE_H
#define	TECHNIQUE_H

#include "math_3d.h"
#include <list>
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QOpenGLFunctions_4_5_Compatibility>

class Technique : public QOpenGLFunctions_4_5_Compatibility
{
public:

    Technique();

    virtual ~Technique();

    virtual bool Init();

    void Enable();

	bool InitDefault();
	void SetDefault(const Matrix4f& mat);
	
protected:

	bool AddShader(GLenum ShaderType, const char* pFilename); // was protected
	
	GLint GetUniformLocation(const char* pUniformName); // was protected

    bool Finalize();
  
    GLint GetProgramParam(GLint param);
    
    GLuint m_shaderProg;    
	GLuint m_Location;
    
private:

    typedef std::list<GLuint> ShaderObjList;
    ShaderObjList m_shaderObjList;
};

#endif	/* TECHNIQUE_H */

