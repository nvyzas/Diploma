#ifndef TECHNIQUE_H
#define	TECHNIQUE_H

// Project
#include "util.h"

// Qt
#include <QtGui\QMatrix4x4>

// Standard C/C++
#include <list>

class Technique : protected QOpenGLFunctions_3_3_Core
{
public:

    Technique();

    virtual ~Technique();

    virtual bool Init();

    void enable();

	bool InitDefault();
	void setMVP(const Matrix4f& MVP);
	void setSpecific(const QMatrix4x4& MVP);

protected:

	bool AddShader(GLenum ShaderType, const char* pFilename); 
	
	GLint GetUniformLocation(const char* pUniformName); 

    bool Finalize();
  
    GLint GetProgramParam(GLint param);
    
    GLuint m_shaderProg;    
	GLuint m_locationMVP;
	GLuint m_locationSpecific;

private:

    typedef std::list<GLuint> ShaderObjList;
    ShaderObjList m_shaderObjList;
};

#endif	/* TECHNIQUE_H */

