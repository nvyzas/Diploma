#ifndef TECHNIQUE_H
#define	TECHNIQUE_H

#include "math_3d.h"
#include "util.h"
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
	void setSpecific(const Matrix4f& MVP);

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

