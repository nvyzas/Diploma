#include "technique.h"
#include <stdio.h>
#include <string.h>
#include "util.h"

Technique::Technique()
{
	//initializeOpenGLFunctions();
    m_shaderProg = 0;	
}
Technique::~Technique()
{
    // Delete the intermediate shader objects that have been added to the program
    // The list will only contain something if shaders were compiled but the object itself
    // was destroyed prior to linking.
    for (ShaderObjList::iterator it = m_shaderObjList.begin() ; it != m_shaderObjList.end() ; it++)
    {
        glDeleteShader(*it);
    }

    if (m_shaderProg != 0)
    {
        glDeleteProgram(m_shaderProg);
        m_shaderProg = 0;
    }
}
bool Technique::Init()
{
	initializeOpenGLFunctions();
    m_shaderProg = glCreateProgram();

    if (m_shaderProg == 0) {
        printf("Error creating shader program\n");
        return false;
	}
	else {
		printf("Created shader program %d\n", m_shaderProg);
		return true;
	}
}
// Use this method to add shaders to the program. When finished - call finalize()
bool Technique::AddShader(GLenum ShaderType, const char* pFilename)
{
    std::string s;
    
    if (!ReadFile(pFilename, s)) {
        return false;
    }
    
    GLuint ShaderObj = glCreateShader(ShaderType);

    if (ShaderObj == 0) {
        printf("Error creating shader type %d\n", ShaderType);
        return false;
    }

    // Save the shader object - will be deleted in the destructor
    m_shaderObjList.push_back(ShaderObj);

    const GLchar* p[1];
    p[0] = s.c_str();
    GLint Lengths[1] = { (GLint)s.size() };

    glShaderSource(ShaderObj, 1, p, Lengths);

    glCompileShader(ShaderObj);

    GLint success;
    glGetShaderiv(ShaderObj, GL_COMPILE_STATUS, &success);

    if (!success) {
        GLchar InfoLog[1024];
        glGetShaderInfoLog(ShaderObj, 1024, NULL, InfoLog);
		printf("Error compiling '%s': '%s'\n", pFilename, InfoLog);
        //fprintf(stderr, "Error compiling '%s': '%s'\n", pFilename, InfoLog);
        return false;
    }

    glAttachShader(m_shaderProg, ShaderObj);

    return true;
}
// After all the shaders have been added to the program call this function
// to link and validate the program.
bool Technique::Finalize()
{
    GLint Success = 0;
    GLchar ErrorLog[1024] = { 0 };

    glLinkProgram(m_shaderProg);

    glGetProgramiv(m_shaderProg, GL_LINK_STATUS, &Success);
	if (Success == 0) {
		glGetProgramInfoLog(m_shaderProg, sizeof(ErrorLog), NULL, ErrorLog);
		printf("Error linking shader program: '%s'\n", ErrorLog);
		//fprintf(stderr, "Error linking shader program: '%s'\n", ErrorLog);
        return false;
	}

    glValidateProgram(m_shaderProg);
    glGetProgramiv(m_shaderProg, GL_VALIDATE_STATUS, &Success);
    if (!Success) {
        glGetProgramInfoLog(m_shaderProg, sizeof(ErrorLog), NULL, ErrorLog);
		printf("Invalid shader program: '%s'\n", ErrorLog);
        //fprintf(stderr, "Invalid shader program: '%s'\n", ErrorLog);
     //   return false;
    }
	
    // Delete the intermediate shader objects that have been added to the program
    for (ShaderObjList::iterator it = m_shaderObjList.begin() ; it != m_shaderObjList.end() ; it++) {
        glDeleteShader(*it);
    }

    m_shaderObjList.clear();
	GLCheckError();
    return GLCheckError();
}
void Technique::enable()
{
    glUseProgram(m_shaderProg);
}
bool Technique::InitDefault()
{
	if (!Technique::Init()) {
		printf("Cannot initialize default technique\n");
		return false;
	}

	if (!AddShader(GL_VERTEX_SHADER, "shaders/test.vs")) {
		printf("Cannot add default vertex shader\n");
		return false;
	}

	if (!AddShader(GL_FRAGMENT_SHADER, "shaders/test.fs")) {
		printf("Cannot add default fragment shader\n");
		return false;
	}
	
	if (!Finalize()) {
		printf("Cannot finalize default shaders\n");
		return false;
	}

	m_Location = GetUniformLocation("gWVP");

	return true;
}
void Technique::SetDefault(const Matrix4f& WVP)
{
	glUniformMatrix4fv(m_Location, 1, GL_TRUE, (const GLfloat*)WVP);
}
GLint Technique::GetUniformLocation(const char* pUniformName)
{
    GLuint Location = glGetUniformLocation(m_shaderProg, pUniformName);

    if (Location == INVALID_UNIFORM_LOCATION) {
		printf("Warning! Unable to get the location of uniform '%s' (Shader program: %d)\n", pUniformName, m_shaderProg);
        //fprintf(stderr, "Warning! Unable to get the location of uniform '%s'\n", pUniformName);
    }

    return Location;
}
GLint Technique::GetProgramParam(GLint param)
{
    GLint ret;
    glGetProgramiv(m_shaderProg, param, &ret);
    return ret;
}
