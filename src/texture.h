#ifndef TEXTURE_H
#define	TEXTURE_H

#include <string>
#include <Magick++.h>
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QOpenGLFunctions_4_5_Compatibility>

class Texture : QOpenGLFunctions_4_5_Compatibility
{
public:
    Texture(GLenum TextureTarget, const std::string& FileName);

    bool Load();

    void Bind(GLenum TextureUnit);

private:
    std::string m_fileName;
    GLenum m_textureTarget;
    GLuint m_textureObj;
    Magick::Image m_image;
    Magick::Blob m_blob;
};


#endif	/* TEXTURE_H */

