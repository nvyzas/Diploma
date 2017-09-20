#ifndef TEXTURE_H
#define	TEXTURE_H

#include "util.h"
#include <Magick++.h>
#include <string>

class Texture : protected OPENGL_FUNCTIONS
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

