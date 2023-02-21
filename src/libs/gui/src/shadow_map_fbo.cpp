#include <gui/shadow_map_fbo.h>
#include <stdio.h>

namespace crl {
namespace gui {

ShadowMapFBO::ShadowMapFBO() {
    fbo = 0;
    shadowMap = 0;
}

ShadowMapFBO::~ShadowMapFBO() {
    if (fbo != 0) {
        glDeleteFramebuffers(1, &fbo);
    }

    if (shadowMap != 0) {
        glDeleteTextures(1, &shadowMap);
    }
}

bool ShadowMapFBO::Init(GLuint bufferWidth, GLuint bufferHeight) {
    // Create the FBO
    glGenFramebuffers(1, &fbo);

    // Create the depth buffer
    glGenTextures(1, &shadowMap);
    glBindTexture(GL_TEXTURE_2D, shadowMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, bufferWidth,
                 bufferHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE,
                    GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                           shadowMap, 0);

    // Disable writes to the color buffer
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    GLenum Status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

    if (Status != GL_FRAMEBUFFER_COMPLETE) {
        printf("FB error, status: 0x%x\n", Status);
        return false;
    }
    this->bufferHeight = bufferHeight;
    this->bufferWidth = bufferWidth;
    return true;
}

void ShadowMapFBO::BindForWriting() {
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
}

void ShadowMapFBO::BindForReading(GLuint TextureUnit) {
    glActiveTexture(TextureUnit);
    glBindTexture(GL_TEXTURE_2D, shadowMap);
}

}  // namespace gui
}  // namespace crl