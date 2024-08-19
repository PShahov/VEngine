#pragma once

#include <GL/glew.h>
#include "Shader.h"

class TextureBufferObject
{
public:
	static const GLuint BufferUsageHint = GL_DYNAMIC_DRAW;
	static const GLsizeiptr DataLength = 5762584 / 2;

	GLuint BufferHandle;
	GLuint TextureHandle;
	GLuint Format;

	TextureBufferObject(GLuint N = 1, float data[] = {}, GLuint format = GL_RGBA32F);

	void setSubData(float data[], GLuint offset);

	void setData(float data[], int elementsCount = 0);

	void Use(Shader *shader);
};

