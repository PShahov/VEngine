#define GLEW_STATIC

#include "TextureBufferObject.h"

TextureBufferObject::TextureBufferObject(GLuint N, float data[], GLuint format)
{
	Format = format;

	BufferHandle = 0;
	TextureHandle = 0;

	//actual max Data length!!!
	//5762584
	glGenBuffers(N, &BufferHandle);
	glBindBuffer(GL_TEXTURE_BUFFER, BufferHandle);
	glBufferData(GL_TEXTURE_BUFFER, DataLength, data, BufferUsageHint);

	glGenTextures(N, &TextureHandle);
	glBindTexture(GL_TEXTURE_BUFFER, TextureHandle);
	glTexBuffer(GL_TEXTURE_BUFFER, Format, BufferHandle);
}

void TextureBufferObject::setSubData(float data[], GLuint offset)
{
	glBindBuffer(GL_TEXTURE_BUFFER, BufferHandle);
	glBufferSubData(GL_TEXTURE_BUFFER, (offset * sizeof(float)), sizeof(data), data);
}

void TextureBufferObject::setData(float data[], int elementsCount)
{
	glBindBuffer(GL_TEXTURE_BUFFER, BufferHandle);
	glBufferData(GL_TEXTURE_BUFFER, elementsCount * sizeof(float), data, BufferUsageHint);
}

void TextureBufferObject::Use(Shader *shader)
{
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_BUFFER, TextureHandle);
	GLuint program = shader->Handle();
	GLint loc = glGetUniformLocation(program, "u_tbo_tex");
	glUniform1i(loc, 0);
	//glUniform1i(glGetUniformLocation(shader.Handle(), "u_tbo_tex2"), 1);
}
