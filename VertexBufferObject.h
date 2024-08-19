#pragma once
#include <GL/glew.h>

#include "VertexInfo.h";
#include "Mesh.h"
#include <iostream>

class VertexBufferObject
{
public:
	static const int MAX_VERTEX_COUNT = 1'000'000;
	static const int MIN_VERTEX_COUNT = 0;

	VertexInfo *BufferVertexInfo = nullptr;
	int VertexCount = 0;
	int IndicesCount = 0;

	GLuint VBO, VAO, EBO;

	GLuint PrimitiveType = GL_TRIANGLES;
	GLuint BufferUsageHint = GL_STATIC_DRAW;

	VertexBufferObject(VertexInfo* vertexInfo, int vertexCount, GLuint bufferUsageHint = GL_STATIC_DRAW);
	VertexBufferObject(VertexInfo* vertexInfo, Mesh *mesh, GLuint bufferUsageHint = GL_STATIC_DRAW);

	~VertexBufferObject();

	void SetData(float* data, int count);
	void SetIndices(int data[], int count);
	void Bind();
	void Draw();
};