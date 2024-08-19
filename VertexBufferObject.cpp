#include "VertexBufferObject.h"

VertexBufferObject::VertexBufferObject(VertexInfo* vertexInfo, int vertexCount, GLuint bufferUsageHint)
{
	if (vertexCount < MIN_VERTEX_COUNT || vertexCount > MAX_VERTEX_COUNT)
	{
		throw std::invalid_argument("\"vertexCount\" value out of range");
	}
	BufferVertexInfo = vertexInfo;
	VertexCount = vertexCount;
	BufferUsageHint = BufferUsageHint;
	IndicesCount = 0;

	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, VertexCount * BufferVertexInfo->Size, nullptr, BufferUsageHint);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

VertexBufferObject::VertexBufferObject(VertexInfo* vertexInfo, Mesh *mesh, GLuint bufferUsageHint)
{
	int vertexCount = mesh->Vertices.size();
	int indicesCount = mesh->Indices.size();
	if (vertexCount < MIN_VERTEX_COUNT || vertexCount > MAX_VERTEX_COUNT)
	{
		throw std::invalid_argument("\"vertexCount\" value out of range");
	}
	BufferVertexInfo = vertexInfo;
	VertexCount = vertexCount;
	IndicesCount = indicesCount;
	BufferUsageHint = BufferUsageHint;


	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);
	// Bind the Vertex Array Object first, then bind and set vertex buffer(s) and attribute pointer(s).
	glBindVertexArray(VAO);


	GLfloat vertices[] = {
		 0.5f,  0.5f, 0.0f,  // Top Right
		 1,  0, 0, 1.0f,  // Top Right

		 0.5f, -0.5f, 0.0f,  // Bottom Right
		 0,  1, 0, 1.0f,  // Top Right

		-0.5f, -0.5f, 0.0f,  // Bottom Left
		 0,  0, 1, 1.0f,  // Top Right

		-0.5f,  0.5f, 0.0f,   // Top Left 
		 1,  1, 1, 1.0f,  // Top Right

	};
	GLuint indices[] = {  // Note that we start from 0!
		0, 1, 3,  // First Triangle
		1, 2, 3   // Second Triangle
	};


	const int _size = mesh->GetVerticesArraySize();
	float arr[512];
	mesh->GetVerticesArray(arr);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, VertexCount * BufferVertexInfo->Size, &arr[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, IndicesCount * sizeof(int), &mesh->Indices[0], GL_STATIC_DRAW);



	//glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

}

VertexBufferObject::~VertexBufferObject()
{
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &EBO);
}

void VertexBufferObject::SetData(float* data, int count)
{
	VertexCount = count;
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, VertexCount * BufferVertexInfo->Size, &data[0]);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void VertexBufferObject::SetIndices(int data[], int count)
{
	IndicesCount = count;
	glGenBuffers(1, &EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, IndicesCount * sizeof(int), &data[0]);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void VertexBufferObject::Bind()
{
	glBindVertexArray(VBO);
	
	int vertexSize = BufferVertexInfo->Size;
	int attributes = Vertex::Info.Attributes.size();
	for (int i = 0; i < attributes; i++) {
		glVertexAttribPointer(
			Vertex::Info.Attributes[i].Index,
			Vertex::Info.Attributes[i].Count,
			GL_FLOAT,
			GL_FALSE,
			vertexSize,
			(GLvoid*)(Vertex::Info.Attributes[i].Offset)
		);
		glEnableVertexAttribArray(Vertex::Info.Attributes[i].Index);
	}


	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)0);
	//glEnableVertexAttribArray(0);
	//glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(float)));
	//glEnableVertexAttribArray(1);

	glBindVertexArray(0);
}

void VertexBufferObject::Draw()
{
	//glBindVertexArray(VertexArrayHandle);
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBufferHandle);

	//glDrawElements(PrimitiveType, 6, GL_UNSIGNED_INT, (void*)0);


	glBindVertexArray(VAO);
	//glDrawArrays(GL_TRIANGLES, 0, 6);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}
