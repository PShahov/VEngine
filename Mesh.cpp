#include "Mesh.h"

void Mesh::GetVerticesArray(float *data)
{
	int vertCount = Vertices.size();
	int offset = 0;
	for (int i = 0; i < vertCount; i++)
	{
		float x = Vertices[i].Position.x;
		data[offset + 0] = x;
		data[offset + 1] = Vertices[i].Position.y;
		data[offset + 2] = Vertices[i].Position.z;

		data[offset + 3] = Vertices[i].Color.R;
		data[offset + 4] = Vertices[i].Color.G;
		data[offset + 5] = Vertices[i].Color.B;
		data[offset + 6] = Vertices[i].Color.A;

		offset += Vertices[i].Info.Size / sizeof(float);
	}
}

int Mesh::GetVerticesArraySize()
{
	return Vertices.size() * (Vertices[0].Info.Size / sizeof(float));
}
