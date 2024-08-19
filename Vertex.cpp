#include "Vertex.h"

VertexInfo Vertex::Info = VertexInfo(
	{
		VertexAttribute("Position", 0, 3, 0),
		VertexAttribute("Color", 1 ,4 ,3 * sizeof(float))
	}
);

Vertex::Vertex()
{
	Position = Vector3(0);
	Color = Color4(1,1,1,1);
}

Vertex::Vertex(Vector3 position, Color4 color)
{
	Position = position;
	Color = color;
}
