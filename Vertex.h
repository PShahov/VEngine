#pragma once
#include "Vector3.h"
#include "Color4.h"
#include "VertexInfo.h"
class Vertex
{
public:
	Vector3 Position;
	Color4 Color;

	static VertexInfo Info;

	Vertex();

	Vertex(Vector3 position, Color4 color);
};

