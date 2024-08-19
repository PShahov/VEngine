#pragma once
#include "VertexAttribute.h"
#include <vector>

class VertexInfo
{
public:
	int Size;
	std::vector< VertexAttribute> Attributes;

	VertexInfo(std::vector< VertexAttribute> attributes);
};