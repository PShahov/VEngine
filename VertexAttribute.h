#pragma once
#include <string>
class VertexAttribute
{
public:
	std::string Name;
	int Index;
	int Count;
	int Offset;
	VertexAttribute();
	VertexAttribute(std::string name, int index, int count, int offset);
};

