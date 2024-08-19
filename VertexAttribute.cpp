#include "VertexAttribute.h"

VertexAttribute::VertexAttribute()
{
	Name = "";
	Index = 0;
	Count = 0;
	Offset = 0;
}

VertexAttribute::VertexAttribute(std::string name, int index, int count, int offset)
{
	Name = name;
	Index = index;
	Count = count;
	Offset = offset;
}
