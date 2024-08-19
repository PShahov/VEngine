#include "VertexInfo.h"

VertexInfo::VertexInfo(std::vector< VertexAttribute> attributes){

	Size = 0;

	Attributes = attributes;

	for (int i = 0; i < Attributes.size(); i++)
	{
		Size += Attributes[i].Count * sizeof(float);
	}
}
