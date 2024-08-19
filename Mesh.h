#pragma once
#include "Vertex.h";
#include <vector>

class Mesh
{
public:
	std::vector<int> Indices;
	std::vector<Vertex> Vertices;

	Mesh(std::vector<Vertex> verticies, std::vector<int> indices) : Vertices(verticies), Indices(indices) {};
	void GetVerticesArray(float *data);
	int GetVerticesArraySize();
};

