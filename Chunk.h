#pragma once
#include "Octree.h"

class Chunk
{
public:
	static constexpr int CHUNK_EDGE = 32;
	static constexpr int MAX_DIVIDE_LAYER = 9;
	static constexpr int MAX_BLOCK_EDGE = 512;

	Octree *ChunkOctree;
	Vector3 Position;
	int DataOffset;
	float* Data;
	int DataLength = 0;

	bool flag_dataUpdateNeeded;
	bool flag_dataPushNeeded;

	Chunk(Vector3 position);
	~Chunk();
	void RegenChunk();
	void UpdateData();
	float* GetData();
};

