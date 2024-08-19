#include "Chunk.h"

Chunk::Chunk(Vector3 position)
{
	Position = position;
	ChunkOctree = new Octree(0, position, VoxelColor(255, 255, 0), 0b00000011, nullptr);
	UpdateData();
}

Chunk::~Chunk()
{
}

void Chunk::RegenChunk()
{
	ChunkOctree = new Octree(0, Position, VoxelColor(255, 255, 0), 0b00000011, nullptr);
	UpdateData();
}

void Chunk::UpdateData()
{
	int childrenCount = ChunkOctree->GetOctreeSize();
	DataLength = (childrenCount * Octree::FLOAT_PER_OCTREE) + 8;
	delete Data;
	Data = new float[DataLength];
	ChunkOctree->OctreeToArray(Data);

	Data[0] = Position.x;
	Data[1] = Position.y;
	Data[2] = Position.z;

	Data[3] = Position.x;
	Data[4] = Position.y;
	Data[5] = Position.z;

	Data[6] = *(float*)&childrenCount;
	Data[7] = 0x00000001;

	flag_dataUpdateNeeded = false;
	flag_dataPushNeeded = true;
}

float* Chunk::GetData()
{
	return Data;
}
