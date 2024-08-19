#pragma once

#include "Vector3.h";
#include "VoxelColor.h"
#include <cstdarg>


enum VoxelStateIndex
{
	FillState = 0,
	Fullfilled = 1,
	Surrounded = 2,
	Opacity = 3,
	Divided = 4,
	Temp = 7
};

class Octree
{
public:
	static constexpr unsigned int FLOAT_PER_OCTREE = 12;
	static constexpr unsigned int MAX_INDEX = 9;
	unsigned char Index = 0;
	//static float InitialSize = Chunk.CHUNK_EDGE;
	static constexpr float INITIAL_SIZE = 32.0f;
	unsigned char State = 0;
	VoxelColor Color;
	Vector3 Position;

	static Vector3 DIAGONAL_DIRECTIONS[];

	Octree *Leafs[8];
	Octree *Parent;

	bool operator==(Octree b);
	bool operator!=(Octree b);

	Octree(int index, Vector3 position, VoxelColor color, char state, Octree* parent = nullptr);
	~Octree();

	bool IsLeaf();
	bool IsRoot();

	bool GetState(char index);
	void SetState(bool value, char index);

	float GetNodeSize();
	static float GetSizeByIndex(int index = 0);
	int GetOctreeSize();
	int GetLowestIndex();
	int GetLeafsCount();

	void OptimizeToTop();
	static void OptimizeToTop(Octree* oct);

	void OptimizeToBottom();
	static void OptimizeToBottom(Octree* oct);

	bool DivideNode();
	void DivideLowestLeafs();
	void RemoveLeafs();
	void AverageColor();
	void AverageFillState();

	void OctreeToArray(float* data, int ptr = 8);

	bool IsPointInside(Vector3 point);
	Octree* OctreeWithPointInside(Vector3 point);



	static int octCreated;
	static int octDeleted;


private:
	float GetOctreeData(char nodeRelativePositionIndex = 0);
};

