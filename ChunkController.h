#pragma once
#include "TextureBufferObject.h";
#include "Chunk.h";
#include "Vector3.h";
#include "AssetsController.h";

static class ChunkController
{
public:

	static int AREA_SIZE_X;
	static int AREA_SIZE_Y;
	static int AREA_SIZE_Z;
	
	static int TOTAL_CHUNKS;

	static bool async_mode;

	static TextureBufferObject *tbo;
	static TextureBufferObject *tbo2;

	static Chunk *Chunks[1];

	static void GenerateArea(Vector3 center);

	static void Tick();


private:
	static void ForceChunksToGpu();
	static float* GetFloats();
};

