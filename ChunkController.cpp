#include "ChunkController.h"

int ChunkController::AREA_SIZE_X = 1;
int ChunkController::AREA_SIZE_Y = 1;
int ChunkController::AREA_SIZE_Z = 1;

int ChunkController::TOTAL_CHUNKS = ChunkController::AREA_SIZE_X * ChunkController::AREA_SIZE_Y * ChunkController::AREA_SIZE_Z;

bool ChunkController::async_mode = false;

TextureBufferObject* ChunkController::tbo = nullptr;
TextureBufferObject* ChunkController::tbo2 = nullptr;

Chunk* ChunkController::Chunks[1] = { nullptr };

void ChunkController::GenerateArea(Vector3 center)
{
	center = center + Vector3((AREA_SIZE_X - 1) * 0.5f, (AREA_SIZE_Y - 1) * 0.5f, (AREA_SIZE_Z - 1) * 0.5f) * Chunk::CHUNK_EDGE;
	Chunks[AREA_SIZE_X];
	for (int x = 0; x < AREA_SIZE_X; x++) {
		for (int y = 0; y < AREA_SIZE_Y; y++) {
			for (int z = 0; z < AREA_SIZE_Z; z++) {
				Vector3 pos = center + Vector3(x * Chunk::CHUNK_EDGE, y * Chunk::CHUNK_EDGE, z * Chunk::CHUNK_EDGE);
				Chunks[x] = new Chunk(pos);
			}
		}
	}
}

void ChunkController::Tick()
{
	bool pushNeeded = false;

	for (int x = 0; x < AREA_SIZE_X; x++) {
		for (int y = 0; y < AREA_SIZE_Y; y++) {
			for (int z = 0; z < AREA_SIZE_Z; z++) {
				if (Chunks[x]->flag_dataUpdateNeeded) {
					Chunks[x]->UpdateData();
				}
				if (Chunks[x]->flag_dataPushNeeded) {
					pushNeeded = true;
					Chunks[x]->flag_dataPushNeeded = false;
				}
			}
		}
	}

	if (pushNeeded)
	{
		ForceChunksToGpu();
	}
}

void ChunkController::ForceChunksToGpu()
{
	AssetsController::Shaders["raymarch"]->Use();
	if (tbo == nullptr) {
		tbo = new TextureBufferObject(1);
		tbo2 = new TextureBufferObject(1);
	}

	glUniform1i(AssetsController::Shaders["raymarch"]->GetParam("u_object_size"), TOTAL_CHUNKS);
	tbo->setData(GetFloats(), Chunks[0]->DataLength);
	Shader *sh = AssetsController::Shaders["raymarch"];
	tbo->Use(sh);
}

float* ChunkController::GetFloats()
{
	//float* data = Chunks[0].GetData();
	return Chunks[0]->GetData();
	return nullptr;
}
