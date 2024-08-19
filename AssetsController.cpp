#define GLEW_STATIC

#include "AssetsController.h"
#include "Chunk.h"
#include "Time.h"

std::string AssetsController::ShaderFolder = "hybrid";
//std::string AssetsController::CONTENT_DIR = "..\\..\\..\\assets\\";
std::string AssetsController::CONTENT_DIR = "assets\\";

std::map<std::string, Shader*> AssetsController::Shaders = {};
std::map<std::string, Mesh*> AssetsController::Meshes = {};
std::map<std::string, VertexBufferObject*> AssetsController::VBOs = {};



std::vector<std::string> AssetsController::ImportModels =
{
	"test.qb",//0
	"dragon.qb",//0
	"town.qb",//1
	"monu3.qb",//2
	"teapot_128.qb",//3
	"monu1_128.qb",//4
	"#red_booth_solid.qb",//5
	"town_1.qb",//6
	"8x8x8_2.qb",//7
	"#skyscraper_03_000-2.qb",//8
	"ghost.qb",//8
	"#christmas_scene.qb",//8
	"castle.qb",//9
};

void AssetsController::Load()
{
	//Shaders["test"] = new Shader(CONTENT_DIR + "shaders/dev/first_vert.glsl", CONTENT_DIR + "shaders/dev/first_frag.glsl");

	Shaders["test_rm"] = new Shader(CONTENT_DIR + "shaders/dev/first_vert.glsl", CONTENT_DIR + "shaders/dev/#version 330 core.glsl");


	std::vector<std::string> raymarch_fragment =
	{
		CONTENT_DIR + "shaders/build/" + ShaderFolder + "/header.glsl",
		CONTENT_DIR + "shaders/build/" + ShaderFolder + "/support.glsl",
		CONTENT_DIR + "shaders/build/" + ShaderFolder + "/vertex.glsl",
	};
	std::vector<std::string> raymarch_vertex =
	{
		CONTENT_DIR + "shaders/build/" + ShaderFolder + "/fragment.glsl"
	};

	Shaders["raymarch"] = new Shader(raymarch_fragment, raymarch_vertex, CONTENT_DIR + "shaders/compiled/vertex.glsl", CONTENT_DIR + "shaders/compiled/fragment.glsl");
}

void AssetsController::ReloadShaders()
{
	//Shaders["test_rm"]->Use();

	std::vector<std::string> raymarch_fragment =
	{
		CONTENT_DIR + "shaders/build/" + ShaderFolder + "/header.glsl",
		CONTENT_DIR + "shaders/build/" + ShaderFolder + "/support.glsl",
		CONTENT_DIR + "shaders/build/" + ShaderFolder + "/vertex.glsl",
	};
	std::vector<std::string> raymarch_vertex =
	{
		CONTENT_DIR + "shaders/build/" + ShaderFolder + "/fragment.glsl"
	};

	//Shaders["raymarch"] = new Shader(raymarch_fragment, raymarch_vertex, CONTENT_DIR + "shaders/compiled/vertex.glsl", CONTENT_DIR + "shaders/compiled/fragment.glsl");
	GLuint oldHandle = Shaders["raymarch"]->Handle();
	Shaders["raymarch"]->ReloadShader(raymarch_fragment, raymarch_vertex, CONTENT_DIR + "shaders/compiled/vertex.glsl", CONTENT_DIR + "shaders/compiled/fragment.glsl");
	std::cout << "Shader reloaded, old handle - " << oldHandle << ", new - " << Shaders["raymarch"]->Handle() << "\n";
}

Mesh* AssetsController::CreatePlane()
{
	std::vector<Vertex> vertices = {
		Vertex(Vector3(1, 1, 0), Color4(1,0,0,1)),//0
		Vertex(Vector3(-1, 1, 0), Color4(0,1,0,1)),//1
		Vertex(Vector3(-1, -1, 0), Color4(0,0,1,1)),//2
		Vertex(Vector3(1, 1, 0), Color4(1,0,0,1)),//0
		Vertex(Vector3(-1, -1, 0), Color4(0,0,1,1)),//2
		Vertex(Vector3(1, -1, 0), Color4(1,1,1,1)),//3
	};

	std::vector<int> indices =
		{
			0, 1, 2,
			3, 4, 5
		};
	Mesh *mesh = new Mesh(vertices, indices);
	return mesh;
}

Octree* AssetsController::LoadQB(std::string path) {
	float _loadTime = Time::Now();
	std::vector<unsigned char> qb = AssetsController::get_contents(CONTENT_DIR + path);

	Octree* rOct = new Octree(0, Vector3::zero, VoxelColor(0, 0, 0, 0), 0b00000011, nullptr);
	Octree* oct = rOct;

	int xSize = int((unsigned char)(qb[28]) << 24 |
		(unsigned char)(qb[27]) << 16 |
		(unsigned char)(qb[26]) << 8 |
		(unsigned char)(qb[25]));
	int ySize = int((unsigned char)(qb[32]) << 24 |
		(unsigned char)(qb[31]) << 16 |
		(unsigned char)(qb[30]) << 8 |
		(unsigned char)(qb[29]));
	int zSize = int((unsigned char)(qb[36]) << 24 |
		(unsigned char)(qb[35]) << 16 |
		(unsigned char)(qb[34]) << 8 |
		(unsigned char)(qb[33]));


	int minVoxelIndex = 0;
	{
		int t = xSize;
		while (t > 1)
		{
			t /= 2;
			minVoxelIndex++;
		}
	}
	float voxSize = Octree::GetSizeByIndex(minVoxelIndex);


	Vector3 pos;
	VoxelColor color = VoxelColor(0, 0, 0, 0);
	int byteIndex = 49;

	Octree tempOct = Octree(0, Vector3::zero, VoxelColor(0, 0, 0, 0), 0b00000011, nullptr);

	color = VoxelColor(qb[byteIndex], qb[byteIndex + 1], qb[byteIndex + 2], qb[byteIndex + 3]);
	bool voxFillState = color.Color[3] == 255;
	oct->SetState(voxFillState, VoxelStateIndex::FillState);
	oct->Color = color;

	for (int x = 0; x < xSize; x++)
	{
		for (int y = 0; y < ySize; y++)
		{
			for (int z = 0; z < zSize; z++)
			{
				if(x == 9 && y == 22 && z == 13)
				std::cout << x << "; " << y << "; " << z << "\n";
				pos = Vector3(
					(x * voxSize + (voxSize * 0.5f)),
					(y * voxSize + (voxSize * 0.5f)),
					(z * voxSize + (voxSize * 0.5f)));

				pos = pos - (Vector3::half * Chunk::CHUNK_EDGE);
				oct = rOct->OctreeWithPointInside(pos);

				int byteIndex = 49;
				int zShift = z * 4;
				int yShift = y * zSize * 4;
				int xShift = x * ySize * zSize * 4;

				byteIndex += xShift + yShift + zShift;

				color = VoxelColor(qb[byteIndex], qb[byteIndex + 1], qb[byteIndex + 2], qb[byteIndex + 3]);
				voxFillState = color.Color[3] == 255;
				tempOct.SetState(voxFillState, VoxelStateIndex::FillState);
				tempOct.Color = color;

				if (oct != &tempOct)
				{
					while (oct->Index < minVoxelIndex)
					{
						oct->DivideNode();
						oct->Color = tempOct.Color;
						oct = oct->OctreeWithPointInside(pos);
					}
					oct->State = tempOct.State;
					oct->Color = tempOct.Color;
				}



				//byteIndex += 4;
			}
		}
	}
	rOct->OptimizeToBottom();
	std::cout << rOct->GetOctreeSize() << "\n";
	std::cout << Time::Now() - _loadTime << "\n";

	std::cout << Octree::octCreated << "\n";
	std::cout << Octree::octDeleted << "\n";
	return rOct;
}

std::vector<unsigned char> AssetsController::get_contents(const std::string& path)
{
	std::ifstream source_file{ path, std::ios::binary };

	auto length{ std::filesystem::file_size(path) };
	std::vector<unsigned char> result(length); // Use parentheses so that we construct a vector of a certain size
	source_file.read(reinterpret_cast<char*>(result.data()), static_cast<long>(length));

	return result;
}