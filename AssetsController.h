#pragma once
#include <string>
#include <map>
#include "Shader.h"
#include "Mesh.h"
#include "VertexBufferObject.h"
#include "Octree.h"
#include <cstdint>
#include <filesystem>
#include <vector>

static class AssetsController
{
public:
	static std::string ShaderFolder;
	static std::string CONTENT_DIR;

	static std::map<std::string, Shader*> Shaders;
	static std::map<std::string, Mesh*> Meshes;

	static std::map<std::string, VertexBufferObject*> VBOs;

	static void Load();
	static void ReloadShaders();

	static Mesh* CreatePlane();
	static Octree* LoadQB(std::string path);

	static std::vector<std::string> ImportModels;



private:
	static std::vector<unsigned char> get_contents(const std::string& path);
};

