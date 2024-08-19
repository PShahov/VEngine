#pragma once


#include <string>
#include <vector>

#include <iostream>
#include <fstream>
#include <sstream>

#include <GL/glew.h>

class Shader
{
public:
	Shader();
	Shader(std::string vertexPath, std::string fragmentPath);
	Shader(const char* vertexShader, const char * fragmentShader);
	Shader(std::vector<std::string> fragmentPathes, std::vector<std::string> vertexPathes, std::string vertexCompiled = "", std::string fragmentCompiled = "");

	~Shader();

	void ReloadShader(std::vector<std::string> fragmentPathes, std::vector<std::string> vertexPathes, std::string vertexCompiled = "", std::string fragmentCompiled = "");

	void Use();
	GLuint GetParam(const char* name);

	GLuint Handle();
	void CheckStatus(GLuint obj, bool isShader);

private:
	GLuint handle;

	void CreateShaderFromSource(std::string vertexShaderSource, std::string fragmentShaderSource);
};

