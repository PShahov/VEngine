#define GLEW_STATIC

#include "Shader.h"


Shader::Shader()
{
	handle = 0;
}

Shader::Shader(std::string vertexPath, std::string fragmentPath)
{
	handle = 0;
	std::ostringstream VertexShaderSource;
	std::ostringstream FragmentShaderSource;

	{
		std::ifstream t(vertexPath);
		std::stringstream buffer;
		buffer << t.rdbuf();
		VertexShaderSource << buffer.str() << "\n\n";
	}

	{
		std::ifstream t(fragmentPath);
		std::stringstream buffer;
		buffer << t.rdbuf();
		FragmentShaderSource << buffer.str() << "\n\n";
	}

	this->CreateShaderFromSource(VertexShaderSource.str(), FragmentShaderSource.str());
}

Shader::Shader(const char* vertexShader, const char* fragmentShader)
{

	this->CreateShaderFromSource(vertexShader, fragmentShader);
}

Shader::Shader(std::vector<std::string> fragmentPathes, std::vector<std::string> vertexPathes, std::string vertexCompiled, std::string fragmentCompiled)
{
	std::string VertexShaderSource;
	std::string FragmentShaderSource;

	for(std::string vertexPath : vertexPathes)
	{
		std::ifstream t(vertexPath);
		std::stringstream buffer;
		buffer << t.rdbuf();


		VertexShaderSource += buffer.str() + "\n\n";
	}


	for (std::string fragmentPath : fragmentPathes)
	{
		std::ifstream t(fragmentPath);
		std::stringstream buffer;
		buffer << t.rdbuf();


		FragmentShaderSource += buffer.str() + "\n\n";

	}

	this->CreateShaderFromSource(VertexShaderSource, FragmentShaderSource);
}

Shader::~Shader()
{
	glDeleteProgram(handle);
}

void Shader::ReloadShader(std::vector<std::string> fragmentPathes, std::vector<std::string> vertexPathes, std::string vertexCompiled, std::string fragmentCompiled)
{
	glDeleteProgram(handle);

	std::string VertexShaderSource;
	std::string FragmentShaderSource;

	for (std::string vertexPath : vertexPathes)
	{
		std::ifstream t(vertexPath);
		std::stringstream buffer;
		buffer << t.rdbuf();


		VertexShaderSource += buffer.str() + "\n\n";
	}


	for (std::string fragmentPath : fragmentPathes)
	{
		std::ifstream t(fragmentPath);
		std::stringstream buffer;
		buffer << t.rdbuf();


		FragmentShaderSource += buffer.str() + "\n\n";

	}

	GLuint nHandle = 0;
	const char* vss = VertexShaderSource.c_str();
	const char* fss = FragmentShaderSource.c_str();
	nHandle = glCreateProgram();


	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vss, NULL);
	glCompileShader(vertexShader);
	CheckStatus(vertexShader, true);
	glAttachShader(nHandle, vertexShader);
	glDeleteShader(vertexShader);


	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fss, NULL);
	glCompileShader(fragmentShader);
	CheckStatus(fragmentShader, true);
	glAttachShader(nHandle, fragmentShader);
	glDeleteShader(fragmentShader);
	glLinkProgram(nHandle);
	CheckStatus(nHandle, false);

	glDeleteProgram(handle);
	handle = nHandle;
}

void Shader::Use()
{
	GLint prog = 0;
	glGetIntegerv(GL_CURRENT_PROGRAM, &prog);
	if(handle != prog)
		glUseProgram(handle);
}

GLuint Shader::GetParam(const char* name)
{
	return glGetUniformLocation(handle, name);
}

GLuint Shader::Handle()
{
	return handle;
}

void Shader::CheckStatus(GLuint obj, bool isShader)
{
	GLint status = GL_FALSE, log[1 << 11] = { 0 };
	(isShader ? glGetShaderiv : glGetProgramiv)(obj, isShader ? GL_COMPILE_STATUS : GL_LINK_STATUS, &status);
	if (status == GL_TRUE) return;
	GLint size = 0;
	(isShader ? glGetShaderInfoLog : glGetProgramInfoLog)(obj, sizeof(log), &size, (GLchar*)log);
	std::cout << (GLchar*)log << "\n";
	std::exit(EXIT_FAILURE);
}

void Shader::CreateShaderFromSource(std::string vertexShaderSource, std::string fragmentShaderSource)
{
	const char* vss = vertexShaderSource.c_str();
	const char* fss = fragmentShaderSource.c_str();
	handle = glCreateProgram();


	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vss, NULL);
	glCompileShader(vertexShader);
	CheckStatus(vertexShader, true);
	glAttachShader(handle, vertexShader);
	glDeleteShader(vertexShader);


	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fss, NULL);
	glCompileShader(fragmentShader);
	CheckStatus(fragmentShader, true);
	glAttachShader(handle, fragmentShader);
	glDeleteShader(fragmentShader);

	glLinkProgram(handle);
	CheckStatus(handle, false);

	//GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	//glShaderSource(vertexShader, 1, &vss, NULL);
	//glCompileShader(vertexShader);
	//Shader::CheckStatus(vertexShader, true);
	//glAttachShader(handle, GL_VERTEX_SHADER);
	//glDeleteShader(vertexShader);

	//GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	//glShaderSource(fragmentShader, 1, &fss, NULL);
	//glCompileShader(fragmentShader);
	//Shader::CheckStatus(fragmentShader, true);
	//glAttachShader(handle, GL_FRAGMENT_SHADER);
	//glDeleteShader(fragmentShader);

	//glLinkProgram(handle);
	//CheckStatus(handle, false);

	////glDetachShader(handle, GL_VERTEX_SHADER);
	////glDetachShader(handle, GL_FRAGMENT_SHADER);

	GLchar log[512] = { 0 };
	GLint size = 0;

	glGetProgramInfoLog(handle, GL_INFO_LOG_LENGTH, &size, log);
	if (size > 0)
		std::cout << log << "\n";

	glGetShaderInfoLog(vertexShader, GL_INFO_LOG_LENGTH, &size, log);
	if (size > 0)
		std::cout << log << "\n";

	glGetShaderInfoLog(fragmentShader, GL_INFO_LOG_LENGTH, &size, log);
	if (size > 0)
		std::cout << log << "\n";

}