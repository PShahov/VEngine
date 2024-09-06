#pragma once
#include <string>
class VoxelColor
{
public:
	unsigned char Color[4];

	VoxelColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255);
	VoxelColor(unsigned int hex = 0xff0000ff);
	VoxelColor(float value);
	
	~VoxelColor();

	void Set(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255);

	float ToFloat(bool backwards = false);
	void Invert();

	std::string ToString();

	bool operator == (VoxelColor b);
	bool operator != (VoxelColor b);
};

