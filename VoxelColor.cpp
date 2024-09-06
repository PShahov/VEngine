#include "VoxelColor.h"

VoxelColor::VoxelColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
	Color[0] = r;
	Color[1] = g;
	Color[2] = b;
	Color[3] = a;
}

VoxelColor::VoxelColor(unsigned int hex)
{
	Color[3] = ((hex >> 24) & 0xff);
	Color[2] = ((hex >> 16) & 0xff);
	Color[1] = ((hex >> 8) & 0xff);
	Color[0] = ((hex >> 0) & 0xff);
}

VoxelColor::VoxelColor(float value)
{
	unsigned int hex = *(int*)&value;

	Color[3] = ((hex >> 24) & 0xff);
	Color[2] = ((hex >> 16) & 0xff);
	Color[1] = ((hex >> 8) & 0xff);
	Color[0] = ((hex >> 0) & 0xff);
}

VoxelColor::~VoxelColor()
{
}

void VoxelColor::Set(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
	Color[0] = r;
	Color[1] = g;
	Color[2] = b;
	Color[3] = a;
}

float VoxelColor::ToFloat(bool backwards)
{
	unsigned int i;
	if(backwards)
		i = Color[0] | (Color[1] << 8) | (Color[2] << 16) | (Color[3] << 24);
	else
		i = Color[3] | (Color[2] << 8) | (Color[1] << 16) | (Color[0] << 24);

	return *(float*)&i;
}

void VoxelColor::Invert()
{
	Color[0] = ~Color[0];
	Color[1] = ~Color[1];
	Color[2] = ~Color[2];
	Color[3] = ~Color[3];
}

bool VoxelColor::operator==(VoxelColor b)
{
	return (Color[0] == b.Color[0] && Color[1] == b.Color[1] && Color[2] == b.Color[2] && Color[3] == b.Color[3]);
}

bool VoxelColor::operator!=(VoxelColor b)
{
	return !(*this == b);
}
