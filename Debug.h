#pragma once

#include <stdexcept>;
#include <string>;
#include <math.h>;
#include <format>;

class Debug
{
private:
	static const std::string SizeSuffixes[];

public:
	static std::string SizeSuffix(unsigned long value, int decimalPlaces = 1);
};

