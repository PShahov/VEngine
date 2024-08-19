#include "Debug.h"

const std::string Debug::SizeSuffixes[] = { "b", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB" };

std::string Debug::SizeSuffix(unsigned long value, int decimalPlaces)
{
	if (decimalPlaces < 0) throw std::invalid_argument("\"decimalPlaces\" received negative value");
	if (value < 0) return "-" + (SizeSuffix(value * -1, decimalPlaces));
	if (value == 0) return "0 bytes";


	int mag = (int)(log(value) / log(1024));
	long double adjustedSize = (long double)value / (1L << (mag * 10));


	if (ceil(adjustedSize * pow(10, decimalPlaces + 1)) / pow(10, decimalPlaces + 1) >= 1000)
	{
		mag += 1;
		adjustedSize /= 1024;
	}
	return std::vformat("{:." + std::to_string(decimalPlaces) + "f}", std::make_format_args(adjustedSize)) + Debug::SizeSuffixes[mag];
}
