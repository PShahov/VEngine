#pragma once
namespace pbl {

	#define E_PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062
	#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062
	#define PI_OVER_4 0.7859382

	template<typename T> // where T : Component - no equivalent
	T Abs(T value) {
		long i = (*(long*)&value) & ~((unsigned int)1 << 31);
		return *(T*)&i;
	}

	float RadiansToDegrees(float rad);
	float DegreesToRadians(float deg);
	int powi(int x, unsigned int p);
}

