#include "pbl_math.h"

float pbl::RadiansToDegrees(float rad)
{
    return rad * 180 / E_PI;
}

float pbl::DegreesToRadians(float deg)
{
    return deg * E_PI / 180;
}

int pbl::powi(int x, unsigned int p)
{
    if (p == 0) return 1;
    if (p == 1) return x;

    int tmp = powi(x, p / 2);
    if (p % 2 == 0) return tmp * tmp;
    else return x * tmp * tmp;
}
