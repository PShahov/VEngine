#include "Time.h"



double Time::_deltaTime = 0;
double Time::_alive = 0;
double Time::_prime = 0;

float Time::DeltaTime()
{
	//return _deltaTime * 1000;
	return _deltaTime;
}

float Time::Alive()
{
	return _alive;
}

void Time::Tick()
{
	tpns tp = clock::now();
	_alive =(double)tp.time_since_epoch().count() / 1000 / 1000 / 1000;
	_deltaTime = _alive - _prime;
	_prime = _alive;
}

float Time::Now()
{
	tpns tp = clock::now();
	return (float)tp.time_since_epoch().count() / 1000 / 1000 / 1000;
}
