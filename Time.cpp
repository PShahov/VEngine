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
	double _p = (double)tp.time_since_epoch().count() / 1000 / 1000 / 1000;
	_deltaTime = _p - _prime;
	_prime = _p;
	_alive =  _alive + _deltaTime;
}

float Time::Now()
{
	tpns tp = clock::now();
	return (float)tp.time_since_epoch().count() / 1000 / 1000 / 1000;
}

void Time::Load()
{
	tpns tp = clock::now();
	_prime = (double)tp.time_since_epoch().count() / 1000 / 1000 / 1000;
}
