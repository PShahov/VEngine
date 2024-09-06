#pragma once
#include <chrono>


class Time
{
using clock = std::chrono::system_clock;
using dns = std::chrono::duration<double, std::nano>;
using tpns = std::chrono::time_point<clock, dns>;

public:
	static float DeltaTime();
	static float Alive();

	static void Tick();

	static float Now();

	static void Load();

private:
	static double _deltaTime;
	static double _alive;
	static double _prime;
};

