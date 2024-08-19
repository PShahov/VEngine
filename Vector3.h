#pragma once
#include <string>
#include <sstream>

#include "pbl_math.h";
#include <math.h>

class Vector3
{
private:
	const float kEpsilon = 0.00001F;
	const float kEpsilonNormalSqrt = 1e-15F;

public:
	float x;
	float y;
	float z;
	

	static Vector3 zero;
	static Vector3 one;
	static Vector3 half;
	static Vector3 up;
	static Vector3 down;
	static Vector3 left;
	static Vector3 right;
	static Vector3 forward;
	static Vector3 backward;

	Vector3(float n = 0);
	Vector3(float x, float y, float z);
	Vector3(const Vector3& v);

	Vector3 operator +(Vector3 b);
	Vector3 operator -(Vector3 b);
	Vector3 operator *(Vector3 b);
	Vector3 operator /(Vector3 b);
	Vector3 operator -();
	Vector3 operator -(float d);
	Vector3 operator +(float d);
	Vector3 operator *(float d);
	Vector3 operator /(float d);
	Vector3 operator =(Vector3 b);
	//static Vector3 operator *(float d, Vector3 a);

	void Normalize();
	static Vector3 Normalize(Vector3 v);

	Vector3 Normalized();

	void ExcludeInfinity();
	static Vector3 ExcludeInfinity(Vector3 v);

	float Magnitude();
	static float Magnitude(Vector3 v);

	float SqrMagnitude();
	static float SqrMagnitude(Vector3 v);

	Vector3 Abs();
	static Vector3 Abs(Vector3 v);

	static float Dot(Vector3 v1, Vector3 v2);

	static Vector3 Cross(Vector3 left, Vector3 right);

	std::string ToString() {
		std::ostringstream oss;
		oss << "X: " << x << "; Y: " << y << "; Z: " << z;
		return oss.str();
	}
};

