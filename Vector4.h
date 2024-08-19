#pragma once
#include <string>
#include <sstream>

#include <math.h>
#include "pbl_math.h"
class Vector4
{
private:
	const float kEpsilon = 0.00001F;
	const float kEpsilonNormalSqrt = 1e-15F;

public:
	float x;
	float y;
	float z;
	float w;


	static Vector4 zero;
	static Vector4 one;
	static Vector4 half;
	static Vector4 up;
	static Vector4 down;
	static Vector4 left;
	static Vector4 right;
	static Vector4 forward;
	static Vector4 backward;

	Vector4(float n = 0);
	Vector4(float x, float y, float z, float w);
	Vector4(const Vector4& v);

	Vector4 operator +(Vector4 b);
	Vector4 operator -(Vector4 b);
	Vector4 operator *(Vector4 b);
	Vector4 operator /(Vector4 b);
	Vector4 operator -();
	Vector4 operator -(float d);
	Vector4 operator +(float d);
	Vector4 operator *(float d);
	Vector4 operator /(float d);
	Vector4 operator =(Vector4 b);
	//static Vector4 operator *(float d, Vector4 a);

	void Normalize();
	static Vector4 Normalize(Vector4 v);

	Vector4 Normalized();

	void ExcludeInfinity();
	static Vector4 ExcludeInfinity(Vector4 v);

	float Magnitude();
	static float Magnitude(Vector4 v);

	float SqrMagnitude();
	static float SqrMagnitude(Vector4 v);

	Vector4 Abs();
	static Vector4 Abs(Vector4 v);

	static float Dot(Vector4 v1, Vector4 v2);

	//static Vector4 Cross(Vector4 left, Vector4 right);

	std::string ToString() {
		std::ostringstream oss;
		oss << "X: " << x << "; Y: " << y << "; Z: " << z << "; W: " << w;
		return oss.str();
	}
};

