#pragma once
#include "Vector3.h"

class Quaternion
{
public:
	float x;
	float y;
	float z;
	float w;

	static constexpr float EPSILON = 0.000001F;
	static Quaternion identity;

	Quaternion operator *(Quaternion b);
	Quaternion operator +(Quaternion b);
	Quaternion operator -(Quaternion b);

	Vector3 operator *(Vector3 point);
	
	Quaternion();
	Quaternion(float x, float y, float z, float w);
	Quaternion(float roll, float pitch, float yaw);
	Quaternion(Vector3 axis, float rads);

	Quaternion(const Quaternion& v);


	float* ToArray();

	Vector3 Forward();
	Vector3 Up();
	Vector3 Right();

	Vector3 Backward();
	Vector3 Down();
	Vector3 Left();

	static Vector3 ToEuler(Quaternion q);
	static float Dot(Quaternion a, Quaternion b);
	
	static Quaternion Invert(Quaternion q);
	static Quaternion Normalize(Quaternion q);

	void Normalize();

	void RotateEuler(Vector3 axis, float eulerAngle);
	void RotateRadians(Vector3 axis, float radians);
};