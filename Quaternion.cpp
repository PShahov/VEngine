#include "Quaternion.h"

Quaternion Quaternion::identity = Quaternion(0, 0, 0, 1);

Quaternion Quaternion::operator*(Quaternion b)
{
	return Quaternion(
		w * b.x + x * b.w + y * b.z - z * b.y,
		w * b.y - x * b.z + y * b.w + z * b.x,
		w * b.z + x * b.y - y * b.x + z * b.w,
		w * b.w - x * b.x - y * b.y - z * b.z
	);
}

Quaternion Quaternion::operator+(Quaternion b)
{
	return Quaternion::Normalize(Quaternion(
		x + b.x,
		y + b.y,
		z + b.z,
		w + b.w
	));
}

Quaternion Quaternion::operator-(Quaternion b)
{
	return Quaternion::Normalize(Quaternion(
		x - b.x,
		y - b.y,
		z - b.z,
		w - b.w
	));
}

Vector3 Quaternion::operator*(Vector3 point)
{
	float _x = x * 2.0F;
	float _y = y * 2.0F;
	float _z = z * 2.0F;
	float xx = x * _x;
	float yy = y * _y;
	float zz = z * _z;
	float xy = x * _y;
	float xz = x * _z;
	float yz = y * _z;
	float wx = w * _x;
	float wy = w * _y;
	float wz = w * _z;

	Vector3 res;
	res.x = (1.0F - (yy + zz)) * point.x + (xy - wz) * point.y + (xz + wy) * point.z;
	res.y = (xy + wz) * point.x + (1.0F - (xx + zz)) * point.y + (yz - wx) * point.z;
	res.z = (xz - wy) * point.x + (yz + wx) * point.y + (1.0F - (xx + yy)) * point.z;
	return res;
}

Quaternion::Quaternion()
{
	x = 0;
	y = 0;
	z = 0;
	w = 1;
}

Quaternion::Quaternion(float x, float y, float z, float w)
{
	this->x = x;
	this->y = y;
	this->z = z;
	this->w = w;
}

Quaternion::Quaternion(float roll, float pitch, float yaw)
{
	float cr = cosf(roll * 0.5f);
	float sr = sinf(roll * 0.5f);
	float cp = cosf(pitch * 0.5f);
	float sp = sinf(pitch * 0.5f);
	float cy = cosf(yaw * 0.5f);
	float sy = sinf(yaw * 0.5f);

	w = cr * cp * cy + sr * sp * sy;
	x = sr * cp * cy - cr * sp * sy;
	y = cr * sp * cy + sr * cp * sy;
	z = cr * cp * sy - sr * sp * cy;
}

Quaternion::Quaternion(Vector3 axis, float rads)
{
	float factor = sinf(rads / 2.0f);

	x = axis.x * factor;
	y = axis.y * factor;
	z = axis.z * factor;
	w = cosf(rads / 2.0f);

	Normalize();
}

Quaternion::Quaternion(const Quaternion& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
	w = v.w;
}

float* Quaternion::ToArray()
{
	static float arr[4] = { x,y,z,w };
	return arr;
}

Vector3 Quaternion::Forward()
{
	return *this * Vector3::forward;
}

Vector3 Quaternion::Up()
{
	return *this * Vector3::up;
}

Vector3 Quaternion::Right()
{
	return *this * Vector3::right;
}

Vector3 Quaternion::Backward()
{
	return -Forward();
}

Vector3 Quaternion::Down()
{
	return -Up();
}

Vector3 Quaternion::Left()
{
	return -Right();
}

Vector3 Quaternion::ToEuler(Quaternion q)
{
	Vector3 angles;

	// x axis
	float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
	float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
	angles.x = atan2(sinr_cosp, cosr_cosp);

	//y axis
	float sinp = 2 * (q.w * q.y - q.z * q.x);
	if (pbl::Abs(sinp) >= 1)
		angles.y = copysignf(E_PI / 2, sinp);
	else
		angles.y = asin(sinp);

	// z axis
	float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	angles.z = atan2(siny_cosp, cosy_cosp);

	return Vector3(pbl::RadiansToDegrees(angles.x), pbl::RadiansToDegrees(angles.y), pbl::RadiansToDegrees(angles.z));
}

float Quaternion::Dot(Quaternion a, Quaternion b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

Quaternion Quaternion::Invert(Quaternion q)
{
	Quaternion nq = Quaternion(-q.x, -q.y, -q.z, q.w);
	return Quaternion::Normalize(nq);
}

Quaternion Quaternion::Normalize(Quaternion q)
{
	float mag = sqrtf(Dot(q, q));

	if (mag < Quaternion::EPSILON)
		return Quaternion::identity;

	return Quaternion(q.x / mag, q.y / mag, q.z / mag, q.w / mag);
}

void Quaternion::Normalize()
{

	float mag = sqrtf(Dot(*this, *this));

	if (mag < Quaternion::EPSILON) {
		x = identity.x;
		y = identity.y;
		z = identity.z;
		w = identity.w;
	}

	x /= mag;
	y /= mag;
	z /= mag;
	w /= mag;
}

void Quaternion::RotateEuler(Vector3 axis, float eulerAngle)
{
	Quaternion q = Quaternion(axis, pbl::DegreesToRadians(eulerAngle)) * *this;
	x = q.x;
	y = q.y;
	z = q.z;
	w = q.w;
}

void Quaternion::RotateRadians(Vector3 axis, float radians)
{
	Quaternion q = Quaternion(axis, radians) * *this;
	x = q.x;
	y = q.y;
	z = q.z;
	w = q.w;
}
