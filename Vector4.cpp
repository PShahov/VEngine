#include "Vector4.h"

Vector4 Vector4::zero = Vector4(0, 0, 0, 0);

//static Vector4 zero;
//static Vector4 one;
//static Vector4 half;
//static Vector4 up;
//static Vector4 down;
//static Vector4 left;
//static Vector4 right;
//static Vector4 forward;
//static Vector4 backward;

Vector4::Vector4(float n)
{
	x = n;
	y = n;
	z = n;
	w = n;
}

Vector4::Vector4(float x, float y, float z, float w)
{
	this->x = x;
	this->y = y;
	this->z = z;
	this->w = w;
}

Vector4::Vector4(const Vector4& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
	w = v.w;
}

Vector4 Vector4::operator+(Vector4 b)
{
	Vector4 res =  Vector4(this->x, this->y, this->z, this->w);
	res.x += b.x;
	res.y += b.y;
	res.z += b.z;
	res.w += b.w;
	return res;
}

Vector4 Vector4::operator-(Vector4 b)
{
	this->x -= b.x;
	this->y -= b.y;
	this->z -= b.z;
	this->w -= b.w;
	return *this;
}

Vector4 Vector4::operator*(Vector4 b)
{
	this->x *= b.x;
	this->y *= b.y;
	this->z *= b.z;
	this->w *= b.w;
	return *this;
}

Vector4 Vector4::operator/(Vector4 b)
{
	this->x /= b.x;
	this->y /= b.y;
	this->z /= b.z;
	this->w /= b.w;
	return *this;
}

Vector4 Vector4::operator-()
{
	this->x *= -1;
	this->y *= -1;
	this->z *= -1;
	this->w *= -1;
	return *this;
}

Vector4 Vector4::operator-(float d)
{
	this->x -= d;
	this->y -= d;
	this->z -= d;
	this->w -= d;
	return *this;
}

Vector4 Vector4::operator+(float d)
{
	this->x += d;
	this->y += d;
	this->z += d;
	this->w += d;
	return *this;
}

Vector4 Vector4::operator*(float d)
{
	this->x *= d;
	this->y *= d;
	this->z *= d;
	this->w *= d;
	return *this;
}

Vector4 Vector4::operator/(float d)
{
	this->x /= d;
	this->y /= d;
	this->z /= d;
	this->w /= d;
	return *this;
}

Vector4 Vector4::operator=(Vector4 b)
{
	this->x = b.x;
	this->y = b.y;
	this->z = b.z;
	this->w = b.w;
	return *this;
}


void Vector4::Normalize()
{
	float d = this->Magnitude();
	x /= d;
	y /= d;
	z /= d;
	w /= d;
}

Vector4 Vector4::Normalize(Vector4 v)
{
	return (v / v.Magnitude());
}

Vector4 Vector4::Normalized()
{
	Vector4 v = *this;
	v.Abs();
	return v;
}

void Vector4::ExcludeInfinity()
{
	if (isinf(this->x)) this->x = 0;
	if (isinf(this->y)) this->y = 0;
	if (isinf(this->z)) this->z = 0;
	if (isinf(this->w)) this->w = 0;
}

Vector4 Vector4::ExcludeInfinity(Vector4 v)
{
	if (isinf(v.x)) v.x = 0;
	if (isinf(v.y)) v.y = 0;
	if (isinf(v.z)) v.z = 0;
	if (isinf(v.w)) v.w = 0;

	return v;
}

float Vector4::Magnitude()
{

	return sqrt(x * x + y * y + z * z + w * w);
}

float Vector4::Magnitude(Vector4 v)
{
	return sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
}

float Vector4::SqrMagnitude()
{
	return (x * x + y * y + z * z + w * w);
}

float Vector4::SqrMagnitude(Vector4 v)
{
	return (v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
}

Vector4 Vector4::Abs()
{
	x = pbl::Abs(x);
	y = pbl::Abs(y);
	z = pbl::Abs(z);
	w = pbl::Abs(w);

	return *this;
}

Vector4 Vector4::Abs(Vector4 v)
{
	v.x = pbl::Abs(v.x);
	v.y = pbl::Abs(v.y);
	v.z = pbl::Abs(v.z);
	v.w = pbl::Abs(v.w);

	return v;
}

float Vector4::Dot(Vector4 v1, Vector4 v2)
{
	return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z) + (v1.w * v2.w);
}
