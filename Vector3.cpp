#include "Vector3.h"

Vector3 Vector3::zero = Vector3(0);
Vector3 Vector3::one = Vector3(1);
Vector3 Vector3::half = Vector3(0.5f);
Vector3 Vector3::up = Vector3(0, 1, 0);
Vector3 Vector3::down = Vector3(0, -1, 0);
Vector3 Vector3::right = Vector3(1, 0, 0);
Vector3 Vector3::left = Vector3(-1, 0, 0);
Vector3 Vector3::forward = Vector3(0, 0, 1);
Vector3 Vector3::backward = Vector3(0, 0, -1);


Vector3::Vector3(float n)
{
	x = n;
	y = n;
	z = n;
}

Vector3::Vector3(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

Vector3::Vector3(const Vector3& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}

Vector3 Vector3::operator+(Vector3 b)
{
	Vector3* res = new Vector3(this->x,this->y,this->z);
	res->x += b.x;
	res->y += b.y;
	res->z += b.z;
	return *res;
}

Vector3 Vector3::operator-(Vector3 b)
{
	Vector3* res = new Vector3(this->x, this->y, this->z);
	res->x -= b.x;
	res->y -= b.y;
	res->z -= b.z;
	return *res;
}

Vector3 Vector3::operator*(Vector3 b)
{
	Vector3* res = new Vector3(this->x, this->y, this->z);
	res->x *= b.x;
	res->y *= b.y;
	res->z *= b.z;
	return *res;
}

Vector3 Vector3::operator/(Vector3 b)
{
	Vector3* res = new Vector3(this->x, this->y, this->z);
	res->x /= b.x;
	res->y /= b.y;
	res->z /= b.z;
	return *res;
}

Vector3 Vector3::operator-()
{
	Vector3* res = new Vector3(this->x, this->y, this->z);
	res->x *= -1;
	res->y *= -1;
	res->z *= -1;
	return *res;
}

Vector3 Vector3::operator-(float d)
{
	Vector3* res = new Vector3(this->x, this->y, this->z);
	res->x -= d;
	res->y -= d;
	res->z -= d;
	return *res;
}

Vector3 Vector3::operator+(float d)
{
	Vector3* res = new Vector3(this->x, this->y, this->z);
	res->x += d;
	res->y += d;
	res->z += d;
	return *res;
}

Vector3 Vector3::operator*(float d)
{
	Vector3* res = new Vector3(this->x, this->y, this->z);
	res->x *= d;
	res->y *= d;
	res->z *= d;
	return *res;
}

Vector3 Vector3::operator/(float d)
{
	Vector3* res = new Vector3(this->x, this->y, this->z);
	res->x /= d;
	res->y /= d;
	res->z /= d;
	return *res;
}

Vector3 Vector3::operator=(Vector3 b)
{
	this->x = b.x;
	this->y = b.y;
	this->z = b.z;
	return *this;
}


void Vector3::Normalize()
{
	float d = this->Magnitude();
	x /= d;
	y /= d;
	z /= d;
}

Vector3 Vector3::Normalize(Vector3 v)
{
	return (v / v.Magnitude());
}

Vector3 Vector3::Normalized()
{
	Vector3 v = *this;
	v.Abs();
	return v;
}

void Vector3::ExcludeInfinity()
{
	if (isinf(this->x)) this->x = 0;
	if (isinf(this->y)) this->y = 0;
	if (isinf(this->z)) this->z = 0;
}

Vector3 Vector3::ExcludeInfinity(Vector3 v)
{
	if (isinf(v.x)) v.x = 0;
	if (isinf(v.y)) v.y = 0;
	if (isinf(v.z)) v.z = 0;

	return v;
}

float Vector3::Magnitude()
{
	
	return sqrt(x * x + y * y + z * z);
}

float Vector3::Magnitude(Vector3 v)
{
	return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

float Vector3::SqrMagnitude()
{
	return (x * x + y * y + z * z);
}

float Vector3::SqrMagnitude(Vector3 v)
{
	return (v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector3 Vector3::Abs()
{
	x = pbl::Abs(x);
	y = pbl::Abs(y);
	z = pbl::Abs(z);

	return *this;
}

Vector3 Vector3::Abs(Vector3 v)
{
	v.x = pbl::Abs(v.x);
	v.y = pbl::Abs(v.y);
	v.z = pbl::Abs(v.z);

	return v;
}

float Vector3::Dot(Vector3 v1, Vector3 v2)
{
	return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}

Vector3 Vector3::Cross(Vector3 left, Vector3 right)
{
	return Vector3(left.y * right.z - left.z * right.y,
		left.z * right.x - left.x * right.z,
		left.x * right.y - left.y * right.x);
}
