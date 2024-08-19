#pragma once
#include "Vector3.h"
#include "Quaternion.h"
#include "GameObject.h"
class Transform
{
public:
	Vector3 position;
	Vector3 localPosition;

	Quaternion rotation;
	Quaternion localRotation;

	Vector3 scale;
	Vector3 localScale;

	class Transform* parent = nullptr;
	class GameObject* gameObject = nullptr;

	Transform();

	Vector3 Position();
	Vector3 LocalPosition();

	Quaternion Rotation();
	Quaternion LocalRotation();

	Vector3 Scale();
	Vector3 LocalScale();

	class Transform* Parent();
	class GameObject* GameObject();

	void RotateAround(Vector3 point, Quaternion rotation);
};

