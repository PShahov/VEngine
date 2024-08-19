#pragma once
#include <vector>
#include <vector>
#include "Transform.h"
#include "Component.h"
#include <map>

class GameObject
{
public:
	class Transform* transform = nullptr;

	GameObject();

	virtual void Start();
	virtual void Update();
    std::vector<class Component*> components;

private:
};