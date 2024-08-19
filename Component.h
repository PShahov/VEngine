#pragma once
#include "GameObject.h"
class Component
{
public:
	class GameObject *gameObject = nullptr;

	Component();
	~Component();
	
	virtual void Start();
	virtual void Update();

private:
	bool _isStarted = false;


};

