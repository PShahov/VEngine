#include "Component.h"

Component::Component()
{
}

Component::~Component()
{
}

void Component::Start()
{
	_isStarted = true;
}

void Component::Update()
{
	if (!_isStarted) {
		Start();
		_isStarted = true;
	}
}
