#include "GameObject.h"

GameObject::GameObject()
{
	transform = new Transform();
	transform->gameObject = this;
}

void GameObject::Start()
{
	int size = components.size();
	for (int i = 0; i < size; i++) {
		components[i]->Start();
	}
}

void GameObject::Update()
{
	int size = components.size();
	for (int i = 0; i < size; i++) {
		components[i]->Update();
	}
}
