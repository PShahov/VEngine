#pragma once
#include "Component.h"
#include "Matrix4.h"
#include "pbl_math.h"

class Camera: public Component
{
public:
	float FOV = 90;
	int Width = 1920;
	int Height = 1080;

	static Camera* MainCamera;

	Camera(GameObject* parent, bool isMain = false);
	~Camera();

	void Start() override;
	void Update() override;

	Matrix4 View;
	Matrix4 Projection;
};

