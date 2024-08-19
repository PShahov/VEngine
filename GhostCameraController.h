#pragma once
#define GLEW_STATIC

#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include "Component.h";
#include "Input.h"
#include "Time.h"
#include "AssetsController.h"

class GhostCameraController: public Component
{
private:

	float _lastClick = 0;
	float _clickCooldown = 0.1f;

public:
	float speed = 2.0f;
	float sensevity = 0.25f;

	bool sendAlive = true;
	bool sendMousePos = true;

	int VISUALIZE_ITERATIONS_MODE = 0;
	int DEV_BACKTRACKING_MODE = 1;
	int CHUNK_RENDER_VARIANT = 0;

	GhostCameraController(GameObject* parent);

	void Update() override;
};

