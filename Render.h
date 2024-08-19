#pragma once

#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include "AssetsController.h"
#include "Input.h"

class Render
{
public:
	bool swapInterval = true;
	int VISUALIZE_ITERATIONS_MODE = 0;
	int DEV_BACKTRACKING_MODE = 1;
	int CHUNK_RENDER_VARIANT = 0;


	int MAX_VISUALIZE_ITERATIONS_MODE = 3;
	int MAX_DEV_BACKTRACKING_MODE = 1;
	int MAX_CHUNK_RENDER_VARIANT = 0;

	void Tick();

	void UpdateValues();
};

