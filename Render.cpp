#include "Render.h"

void Render::Tick()
{
	if (Input::IsKeyPressed(88))
	{
		VISUALIZE_ITERATIONS_MODE++;
	}
	if (Input::IsKeyPressed(GLFW_KEY_Z))
	{
		DEV_BACKTRACKING_MODE++;
	}
	//if (Input::IsKeyPressed(GLFW_KEY_Z))
	//{
	//	CHUNK_RENDER_VARIANT++;
	//}
	if (Input::IsKeyPressed(GLFW_KEY_F3)) {
		swapInterval = !swapInterval;
	}

	UpdateValues();
}

void Render::UpdateValues()
{
	glUniform1i(AssetsController::Shaders["raymarch"]->GetParam("VISUALIZE_ITERATIONS_MODE"), VISUALIZE_ITERATIONS_MODE);
	glUniform1i(AssetsController::Shaders["raymarch"]->GetParam("DEV_BACKTRACKING_MODE"), DEV_BACKTRACKING_MODE);
	glUniform1i(AssetsController::Shaders["raymarch"]->GetParam("CHUNK_RENDER_VARIANT"), CHUNK_RENDER_VARIANT);
}
