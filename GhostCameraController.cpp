#define GLEW_STATIC

#include "GhostCameraController.h"

GhostCameraController::GhostCameraController(GameObject* parent)
{
	gameObject = parent;
}

void GhostCameraController::Update()
{
	if (Input::IsKeyPressed(GLFW_KEY_1)) sendAlive = !sendAlive;
	if (Input::IsKeyPressed(GLFW_KEY_2)) sendAlive = !sendMousePos;

	Vector3 dir = Vector3::zero;
	dir.z += Input::IsKeyDown(GLFW_KEY_W) ? 1 : 0;
	dir.z += Input::IsKeyDown(GLFW_KEY_S) ? -1 : 0;

	dir.y += Input::IsKeyDown(GLFW_KEY_SPACE) ? 1 : 0;
	dir.y += Input::IsKeyDown(GLFW_KEY_C) ? -1 : 0;

	dir.x += Input::IsKeyDown(GLFW_KEY_A) ? -1 : 0;
	dir.x += Input::IsKeyDown(GLFW_KEY_D) ? 1 : 0;

	dir = gameObject->transform->Rotation().Forward() * dir.z +
		gameObject->transform->Rotation().Left() * dir.x +
		gameObject->transform->Rotation().Up() * dir.y;

	if (Input::IsKeyDown(GLFW_KEY_LEFT_SHIFT)) dir = dir * 10;

	gameObject->transform->position = gameObject->transform->position + (dir * speed * Time::DeltaTime());

	dir = Input::MouseDelta();
	dir = dir * sensevity;

	if (Input::GetCursorMode() == GLFW_CURSOR_DISABLED) {

		gameObject->transform->rotation.RotateEuler(
			Vector3(0, 1, 0),
			dir.x * -1);

		gameObject->transform->rotation.RotateEuler(
			gameObject->transform->rotation.Left(),
			dir.y * -1);
	}




	AssetsController::Shaders["raymarch"]->Use();

	Vector3 forward = gameObject->transform->rotation.Forward();
	Vector3 right = gameObject->transform->rotation.Left();
	Vector3 up = gameObject->transform->rotation.Up();

	Vector3 pos = gameObject->transform->position;

	glUniform3f(AssetsController::Shaders["raymarch"]->GetParam("u_camera_position"), pos.x, pos.y, pos.z);
	glUniform3f(AssetsController::Shaders["raymarch"]->GetParam("u_camera_forward"), forward.x, forward.y, forward.z);
	glUniform3f(AssetsController::Shaders["raymarch"]->GetParam("u_camera_right"), right.x, right.y, right.z);
	glUniform3f(AssetsController::Shaders["raymarch"]->GetParam("u_camera_up"), up.x, up.y, up.z);
	//glUniform1i(AssetsController::Shaders["raymarch"]->GetParam("u_chunks_count"), ChunkController.TOTAL_CHUNKS);

	/*if (ghost.sendMousePos)
		GL.Uniform2(AssetsController::Shaders["raymarch"]->GetParam("u_mouse"), this.MousePosition);
	if (ghost.sendAlive)
		glUniform1i(AssetsController::Shaders["raymarch"]->GetParam("u_time"), Time.alive);*/
}
