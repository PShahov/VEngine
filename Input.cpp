#define GLEW_STATIC

#include "Input.h"

int	Input::key_state[512];
int	Input::mouse_button_state[8];
int Input::_cursor_mode = GLFW_CURSOR_NORMAL;

Vector3 Input::_MousePosition = Vector3(0);
Vector3 Input::_MouseDelta = Vector3(0);

GLFWwindow* Input::window = 0;

void Input::InputStartup(GLFWwindow* window)
{
	/* Keyboard input callback */
	glfwSetKeyCallback(window, Input::key_callback);
	glfwSetCursorPosCallback(window, Input::cursor_position_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);

	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	_MousePosition.x = (float)xpos;
	_MousePosition.y = (float)ypos;

	Input::window = window;
}

bool Input::IsKeyPressed(int key)
{
	return key_state[key] == PBL_KEYSTATE_PRESSED;
}

bool Input::IsKeyDown(int key)
{
	return key_state[key] == PBL_KEYSTATE_DOWN || key_state[key] == PBL_KEYSTATE_PRESSED;
}

bool Input::IsKeyReleased(int key)
{
	return key_state[key] == PBL_KEYSTATE_RELEASED;
}

void Input::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT) {
		switch (key_state[key]) {
			case PBL_KEYSTATE_UP: {
				key_state[key] = PBL_KEYSTATE_PRESSED;
				break;
			}
			case PBL_KEYSTATE_DOWN: {
				key_state[key] = PBL_KEYSTATE_DOWN;
				break;
			}
			case PBL_KEYSTATE_PRESSED: {
				key_state[key] = PBL_KEYSTATE_DOWN;
				break;
			}
			case PBL_KEYSTATE_RELEASED: {
				key_state[key] = PBL_KEYSTATE_PRESSED;
				break;
			}
		}
	}
	else if(action == GLFW_RELEASE) {
		key_state[key] = PBL_KEYSTATE_RELEASED;
	}
}

void Input::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT) {
		switch (mouse_button_state[button]) {
		case PBL_KEYSTATE_UP: {
			mouse_button_state[button] = PBL_KEYSTATE_PRESSED;
			break;
		}
		case PBL_KEYSTATE_DOWN: {
			mouse_button_state[button] = PBL_KEYSTATE_DOWN;
			break;
		}
		case PBL_KEYSTATE_PRESSED: {
			mouse_button_state[button] = PBL_KEYSTATE_DOWN;
			break;
		}
		case PBL_KEYSTATE_RELEASED: {
			mouse_button_state[button] = PBL_KEYSTATE_PRESSED;
			break;
		}
		}
	}
	else if (action == GLFW_RELEASE) {
		mouse_button_state[button] = PBL_KEYSTATE_RELEASED;
	}
}

bool Input::IsMouseButtonPressed(int key)
{
	return mouse_button_state[key] == PBL_KEYSTATE_PRESSED;
}

bool Input::IsMouseButtonDown(int key)
{
	return mouse_button_state[key] == PBL_KEYSTATE_DOWN || mouse_button_state[key] == PBL_KEYSTATE_PRESSED;
}

bool Input::IsMouseButtonReleased(int key)
{
	return mouse_button_state[key] == PBL_KEYSTATE_RELEASED;
}

Vector3 Input::MousePosition()
{
	return Input::_MousePosition;
}

Vector3 Input::MouseDelta()
{
	return Input::_MouseDelta;
}

void Input::SetCursorMode(int mode)
{
	_cursor_mode = mode;
	glfwSetInputMode(window, GLFW_CURSOR, mode);
}

int Input::GetCursorMode()
{
	return _cursor_mode;
}

void Input::key_tick()
{
	for (int i = 0; i < 512; i++) {
		switch (key_state[i])
		{
		case PBL_KEYSTATE_RELEASED: {
			key_state[i] = PBL_KEYSTATE_UP;
			break;
		}
		case PBL_KEYSTATE_PRESSED: {
			key_state[i] = PBL_KEYSTATE_DOWN;
			break;
		}
		}
	}
	for (int i = 0; i < 8; i++) {
		switch (mouse_button_state[i])
		{
		case PBL_KEYSTATE_RELEASED: {
			mouse_button_state[i] = PBL_KEYSTATE_UP;
			break;
		}
		case PBL_KEYSTATE_PRESSED: {
			mouse_button_state[i] = PBL_KEYSTATE_DOWN;
			break;
		}
		}
	}
	Input::_MouseDelta = Vector3::zero;
}

void Input::cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
	_MouseDelta.x = (float)xpos - _MousePosition.x;
	_MouseDelta.y = (float)ypos - _MousePosition.y;

	_MousePosition.x = (float)xpos;
	_MousePosition.y = (float)ypos;
}
