#pragma once

#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>;

#include "Vector3.h";

#define PBL_KEYSTATE_UP 0
#define PBL_KEYSTATE_DOWN 1
#define PBL_KEYSTATE_RELEASED 3
#define PBL_KEYSTATE_PRESSED 2

class Input
{
public:
	static void InputStartup(GLFWwindow* window);

	static bool IsKeyPressed(int key);
	static bool IsKeyDown(int key);
	static bool IsKeyReleased(int key);

	static bool IsMouseButtonPressed(int key);
	static bool IsMouseButtonDown(int key);
	static bool IsMouseButtonReleased(int key);

	static Vector3 MousePosition();
	static Vector3 MouseDelta();

	static void SetCursorMode(int mode);
	static int GetCursorMode();


	static void key_tick();
private:
	static int key_state[];
	static int mouse_button_state[];

	static Vector3 _MousePosition;
	static Vector3 _MouseDelta;

	static int _cursor_mode;

	static GLFWwindow* window;

	static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
	static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
};

