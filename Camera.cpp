#include "Camera.h"

Camera* Camera::MainCamera = nullptr;

Camera::Camera(GameObject* parent, bool isMain)
{
	if (isMain || Camera::MainCamera == nullptr) Camera::MainCamera = this;
	gameObject = parent;

	View = Matrix4::LookAt(gameObject->transform->Position(), gameObject->transform->Position() + gameObject->transform->Rotation().Forward(), gameObject->transform->Rotation().Up());
	Projection = Matrix4::CreatePerspectiveFieldOfView(PI_OVER_4, Width / Height, 0.01f, 1000);
}

Camera::~Camera()
{
	if (Camera::MainCamera == this) Camera::MainCamera = nullptr;
}

void Camera::Start()
{
	View = Matrix4::LookAt(gameObject->transform->Position(), gameObject->transform->Position() + gameObject->transform->Rotation().Forward(), gameObject->transform->Rotation().Up());
	Projection = Matrix4::CreatePerspectiveFieldOfView(PI_OVER_4, Width / Height, 0.01f, 1000);
}
void Camera::Update()
{
	View = Matrix4::LookAt(gameObject->transform->Position(), gameObject->transform->Position() + gameObject->transform->Rotation().Forward(), gameObject->transform->Rotation().Up());
}
