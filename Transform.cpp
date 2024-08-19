#include "Transform.h"

Transform::Transform()
{
    position = Vector3(0);
    localPosition = Vector3(0);

    scale = Vector3(1);
    localScale = Vector3(1);

    rotation = Quaternion(0,0,0);
    localRotation = Quaternion(0, 0, 0);
}

Vector3 Transform::Position()
{
    return position;
}

Vector3 Transform::LocalPosition()
{
    if (parent == nullptr) {
        return position;
    }
    else {
        return position - parent->position;
    }
}

Quaternion Transform::Rotation()
{
    return rotation;
}

Quaternion Transform::LocalRotation()
{
    if (parent == nullptr) {
        return rotation;
    }
    else {
        return rotation - parent->rotation;
    }
}

Vector3 Transform::Scale()
{
    return scale;
}

Vector3 Transform::LocalScale()
{
    return Vector3();
}

Transform* Transform::Parent()
{
    return parent;
}

GameObject* Transform::GameObject()
{
    return gameObject;
}

void Transform::RotateAround(Vector3 point, Quaternion rotation)
{
    position = rotation * (position - point) + point;
    rotation = rotation * rotation;
}
