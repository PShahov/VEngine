#pragma once
#include "Vector4.h"
#include "Vector3.h"
class Matrix4
{
public:
    Vector4 Row0;
    Vector4 Row1;
    Vector4 Row2;
    Vector4 Row3;

    static Matrix4 identity;

    Matrix4 operator*(Matrix4 right);

    static Matrix4 Mult(Matrix4 left, Matrix4 right);

    Matrix4();
    Matrix4(float n);
    Matrix4(Vector4 row0, Vector4 row1, Vector4 row2, Vector4 row3);
    Matrix4(float m00, float m01, float m02, float m03, float m10, float m11, float m12, float m13, float m20, float m21, float m22, float m23, float m30, float m31, float m32, float m33);

    static Matrix4 LookAt(Vector3 eye, Vector3 target, Vector3 up);
    static Matrix4 CreateTranslation(Vector3 vector);
    static Matrix4 CreatePerspectiveFieldOfView(float fovy, float aspect, float zNear, float zFar);
    static Matrix4 CreatePerspectiveOffCenter(float left, float right, float bottom, float top, float zNear, float zFar);
};

