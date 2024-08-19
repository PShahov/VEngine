#include "Matrix4.h"

Matrix4 Matrix4::identity = Matrix4(
    Vector4(1, 0, 0, 0),
    Vector4(0, 1, 0, 0),
    Vector4(0, 0, 1, 0),
    Vector4(0, 0, 0, 1)
);

Matrix4 Matrix4::operator*(Matrix4 right)
{
    return Matrix4::Mult(*this, right);
}

Matrix4 Matrix4::Mult(Matrix4 left, Matrix4 right)
{
    Matrix4 result = Matrix4(0);
    float lM11 = left.Row0.x, lM12 = left.Row0.y, lM13 = left.Row0.z, lM14 = left.Row0.w,
        lM21 = left.Row1.x, lM22 = left.Row1.y, lM23 = left.Row1.z, lM24 = left.Row1.w,
        lM31 = left.Row2.x, lM32 = left.Row2.y, lM33 = left.Row2.z, lM34 = left.Row2.w,
        lM41 = left.Row3.x, lM42 = left.Row3.y, lM43 = left.Row3.z, lM44 = left.Row3.w,
        rM11 = right.Row0.x, rM12 = right.Row0.y, rM13 = right.Row0.z, rM14 = right.Row0.w,
        rM21 = right.Row1.x, rM22 = right.Row1.y, rM23 = right.Row1.z, rM24 = right.Row1.w,
        rM31 = right.Row2.x, rM32 = right.Row2.y, rM33 = right.Row2.z, rM34 = right.Row2.w,
        rM41 = right.Row3.x, rM42 = right.Row3.y, rM43 = right.Row3.z, rM44 = right.Row3.w;

    result.Row0.x = (((lM11 * rM11) + (lM12 * rM21)) + (lM13 * rM31)) + (lM14 * rM41);
    result.Row0.y = (((lM11 * rM12) + (lM12 * rM22)) + (lM13 * rM32)) + (lM14 * rM42);
    result.Row0.z = (((lM11 * rM13) + (lM12 * rM23)) + (lM13 * rM33)) + (lM14 * rM43);
    result.Row0.w = (((lM11 * rM14) + (lM12 * rM24)) + (lM13 * rM34)) + (lM14 * rM44);
    result.Row1.x = (((lM21 * rM11) + (lM22 * rM21)) + (lM23 * rM31)) + (lM24 * rM41);
    result.Row1.y = (((lM21 * rM12) + (lM22 * rM22)) + (lM23 * rM32)) + (lM24 * rM42);
    result.Row1.z = (((lM21 * rM13) + (lM22 * rM23)) + (lM23 * rM33)) + (lM24 * rM43);
    result.Row1.w = (((lM21 * rM14) + (lM22 * rM24)) + (lM23 * rM34)) + (lM24 * rM44);
    result.Row2.x = (((lM31 * rM11) + (lM32 * rM21)) + (lM33 * rM31)) + (lM34 * rM41);
    result.Row2.y = (((lM31 * rM12) + (lM32 * rM22)) + (lM33 * rM32)) + (lM34 * rM42);
    result.Row2.z = (((lM31 * rM13) + (lM32 * rM23)) + (lM33 * rM33)) + (lM34 * rM43);
    result.Row2.w = (((lM31 * rM14) + (lM32 * rM24)) + (lM33 * rM34)) + (lM34 * rM44);
    result.Row3.x = (((lM41 * rM11) + (lM42 * rM21)) + (lM43 * rM31)) + (lM44 * rM41);
    result.Row3.y = (((lM41 * rM12) + (lM42 * rM22)) + (lM43 * rM32)) + (lM44 * rM42);
    result.Row3.z = (((lM41 * rM13) + (lM42 * rM23)) + (lM43 * rM33)) + (lM44 * rM43);
    result.Row3.w = (((lM41 * rM14) + (lM42 * rM24)) + (lM43 * rM34)) + (lM44 * rM44);

    return result;
}

Matrix4::Matrix4(Vector4 row0, Vector4 row1, Vector4 row2, Vector4 row3)
{
	Row0 = row0;
	Row1 = row1;
	Row2 = row2;
	Row3 = row3;
}

Matrix4::Matrix4(float m00, float m01, float m02, float m03, float m10, float m11, float m12, float m13, float m20, float m21, float m22, float m23, float m30, float m31, float m32, float m33)
{
    Row0 = Vector4(m00, m01, m02, m03);
    Row1 = Vector4(m10, m11, m12, m13);
    Row2 = Vector4(m20, m21, m22, m23);
    Row3 = Vector4(m30, m31, m32, m33);
}

Matrix4::Matrix4()
{
    Row0 = Vector4(0);
    Row1 = Vector4(0);
    Row2 = Vector4(0);
    Row3 = Vector4(0);
}

Matrix4::Matrix4(float n)
{
    Row0 = Vector4(n);
    Row1 = Vector4(n);
    Row2 = Vector4(n);
    Row3 = Vector4(n);
}

Matrix4 Matrix4::LookAt(Vector3 eye, Vector3 target, Vector3 up)
{
    Vector3 z = Vector3::Normalize(eye - target);
    Vector3 x = Vector3::Normalize(Vector3::Cross(up, z));
    Vector3 y = Vector3::Normalize(Vector3::Cross(z, x));

    Matrix4 rot = Matrix4(Vector4(x.x, y.x, z.x, 0.0f),
        Vector4(x.y, y.y, z.y, 0.0f),
        Vector4(x.z, y.z, z.z, 0.0f),
        Vector4(0,0,0,1));

    Matrix4 trans = Matrix4::CreateTranslation(-eye);

    return trans * rot;
}

Matrix4 Matrix4::CreateTranslation(Vector3 vector)
{
    Matrix4 result = Matrix4::identity;
    result.Row3 = Vector4(vector.x, vector.y, vector.z, 1);
    return result;
}

Matrix4 Matrix4::CreatePerspectiveFieldOfView(float fovy, float aspect, float zNear, float zFar)
{
    Matrix4 result = Matrix4(0);

    float yMax = zNear * (float)tan(0.5f * fovy);
    float yMin = -yMax;
    float xMin = yMin * aspect;
    float xMax = yMax * aspect;

    return CreatePerspectiveOffCenter(xMin, xMax, yMin, yMax, zNear, zFar);
}

Matrix4 Matrix4::CreatePerspectiveOffCenter(float left, float right, float bottom, float top, float zNear, float zFar)
{
    float x = (2.0f * zNear) / (right - left);
    float y = (2.0f * zNear) / (top - bottom);
    float a = (right + left) / (right - left);
    float b = (top + bottom) / (top - bottom);
    float c = -(zFar + zNear) / (zFar - zNear);
    float d = -(2.0f * zFar * zNear) / (zFar - zNear);

    return Matrix4(x, 0, 0, 0,
        0, y, 0, 0,
        a, b, c, -1,
        0, 0, d, 0);
}
