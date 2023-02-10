#pragma once

#include <cmath>

union Vector4
{
    struct
    {
        float x, y, z, w;
    };
    float arr[4];

    Vector4()
    {
        x = y = z = w = 0.0f;
    }

    explicit Vector4(const float &value)
    {
        x = y = z = w = value;
    }

    Vector4(float x, float y, float z)
        : x(x), y(y), z(z), w(0.0f)
    {
    }

    Vector4(float x, float y, float z, float w)
        : x(x), y(y), z(z), w(w)
    {
    }

    Vector4 max(const Vector4 &other) const
    {
        return {std::fmax(x, other.x), std::fmax(y, other.y), std::fmax(z, other.z)};
    }

    Vector4 min(const Vector4 &other) const
    {
        return {std::fmin(x, other.x), std::fmin(y, other.y), std::fmin(z, other.z)};
    }

    float max_elem3() const
    {
        return std::fmax(x, std::fmax(y, z));
    }

    float min_elem3() const
    {
        return std::fmin(x, std::fmin(y, z));
    }

    float length3() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vector4 normalized3() const
    {
        return (*this) / length3();
    }

    Vector4 cross3(const Vector4 &other) const
    {
        return {(y * other.z - z * other.y), (z * other.x - x * other.z), (x * other.y - y * other.x)};
    }

    float dot3(const Vector4 &other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    Vector4 operator*(const Vector4 &other) const
    {
        return {x * other.x, y * other.y, z * other.z};
    }

    Vector4 operator*(const float &other) const
    {
        return {x * other, y * other, z * other};
    }

    Vector4 operator/(const Vector4 &other) const
    {
        return {x / other.x, y / other.y, z / other.z};
    }

    Vector4 operator/(const float &other) const
    {
        return {x / other, y / other, z / other};
    }

    Vector4 operator+(const Vector4 &other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vector4 operator-(const Vector4 &other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    float &operator[](size_t i)
    {
        return arr[i];
    }
};

static Vector4 operator/(const float &rhs, const Vector4 &lhs)
{
    return {rhs / lhs.x, rhs / lhs.y, rhs / lhs.z};
}

static Vector4 operator*(const float &rhs, const Vector4 &lhs)
{
    return {rhs * lhs.x, rhs * lhs.y, rhs * lhs.z};
}
