#pragma once

#include <cmath>
#include <immintrin.h>

union Vector4
{
    struct
    {
        float x, y, z, w;
    };
    __m128 mm;
    float arr[4];

    Vector4()
    {
        mm = _mm_set1_ps(0.0f);
    }

    Vector4(__m128 mm)
        : mm(mm)
    {
    }

    explicit Vector4(const float &value)
    {
        mm = _mm_set1_ps(value);
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
        return _mm_max_ps(mm, other.mm);
    }

    Vector4 min(const Vector4 &other) const
    {
        return _mm_min_ps(mm, other.mm);
    }

    float max_elem3() const
    {
        __m128 a = _mm_unpacklo_ps(mm, mm); // x x y y
        __m128 b = _mm_unpackhi_ps(mm, mm); // z z w w
        __m128 c = _mm_max_ps(a, b);        // ..., max(x, z), ..., ...
        Vector4 res = _mm_max_ps(mm, c);    // ..., max(y, max(x, z)), ..., ...
        return res.y;
    }

    float min_elem3() const
    {
        __m128 a = _mm_unpacklo_ps(mm, mm); // x x y y
        __m128 b = _mm_unpackhi_ps(mm, mm); // z z w w
        __m128 c = _mm_min_ps(a, b);        // ..., min(x, z), ..., ...
        Vector4 res = _mm_min_ps(mm, c);    // ..., min(y, min(x, z)), ..., ...
        return res.y;
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
        return _mm_mul_ps(mm, other.mm);
    }

    Vector4 operator*(const float &other) const
    {
        return _mm_mul_ps(mm, _mm_set1_ps(other));
    }

    Vector4 operator/(const Vector4 &other) const
    {
        return _mm_div_ps(mm, other.mm);
    }

    Vector4 operator/(const float &other) const
    {
        return _mm_div_ps(mm, _mm_set1_ps(other));
    }

    Vector4 operator+(const Vector4 &other) const
    {
        return _mm_add_ps(mm, other.mm);
    }

    Vector4 operator-(const Vector4 &other) const
    {
        return _mm_sub_ps(mm, other.mm);
    }

    float &operator[](size_t i)
    {
        return arr[i];
    }
};

static Vector4 operator/(const float &rhs, const Vector4 &lhs)
{
    return _mm_div_ps(_mm_set1_ps(rhs), lhs.mm);
}

static Vector4 operator*(const float &rhs, const Vector4 &lhs)
{
    return _mm_mul_ps(_mm_set1_ps(rhs), lhs.mm);
}
