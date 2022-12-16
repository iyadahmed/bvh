#pragma once

#include <immintrin.h>

union Vector4 {
    struct {
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

    Vector4(float values[3])
        : x(values[0])
        , y(values[1])
        , z(values[2])
        , w(0.0f)
    {
    }

    explicit Vector4(const float& value)
    {
        mm = _mm_set1_ps(value);
    }

    Vector4(float x, float y, float z)
        : x(x)
        , y(y)
        , z(z)
        , w(0.0f)
    {
    }

    Vector4(float x, float y, float z, float w)
        : x(x)
        , y(y)
        , z(z)
        , w(w)
    {
    }

    Vector4 max(const Vector4& other) const
    {
        return _mm_max_ps(mm, other.mm);
    }

    Vector4 min(const Vector4& other) const
    {
        return _mm_min_ps(mm, other.mm);
    }

    Vector4 operator*(const Vector4& other) const
    {
        return _mm_mul_ps(mm, other.mm);
    }

    Vector4 operator*(const float& other) const
    {
        return _mm_mul_ps(mm, _mm_set1_ps(other));
    }

    Vector4 operator/(const Vector4& other) const
    {
        return _mm_div_ps(mm, other.mm);
    }

    Vector4 operator/(const float& other) const
    {
        return _mm_div_ps(mm, _mm_set1_ps(other));
    }

    Vector4 operator+(const Vector4& other) const
    {
        return _mm_add_ps(mm, other.mm);
    }

    Vector4 operator-(const Vector4& other) const
    {
        return _mm_sub_ps(mm, other.mm);
    }

    float& operator[](size_t i)
    {
        return arr[i];
    }
};