#pragma once

#include "vec4.hpp"

class Camera
{
private:
    Vector4 pos, up, right, forward;
    float fov, pitch, yaw;

public:
    Camera(Vector4 pos, Vector4 target)
    {
        forward = (target - pos).normalized3();
        Vector4 global_up(0, 0, 1);
        right = global_up.cross3(forward).normalized3();
        up = forward.cross3(right);
        this->pos = pos;

        fov = 3.14f / 4.0f;
        pitch = 0.0f;
        yaw = 0.0f;
    }

    void calc_vectors(Vector4 *p_up, Vector4 *p_right, Vector4 *p_forward) const
    {
        // Calculate new up and right vectors based on yaw
        Vector4 new_forward = right * std::sin(yaw) + forward * std::cos(yaw);
        Vector4 new_right = new_forward.cross3(up);

        Vector4 new_up = new_forward * std::sin(pitch) + up * std::cos(pitch);
        new_forward = new_up.cross3(new_right);

        *p_forward = new_forward;
        *p_up = new_up;
        *p_right = new_right;
    }

    float get_fov() const
    {
        return fov;
    }

    Vector4 get_pos() const
    {
        return pos;
    }

    void rotate(float d_yaw, float d_pitch)
    {
        yaw -= d_yaw;
        pitch += d_pitch;
    }

    void move(Vector4 vector)
    {
        pos = pos + vector;
    }
};
