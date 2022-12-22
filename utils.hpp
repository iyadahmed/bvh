#pragma once

#include "bvh.hpp"

void render_to_ppm(const BVH::BVH& bvh, const char* filepath, int width, int height, Vector4 cam_pos, Vector4 p0, Vector4 p1, Vector4 p2);