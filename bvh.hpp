#pragma once

#include <vector>

#include "vec4.hpp"

namespace BVH {

    struct Triangle {
        Vector4 vertices[3];

        Vector4 calc_centroid() const {
            return (vertices[0] + vertices[1] + vertices[2]) / 3;
        }
    };

    class BVH {

    private:
        std::vector<Triangle> tris;
        void *root = nullptr;

    public:
        BVH(const std::vector<Triangle> &tris);

        ~BVH();

        bool does_intersect_ray(Vector4 origin, Vector4 direction, float *t_out) const;
        void print_stats() const;

        // Make instances non-copyable
        BVH(const BVH &) = delete;

        BVH &operator=(const BVH &) = delete;
    };

}