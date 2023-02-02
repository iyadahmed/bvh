#pragma once

#include <algorithm>
#include <cassert>
#include <vector>
#include <limits>

#include "bvh.hpp"
#include "vec4.hpp"

namespace BVH {

    void BVH::subdivide(Node *parent, float aabb_expansion) {
        auto begin = parent->begin;
        auto end = parent->end;

        assert(begin < end);

        Vector4 &upper = parent->aabb.upper;
        Vector4 &lower = parent->aabb.lower;

        upper = Vector4(-1 * std::numeric_limits<float>::infinity());
        lower = Vector4(std::numeric_limits<float>::infinity());

        // Calculate variance to determine split axis based on axis with the largest variance,
        // this produces more balanced trees and overcomes an issue that happens with meshes that contain
        // long thin triangles, where normal largest-bounding-box-split-axis fails.
        // P.S.: we also calculate the node's bounding box in same loop while we are at it
        Vector4 mean(0.0f);
        Vector4 mean_of_squares(0.0f);
        long num_tris = std::distance(begin, end);
        assert(num_tris > 0);
        for (auto it = begin; it != end; ++it) {
            for (auto vertex : it->vertices) {
                upper = upper.max(vertex);
                lower = lower.min(vertex);
            }

            Vector4 triangle_center = it->calc_centroid();
            mean = mean + triangle_center / num_tris;
            mean_of_squares = mean_of_squares + (triangle_center * triangle_center) / num_tris;
        }
        Vector4 variance = mean_of_squares - mean * mean;

        // Expand bounding box by some value,
        // this helps increase the robustness of queries
        // (e.g. tangent rays or very thin bounding boxes)
        upper = upper + Vector4(aabb_expansion);
        lower = lower - Vector4(aabb_expansion);

        int split_axis = 0;

        if (variance[1] > variance[0]) {
            split_axis = 1;
        }

        if (variance[2] > variance[split_axis]) {
            split_axis = 2;
        }

        float split_pos = mean[split_axis];

        auto middle = std::partition(begin, end, [split_axis, split_pos](const Triangle &t) {
            return t.calc_centroid()[split_axis] < split_pos;
        });

        if ((middle == begin) || (middle == end)) {
            return;
        }

        Node *left = new_node(begin, middle);
        Node *right = new_node(middle, end);

        parent->left = left;
        parent->right = right;

        subdivide(left, aabb_expansion);
        subdivide(right, aabb_expansion);
    }

}