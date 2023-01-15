#pragma once

#include <algorithm>
#include <cassert>
#include <vector>
#include <limits>

#include "bvh.hpp"
#include "vec4.hpp"

namespace BVH {

    void BVH::subdivide(Node *parent) {
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

        Vector4 plane_normal = variance.normalized3();
        Vector4 plane_point = mean;

        parent->plane_normal = plane_normal;
        parent->plane_point = plane_point;

        // Expand bounding box by an epsilon;
        // fixes an issue where rays that are tangent to the bounding box miss,
        // hopefully this does not strike back and need extra margins in the future,
        // P.S.: this is probably related to numeric precision of intrinsics and order of floating-point operations
        upper = upper + Vector4(std::numeric_limits<float>::epsilon());
        lower = lower - Vector4(std::numeric_limits<float>::epsilon());

        int split_axis = 0;

        if (variance[1] > variance[0]) {
            split_axis = 1;
        }

        if (variance[2] > variance[split_axis]) {
            split_axis = 2;
        }

        float split_pos = mean[split_axis];

        auto middle = std::partition(begin, end, [plane_normal, plane_point](const Triangle &t) {
            //return t.calc_centroid()[split_axis] < split_pos;
            return (t.calc_centroid() - plane_point).normalized3().dot3(plane_normal) < 0;
        });

        if ((middle == begin) || (middle == end)) {
            return;
        }

        Node *left = new_node(begin, middle);
        Node *right = new_node(middle, end);

        parent->left = left;
        parent->right = right;

        subdivide(left);
        subdivide(right);
    }

}