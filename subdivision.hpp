#pragma once

#include <algorithm>
#include <cassert>
#include <vector>
#include <limits>

#include "bvh.hpp"
#include "vec4.hpp"

namespace BVH {

    void subdivide(Node *parent) {
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
        for (std::vector<Triangle>::iterator it = begin; it != end; ++it) {
            for (int i = 0; i < 3; i++) {
                upper = upper.max(it->vertices[i]);
                lower = lower.min(it->vertices[i]);
            }

            Vector4 tc = it->calc_centroid();
            mean = mean + tc / num_tris;
            mean_of_squares = mean_of_squares + (tc * tc) / num_tris;
        }
        Vector4 variance = mean_of_squares - mean * mean;

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

        auto middle = std::partition(begin, end, [split_axis, split_pos](const Triangle &t) {
            return t.calc_centroid()[split_axis] < split_pos;
        });

        if ((middle == begin) || (middle == end)) {
            return;
        }

        Node *left = new Node(begin, middle);
        Node *right = new Node(middle, end);

        parent->left = left;
        parent->right = right;

        subdivide(left);
        subdivide(right);
    }

}