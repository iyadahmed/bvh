#pragma once

#include <algorithm>
#include <cassert>
#include <vector>

#include "bvh.hpp"
#include "vec4.hpp"

#define USE_ORDERED_TRAVERSAL 1

namespace BVH {

    struct AABB {
        Vector4 upper, lower;
    };

    struct Ray {
    private:
        Vector4 O, D, rD;
        float t;

    public:
        Ray(Vector4 origin, Vector4 direction) {
            O = origin;
            D = direction;
            rD = 1.0f / direction;
            t = std::numeric_limits<float>::max();
        }

        Vector4 get_reciprocal_direction() const {
            return rD;
        }

        Vector4 get_direction() const {
            return D;
        }

        Vector4 get_origin() const {
            return O;
        }

        float get_t() const {
            return t;
        }

        void set_t(float t) {
            this->t = t;
        }
    };

    struct Node {
        std::vector<Triangle>::iterator begin, end;
        Node *left = nullptr, *right = nullptr;
        AABB aabb;

        Node(std::vector<Triangle>::iterator begin, std::vector<Triangle>::iterator end) {
            this->begin = begin;
            this->end = end;
            left = nullptr;
            right = nullptr;
        }

        bool is_leaf() const {
            return (left == nullptr) && (right == nullptr);
        }
    };

    int count_nodes(Node *node) {
        if (node == nullptr)
            return 0;

        return 1 + count_nodes(node->left) + count_nodes(node->right);
    }

// https://stackoverflow.com/a/9181223/8094047
    void free_tree(Node *node) {
        if (node == nullptr)
            return;

        free_tree(node->left);
        free_tree(node->right);

        delete node;
    }

    int count_leaf_triangles(Node *node) {
        if (node == nullptr) {
            return 0;
        } else if (node->is_leaf()) {
            return std::abs(std::distance(node->begin, node->end));
        } else {
            return count_leaf_triangles(node->left) + count_leaf_triangles(node->right);
        }
    }

    int count_leaf_nodes(Node *node) {
        if (node == nullptr) {
            return 0;
        } else if (node->is_leaf()) {
            return 1;
        } else {
            return count_leaf_nodes(node->left) + count_leaf_nodes(node->right);
        }
    }

    void subdivide(Node *parent) {
        auto begin = parent->begin;
        auto end = parent->end;

        assert(begin < end);

        Vector4 &upper = parent->aabb.upper;
        Vector4 &lower = parent->aabb.lower;

        upper = Vector4(-1 * std::numeric_limits<float>::infinity());
        lower = Vector4(std::numeric_limits<float>::infinity());

        // Calculate variance to determine split axis based on axis with largest variance,
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
        // hopefully this does not strike back and need extra margins in future,
        // P.S.: this is probably related to numeric precision of instrinsics and order of floating-point operations
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

    void intersect_ray_triangle(Ray &ray, const Triangle &tri) {
        constexpr float EPSILON = 0.0f;
        const Vector4 edge1 = tri.vertices[1] - tri.vertices[0];
        const Vector4 edge2 = tri.vertices[2] - tri.vertices[0];
        const Vector4 h = ray.get_direction().cross3(edge2);
        const float a = edge1.dot3(h);
        if (a > -EPSILON && a < EPSILON)
            return; // ray parallel to triangle
        const float f = 1 / a;
        const Vector4 s = ray.get_origin() - tri.vertices[0];
        const float u = f * s.dot3(h);
        if (u < 0 || u > 1)
            return;
        const Vector4 q = s.cross3(edge1);
        const float v = f * ray.get_direction().dot3(q);
        if (v < 0 || u + v > 1)
            return;
        const float t = f * edge2.dot3(q);
        if (t > EPSILON) {
            ray.set_t(std::min(ray.get_t(), t));
        }
    }

    float intersect_ray_aabb(const Ray &ray, const AABB &aabb) {
        Vector4 t_upper = (aabb.upper - ray.get_origin()) * ray.get_reciprocal_direction();
        Vector4 t_lower = (aabb.lower - ray.get_origin()) * ray.get_reciprocal_direction();
        Vector4 t_min_v = t_upper.min(t_lower);
        Vector4 t_max_v = t_upper.max(t_lower);

        float t_min = t_min_v.max_elem3();
        float t_max = t_max_v.min_elem3();

        if (t_max >= t_min && t_min < ray.get_t() && t_max > 0)
            return t_min;
        else
            return std::numeric_limits<float>::max();
    }

#if !USE_ORDERED_TRAVERSAL
    void intersect_ray_bvh(Ray& ray, Node* node)
    {
        if (intersect_ray_aabb(ray, node->aabb) == std::numeric_limits<float>::max()) {
            return;
        }

        if (node->is_leaf()) {
            for (std::vector<Triangle>::iterator it = node->begin; it != node->end; ++it) {
                intersect_ray_triangle(ray, *it);
            }
        } else {
            intersect_ray_bvh(ray, node->left);
            intersect_ray_bvh(ray, node->right);
        }
    }
#else

    void intersect_ray_bvh(Ray &ray, Node *node) {
        Node *stack[64];
        unsigned int stack_ptr = 0;
        while (true) {
            if (node->is_leaf()) {
                for (std::vector<Triangle>::iterator it = node->begin; it != node->end; ++it) {
                    intersect_ray_triangle(ray, *it);
                }
                if (stack_ptr == 0)
                    break;
                else
                    node = stack[--stack_ptr];

                continue;
            }

            Node *child1 = node->left;
            Node *child2 = node->right;

            float dist1 = intersect_ray_aabb(ray, child1->aabb);
            float dist2 = intersect_ray_aabb(ray, child2->aabb);

            if (dist1 > dist2) {
                std::swap(dist1, dist2);
                std::swap(child1, child2);
            }

            if (dist1 == std::numeric_limits<float>::max()) {
                if (stack_ptr == 0)
                    break;
                else
                    node = stack[--stack_ptr];
            } else {
                node = child1;
                if (dist2 != std::numeric_limits<float>::max())
                    stack[stack_ptr++] = child2;
            }
        }
    }

#endif

}