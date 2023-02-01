#pragma once

#include <limits>

#include "bvh.hpp"
#include "vec4.hpp"


namespace BVH {

    struct Ray {
    private:
        Vector4 m_origin, m_direction, m_reciprocal_direction;
        float m_t;

    public:
        Ray(Vector4 origin, Vector4 direction) {
            m_origin = origin;
            m_direction = direction;
            m_reciprocal_direction = 1.0f / direction;
            m_t = std::numeric_limits<float>::max();
        }

        Vector4 get_reciprocal_direction() const {
            return m_reciprocal_direction;
        }

        Vector4 get_direction() const {
            return m_direction;
        }

        Vector4 get_origin() const {
            return m_origin;
        }

        float get_t() const {
            return m_t;
        }

        void set_t(float t) {
            this->m_t = t;
        }
    };

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

    bool intersect_ray_aabb(const Ray &ray, const AABB &aabb) {
        Vector4 t_upper = (aabb.upper - ray.get_origin()) * ray.get_reciprocal_direction();
        Vector4 t_lower = (aabb.lower - ray.get_origin()) * ray.get_reciprocal_direction();
        Vector4 t_min_v = t_upper.min(t_lower);
        Vector4 t_max_v = t_upper.max(t_lower);

        // float t_min = t_min_v.max_elem3();
        // float t_max = t_max_v.min_elem3();

        __m128 a = _mm_movehdup_ps(t_min_v.mm); // y y w w
        __m128 b = _mm_unpackhi_ps(t_min_v.mm, t_min_v.mm); // z z w w
        __m128 c = _mm_max_ps(a, b); // max(y, z), ..., ...
        __m128 t_min = _mm_max_ps(t_min_v.mm, c); // max(x, y, z), ..., ...

        __m128 e = _mm_movehdup_ps(t_max_v.mm); // y y w w
        __m128 f = _mm_unpackhi_ps(t_max_v.mm, t_max_v.mm); // z z w w
        __m128 g = _mm_min_ps(e, f); // min(y, z), ..., ...
        __m128 t_max = _mm_min_ps(t_max_v.mm, g); // min(x, y, z), ..., ...

        return _mm_ucomigt_ss(t_max, t_min);


//        return (t_max >= t_min && t_min < ray.get_t() && t_max > 0);
        // return t_max > t_min;
    }

    void intersect_ray_bvh(Ray &ray, Node *node) {
        if (node == nullptr) {
            return;
        }

        if (!intersect_ray_aabb(ray, node->aabb)) {
            return;
        }

        if (node->is_leaf()) {
            for (auto it = node->begin; it != node->end; ++it) {
                intersect_ray_triangle(ray, *it);
            }
        } else {
            intersect_ray_bvh(ray, node->left);
            intersect_ray_bvh(ray, node->right);
        }
    }

}