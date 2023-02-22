#pragma once

#include <limits>

#include "bvh.hpp"
#include "utils.hpp"
#include "vec4.hpp"

namespace BVH
{

    struct Ray
    {
    private:
        Vector4 m_origin, m_direction, m_reciprocal_direction;
        float m_t;

    public:
        Ray(Vector4 origin, Vector4 direction)
        {
            m_origin = origin;
            m_direction = direction;
            m_reciprocal_direction = 1.0f / direction;
            m_t = std::numeric_limits<float>::max();
        }

        Vector4 get_reciprocal_direction() const
        {
            return m_reciprocal_direction;
        }

        Vector4 get_direction() const
        {
            return m_direction;
        }

        Vector4 get_origin() const
        {
            return m_origin;
        }

        float get_t() const
        {
            return m_t;
        }

        void set_t(float t)
        {
            this->m_t = t;
        }
    };

    void intersect_ray_triangle(Ray &ray, const Triangle &tri)
    {
        // TODO: reduce code duplication,
        //       same code is repeated in segment/triangle intersection
        constexpr float COPLANAR_THRESHOLD = 0.00001;
        Vector4 e1 = tri.vertices[1] - tri.vertices[0];
        Vector4 e2 = tri.vertices[2] - tri.vertices[1];
        Vector4 e3 = tri.vertices[0] - tri.vertices[2];
        Vector4 normal = e2.cross3(e3).normalized3(); // normalizing is very important for comparing thresholds later
        Vector4 p1_n = normal.cross3(e1);
        Vector4 p2_n = normal.cross3(e2);
        Vector4 p3_n = normal.cross3(e3);

        float denom = ray.get_direction().dot3(normal);
        if (std::abs(denom) <= COPLANAR_THRESHOLD)
        {
            // TODO:    handle coplanar case by intersecting segment/ray with
            //          the 3 planes that define the triangle
            return;
        }
        float t = (tri.vertices[0] - ray.get_origin()).dot3(normal) / denom;
        if (t < 0)
        {
            return;
        }
        Vector4 p = t * ray.get_direction() + ray.get_origin();
        if (is_point_above_plane(p, p1_n, tri.vertices[0]) &&
            is_point_above_plane(p, p2_n, tri.vertices[1]) &&
            is_point_above_plane(p, p3_n, tri.vertices[2]))
        {
            ray.set_t(std::min(ray.get_t(), t));
        }
    }

    bool intersect_ray_aabb(const Ray &ray, const AABB &aabb)
    {
        Vector4 t_upper = (aabb.upper - ray.get_origin()) * ray.get_reciprocal_direction();
        Vector4 t_lower = (aabb.lower - ray.get_origin()) * ray.get_reciprocal_direction();
        Vector4 t_min_v = t_upper.min(t_lower);
        Vector4 t_max_v = t_upper.max(t_lower);

        float t_min = t_min_v.max_elem3();
        float t_max = t_max_v.min_elem3();

        // return (t_max >= t_min && t_min < ray.get_t() && t_max > 0);
        return t_max > t_min;
    }

    void intersect_ray_bvh(Ray &ray, Node *node)
    {
        if (node == nullptr)
        {
            return;
        }

        if (!intersect_ray_aabb(ray, node->aabb))
        {
            return;
        }

        if (node->is_leaf())
        {
            for (auto it = node->begin; it != node->end; ++it)
            {
                intersect_ray_triangle(ray, *it);
            }
        }
        else
        {
            intersect_ray_bvh(ray, node->left);
            intersect_ray_bvh(ray, node->right);
        }
    }

}
