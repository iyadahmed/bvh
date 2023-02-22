#pragma once

#include <vector>

#include "bvh.hpp"
#include "utils.hpp"
#include "vec4.hpp"

namespace BVH
{
    class Segment
    {
    private:
        float m_length;
        Vector4 m_origin, m_direction;

    public:
        Segment(Vector4 a, Vector4 b)
        {
            auto v = b - a;
            m_length = v.length3();
            m_origin = a;
            m_direction = v / m_length;
        }

        float get_length() const
        {
            return m_length;
        }

        Vector4 get_origin() const
        {
            return m_origin;
        }

        Vector4 get_direction() const
        {
            return m_direction;
        }
    };

    void intersect_segment_triangle(Segment &segment, const Triangle &tri, std::vector<Vector4> output)
    {
        constexpr float COPLANAR_THRESHOLD = 0.0001;
        Vector4 e1 = tri.vertices[1] - tri.vertices[0];
        Vector4 e2 = tri.vertices[2] - tri.vertices[1];
        Vector4 e3 = tri.vertices[0] - tri.vertices[2];
        Vector4 normal = e2.cross3(e3);
        Vector4 p1_n = normal.cross3(e1);
        Vector4 p2_n = normal.cross3(e2);
        Vector4 p3_n = normal.cross3(e3);

        float denom = segment.get_direction().dot3(normal);
        if (std::abs(denom) <= COPLANAR_THRESHOLD)
        {
            // TODO:    handle coplanar case by intersecting segment/ray with
            //          the 3 planes that define the triangle
            return;
        }
        float t = (tri.vertices[0] - segment.get_origin()).dot3(normal) / denom;
        if (t < 0)
        {
            return;
        }
        Vector4 p = t * segment.get_direction() + segment.get_origin();
        if (is_point_above_plane(p, p1_n, tri.vertices[0]) &&
            is_point_above_plane(p, p2_n, tri.vertices[1]) &&
            is_point_above_plane(p, p3_n, tri.vertices[2]))
        {
            output.push_back(p);
        }
    }
}
