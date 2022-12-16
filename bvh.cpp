#include <algorithm> // for std::partition
#include <iostream>
#include <limits>
#include <stdio.h>
#include <vector>

#include <tiny_stl/tiny_stl.hpp>

#include "vec4.hpp"

struct Triangle {
    Vector4 vertices[3];

    Vector4 calc_centroid() const
    {
        return (vertices[0] + vertices[1] + vertices[2]) / 3;
    }
};

static void bvh(std::vector<Triangle>::iterator begin, std::vector<Triangle>::iterator end)
{
    if (begin == end)
        return;

    Vector4 upper = Vector4(-1 * std::numeric_limits<float>::infinity());
    Vector4 lower = Vector4(std::numeric_limits<float>::infinity());

    for (std::vector<Triangle>::iterator it = begin; it != end; ++it) {
        for (int i = 0; i < 3; i++) {
            upper = upper.max(it->vertices[i]);
            lower = lower.min(it->vertices[i]);
        }
    }

    Vector4 dims = upper - lower;

    int split_axis = 0;

    if (dims[1] > dims[0]) {
        split_axis = 1;
    }

    if (dims[2] > dims[split_axis]) {
        split_axis = 2;
    }

    float split_pos = lower[split_axis] + dims[split_axis] * 0.5f;

    auto middle = std::partition(begin, end, [split_axis, split_pos](const Triangle& t) {
        return t.calc_centroid()[split_axis] < split_pos;
    });

    bvh(begin, middle);
    bvh(middle, end);
}

int main(int argc, char* argv[])
{
    if (argc != 2) {
        puts("Expected arguments: mesh.stl");
        return 1;
    }

    auto reader = STL_Mesh_IO::create_reader(argv[1]);

    std::vector<Triangle> tris;

    STL_Mesh_IO::Triangle t;
    while (reader->read_next_triangle(&t)) {
        tris.push_back({ t.vertices[0], t.vertices[1], t.vertices[2] });
    }

    std::cout << "Number of triangles: " << tris.size() << std::endl;

    bvh(tris.begin(), tris.end());

    return 0;
}