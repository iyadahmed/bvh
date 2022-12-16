#include <iostream>
#include <limits>
#include <stdio.h>
#include <vector>

#include <tiny_stl/tiny_stl.hpp>

#include "vec4.hpp"

struct Triangle {
    Vector4 vertices[3];
};

static void bvh(std::vector<Triangle>::iterator begin, std::vector<Triangle>::iterator end)
{
    if (begin == end)
        return;

    Vector4 upper = Vector4(-1 * std::numeric_limits<float>::infinity());
    Vector4 lower = Vector4(std::numeric_limits<float>::infinity());

    for (auto& it = begin; it != end; ++it) {
        for (int i = 0; i < 3; i++) {
            upper = upper.max(it->vertices[i]);
            lower = lower.min(it->vertices[i]);
        }
    }

    std::cout << upper.x << " " << upper.y << " " << upper.z << std::endl;
    std::cout << lower.x << " " << lower.y << " " << lower.z << std::endl;

    // TODO: recursively parition the triangles vector, and store nodes
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