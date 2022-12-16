#include <iostream>
#include <stdio.h>
#include <vector>

#include <tiny_stl/tiny_stl.hpp>

#include "vec4.hpp"

struct Triangle {
    Vector4 vertices[3];
};

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
        for (int i = 0; i < 3; i++) {
        }

        tris.push_back({ t.vertices[0], t.vertices[1], t.vertices[1] });
    }

    for (const auto& v : tris) {
        for (int i = 0; i < 3; i++)
            std::cout << v.vertices[i].x << ' ' << v.vertices[i].y << ' ' << v.vertices[i].z << std::endl;
    }

    return 0;
}