#include <chrono>
#include <cstdio>
#include <iostream>

#include "bvh.hpp"
#include "tiny_stl/tiny_stl.hpp"
#include "utils.hpp"

int main(int argc, char* argv[])
{
    if (argc != 2) {
        puts("Expected arguments: mesh.stl");
        return 1;
    }

    const char* filepath = argv[1];
    auto reader = STL_Mesh_IO::create_reader(filepath);

    std::vector<BVH::Triangle> tris;
    STL_Mesh_IO::Triangle t;
    while (reader->read_next_triangle(&t)) {
        tris.push_back({ t.vertices[0], t.vertices[1], t.vertices[1] });
    }
    BVH::BVH bvh(tris);

    Vector4 cam_pos(0.0f, 0.0f, 2.5f);
    Vector4 p0(-1, 1, -2), p1(1, 1, -2), p2(-1, -1, -2);

    render_to_ppm(bvh, "raytrace.ppm", 640, 640, cam_pos, p0, p1, p2);
}