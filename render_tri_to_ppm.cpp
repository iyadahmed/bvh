#include <vector>

#include "bvh.hpp"
#include "utils.hpp"

int main(int argc, char* argv[])
{
    if (argc != 2) {
        puts("Expected arguments: mesh.tri");
        return 1;
    }

    const char* filepath = argv[1];

    // Load .tri mesh file
    std::vector<BVH::Triangle> tris;
    {

        FILE* file = fopen(filepath, "r");
        if (file == NULL) {
            puts("Failed to open file");
            return 1;
        }
        float a, b, c, d, e, f, g, h, i;
        while (fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
                   &a, &b, &c, &d, &e, &f, &g, &h, &i)
            == 9) {
            Vector4 v1(a, b, c);
            Vector4 v2(d, e, f);
            Vector4 v3(g, h, i);
            tris.push_back({ v1, v2, v3 });
        }
        fclose(file);
    }

    BVH::BVH bvh(tris);

    Vector4 cam_pos(-1.5f, -0.2f, 2.5f);
    Vector4 p0(-1, 1, -2), p1(1, 1, -2), p2(-1, -1, -2);

    render_to_ppm(bvh, "raytrace.ppm", 640, 640, cam_pos, p0, p1, p2);
}