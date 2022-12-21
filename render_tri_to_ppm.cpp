#include <chrono>
#include <cstdio>
#include <iostream>
#include <vector>

#include "bvh.hpp"

int main(int argc, char* argv[])
{
    if (argc != 2) {
        puts("Expected arguments: mesh.tri");
        return 1;
    }

    const char* filepath = argv[1];

    // Load .tri mesh file
    std::vector<Triangle> tris;
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

    // Build BVH
    Node* root = new Node(tris.begin(), tris.end());
    subdivide(root);
    assert(count_leaf_triangles(root) == tris.size());

    // Render to image
    constexpr int IMAGE_WIDTH = 640;
    constexpr int IMAGE_HEIGHT = 640;

    FILE* image = fopen("raytrace.ppm", "wb");
    fputs("P3", image); // "P3" means this is an RGB PPM color image in ASCII
    fprintf(image, "%d %d 255\n", IMAGE_WIDTH, IMAGE_HEIGHT);

    Vector4 cam_pos(-1.5f, -0.2f, 2.5);
    Vector4 p0(-1, 1, -2), p1(1, 1, -2), p2(-1, -1, -2);

    auto t1 = std::chrono::high_resolution_clock::now();
    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            Vector4 pixel_pos = cam_pos + p0 + (p1 - p0) * (x / (float)IMAGE_WIDTH) + (p2 - p0) * (y / (float)IMAGE_HEIGHT);
            Ray ray(cam_pos, (pixel_pos - cam_pos).normalized3());

            intersect_ray_bvh(ray, root);

            if (ray.t < std::numeric_limits<float>::max()) {
                unsigned char c = 500 - (int)(ray.t * 42);
                fprintf(image, "%u %u %u ", c, c, c);
            } else {
                fputs("0 0 0 ", image);
            }
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Rendering took: " << (t2 - t1).count() / 1'000'000 << " milli seconds" << std::endl;

    free_tree(root);
    fclose(image);
}