#include <chrono>
#include <iostream>

#include "utils.hpp"
#include "vec4.hpp"

void render_to_ppm(const BVH::BVH& bvh, const char* filepath, int width, int height, Vector4 cam_pos, Vector4 p0, Vector4 p1, Vector4 p2)
{
    // FIXME: support non square aspect ratio
    FILE* image = fopen("raytrace.ppm", "wb");
    fputs("P3", image); // "P3" means this is an RGB PPM color image in ASCII
    fprintf(image, "%d %d 255\n", width, height);

    auto t1 = std::chrono::high_resolution_clock::now();
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Vector4 pixel_pos = cam_pos + p0 + (p1 - p0) * (x / (float)width) + (p2 - p0) * (y / (float)height);
            Vector4 ray_origin = cam_pos;
            Vector4 ray_direction = (pixel_pos - cam_pos).normalized3();

            float t = 0.0f;
            if (bvh.does_intersect_ray(cam_pos, ray_direction, &t)) {
                unsigned char c = 500 - (int)(t * 42);
                fprintf(image, "%u %u %u ", c, c, c);
            } else {
                fputs("0 0 0 ", image);
            }
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Rendering took: " << (t2 - t1).count() / 1'000'000 << " milli seconds" << std::endl;

    fclose(image);
}
