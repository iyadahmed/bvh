#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include <SDL.h>

#include "bvh.hpp"
#include "camera.hpp"
#include "raytrace.hpp"
#include "vec4.hpp"

constexpr int WINDOW_WIDTH = 640;
constexpr int WINDOW_HEIGHT = 640;
constexpr float ROTATION_SPEED = .001;
constexpr float MOVEMENT_SPEED = .1;

Camera cam({0, -2, 0}, {0, 0, 0});
double total_time_ns = 0.0;
size_t num_frames = 0;

struct Color
{
    unsigned char a, b, g, r;
};

static void render(Color *pixels, const BVH::AABBTree &bvh, int width, int height)
{
    float fov = cam.get_fov();
    float tan_half_fov = std::tan(fov / 2);
    Vector4 cam_pos = cam.get_pos();
    Vector4 up;
    Vector4 right;
    Vector4 forward;
    cam.calc_vectors(&up, &right, &forward);

    float aspect_ratio;
    if (width > height)
    {
        aspect_ratio = width / (float)height;
    }
    else
    {
        aspect_ratio = height / (float)width;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
#pragma omp parallel for default(none) firstprivate(aspect_ratio, width, height, cam_pos, forward, right, tan_half_fov, up) shared(bvh, pixels)
    for (int i = 0; i < width * height; i++)
    {
        int pixel_x = i % width;
        int pixel_y = i / width;
        float pixel_x_normalized = pixel_x / (float)width;
        float pixel_y_normalized = pixel_y / (float)height;

        pixel_x_normalized = 2 * pixel_x_normalized - 1;
        pixel_x_normalized *= aspect_ratio;
        pixel_y_normalized = 1 - 2 * pixel_y_normalized;

        Vector4 pixel_pos = cam_pos + forward + right * tan_half_fov * pixel_x_normalized + up * tan_half_fov * pixel_y_normalized;

        Vector4 ray_origin = cam_pos;
        Vector4 ray_direction = (pixel_pos - cam_pos).normalized3();

        float t = 0.0f;
        if (bvh.does_intersect_ray(ray_origin, ray_direction, &t))
        {
            // Map t from [0, inf[ to [0, 1[
            // https://math.stackexchange.com/a/3200751/691043
            float t_normalized = std::atan(t) / (3.14 / 2);
            unsigned char pixel_color = (t_normalized * t_normalized) * 255;
            pixels[pixel_x + pixel_y * width] = {255, pixel_color, pixel_color, pixel_color};
        }
        else
        {
            pixels[pixel_x + pixel_y * width] = {255, 0, 0, 0};
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    auto frame_time_ns = (t2 - t1).count();
    total_time_ns += frame_time_ns;
    num_frames++;
    std::cout << "Rendering took: " << frame_time_ns / 1'000'000 << " milli seconds" << std::endl;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        puts("Expected arguments: mesh.[stl|tri]");
        return 1;
    }

    const char *filepath = argv[1];
    std::vector<BVH::Triangle> tris = load_bvh_tris_from_mesh_file(filepath, 0.01f);
    std::cout << "Loaded " << tris.size() << " triangles from " << filepath << std::endl;
    BVH::AABBTree bvh(tris, 0.001f);
    bvh.print_stats();

    SDL_Event event;
    SDL_Renderer *renderer;
    SDL_Window *window;

    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window, &renderer);
    SDL_SetRelativeMouseMode(SDL_TRUE);

    bool is_running = true;
    int32_t dx, dy;
    auto *pixels = static_cast<Color *>(malloc(WINDOW_WIDTH * WINDOW_HEIGHT * 4));

    SDL_Texture *buffer = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING,
                                            WINDOW_WIDTH, WINDOW_HEIGHT);

    while (true)
    {
        while (SDL_PollEvent(&event) != 0)
        {
            Vector4 up;
            Vector4 right;
            Vector4 forward;
            cam.calc_vectors(&up, &right, &forward);

            switch (event.type)
            {
            case SDL_QUIT:
                is_running = false;
                break;
            case SDL_MOUSEMOTION:
                dx = event.motion.xrel;
                dy = event.motion.yrel;
                cam.rotate(dx * ROTATION_SPEED, dy * ROTATION_SPEED);
                break;
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE)
                {
                    is_running = false;
                }
                else if (event.key.keysym.sym == SDLK_w)
                {
                    cam.move(forward * MOVEMENT_SPEED);
                }
                else if (event.key.keysym.sym == SDLK_s)
                {
                    cam.move(forward * MOVEMENT_SPEED * -1);
                }
                else if (event.key.keysym.sym == SDLK_a)
                {
                    cam.move(right * MOVEMENT_SPEED * -1);
                }
                else if (event.key.keysym.sym == SDLK_d)
                {
                    cam.move(right * MOVEMENT_SPEED);
                }
                break;
            }
        }

        if (!is_running)
            break;

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        render(pixels, bvh, WINDOW_WIDTH, WINDOW_HEIGHT);
        SDL_UpdateTexture(buffer, nullptr, pixels, WINDOW_WIDTH * 4);
        SDL_RenderCopy(renderer, buffer, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    free(pixels);

    std::cout << "Avreage milliseconds per frame = " << (total_time_ns / num_frames) / 1'000'000 << std::endl;

    return 0;
}
