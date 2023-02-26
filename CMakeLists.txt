cmake_minimum_required(VERSION 3.16)
project(BVHDemos)

add_subdirectory(extern EXCLUDE_FROM_ALL)

find_package(OpenMP REQUIRED)

add_library(bvh "bvh.cpp" "bvh.hpp" "subdivision.hpp" "ray_intersection.hpp" "utils.hpp" "non_copyable.hpp")
add_executable(raytrace "raytrace.cpp" "camera.hpp" "raytrace.hpp")

# Note: SDL2::SDL2main has to come before SDL2::SDL2
# https://github.com/msys2/MINGW-packages/issues/10459#issuecomment-1003700201
target_link_libraries(raytrace tiny_stl bvh SDL2::SDL2main SDL2::SDL2-static OpenMP::OpenMP_CXX)
