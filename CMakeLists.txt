cmake_minimum_required(VERSION 3.16)

project(BVHDemos)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)

FETCHCONTENT_DECLARE(
    sdl
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-2.26.1
    GIT_SHALLOW ON
    GIT_PROGRESS TRUE
)
FETCHCONTENT_MAKEAVAILABLE(sdl)

FETCHCONTENT_DECLARE(
    tiny_stl
    GIT_REPOSITORY https://github.com/iyadahmed/tiny_stl.git
    GIT_TAG a928bb9ff65bfe0cb0e2335c70abd16e8bdce0d7
    GIT_PROGRESS TRUE
)
FETCHCONTENT_MAKEAVAILABLE(tiny_stl)

find_package(OpenMP REQUIRED)

add_library(bvh "bvh.cpp" "bvh.hpp" "subdivision.hpp" "ray_intersection.hpp" "utils.hpp")
add_executable(raytrace "raytrace.cpp" "camera.hpp" "raytrace.hpp")

# Note: SDL2::SDL2main has to come before SDL2::SDL2
# https://github.com/msys2/MINGW-packages/issues/10459#issuecomment-1003700201
target_link_libraries(raytrace tiny_stl bvh SDL2::SDL2main SDL2::SDL2-static OpenMP::OpenMP_CXX)
