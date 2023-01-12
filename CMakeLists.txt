cmake_minimum_required(VERSION 3.16)

project(BVHDemos)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)
#FETCHCONTENT_DECLARE(
#        sdl
#        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
#        GIT_TAG release-2.26.1
#        GIT_SHALLOW ON
#        GIT_PROGRESS TRUE
#)
#FETCHCONTENT_MAKEAVAILABLE(sdl)
find_package(SDL2 REQUIRED)

FETCHCONTENT_DECLARE(
        tiny_stl
        GIT_REPOSITORY https://github.com/iyadahmed/tiny_stl.git
        GIT_TAG dee148c1af0629b47daed11d984ffcbd93df2a6a
        GIT_SHALLOW ON
        GIT_PROGRESS TRUE
)
FETCHCONTENT_MAKEAVAILABLE(tiny_stl)

add_library(BVH "bvh.cpp" "bvh.hpp" "subdivision.hpp" "ray_intersection.hpp" "utils.hpp")
add_executable(main "main.cpp" "camera.hpp" "main.hpp")
# Note: SDL2::SDL2main has to come before SDL2::SDL2
# https://github.com/msys2/MINGW-packages/issues/10459#issuecomment-1003700201
target_link_libraries(main tiny_stl BVH SDL2::SDL2main SDL2::SDL2-static)
