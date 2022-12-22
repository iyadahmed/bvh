cmake_minimum_required(VERSION 3.16)

project(BVHDemos)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
	sdl
	GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
	GIT_TAG release-2.26.1
	GIT_SHALLOW    ON
	GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(sdl)

add_subdirectory(tiny_stl)

add_library(BVH "bvh.cpp" "bvh.hpp")
add_library(Utils "utils.cpp" "utils.hpp")

add_executable(render_stl_to_ppm "render_stl_to_ppm.cpp")
target_link_libraries(render_stl_to_ppm PRIVATE BVH tiny_stl Utils)

add_executable(render_tri_to_ppm "render_tri_to_ppm.cpp")
target_link_libraries(render_tri_to_ppm PRIVATE BVH Utils)

add_executable(main "main.cpp")
target_link_libraries(main BVH SDL2::SDL2 SDL2::SDL2main)
