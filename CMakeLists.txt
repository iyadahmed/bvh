cmake_minimum_required(VERSION 3.16)

project(bvh)

include(FetchContent) # Needs to be included first
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

add_library(vec4 INTERFACE "vec4.hpp")
add_library(bvh INTERFACE "bvh.hpp")
target_link_libraries(bvh INTERFACE vec4)

add_executable(main "main.cpp")
target_link_libraries(main INTERFACE bvh vec4 PUBLIC tiny_stl SDL2::SDL2 SDL2::SDL2main)

add_executable(render_tri_to_ppm "render_tri_to_ppm.cpp")
target_link_libraries(render_tri_to_ppm INTERFACE bvh vec4)

if(MSVC)
  target_link_options(main PRIVATE $<$<CONFIG:RELWITHDEBINFO>:/PROFILE>)
endif()