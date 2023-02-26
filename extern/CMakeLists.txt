add_subdirectory(tiny_stl EXCLUDE_FROM_ALL)

# SDL2
include(FetchContent)
set(FetchContent_Quiet FALSE)
FetchContent_Declare(
    sdl
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-2.26.3
    GIT_SHALLOW ON
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(sdl)
