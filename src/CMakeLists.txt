cmake_minimum_required(VERSION 3.14)

add_executable(cmucam-viewer main.c)

target_link_libraries(cmucam-viewer
    PRIVATE
        OpenGL::OpenGL
        SDL2::SDL2
)
