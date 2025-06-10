cmake_minimum_required(VERSION 3.14)

add_executable(cmucam-viewer
    main.c
    shaders.c
    cmucam.c
)

set_target_properties(cmucam-viewer
    PROPERTIES
        POSITION_INDEPENDENT_CODE TRUE
)

target_link_libraries(cmucam-viewer
    PRIVATE
        OpenGL::OpenGL
        SDL2::SDL2
        GLEW::GLEW
        m
)
