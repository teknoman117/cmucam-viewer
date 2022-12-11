#include <GL/glew.h>
#include <GL/gl.h>

#include <SDL.h>

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "shaders.h"

#define CMUCAM_IMAGE_WIDTH 81
#define CMUCAM_IMAGE_HEIGHT 143

int main(int argc, char** argv) {
    // Setup an OpenGL 4.3 rendering environment
    int rc = SDL_Init(SDL_INIT_VIDEO);
    if (rc < 0) {
        fprintf(stderr, "failed to initialize SDL with error: %s (%d)\n", SDL_GetError(), -rc);
        return EXIT_FAILURE;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    SDL_Window *window = SDL_CreateWindow(
            "CMUCam v1.12 Viewer",
            SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED,
            CMUCAM_IMAGE_WIDTH * 2,
            CMUCAM_IMAGE_HEIGHT,
            SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE);
    if (!window) {
        fprintf(stderr, "failed to create SDL window with error: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }

    SDL_GLContext context = SDL_GL_CreateContext(window);
    if (!context) {
        fprintf(stderr, "failed to create SDL OpenGL context with error: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }

    // Setup OpenGL resources
    const GLubyte *version = glGetString(GL_VERSION);
    printf("OpenGL: %s\n", version);

    glewExperimental = true;
    GLenum ret = glewInit();
    if (ret != GLEW_OK) {
        fprintf(stderr, "failed to initialize GLEW with error: %s\n", glewGetErrorString(ret));
        return EXIT_FAILURE;
    }

    rc = loadShaders();
    if (rc < 0) {
        // NOTE: prints its own error message
        return EXIT_FAILURE;
    }

    const GLfloat quad[] = {
            -1.0f, -1.0f, 0.0f, 1.0f,
            -1.0f,  1.0f, 0.0f, 0.0f,
            1.0f, -1.0f, 1.0f, 1.0f,
            1.0f,  1.0f, 1.0f, 0.0f,
    };

    GLuint quadVAO;
    glGenVertexArrays(1, &quadVAO);
    glBindVertexArray(quadVAO);

    GLuint quadVertexBuffer;
    glGenBuffers(1, &quadVertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, quadVertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad[0]) * 16, quad, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(quad[0]) * 4,
            (const void*) 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(quad[0]) * 4,
            (const void*) (sizeof(quad[0]) * 2));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    ret = glGetError();
    if (ret != GL_NO_ERROR) {
        fprintf(stderr, "there is a GL error!!! 0\n");
    }

    // use nearest filtering for integer scaling
    GLuint frame;
    glGenTextures(1, &frame);
    glBindTexture(GL_TEXTURE_2D, frame);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGB8, CMUCAM_IMAGE_WIDTH, CMUCAM_IMAGE_HEIGHT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    SDL_GL_SetSwapInterval(0);
    glClearColor(0, 0, 0, 1);
    
    // Draw CMUcam image until exit is requested
    SDL_ShowWindow(window);
    bool quit = false;
    while (!quit) {
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                printf("Exit requested\n");
                quit = true;
            }
        }

        // process incoming frame from CMUcam

        // redraw window
        int width;
        int height;
        SDL_GetWindowSize(window, &width, &height);

        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(quad_shader_prog);
        glBindVertexArray(quadVAO);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, frame);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindVertexArray(0);
        SDL_GL_SwapWindow(window);
    }

    return EXIT_SUCCESS;
}
