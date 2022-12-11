#include <GL/gl.h>
#include <SDL.h>

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#define CMUCAM_IMAGE_WIDTH 81
#define CMUCAM_IMAGE_HEIGHT 143

int main(int argc, char** argv) {
    int rc = SDL_Init(SDL_INIT_VIDEO);
    if (rc < 0) {
        fprintf(stderr, "failed to initialize SDL with error: %s (%d)\n", SDL_GetError(), -rc);
        return EXIT_FAILURE;
    }

    // Setup an OpenGL 4.3 rendering environment
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

    SDL_Window *window = SDL_CreateWindow(
            "CMUCam v1.12 Viewer",
            SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED,
            CMUCAM_IMAGE_WIDTH * 2,
            CMUCAM_IMAGE_HEIGHT,
            SDL_WINDOW_OPENGL);
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

    SDL_GL_SetSwapInterval(0);
    glClearColor(0, 0, 0, 1);
    
    // Draw CMUcam image until exit is requested
    bool quit = false;
    while (!quit) {
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                printf("Exit requested\n");
                quit = true;
            }
        }

        glClear(GL_COLOR_BUFFER_BIT);
        SDL_GL_SwapWindow(window);
    }

    return EXIT_SUCCESS;
}
