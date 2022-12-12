#include <GL/glew.h>
#include <GL/gl.h>

#include <SDL.h>

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include "shaders.h"

#define CMUCAM_IMAGE_WIDTH 80
#define CMUCAM_IMAGE_HEIGHT 143

int cmucam_readbyte(int fd) {
    uint8_t b = 0;
    int rc = 0;
    do {
        rc = read(fd, &b, 1);
        if (rc < 0) {
            perror("failed to read from CMUcam");
            return rc;
        }
    } while (rc != 1);
    return b;
}

int cmucam_wait(int fd) {
    int rc = 0;
    fsync(fd);
    do {
        rc = cmucam_readbyte(fd);
        if (rc < 0) {
            perror("failed to read from CMUcam");
            return rc;
        }

        // sanitize byte for printing
        char c = (rc > 31 && rc < 127) ? rc : '.';
        printf("got byte: %u (%c)\n", rc, c);
    } while (rc != ':');
    return 0;
}

int cmucam_exit_raw_mode(int fd) {
    const uint8_t cmucam_reset_cmd_raw[] = {'R', 'M', 1, 0};
    int rc = write(fd, cmucam_reset_cmd_raw, 4);
    if (rc < 0 || rc != 4) {
        perror("failed to write to CMUcam");
        return rc;
    }

    return cmucam_wait(fd);
}

int cmucam_dump_frame(int fd) {
    const uint8_t cmucam_dumpframe_cmd_raw[] = {'D', 'F', 0};
    int rc = write(fd, &cmucam_dumpframe_cmd_raw, 3);
    if (rc < 0 || rc != 3) {
        perror("failed to write to CMUcam");
        return rc;
    }
}

int cmucam_dump_frame_get_column(int fd, uint8_t* column) {
    int rc = cmucam_readbyte(fd);
    if (rc < 0) {
        return rc;
    }

#define CMUCAM_DF_START_OF_FRAME 1
#define CMUCAM_DF_START_OF_COLUMN 2
#define CMUCAM_DF_END_OF_FRAME 3

    size_t received = 0;
    if (rc == CMUCAM_DF_START_OF_FRAME) {
        // start of frame, no action
    } else if (rc == CMUCAM_DF_START_OF_COLUMN) {
        // new column, but we have to check for end-of-frame
        rc = cmucam_readbyte(fd);
        if (rc == CMUCAM_DF_END_OF_FRAME) {
            // end of frame, sync with camera shell
            rc = cmucam_wait(fd);
            if (rc < 0) {
                return rc;
            }
            return 1;
        } else {
            // otherwise we've just received the first byte of the column
            column[0] = rc;
            received = 1;
        }
    } else {
        // we're out of sync with the camera
        return -EIO;
    }

    // wait for the column from the camera
    const size_t column_size = CMUCAM_IMAGE_HEIGHT * 3;
    do {
        rc = read(fd, column + received, column_size - received);
        if (rc < 0) {
            perror("failed to read from CMUcam");
            return rc;
        }

        received += rc;
    } while (received != column_size);
    return 0;
}

int cmucam_open(const char* path) {
    // configure 115200 8n1, no in kernel processing
    int fd = open(path, O_RDWR);
    if (fd < 0) {
        perror("failed to open CMUcam");
        return fd;
    }

    struct termios attrs;
    int rc = tcgetattr(fd, &attrs);
    if (rc < 0) {
        perror("failed to get termios attributes");
        return rc;
    }

    attrs.c_cflag &= ~PARENB;
    attrs.c_cflag &= ~CSTOPB;
    attrs.c_cflag &= ~CSIZE;
    attrs.c_cflag |= CS8;
    attrs.c_cflag &= ~CRTSCTS;
    attrs.c_cflag |= CREAD | CLOCAL;

    attrs.c_lflag &= ~ICANON;
    attrs.c_lflag &= ~ECHO;
    attrs.c_lflag &= ~ECHOE;
    attrs.c_lflag &= ~ECHONL;
    attrs.c_lflag &= ~ISIG;
    attrs.c_lflag &= ~ECHOK;
    attrs.c_lflag &= ~IEXTEN;
    attrs.c_lflag &= ~ECHOCTL;
    attrs.c_lflag &= ~ECHOKE;

    attrs.c_iflag &= ~(IXON | IXOFF | IXANY);
    attrs.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    attrs.c_oflag &= ~OPOST;
    attrs.c_oflag &= ~ONLCR;

    attrs.c_cc[VTIME] = 10;
    attrs.c_cc[VMIN] = 0;

    cfsetispeed(&attrs, B115200);
    cfsetospeed(&attrs, B115200);

    rc = tcsetattr(fd, TCSANOW, &attrs);
    if (rc < 0) {
        perror("failed to set termios attributes");
        return rc;
    }

    return fd;
}

int cmucam_setup(int fd) {
    // Boot the camera (get a prompt)
    const char* cmucam_boot_cmd = "\r";
    int rc = write(fd, &cmucam_boot_cmd, 1);
    if (rc < 0 || rc != 1) {
        fprintf(stderr, "failed to write to CMUcam - %d\n", rc);
    }

    rc = cmucam_wait(fd);
    if (rc < 0) {
        return rc;
    }
    printf("CMUcam booted\n");

    // Put the camera in raw mode
    const char* cmucam_rawmode_cmd = "RM 7\r";
    rc = write(fd, cmucam_rawmode_cmd, 5);
    if (rc < 0 || rc != 5) {
        fprintf(stderr, "failed to write to CMUcam - %d\n", rc);
        return rc;
    }

    rc = cmucam_wait(fd);
    if (rc < 0) {
        return rc;
    }
    printf("CMUcam in raw mode\n");

    return fd;
}

int main(int argc, char** argv) {
    int cmucam = cmucam_open("/dev/ttyUSB0");
    if (cmucam < 0) {
        return EXIT_FAILURE;
    }

    int rc = cmucam_setup(cmucam);
    if (rc < 0) {
        return EXIT_FAILURE;
    }

    // Setup an OpenGL 4.3 rendering environment
    rc = SDL_Init(SDL_INIT_VIDEO);
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
            -1.0f, -1.0f, 0.0f, 0.0f,
            -1.0f,  1.0f, 1.0f, 0.0f,
            1.0f, -1.0f, 0.0f, 1.0f,
            1.0f,  1.0f, 1.0f, 1.0f,
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
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGB8, CMUCAM_IMAGE_HEIGHT, CMUCAM_IMAGE_WIDTH);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    glClearColor(0, 0, 0, 1);
    SDL_GL_SetSwapInterval(0);
    SDL_ShowWindow(window);
    
    // Render CMUcam frame dumps
    GLint column = 0;
    bool quit = false;

    rc = cmucam_dump_frame(cmucam);
    if (rc < 0) {
        return rc;
    }

    while (true) {
        // Fetch the next column from the camera
        uint8_t column_data[CMUCAM_IMAGE_HEIGHT * 3];
        rc = cmucam_dump_frame_get_column(cmucam, column_data);
        if (rc < 0) {
            fprintf(stderr, "failed to get next column from CMUcam\n");
            return EXIT_FAILURE;
        }

        if (rc == 1) {
            // got end-of-frame from CMUcam
            if (quit) {
                break;
            }

            rc = cmucam_dump_frame(cmucam);
            if (rc < 0) {
                fprintf(stderr, "failed to start frame dump from CMUcam\n");
                return rc;
            }

            column = 0;
            continue;
        }

        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                quit = true;
            } else if (event.type == SDL_WINDOWEVENT) {
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    // update viewport with new window size
                    int width;
                    int height;
                    SDL_GetWindowSize(window, &width, &height);
                    glViewport(0, 0, width, height);
                }
            }
        }

        // update texture
        // we store the texture rotated so that the column updates from the CMUcam are sent to
        // OpenGL as row updates, which is generally more performant.
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, frame);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, column, CMUCAM_IMAGE_HEIGHT, 1, GL_RGB, GL_UNSIGNED_BYTE, column_data);
        column++;

        // redraw window
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(quad_shader_prog);
        glBindVertexArray(quadVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
        SDL_GL_SwapWindow(window);
    }

    cmucam_exit_raw_mode(cmucam);
    return EXIT_SUCCESS;
}
