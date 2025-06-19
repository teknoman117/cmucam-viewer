/*
 * Copyright (c) 2022 Nathan Lewis <github@nrlewis.dev>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of mosquitto nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "cmucam.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <sys/epoll.h>

#define CMUCAM_DF_START_OF_FRAME 1
#define CMUCAM_DF_START_OF_COLUMN 2
#define CMUCAM_DF_END_OF_FRAME 3

#define CMUCAM_MAX_CAMERAS 8

static const uint8_t cmucam_cmd_ping[] = {'\r'};

static const uint8_t cmucam_rcmd_dumpframe[] = {'D', 'F', 0};
static const uint8_t cmucam_rcmd_rawmode_exit[] = {'R', 'M', 1, 0};
static const uint8_t cmucam_rcmd_reset[] = {'R', 'S', 0};
static const uint8_t cmucam_rcmd_reset_window[] = {'S', 'W', 0};
static const uint8_t cmucam_rcmd_get_mean[] = {'G', 'M', 0};
static const uint8_t cmucam_rcmd_track_window[] = {'T', 'W', 0};

static const uint8_t cmucam_acmd_rawmode_enter[] = {'R', 'M', ' ', '7', '\r'};
static const uint8_t cmucam_acmd_reset[] = {'R', 'S', '\r'};

static int epoll_fd = -1;

// wait for the CMUcam to send some data to the host
static int cmucam_await(int fd, int timeout) {
    struct epoll_event events[CMUCAM_MAX_CAMERAS];
    int nfds = epoll_wait(epoll_fd, events, CMUCAM_MAX_CAMERAS, timeout);
    if (nfds < 0) {
        perror("failed to poll CMUcam");
        return nfds;
    } else {
        for (int i = 0; i < nfds; i++) {
            if (events[i].data.fd == fd) {
                // this CMUcam has pending reads
                return 0;
            }
        }
    }
    return -ETIME;
}

// wait for the CMUcam to stop sending data
static int cmucam_quiesce(int fd) {
    int rc = 0;
    do {
        uint8_t discard[1024];
        rc = read(fd, discard, sizeof discard);
        if (rc < 0) {
            perror("failed to read from CMUcam");
            return rc;
        }

        rc = cmucam_await(fd, 100);
        if (rc < 0 && rc != -ETIME) {
            return rc;
        }
    } while (rc != -ETIME);
    return 0;
}

// wait on a byte to be available from the CMUcam
static int cmucam_readbyte(int fd) {
    int rc = cmucam_await(fd, 200);
    if (rc < 0) {
        return rc;
    }

    uint8_t b = 0;
    rc = read(fd, &b, 1);
    if (rc < 0) {
        perror("failed to read from CMUcam");
        return rc;
    } else if (rc == 0) {
        fprintf(stderr, "failed to read byte from CMUcam\n");
        return -EIO;
    }
    return b;
}

// read data until the prompt is found (a ':' character)
static int cmucam_find_prompt(int fd) {
    int rc = 0;
    do {
        rc = cmucam_readbyte(fd);
        if (rc < 0) {
            return rc;
        }
    } while (rc != ':');
    return 0;
}

// ping CMUcam until we receive a prompt
static int cmucam_synchronize_prompt(int fd) {
    fprintf(stderr, "[DEBUG] quiescing CMUcam\n");
    int rc = cmucam_quiesce(fd);
    if (rc < 0) {
        return rc;
    }

    // attempt to get a prompt
    //
    // "attempts" starts at 15 as that's the most '\r''s that need to be sent to recover from
    // sending the ascii version of the reset command when the camera is in raw mode
    int attempts = 15;
    do {
        fprintf(stderr, "[DEBUG] pinging CMUcam\n");
        rc = write(fd, cmucam_cmd_ping, sizeof cmucam_cmd_ping);
        if (rc < 0) {
            perror("failed to write to CMUcam");
            return rc;
        } else if (rc != sizeof cmucam_cmd_ping) {
            fprintf(stderr, "failed to write all command bytes to CMUcam\n");
            return -EIO;
        }

        rc = cmucam_await(fd, 100);
        if (rc < 0 && rc != -ETIME) {
            return rc;
        } else if (!rc) {
            // some data has come back from the CMUcam
            break;
        }
    } while (--attempts);

    if (!attempts || rc == -ETIME) {
        fprintf(stderr, "[DEBUG] timed out waiting for CMUcam\n");
        return -ETIME;
    }

    fprintf(stderr, "[DEBUG] pong from CMUcam\n");
    return cmucam_find_prompt(fd);
}

static int cmucam_reset_asciimode(int fd) {
    int rc = write(fd, cmucam_acmd_reset, sizeof cmucam_acmd_reset);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if (rc != sizeof cmucam_acmd_reset) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }

    return cmucam_find_prompt(fd);
}

static int cmucam_reset_rawmode(int fd) {
    int rc = write(fd, cmucam_rcmd_reset, sizeof cmucam_rcmd_reset);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if (rc != sizeof cmucam_rcmd_reset) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }

    return cmucam_find_prompt(fd);
}

// get the CMUcam into a known state
static int cmucam_synchronize(int fd) {
    int rc = cmucam_synchronize_prompt(fd);
    if (rc < 0) {
        return rc;
    }

    // we don't know what state the camera is in other than that it's at the prompt
    // sending ascii commands in raw mode is BAD (will expect '\r' == 13 bytes of args after cmd)
    // sending raw mode commands in ascii mode will time out, a subsequent '\r' will give us a NCK

    // start by trying to send the reset command in raw mode
    fprintf(stderr, "[DEBUG] attempting raw-mode reset\n");
    rc = cmucam_reset_rawmode(fd);
    if (rc < 0 && rc != -ETIME) {
        return rc;
    }

    // if we find the prompt, we were in raw mode and the camera is now properly reset
    if (!rc) {
        return 0;
    }
    fprintf(stderr, "[DEBUG] raw-mode reset timed out\n");

    // since we timed out, we were probably in ascii mode. Need to resynchronize with the prompt
    rc = cmucam_synchronize_prompt(fd);
    if (rc < 0) {
        return rc;
    }

    // send reset command in ascii mode
    fprintf(stderr, "[DEBUG] attempting ascii-mode reset\n");
    return cmucam_reset_asciimode(fd);
}

int cmucam_rawmode_enter(int fd) {
    int rc = write(fd, cmucam_acmd_rawmode_enter, sizeof cmucam_acmd_rawmode_enter);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if (rc != sizeof cmucam_acmd_rawmode_enter) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }

    return cmucam_find_prompt(fd);
}

int cmucam_rawmode_exit(int fd) {
    int rc = write(fd, cmucam_rcmd_rawmode_exit, sizeof cmucam_rcmd_rawmode_exit);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if (rc != sizeof cmucam_rcmd_rawmode_exit) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }

    return cmucam_find_prompt(fd);
}

int cmucam_end_stream(int fd) {
    const uint8_t cmucam_rcmd_end_stream[] = {'\r'};
    int rc = write(fd, cmucam_rcmd_end_stream, sizeof cmucam_rcmd_end_stream);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if(rc != sizeof cmucam_rcmd_end_stream) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }

    return cmucam_find_prompt(fd);
}

int cmucam_set_color_mode(int fd, bool yuv, bool auto_white_balance) {
    const uint8_t color_mode = (1u << 5)
            | (!yuv ? (1u << 3) : 0u)
            | (auto_white_balance ? (1u << 2) : 0u);
    const uint8_t cmucam_rcmd_set_color_mode[] = { 'C', 'R', 2, 18, color_mode };

    int rc = write(fd, cmucam_rcmd_set_color_mode, sizeof cmucam_rcmd_set_color_mode);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if(rc != sizeof cmucam_rcmd_set_color_mode) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }
    return cmucam_find_prompt(fd);
}

int cmucam_set_auto_exposure(int fd, bool on) {
    const uint8_t autoexposure_mode = on ? 33 : 32;
    const uint8_t cmucam_rcmd_set_autoexposure[] = { 'C', 'R', 2, 19, autoexposure_mode };

    int rc = write(fd, cmucam_rcmd_set_autoexposure, sizeof cmucam_rcmd_set_autoexposure);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if(rc != sizeof cmucam_rcmd_set_autoexposure) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }
    return cmucam_find_prompt(fd);
}

int cmucam_set_line_mode(int fd, bool on) {
    const uint8_t cmucam_rcmd_set_linemode[] = { 'L', 'M', 1, on ? 1 : 0 };

    int rc = write(fd, cmucam_rcmd_set_linemode, sizeof cmucam_rcmd_set_linemode);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if(rc != sizeof cmucam_rcmd_set_linemode) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }
    return cmucam_find_prompt(fd);
}

int cmucam_set_noise_filter(int fd, bool on) {
    const uint8_t cmucam_rcmd_set_noise_filter[] = { 'N', 'F', 1, on ? 1 : 0 };

    int rc = write(fd, cmucam_rcmd_set_noise_filter, sizeof cmucam_rcmd_set_noise_filter);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if(rc != sizeof cmucam_rcmd_set_noise_filter) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }
    return cmucam_find_prompt(fd);
}

int cmucam_dumpframe(int fd) {
    int rc = write(fd, cmucam_rcmd_dumpframe, sizeof cmucam_rcmd_dumpframe);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if(rc != sizeof cmucam_rcmd_dumpframe) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }
    return 0;
}

int cmucam_dumpframe_next_column(int fd, uint8_t* column) {
    int rc = cmucam_readbyte(fd);
    if (rc < 0) {
        return rc;
    }

    size_t received = 0;
    if (rc == CMUCAM_DF_START_OF_FRAME) {
        // start of frame, no action
    } else if (rc == CMUCAM_DF_START_OF_COLUMN) {
        // new column, but we have to check for end-of-frame sentinel in next byte
        rc = cmucam_readbyte(fd);
        if (rc < 0) {
            return rc;
        } else if (rc == CMUCAM_DF_END_OF_FRAME) {
            rc = cmucam_find_prompt(fd);
            if (rc < 0) {
                return rc;
            }
            return 1;
        } else {
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
        rc = cmucam_await(fd, 1000);
        if (rc < 0) {
            perror("failed to read from CMUcam");
            return rc;
        }

        rc = read(fd, column + received, column_size - received);
        if (rc < 0) {
            perror("failed to read from CMUcam");
            return rc;
        }

        received += rc;
    } while (received != column_size);
    return 0;
}

int cmucam_set_window(int fd, int x, int y, int x2, int y2) {
    const uint8_t cmucam_rcmd_set_window[] = {
        'S', 'W', 4,
        x2 < x ? x2 : x, y2 < y ? y2 : y,
        x2 > x ? x2 : x, y2 > y ? y2 : y
    };

    int rc = write(fd, cmucam_rcmd_set_window, sizeof cmucam_rcmd_set_window);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if(rc != sizeof cmucam_rcmd_set_window) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }
    return cmucam_find_prompt(fd);
}

int cmucam_reset_window(int fd) {
    int rc = write(fd, cmucam_rcmd_reset_window, sizeof cmucam_rcmd_reset_window);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if(rc != sizeof cmucam_rcmd_reset_window) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }
    return cmucam_find_prompt(fd);
}

int cmucam_track_window(int fd) {
    int rc = write(fd, cmucam_rcmd_track_window, sizeof cmucam_rcmd_track_window);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if(rc != sizeof cmucam_rcmd_track_window) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }
    return 0;
}

int cmucam_track_color(int fd, uint8_t rmin, uint8_t rmax, uint8_t gmin, uint8_t gmax,
        uint8_t bmin, uint8_t bmax) {
    const uint8_t cmucam_rcmd_track_color[] = { 'T', 'C', 6, rmin, rmax, gmin, gmax, bmin, bmax };

    int rc = write(fd, cmucam_rcmd_track_color, sizeof cmucam_rcmd_track_color);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if(rc != sizeof cmucam_rcmd_track_color) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }
    return 0;
}

int cmucam_get_mean(int fd) {
    int rc = write(fd, cmucam_rcmd_get_mean, sizeof cmucam_rcmd_get_mean);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if(rc != sizeof cmucam_rcmd_reset_window) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }

    return 0;
}

int cmucam_read_packet(int fd, uint8_t *packet, uint8_t n) {
    // find the start of a packet
    int rc = 0;
    do {
        rc = cmucam_readbyte(fd);
        if (rc < 0) {
            return rc;
        }
    } while (rc != 255);

    // get the packet type
    uint8_t packet_size = 255;
    for (int i = 0; i < packet_size; i++) {
        rc = cmucam_readbyte(fd);
        if (rc < 0) {
            return rc;
        }
        if (i < n) {
            packet[i] = rc;
        }

        // identify packet size based on type
        if (packet_size == 255) {
            switch (rc) {
                case CMUCAM_PACKET_TYPE_C:
                    packet_size = CMUCAM_PACKET_TYPE_C_SIZE;
                    break;
                case CMUCAM_PACKET_TYPE_M:
                    packet_size = CMUCAM_PACKET_TYPE_M_SIZE;
                    break;
                case CMUCAM_PACKET_TYPE_S:
                    packet_size = CMUCAM_PACKET_TYPE_S_SIZE;
                    break;
                default:
                    fprintf(stderr, "CMUcam sent unknown packet type - %c\n", rc);
                    return -EINVAL;
            }
        }
    }
    return 0;
}

static int cmucam_open_port(const char* path) {
    // configure 115200 8n1, no in kernel processing
    int fd = open(path, O_RDWR | O_NONBLOCK);
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

    attrs.c_cc[VTIME] = 0;
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

int cmucam_open(const char* path) {
    int fd = cmucam_open_port(path);
    if (fd < 0) {
        return fd;
    }

    // add camera to event loop
    struct epoll_event ev = {0};
    ev.events = EPOLLIN;
    ev.data.fd = fd;
    int rc = epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev);
    if (rc < 0) {
        perror("failed to add CMUcam to event loop");
        close(fd);
        return rc;
    }

    rc = cmucam_synchronize(fd);
    if (rc < 0) {
        return rc;
    }

    rc = cmucam_rawmode_enter(fd);
    if (rc < 0) {
        return rc;
    }

    return fd;
}

int cmucam_close(int fd) {
    int rc = cmucam_rawmode_exit(fd);
    int rc2 = epoll_ctl(epoll_fd, EPOLL_CTL_DEL, fd, NULL);
    close(fd);
    return (rc < 0) ? rc : rc2;
}

int cmucam_initialize() {
    if (epoll_fd != -1) {
        return 0;
    }

    epoll_fd = epoll_create1(0);
    if (epoll_fd < 0) {
        perror("failed to create epoll handler");
        return epoll_fd;
    }
    return 0;
}