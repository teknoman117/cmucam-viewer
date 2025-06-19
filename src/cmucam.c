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
        if (errno == EINTR) {
            return cmucam_await(fd, timeout);
        }
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
static int cmucam_read_byte(int fd) {
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

static int cmucam_read_bytes(int fd, uint8_t *data, size_t n) {
    size_t c = 0;
    do {
        int rc = cmucam_await(fd, 200);
        if (rc < 0) {
            return rc;
        }

        rc = read(fd, data + c, n - c);
        if (rc < 0) {
            return rc;
        }
        c += rc;
    } while (c != n);
    return 0;
}

// read data until the prompt is found (a ':' character)
static int cmucam_find_prompt(int fd) {
    int rc = 0;
    do {
        rc = cmucam_read_byte(fd);
        if (rc < 0) {
            return rc;
        }
    } while (rc != ':');
    return 0;
}

// send a command that will not end in a prompt
static int cmucam_command_noprompt(int fd, const uint8_t *cmd, size_t n) {
    int rc = write(fd, cmd, n);
    if (rc < 0) {
        perror("failed to write to CMUcam");
        return rc;
    } else if (rc != n) {
        fprintf(stderr, "failed to write all command bytes to CMUcam\n");
        return -EIO;
    }

    return 0;
}

// send a command and find prompt afterwards
static int cmucam_command(int fd, const uint8_t *cmd, size_t n) {
    int rc = cmucam_command_noprompt(fd, cmd, n);
    return rc ? rc : cmucam_find_prompt(fd);
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
        rc = cmucam_command_noprompt(fd, cmucam_cmd_ping, sizeof cmucam_cmd_ping);
        if (rc) {
            return rc;
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
    return cmucam_command(fd, cmucam_acmd_reset, sizeof cmucam_acmd_reset);
}

static int cmucam_reset_rawmode(int fd) {
    return cmucam_command(fd, cmucam_rcmd_reset, sizeof cmucam_rcmd_reset);
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

static int cmucam_rawmode_enter(int fd) {
    return cmucam_command(fd, cmucam_acmd_rawmode_enter, sizeof cmucam_acmd_rawmode_enter);
}

static int cmucam_rawmode_exit(int fd) {
    return cmucam_command(fd, cmucam_rcmd_rawmode_exit, sizeof cmucam_rcmd_rawmode_exit);
}

int cmucam_end_stream(int fd) {
    const uint8_t cmucam_rcmd_end_stream[] = {'\r'};

    return cmucam_command(fd, cmucam_rcmd_end_stream, sizeof cmucam_rcmd_end_stream);
}

int cmucam_set_color_mode(int fd, bool yuv, bool auto_white_balance) {
    const uint8_t color_mode = (1u << 5)
            | (!yuv ? (1u << 3) : 0u)
            | (auto_white_balance ? (1u << 2) : 0u);
    const uint8_t cmucam_rcmd_set_color_mode[] = { 'C', 'R', 2, 18, color_mode };

    return cmucam_command(fd, cmucam_rcmd_set_color_mode, sizeof cmucam_rcmd_set_color_mode);
}

int cmucam_set_auto_exposure(int fd, bool on) {
    const uint8_t cmucam_rcmd_set_autoexposure[] = { 'C', 'R', 2, 19, on ? 33 : 32 };

    return cmucam_command(fd, cmucam_rcmd_set_autoexposure, sizeof cmucam_rcmd_set_autoexposure);
}

int cmucam_set_line_mode(int fd, bool on) {
    const uint8_t cmucam_rcmd_set_linemode[] = { 'L', 'M', 1, on ? 1 : 0 };

    return cmucam_command(fd, cmucam_rcmd_set_linemode, sizeof cmucam_rcmd_set_linemode);
}

int cmucam_set_noise_filter(int fd, bool on) {
    const uint8_t cmucam_rcmd_set_noise_filter[] = { 'N', 'F', 1, on ? 1 : 0 };

    return cmucam_command(fd, cmucam_rcmd_set_noise_filter, sizeof cmucam_rcmd_set_noise_filter);
}

int cmucam_set_window(int fd, int x, int y, int x2, int y2) {
    const uint8_t cmucam_rcmd_set_window[] = {
        'S', 'W', 4,
        x2 < x ? x2 : x, y2 < y ? y2 : y,
        x2 > x ? x2 : x, y2 > y ? y2 : y
    };

    return cmucam_command(fd, cmucam_rcmd_set_window, sizeof cmucam_rcmd_set_window);
}

int cmucam_reset_window(int fd) {
    return cmucam_command(fd, cmucam_rcmd_reset_window, sizeof cmucam_rcmd_reset_window);
}

int cmucam_track_window(int fd) {
    return cmucam_command(fd, cmucam_rcmd_track_window, sizeof cmucam_rcmd_track_window);
}

int cmucam_track_color(int fd, uint8_t rmin, uint8_t rmax, uint8_t gmin, uint8_t gmax,
        uint8_t bmin, uint8_t bmax) {
    const uint8_t cmucam_rcmd_track_color[] = { 'T', 'C', 6, rmin, rmax, gmin, gmax, bmin, bmax };

    return cmucam_command_noprompt(fd, cmucam_rcmd_track_color, sizeof cmucam_rcmd_track_color);
}

int cmucam_get_mean(int fd) {
    return cmucam_command_noprompt(fd, cmucam_rcmd_get_mean, sizeof cmucam_rcmd_get_mean);
}

int cmucam_dump_frame(int fd) {
    return cmucam_command_noprompt(fd, cmucam_rcmd_dumpframe, sizeof cmucam_rcmd_dumpframe);
}

static int cmucam_read_f_packet(int fd, struct cmucam_packet *packet) {
    // for f-type packets, first byte could be end-of-frame sentinel
    int rc = cmucam_read_byte(fd);
    if (rc < 0) {
        return rc;
    }
    if (rc == CMUCAM_PACKET_TYPE_F_END) {
        packet->type = CMUCAM_PACKET_TYPE_F_END;
        return cmucam_find_prompt(fd);
    }

    // read column into extended buffer
    if (packet->extended.data != NULL
            && packet->extended.capacity >= CMUCAM_PACKET_TYPE_F_NEXT_SIZE) {
        packet->extended.data[0] = rc;
        packet->extended.length = CMUCAM_PACKET_TYPE_F_NEXT;
        return cmucam_read_bytes(fd, packet->extended.data + 1, CMUCAM_PACKET_TYPE_F_NEXT_SIZE - 1);
    }

    // no extended buffer, discard data
    for (int i = 1; i < CMUCAM_PACKET_TYPE_F_NEXT_SIZE; i++) {
        rc = cmucam_read_byte(fd);
        if (rc < 0) {
            return rc;
        }
    }
    return 0;
}

static int cmucam_read_lm_track_packet(int fd, struct cmucam_packet *packet) {
    packet->extended.length = 0;
    while (1) {
        int rc = cmucam_read_byte(fd);
        if (rc < 0) {
            return rc;
        } else if (rc == 0xAA) {
            break;
        }

        if (packet->extended.data != NULL
                && packet->extended.length < packet->extended.capacity) {
            packet->extended.data[packet->extended.length++] = rc;
        }
    }

    // read one final 0xAA packet
    int rc = cmucam_read_byte(fd);
    if (rc < 0) {
        return rc;
    } else if (rc != 0xAA) {
        return -EINVAL;
    }
    return 0;
}

static int cmucam_read_lm_mean_packet(int fd, struct cmucam_packet *packet) {
    packet->extended.length = 0;
    while (1) {
        int rc = cmucam_read_byte(fd);
        if (rc < 0) {
            return rc;
        } else if (rc == 0xFD) {
            break;
        }

        if (packet->extended.data != NULL
                && packet->extended.length < packet->extended.capacity) {
            packet->extended.data[packet->extended.length++] = rc;
        }
    }
    return 0;
}

static int cmucam_detect_packet(int fd) {
    while (1) {
        int rc = cmucam_read_byte(fd);
        if (rc < 0) {
            return rc;
        }

        switch (rc) {
            case CMUCAM_PACKET_TYPE_BASIC:
                // this is a basic data packet
                return cmucam_read_byte(fd);
            case CMUCAM_PACKET_TYPE_F_START:
            case CMUCAM_PACKET_TYPE_F_NEXT:
            case CMUCAM_PACKET_TYPE_LM_MEAN:
            case CMUCAM_PACKET_TYPE_LM_TRACK:
                // this is an extended type packet (frame dump, line mode, etc.)
                return rc;
            default:
                continue;
        }
    }
}

int cmucam_read_packet(int fd, struct cmucam_packet *packet) {
    int rc = cmucam_detect_packet(fd);
    if (rc < 0) {
        return rc;
    }

    packet->type = rc;
    switch (rc) {
        case CMUCAM_PACKET_TYPE_C:
            return cmucam_read_bytes(fd, (uint8_t *) &packet->c, sizeof packet->c);
        case CMUCAM_PACKET_TYPE_M:
            return cmucam_read_bytes(fd, (uint8_t *) &packet->m, sizeof packet->m);
        case CMUCAM_PACKET_TYPE_N:
            return cmucam_read_bytes(fd, (uint8_t *) &packet->n, sizeof packet->n);
        case CMUCAM_PACKET_TYPE_S:
            return cmucam_read_bytes(fd, (uint8_t *) &packet->s, sizeof packet->s);
        case CMUCAM_PACKET_TYPE_F_START:
        case CMUCAM_PACKET_TYPE_F_NEXT:
            return cmucam_read_f_packet(fd, packet);
        case CMUCAM_PACKET_TYPE_LM_TRACK:
            return cmucam_read_lm_track_packet(fd, packet);
        case CMUCAM_PACKET_TYPE_LM_MEAN:
            return cmucam_read_lm_mean_packet(fd, packet);
        default:
            break;
    }
    return rc;
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