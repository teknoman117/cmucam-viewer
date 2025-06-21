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

#ifndef CMUCAM_VIEWER_CMUCAM_H_
#define CMUCAM_VIEWER_CMUCAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#define CMUCAM_IMAGE_WIDTH 80
#define CMUCAM_IMAGE_HEIGHT 143

#define CMUCAM_PACKET_TYPE_C ((uint8_t) 'C')
#define CMUCAM_PACKET_TYPE_C_SIZE 7
#define CMUCAM_PACKET_TYPE_M ((uint8_t) 'M')
#define CMUCAM_PACKET_TYPE_M_SIZE 9
#define CMUCAM_PACKET_TYPE_N ((uint8_t) 'N')
#define CMUCAM_PACKET_TYPE_N_SIZE 10
#define CMUCAM_PACKET_TYPE_S ((uint8_t) 'S')
#define CMUCAM_PACKET_TYPE_S_SIZE 7

#define CMUCAM_PACKET_TYPE_BASIC ((uint8_t ) 255)
#define CMUCAM_PACKET_TYPE_F_START ((uint8_t) 1)
#define CMUCAM_PACKET_TYPE_F_START_SIZE (CMUCAM_IMAGE_HEIGHT * 3)
#define CMUCAM_PACKET_TYPE_F_NEXT ((uint8_t) 2)
#define CMUCAM_PACKET_TYPE_F_NEXT_SIZE (CMUCAM_PACKET_TYPE_F_START_SIZE)
#define CMUCAM_PACKET_TYPE_F_END ((uint8_t) 3)
#define CMUCAM_PACKET_TYPE_F_END_SIZE (0)

// Line-Mode Extended Tracking Data packet size is 80x48 bit-pixels (so 10 bytes * 48 rows)
#define CMUCAM_PACKET_TYPE_LM_TRACK ((uint8_t) 0xAA)
#define CMUCAM_PACKET_TYPE_LM_TRACK_SIZE (10*48)

// Line-Mode Extended Mean Data packet size is 143/2 lines (so ?)
#define CMUCAM_PACKET_TYPE_LM_MEAN ((uint8_t) 0xFE)
#define CMUCAM_PACKET_TYPE_LM_MEAN_SIZE ((143/2) * 3)

struct cmucam_packet {
    uint8_t type;
    union {
        struct {
            uint8_t lx;
            uint8_t ly;
            uint8_t rx;
            uint8_t ry;
            uint8_t pixels;
            uint8_t confidence;
        } c;
        struct {
            uint8_t mx;
            uint8_t my;
            uint8_t lx;
            uint8_t ly;
            uint8_t rx;
            uint8_t ry;
            uint8_t pixels;
            uint8_t confidence;
        } m;
        struct {
            uint8_t spos;
            uint8_t mx;
            uint8_t my;
            uint8_t lx;
            uint8_t ly;
            uint8_t rx;
            uint8_t ry;
            uint8_t pixels;
            uint8_t confidence;
        } n;
        struct {
            uint8_t rmean;
            uint8_t gmean;
            uint8_t bmean;
            uint8_t rdeviation;
            uint8_t gdeviation;
            uint8_t bdeviation;
        } s;
        struct {
            uint8_t *data;
            size_t capacity;
            size_t length;
        } extended;
    };
};

int cmucam_initialize();

int cmucam_open(const char* path, bool skip_init);
int cmucam_close(int fd);

int cmucam_set_color_mode(int fd, bool yuv, bool auto_white_balance);
int cmucam_set_auto_exposure(int fd, bool on);
int cmucam_set_line_mode(int fd, bool on);
int cmucam_set_noise_filter(int fd, bool on);

int cmucam_set_window(int fd, int x, int y, int x2, int y2);
int cmucam_reset_window(int fd);

int cmucam_end_stream(int fd);

int cmucam_track_window(int fd);
int cmucam_track_color(int fd, uint8_t rmin, uint8_t rmax, uint8_t gmin, uint8_t gmax,
        uint8_t bmin, uint8_t bmax);

int cmucam_get_mean(int fd);

int cmucam_dump_frame(int fd);

int cmucam_read_packet(int fd, struct cmucam_packet *packet);

#ifdef __cplusplus
}
#endif

#endif /* CMUCAM_VIEWER_CMUCAM_H_ */
