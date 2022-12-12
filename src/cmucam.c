#include "cmucam.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#define CMUCAM_DF_START_OF_FRAME 1
#define CMUCAM_DF_START_OF_COLUMN 2
#define CMUCAM_DF_END_OF_FRAME 3

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

int cmucam_dump_frame_next_column(int fd, uint8_t* column) {
    int rc = cmucam_readbyte(fd);
    if (rc < 0) {
        return rc;
    }

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

static int cmucam_open_port(const char* path) {
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

int cmucam_open(const char* path) {
    int fd = cmucam_open_port(path);
    if (fd < 0) {
        return fd;
    }

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

int cmucam_close(int fd) {
    return cmucam_exit_raw_mode(fd);
}