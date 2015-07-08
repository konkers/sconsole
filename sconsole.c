/* sconsole - cheap serial console (for xterm, etc)
 *
 * Copyright (c) 2005-2015
 *	Brian Swetland.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the authors nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <termios.h>
#include <signal.h>
#include <poll.h>
#include <unistd.h>

#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <sys/types.h>

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#define B4000000 4000000
#define B3500000 3500000
#define B3000000 3000000
#define B2500000 2500000
#define B2000000 2000000
#define B1500000 1500000
#define B1152000 1152000
#define B1000000 1000000
#define B921600 921600
#endif

static struct termios tio_save;

static void stdin_raw_init(void)
{
	struct termios tio;

	if (tcgetattr(0, &tio))
		return;
	if (tcgetattr(0, &tio_save))
		return;

	/* disable CANON, ECHO*, etc */
	tio.c_lflag = 0;

	/* no timeout but request at least one character per read */
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN] = 1;

	tcsetattr(0, TCSANOW, &tio);
	tcflush(0, TCIFLUSH);
}

static void stdin_raw_restore(void)
{
	tcsetattr(0, TCSANOW, &tio_save);
	tcflush(0, TCIFLUSH);
}

void oops(int x)
{
	char *msg = "\n[ killed by signal ]\n";
	write(2, msg, strlen(msg));
	stdin_raw_restore();
	exit(1);
}

int text2speed(const char *s)
{
	char *e;
	unsigned n = strtoul(s, &e, 10);
	switch (*e) {
	case 'k':
	case 'K':
		n *= 1000;
		break;
	case 'm':
	case 'M':
		n *= 1000000;
		break;
	}
	switch (n) {
	case 4000000:	return B4000000;
	case 3500000:	return B3500000;
	case 3000000:	return B3000000;
	case 2500000:	return B2500000;
	case 2000000:	return B2000000;
	case 1500000:	return B1500000;
	case 1152000:	return B1152000;
	case 1000000:	return B1000000;
	case 921600:	return B921600;
	case 115200:	return B115200;
	case 57600:	return B57600;
	case 38400:	return B38400;
	case 19200:	return B19200;
	case 9600:	return B9600;
	default:
		fprintf(stderr, "unsupported baud rate %dbps\n", n);
		return B115200;
	}
}

int openserial(const char *device, int speed)
{
	struct termios tio;
	int fd;
	int fl;

	/* open the serial port non-blocking to avoid waiting for cd */
	if ((fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
		return -1;
	}

	/* then switch the fd to blocking */
	if ((fl = fcntl(fd, F_GETFL, 0)) < 0) {
		close(fd);
		return -1;
	}
	if ((fl = fcntl(fd, F_SETFL,  fl & ~O_NDELAY)) < 0) {
		close(fd);
		return -1;
	}

	if (tcgetattr(fd, &tio)) {
		memset(&tio, 0, sizeof(tio));
	}

	tio.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
	tio.c_ispeed = B57600;
	tio.c_ospeed = B57600;
	tio.c_iflag = IGNPAR;
	tio.c_oflag &= ~ONLCR;
	tio.c_lflag = 0; /* turn of CANON, ECHO*, etc */
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN] = 1;
	tcsetattr(fd, TCSANOW, &tio);
	tcflush(fd, TCIFLUSH);

#ifdef __APPLE__
	tio.c_cflag =  CS8 | CLOCAL | CREAD;
#else
	tio.c_cflag =  speed | CS8 | CLOCAL | CREAD;
#endif
	tio.c_ispeed = speed;
	tio.c_ospeed = speed;

	tcsetattr(fd, TCSANOW, &tio);

#ifdef __APPLE__
	if (speed >= 921600) {
		if (ioctl(fd, IOSSIOSPEED, &speed) == -1) {
			fprintf(stderr, "error IOSSIOSPEED ioctl: %s(%d)\n",
				strerror(errno), errno);
		}
	}
#endif

	tcflush(fd, TCIFLUSH);
	return fd;
}

static unsigned char valid[256];

#define STATE_IDLE    0
#define STATE_PREFIX  1
#define STATE_COMMAND 2

#ifdef __APPLE__
/* Darwin's poll is broken.  Implement a replacement using select */
int select_poll(struct pollfd *fds, int nfds, int timeout)
{
	fd_set rfds;
	fd_set wfds;
	fd_set efds;
	int max_fd = 0;
	int retval;
	struct timeval tv;
	int i;

	FD_ZERO(&rfds);
	FD_ZERO(&wfds);
	FD_ZERO(&efds);

	for (i = 0; i < nfds; i++) {
		if (fds[i].events == POLLIN)
			FD_SET(fds[i].fd, &rfds);
		if (fds[i].events == POLLOUT)
			FD_SET(fds[i].fd, &wfds);
		FD_SET(fds[i].fd, &efds);

		if (fds[i].fd > max_fd)
			max_fd = fds[i].fd;

		fds[i].revents = 0;
	}

	retval = select(max_fd + 1, &rfds, &wfds, &efds, NULL);

	if (errno == EAGAIN) {
		return 0;
	}

	if (retval < 0) {
		return -1;
	}

	for (i = 0; i < nfds; i++ ) {
		if (FD_ISSET(fds[i].fd, &rfds))
			fds[i].revents |= POLLIN;
		if (FD_ISSET(fds[i].fd, &wfds))
			fds[i].revents |= POLLOUT;
		if (FD_ISSET(fds[i].fd, &efds))
			fds[i].revents |= POLLERR;
	}

	return 1;
}

#define poll select_poll

#endif /* __APPLE__ */

void usage(void) {
	fprintf(stderr,
		"usage: sconsole [ <flag> ]* [ <device> [ <speed> ] ]\n"
		"\n"
		"flags:   -t    transparent mode (don't filter unprintables, etc)\n"
		"         -l    log to console.log (or -lsomeotherlogfile)\n"
		"         -c    convert NL to CR on transmit\n"
		"         -x    display characters in hex\n"
		"\n"
		"default device /dev/ttyUSB and speed 115200\n"
		);
}

typedef enum {
	SWV_SYNC,
	SWV_1X1,
	SWV_1X2,
	SWV_1X4,
	SWV_2X2,
	SWV_2X4,
	SWV_3X4,
	SWV_4X4,
	SWV_PROTO,
	SWV_IDLE,
} swv_state_t;

typedef struct {
	swv_state_t state;
	unsigned zcount;
	unsigned ccount;
	unsigned id;
	unsigned val;
	unsigned char data[8];
} swv_t;

static swv_t swv = {
	.state = SWV_SYNC,
	.zcount = 0
};

void handle_swv_src(unsigned id, unsigned val, unsigned n) {
	printf("SRC %s %02x %08x\n", (id & 0x100) ? "HW" : "SW", id & 0xFF, val);
}

void handle_swv_proto(unsigned char *data, unsigned len) {
	switch (len) {
	case 1:
		printf("PRO %02x\n", data[0]);
		break;
	case 2:
		printf("PRO %02x %02x\n", data[0], data[1]);
		break;
	case 3:
		printf("PRO %02x %02x %02x\n",
			data[0], data[1], data[2]);
		break;
	case 4:
		printf("PRO %02x %02x %02x %02x\n",
			data[0], data[1], data[2], data[3]);
		break;
	case 5:
		printf("PRO %02x %02x %02x %02x %02x\n",
			data[0], data[1], data[2], data[3], data[4]);;
		break;
	case 6:
		printf("PRO %02x %02x %02x %02x %02x %02x\n",
			data[0], data[1], data[2], data[3],
			data[4], data[5]);
		break;
	case 7:
		printf("PRO %02x %02x %02x %02x %02x %02x %02x\n",
			data[0], data[1], data[2], data[3],
			data[4], data[5], data[6]);
		break;
	}
}

void handle_swv(swv_t *swv, unsigned x) {
	//printf("%02x ", x); fflush(stdout);
	// any sequence ending in 00 00 00 00 00 80 is a re-sync
	if (x == 0) {
		swv->zcount++;
	} else {
		if ((swv->zcount >= 5) && (x == 0x80)) {
			swv->state = SWV_IDLE;
			swv->zcount = 0;
			printf("SYNC\n");
			return;
		}
		swv->zcount = 0;
	}

	switch (swv->state) {
	case SWV_IDLE:
		if (x & 7) {
			// AAAAAHSS source packet
			swv->id = (x >> 3) | ((x & 4) << 6);
			swv->val = 0;
			swv->state = (x & 3);
		} else if (x != 0) {
			// CXXXXX00 protocol packet
			swv->data[0] = x;
			if (x & 0x80) {
				swv->ccount = 1;
				swv->state = SWV_PROTO;
			} else {
				handle_swv_proto(swv->data, 1);
			}
		} else {
			// 00 packets are for sync, ignore
		}
		break;
	case SWV_PROTO:
		swv->data[swv->ccount++] = x;
		// protocol packets end at 7 total bytes or a byte with bit7 clear
		if ((swv->ccount == 7) || (!(x & 0x80))) {
			handle_swv_proto(swv->data, swv->ccount);
			swv->state = SWV_IDLE;
		}
		break;
	case SWV_1X1:
		handle_swv_src(swv->id, x, 1);
		swv->state = SWV_IDLE;
		break;
	case SWV_1X2:
		swv->val = x;
		swv->state = SWV_2X2;
		break;
	case SWV_2X2:
		handle_swv_src(swv->id, swv->val | (x << 8), 2);
		swv->state = SWV_IDLE;
		break;
	case SWV_1X4:
		swv->val = x;
		swv->state = SWV_2X4;
		break;
	case SWV_2X4:
		swv->val |= (x << 8);
		swv->state = SWV_3X4;
		break;
	case SWV_3X4:
		swv->val |= (x << 16);
		swv->state = SWV_4X4;
		break;
	case SWV_4X4:
		handle_swv_src(swv->id, swv->val | (x << 24), 4);
		swv->state = SWV_IDLE;
		break;
	case SWV_SYNC:
		break;
	default:
		// impossible
		printf("fatal error, bad state %d\n", swv->state);
		exit(1);
	}
}

int main(int argc, char *argv[])
{
	struct pollfd fds[2];
	int speed = B115200;
#ifdef __APPLE__
	const char *device = "/dev/tty.usbserial";
#else
	const char *device = "/dev/ttyUSB0";
#endif
	const char *logfile = "console.log";
	int fd, n;
	int map_nl_to_cr = 0;
	int escape = 0;
	int logfd = -1;
	int hexmode = 0;
	unsigned char ESC = 27;
	int decode_swv = 0;

	for (n = ' '; n < 127; n++)
		valid[n] = 1;

	valid[7] = -1; /* bell */
	valid[8] = 1; /* backspace */
	valid[9] = 1; /* tab */
	valid[10] = 1; /* newline */
	valid[13] = 1; /* carriage return */

	while ((argc > 1) && (argv[1][0] == '-')) {
		switch (argv[1][1]) {
		case 'x':
			hexmode = 1;
			break;
		case 't':
			/* transparent mode */
			for (n = 0; n < 256; n++)
				valid[n] = 1;
			break;
		case 'l':
			/* log */
			if (argv[1][2])
				logfile = &argv[1][2];
			logfd = open(logfile, O_CREAT | O_WRONLY, 0644);
			break;
		case 'c':
			/* carriage return mode -- map \n to \r */
			map_nl_to_cr = 1;
			break;
		case 'h':
			usage();
			return 0;
		case 's':
			decode_swv = 1;
			break;
		default:
			fprintf(stderr, "error: unknown option %s\n\n", argv[1]);
			usage();
			return 1;
		}
		argv++;
		argc--;
	}

	if (argc > 1) {
		device = argv[1];
		argc--;
		argv++;
	}

	if (argc > 1) {
		speed = text2speed(argv[1]);
		fprintf(stderr, "SPEED: %s %08x\n", argv[1], speed);
		argc--;
		argv++;
	}

	fd = openserial(device, speed);
	if (fd < 0) {
		fprintf(stderr, "error opening '%s': %s\n",
			device, strerror(errno));
		return -1;
	}

	stdin_raw_init();
	signal(SIGINT, oops);

	fds[0].fd = 0;
	fds[0].events = POLLIN;

	fds[1].fd = fd;
	fds[1].events = POLLIN;

	fprintf(stderr, "[ %s ]\n", device);

	for (;;) {
		unsigned char x, t;

		if (poll(fds, 2, -1) < 1)
			continue;

		if (fds[0].revents & (POLLERR | POLLHUP)) {
			fprintf(stderr, "\n[ stdin port closed ]\n");
			break;
		}
		if (fds[1].revents & (POLLERR | POLLHUP)) {
			fprintf(stderr, "\n[ serial port closed ]\n");
			break;
		}
		if ((fds[0].revents & POLLIN) && (read(0, &x, 1) == 1)) {
			switch (escape) {
			case 0:
				if (x == 27) {
					escape = 1;
				} else {
					if ((x == '\n') && map_nl_to_cr) {
						x = '\r';
						write(fd, &x, 1);
					} else {
						write(fd, &x, 1);
					}
				}
				break;
			case 1:
				if (x == 27) {
					escape = 2;
					fprintf(stderr, "\n[ (b)reak? e(x)it? ]\n");
				} else {
					escape = 0;
					write(fd, &ESC, 1);
					write(fd, &x, 1);
				}
				break;
			case 2:
				escape = 0;
				switch (x) {
				case 27:
					write(fd, &x, 1);
					break;
				case 'b':
					fprintf(stderr, "[ break ]\n");
					tcsendbreak(fd, 0);
					break;
				case 'x':
					fprintf(stderr, "[ exit ]\n");
					goto done;
				default:
					fprintf(stderr, "[ huh? ]\n");
					break;
				}
				break;
			}
		}
		if ((fds[1].revents & POLLIN) && (read(fd, &x, 1) == 1)) {
			if (hexmode) {
				char hex[4];
				sprintf(hex, "%02x ", x);
				write(1, hex, 3);
			} else if (decode_swv) {
				handle_swv(&swv, x);
			} else {
				unsigned char c = x;
				if (!valid[x])
					c = '.';

				if (valid[x] != -1) {
					write(1, &c, 1);
					if (logfd != -1)
						write(logfd, &c, 1);
				}
			}
		}
	}

done:
	stdin_raw_restore();
	return 0;
}
