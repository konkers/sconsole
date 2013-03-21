/* sconsole - cheap serial console (for xterm, etc)
 *
 * Copyright (c) 2005-2010
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
	int n = atoi(s);
	switch (n) {
	case 115200:
		return B115200;
	case 57600:
		return B57600;
	case 38400:
		return B38400;
	case 19200:
		return B19200;
	case 9600:
		return B9600;
	default:
		return B115200;
	}
}

int openserial(const char *device, int speed)
{
	struct termios tio;
	int fd;
	int fl;

	/* open the serial port non-blocking to avoid waiting for cd */
	fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd < 0)
		return -1;


	/* then switch the fd to blocking */
	fl = fcntl(fd, F_GETFL, 0);
	if (fl < 0) {
		close(fd);
		return -1;
	}
	fl = fcntl(fd, F_SETFL,  fl & ~O_NDELAY);
	if (fl < 0) {
		close(fd);
		return -1;
	}

	if (tcgetattr(fd, &tio))
		memset(&tio, 0, sizeof(tio));

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

int main(int argc, char *argv[])
{
	struct pollfd fds[2];
	int speed = B115200;
	const char *device = "/dev/ttyUSB0";
	const char *logfile = "console.log";
	int fd, n;
	int escape = 0;
	int logfd = -1;
	unsigned char ESC = 27;

	for (n = ' '; n < 127; n++)
		valid[n] = 1;

	valid[7] = -1; /* bell */
	valid[8] = 1; /* backspace */
	valid[9] = 1; /* tab */
	valid[10] = 1; /* newline */
	valid[13] = 1; /* carriage return */

	while ((argc > 1) && (argv[1][0] == '-')) {
		switch (argv[1][1]) {
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
		default:
			fprintf(stderr, "unknown option %s\n", argv[1]);
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
		fprintf(stderr, "stderr open '%s'\n", device);
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

		if (poll(fds, 2, -1) > 0) {
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
						write(fd, &x, 1);
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
