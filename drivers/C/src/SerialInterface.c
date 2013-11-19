/***************************************************************************
 * Humanistic Robotics VSC Interface Library                               *
 * Version 1.1                                                             *
 * Copyright 2013, Humanistic Robotics, Inc                                *
 ***************************************************************************/
/*
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

#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>

#include "SerialInterface.h"

/**
 * Open a serial port
 *
 * Supported baud rate for Linux:
 *    110, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
 *
 *
 * @param device Port name /dev/ttyS0, /dev/ttyACM0, /dev/ttyUSB0, ...
 * @param baud Baud rate of the serial port
 * @return fd success, -1 on error
 */
int open_serial_interface(const char *device, const unsigned int baud) {
	struct termios options;
	int fd = -1;
	speed_t speed = B115200;

	/* Open device */
	fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (fd == -1) {
		return -1;
	}

	/* Set parameters */
	tcgetattr(fd, &options); /* Get the current options of the port */
	memset(&options, 0, sizeof(options)); /* Clear all the options */
	switch (baud) {
	case 110:
		speed = B110;
		break;
	case 300:
		speed = B300;
		break;
	case 600:
		speed = B600;
		break;
	case 1200:
		speed = B1200;
		break;
	case 2400:
		speed = B2400;
		break;
	case 4800:
		speed = B4800;
		break;
	case 9600:
		speed = B9600;
		break;
	case 19200:
		speed = B19200;
		break;
	case 38400:
		speed = B38400;
		break;
	case 57600:
		speed = B57600;
		break;
	case 115200:
		speed = B115200;
		break;
	default:
		return -1;
	}

	/* Set the speed (Bauds) */
	options.c_cflag = speed | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;

	options.c_cc[VTIME] = 0; /* Timer unused */
	options.c_cc[VMIN] = 0; /* At least on character before satisfy reading */
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options); /* Activate the settings */

	/* Return File Descriptor */
	return fd;
}

/**
 * Close the connection with the current device
 *
 * @param fd The file descriptor to close
 */
void close_serial_interface(int fd) {
	if (fd != -1) {
		close(fd);
	}
}

/**
 * Write bytes to serial port from buffer
 *
 * @param buffer The data buffer to write data from
 * @param bytes The number of bytes to write from the buffer
 * @return number of bytes written on success, -1 on error
 */
int write_to_serial(int fd, const void *buffer, const unsigned int bytes) {
	if (fd == -1) {
		return -1;
	}

	return write(fd, buffer, bytes);
}

/**
 * Read bytes from serial port
 *
 * @param buffer The data buffer to write data into
 * @param bytes The number of bytes to read into the buffer
 * @return number of bytes read on success, -1 on error
 */
int read_from_serial(int fd, void *buffer, unsigned int bytes) {
	if (fd == -1) {
		return -1;
	}

	return read(fd, buffer, bytes);
}

