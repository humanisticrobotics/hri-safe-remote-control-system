/***************************************************************************
 * Humanistic Robotics VSC Interface Library                               *
 * Version 1.0                                                             *
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

#ifndef __SERIAL_INTERFACE_INCLUDED__
#define __SERIAL_INTERFACE_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

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
int open_serial_interface(const char *device, const unsigned int baud);

/**
 * Close the connection with the current device
 *
 * @param fd The file descriptor to close
 */
void close_serial_interface(int fd);

/**
 * Write bytes to serial port from buffer
 *
 * @param buffer The data buffer to write data from
 * @param bytes The number of bytes to write from the buffer
 * @return number of bytes written on success, -1 on error
 */
int write_to_serial(int fd, const void *buffer, const unsigned int bytes);

/**
 * Read bytes from serial port
 *
 * @param buffer The data buffer to write data into
 * @param bytes The number of bytes to read into the buffer
 * @return number of bytes read on success, -1 on error
 */
int read_from_serial(int fd, void *buffer,unsigned int bytes);

#ifdef __cplusplus
}
#endif


#endif

