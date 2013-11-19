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

#ifndef __VEHICLE_INTERFACE_INCLUDED__
#define __VEHICLE_INTERFACE_INCLUDED__

#include "VehicleMessages.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIZE_RECEIVE_BUFFER   1000

typedef struct {
	uint32_t front;
	uint32_t back;
	int32_t fd;
	uint8_t  recvbuffer[SIZE_RECEIVE_BUFFER];
} VscInterfaceType;

/**
 * Initialize the VSC interface
 *
 * This function attempts to initialize the VSC interface.
 *
 * @param device Character string of serial device to open.
 * @param baud Baud rate of serial device to open.
 * @return VSC Interface Structure if successful or NULL on error
 */
VscInterfaceType* vsc_initialize(const char *device, const unsigned int baud);

/**
 * Cleanup the VSC interface
 *
 * This function attempts to close and cleanup the VSC interface.
 *
 * @param vscInterface VSC Interface Structure to close.  Set to NULL upon exit
 */
void vsc_cleanup(VscInterfaceType* vscInterface);

/**
 * Get Serial Interface File Descriptor
 *
 * This function returns the file descriptor of the serial interface to the VSC.
 *
 * @param vscInterface VSC Interface Structure.
 * @return File Descriptor of serial interface
 */
int32_t vsc_get_fd(VscInterfaceType* vscInterface);

/**
 * Send Message to the VSC
 *
 * This function attempts to send a message to the VSC.  It will insert the header
 * and checksum into the structure.
 *
 * @param vscInterface VSC Interface Structure.
 * @param sendMsg Pointer to structer with message to send to VSC.
 * @return File Descriptor of serial interface
 */
int32_t vsc_send_msg(VscInterfaceType* vscInterface, VscMsgType *sendMsg);

/**
 * Read next message from the VSC
 *
 * This function attempts to read any messages from the VSC data buffer.
 *
 * @param vscInterface VSC Interface Structure.
 * @param[out] newMsg Pointer to structure that is filled with new message from buffer.
 * @return 1 if newMsg is populated or -1 if no new messages are available
 */
int32_t vsc_read_next_msg(VscInterfaceType* vscInterface, VscMsgType *newMsg);

/**
 * Translate joystick value from structure into integer
 *
 * This function translates the value of a joystick axis from the J1939 like structure
 * into an integer value.
 *
 * @param joystick Axis to translate.
 * @return Joystick value from -1024 to 1024
 */
int32_t vsc_get_stick_value(JoystickType joystick);

/**
 * Translate button value from structure into integer
 *
 * This function translates the value of a joystick button from the J1939 like structure
 * into an integer value.
 *
 * @param button button to translate
 * @return 1 if set, 0 if not set
 */
int32_t vsc_get_button_value(uint8_t button);

/**
 * Send user feedback message to VSC
 *
 * This function packages and sends the user feedback data to the VSC.
 * NOTE:  The SRC can only display 16 Bit values.  Any values greater will be displayed as "XXXXX"
 *
 * @param vscInterface VSC Interface Structure.
 * @param key Index for which user feedback value to update.
 * @param value Value to update.
 */
void vsc_send_user_feedback(VscInterfaceType* vscInterface, uint8_t key, int32_t value);

/**
 * Send user feedback name message to VSC
 *
 * This function packages and sends the user feedback data to the VSC.
 *
 * @param vscInterface VSC Interface Structure.
 * @param key Index for which user feedback value to update.
 * @param value Value to update.
 */
void vsc_send_user_feedback_string(VscInterfaceType* vscInterface, uint8_t key, const char* value);

/**
 * Send heartbeat message to VSC
 *
 * This function packages and sends the heartbeat data to the VSC.
 *
 * @param vscInterface VSC Interface Structure.
 * @param EStopStatus value > 0 will indicate an E-STOP condition from SW.
 */
void vsc_send_heartbeat(VscInterfaceType* vscInterface, uint8_t EStopStatus);

#ifdef __cplusplus
}
#endif

#endif
