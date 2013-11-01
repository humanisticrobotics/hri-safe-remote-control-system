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
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>

#include "VehicleMessages.h"
#include "VehicleInterface.h"
#include "SerialInterface.h"

/**
 * Calculate the Fletcher 16 Checksum
 *
 * The Fletcher checksum is an algorithm for computing a position-dependent checksum. The
 * objective of the Fletcher checksum was to provide error-detection properties approaching
 * those of a cyclic redundancy check but with the lower computational effort associated
 * with summation techniques.
 *
 * @param data A pointer to an array of data to calculate the checksum of.
 * @param count Number of bytes in the array of data.
 * @return 16-Bit checksum value
 */
static uint16_t checksum_16(uint8_t* data, int count) {
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;
	int index;

	for (index = 0; index < count; index++) {
		sum1 = (sum1 + data[index]) & 0xff;
		sum2 = (sum2 + sum1) & 0xff;
	}

	return (sum2 << 8) | sum1;
}

/**
 * Initialize the VSC interface
 *
 * This function attempts to initialize the VSC interface.
 *
 * @param device Character string of serial device to open.
 * @param baud Baud rate of serial device to open.
 * @return VSC Interface Structure if successful or NULL on error
 */
VscInterfaceType* vsc_initialize(const char *device, const unsigned int baud) {
	VscInterfaceType* newVscInterface = malloc(sizeof(VscInterfaceType));

	printf("Opening serial port (%s,%i).\n", device, baud);
	newVscInterface->fd = open_serial_interface(device, baud);
	newVscInterface->front = newVscInterface->back = 0;
	if (newVscInterface->fd == -1) {
		printf("Opening serial port (%s,%i) failed.\n", device, baud);
		free(newVscInterface);
		newVscInterface = NULL;
	}

	return newVscInterface;
}

/**
 * Cleanup the VSC interface
 *
 * This function attempts to close and cleanup the VSC interface.
 *
 * @param vscInterface VSC Interface Structure to close.  Set to NULL upon exit
 */
void vsc_cleanup(VscInterfaceType* vscInterface) {
	if (vscInterface) {
		if (vscInterface->fd != -1) {
			close_serial_interface(vscInterface->fd);
		}

		free(vscInterface);
		vscInterface = NULL;
	}
}

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
int vsc_send_msg(VscInterfaceType* vscInterface, VscMsgType *sendMsg) {
	uint16_t checksum;
	uint32_t retval;

	/* Verify Msg is valid. */
	if (sendMsg->msg.length > VSC_MAX_MESSAGE_LENGTH)
		return -1;

	sendMsg->msg.header_1 = VSC_MESSAGE_HEADER_1;
	sendMsg->msg.header_2 = VSC_MESSAGE_HEADER_2;

	/* Calculate Fletchers Checksum and add to buffer. */
	checksum = checksum_16((uint8_t*) sendMsg->msg.buffer,
			sendMsg->msg.length + VSC_HEADER_OVERHEAD);

	/* Add checksum to end of buffer */
	sendMsg->msg.buffer[sendMsg->msg.length + VSC_HEADER_OVERHEAD] = checksum
			& 0xff;
	sendMsg->msg.buffer[sendMsg->msg.length + VSC_HEADER_OVERHEAD + 1] =
			(checksum >> 8) & 0xff;

	/* Send message over VSC connection. */
	retval = write_to_serial(vscInterface->fd, (void*) sendMsg->msg.buffer,
			sendMsg->msg.length + VSC_HEADER_OVERHEAD + VSC_FOOTER_OVERHEAD);

	return retval;
}

/**
 * Read next message from the VSC
 *
 * This function attempts to read any messages from the VSC data buffer.
 *
 * @param vscInterface VSC Interface Structure.
 * @param[out] newMsg Pointer to structure that is filled with new message from buffer.
 * @return 1 if newMsg is populated or -1 if no new messages are available
 */
int vsc_read_next_msg(VscInterfaceType* vscInterface, VscMsgType *newMsg) {
	VscMsgType *msgPtr;
	int retval = -1;
	bool done = false;
	int bytesRead;

	/* Perform non-blocking read on serial terminal into receive buffer */
	bytesRead = read_from_serial(vscInterface->fd,
			(void*) (vscInterface->recvbuffer + vscInterface->front),
			(SIZE_RECEIVE_BUFFER - vscInterface->front));

	/* Received Data */
	if (bytesRead >= 0) {
		vscInterface->front += bytesRead;
	} else {
		fprintf(stderr, "VscInterface: Receive Error. (%i) (%i)\n", retval,
				errno);
	}

	/* Check for messages in queue when all of the following are true:
	 *  - Queue is not empty
	 *  - Queue size is at least as big as smallest message
	 *  - Haven't already found a message
	 *
	 * Handle:
	 * Buffer starting mid message (trash)
	 * Buffer containing multiple messages (return first, update pointers)
	 * Buffer containing half message (leave buffer as is)
	 * Buffer containing full and half message (return first, update pointers)
	 * Buffer containing full message (return message, update pointers)
	 */
	while (vscInterface->front != vscInterface->back
			&& (vscInterface->front - vscInterface->back
					>= VSC_MIN_MESSAGE_LENGTH) && !done) {
		msgPtr = (VscMsgType *) (vscInterface->recvbuffer + vscInterface->back);
		if (msgPtr->msg.header_1 == VSC_MESSAGE_HEADER_1
				&& msgPtr->msg.header_2 == VSC_MESSAGE_HEADER_2) {
			/* Verify length */
			uint32_t expectedLength = msgPtr->msg.length + VSC_HEADER_OVERHEAD
					+ VSC_FOOTER_OVERHEAD;

			if ((vscInterface->front - vscInterface->back) < expectedLength) {
				/* Received partial message */
				done = true;
			} else {

				/* Calculate Fletchers Checksum verify */
				uint16_t checksum = checksum_16((uint8_t*) msgPtr->msg.buffer,
						msgPtr->msg.length + VSC_HEADER_OVERHEAD);
				if ((msgPtr->msg.buffer[msgPtr->msg.length + VSC_HEADER_OVERHEAD]
						== (checksum & 0xff))
						&& (msgPtr->msg.buffer[msgPtr->msg.length
								+ VSC_HEADER_OVERHEAD + 1]
								== ((checksum >> 8) & 0xff))) {

					/* Copy to output */
					memcpy((void*) newMsg, (void*) msgPtr, expectedLength);

					/* Update indices */
					vscInterface->back += expectedLength;

					done = true;
					retval = 1;
				} else {
					fprintf(stderr,
							"VscInterface: Invalid checksum 0x%02x 0x%02x received when expecting 0x%02x 0x%02x\n",
							msgPtr->msg.buffer[msgPtr->msg.length
									+ VSC_HEADER_OVERHEAD],
							msgPtr->msg.buffer[msgPtr->msg.length
									+ VSC_HEADER_OVERHEAD + 1], checksum & 0xff,
							(checksum >> 8) & 0xff);
					vscInterface->back += expectedLength;
				}
			}
		} else {
			/* Search for start of message. */
			fprintf(stderr,
					"VscInterface: Invalid bytes 0x%02x 0x%02x received when expecting message headers\n",
					msgPtr->msg.header_1, msgPtr->msg.header_2);
			vscInterface->back++;
		}
	}

	/* Reset pointers */
	if (vscInterface->front == vscInterface->back) {
		vscInterface->front = vscInterface->back = 0;
	} else if (vscInterface->front == SIZE_RECEIVE_BUFFER) {
		/* Handle the case where the buffer is FULL from a backlog, and a partial message is on the boundary.
		 * Move partial message to back of queue to open up space to read remainder of message.
		 */
		memmove(vscInterface->recvbuffer,
				vscInterface->recvbuffer + vscInterface->back,
				vscInterface->front - vscInterface->back);
		vscInterface->front = vscInterface->front - vscInterface->back;
		vscInterface->back = 0;
	}

	return retval;
}

/**
 * Translate joystick value from structure into integer
 *
 * This function translates the value of a joystick axis from the J1939 like structure
 * into an integer value.
 *
 * @param joystick Axis to translate.
 * @return Joystick value from -1024 to 1024
 */
int32_t vsc_get_stick_value(JoystickType joystick) {
	int32_t magnitude = (joystick.magnitude << 2) + joystick.mag_lsb;

	if (joystick.neutral_status == STATUS_SET) {
		return 0;
	} else if (joystick.negative_status == STATUS_SET) {
		return -1 * magnitude;
	} else if (joystick.positive_status == STATUS_SET) {
		return magnitude;
	}

	return 0;
}

/**
 * Translate button value from structure into integer
 *
 * This function translates the value of a joystick button from the J1939 like structure
 * into an integer value.
 *
 * @param button button to translate
 * @return 1 if set, 0 if not set
 */
int32_t vsc_get_button_value(uint8_t button) {
	if (button == STATUS_SET) {
		return 1;
	}

	return 0;
}

/**
 * Send user feedback message to VSC
 *
 * This function packages and sends the user feedback data to the VSC.
 *
 * @param vscInterface VSC Interface Structure.
 * @param key Index for which user feedback value to update.
 * @param value Value to update.
 */
void vsc_send_user_feedback(VscInterfaceType* vscInterface, uint8_t key, int32_t value) {
	VscMsgType feedbackMsg;
	UserFeedbackMsgType *msgPtr = (UserFeedbackMsgType*) feedbackMsg.msg.data;

	/* Fill Message */
	feedbackMsg.msg.msgType = MSG_USER_FEEDBACK;
	feedbackMsg.msg.length = sizeof(UserFeedbackMsgType);

	/* Set this value > 0 to indicate an E-STOP condition from SW */
	msgPtr->key = key;
	msgPtr->value = value;

	/* Send Message */
	if (vsc_send_msg(vscInterface, &feedbackMsg) < 0) {
		fprintf(stderr, "vsc_example: Send Message Failure (Errno: %i)\n", errno);
	}
}

/**
 * Send user feedback name message to VSC
 *
 * This function packages and sends the user feedback data to the VSC.
 *
 * @param vscInterface VSC Interface Structure.
 * @param key Index for which user feedback value to update.
 * @param value Value to update.
 */
void vsc_send_user_feedback_string(VscInterfaceType* vscInterface, uint8_t key, const char* value) {
	VscMsgType feedbackMsg;
	UserFeedbackStringMsgType *msgPtr = (UserFeedbackStringMsgType*) feedbackMsg.msg.data;
        memset(feedbackMsg.msg.data, 0, sizeof(feedbackMsg.msg.data));

	/* Fill Message */
	feedbackMsg.msg.msgType = MSG_USER_FEEDBACK_STRING;
	feedbackMsg.msg.length = sizeof(UserFeedbackStringMsgType);

	/* Set this value > 0 to indicate an E-STOP condition from SW */
	msgPtr->key = key;
	strncpy(msgPtr->value, value, VSC_USER_FEEDBACK_STRING_LENGTH);

	/* Send Message */
	if (vsc_send_msg(vscInterface, &feedbackMsg) < 0) {
		fprintf(stderr, "vsc_example: Send Message Failure (Errno: %i)\n", errno);
	}
}

/**
 * Send heartbeat message to VSC
 *
 * This function packages and sends the heartbeat data to the VSC.
 *
 * @param vscInterface VSC Interface Structure.
 * @param EStopStatus value > 0 will indicate an E-STOP condition from SW.
 */
void vsc_send_heartbeat(VscInterfaceType* vscInterface, uint8_t EStopStatus) {
	VscMsgType heartbeatMsg;
	UserHeartbeatMsgType *msgPtr = (UserHeartbeatMsgType*) heartbeatMsg.msg.data;

	/* Fill Message */
	heartbeatMsg.msg.msgType = MSG_USER_HEARTBEAT;
	heartbeatMsg.msg.length = sizeof(UserHeartbeatMsgType);

	/* Set this value > 0 to indicate an E-STOP condition from SW */
	msgPtr->EStopStatus = EStopStatus;

	/* Send Message */
	if (vsc_send_msg(vscInterface, &heartbeatMsg) < 0) {
		fprintf(stderr, "vsc_example: Send Message Failure (Errno: %i)\n", errno);
	}

}

/**
 * Get Serial Interface File Descriptor
 *
 * This function returns the file descriptor of the serial interface to the VSC.
 *
 * @param vscInterface VSC Interface Structure.
 * @return File Descriptor of serial interface
 */
int vsc_get_fd(VscInterfaceType* vscInterface) {
	return vscInterface->fd;
}

