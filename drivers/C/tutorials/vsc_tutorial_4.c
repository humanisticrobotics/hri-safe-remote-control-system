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

/***************************************************************************
 * VSC_TUTORIAL_4 - Value Display Mode Example with Motor Control
 *
 *   The third tutorial shows how to switch the SRC into the user value display
 *   mode.  The example shows how to constantly update user feedback values and
 *   also send motor control commands to the SRC.
 *
 ***************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>

#include "VehicleMessages.h"
#include "VehicleInterface.h"

/* File descriptor for VSC Interface */
VscInterfaceType* vscInterface;

void signal_handler(int s) {
	printf("Caught signal %d - Shutting down.\n", s);
	vsc_cleanup(vscInterface);

	exit(0);
}

unsigned long diffTime(struct timespec start, struct timespec end, struct timespec *temp) {
	if ((end.tv_nsec - start.tv_nsec) < 0) {
		temp->tv_sec = end.tv_sec - start.tv_sec - 1;
		temp->tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
	} else {
		temp->tv_sec = end.tv_sec - start.tv_sec;
		temp->tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	return temp->tv_nsec;
}

void handleJoystickMsg(VscMsgType *recvMsg) {
	JoystickMsgType *joyMsg = (JoystickMsgType*) recvMsg->msg.data;

	printf("Joystick (L / R): %5i, %5i, %5i / %5i, %5i, %5i ",
			vsc_get_stick_value(joyMsg->leftX), vsc_get_stick_value(joyMsg->leftY),
			vsc_get_stick_value(joyMsg->leftZ), vsc_get_stick_value(joyMsg->rightX),
			vsc_get_stick_value(joyMsg->rightY), vsc_get_stick_value(joyMsg->rightZ));

	printf("\tJoystick (L Pad / R Pad): %i, %i, %i, %i / %i, %i, %i, %i\n",
			vsc_get_button_value(joyMsg->leftSwitch.home), vsc_get_button_value(joyMsg->leftSwitch.first),
			vsc_get_button_value(joyMsg->leftSwitch.second), vsc_get_button_value(joyMsg->leftSwitch.third),
			vsc_get_button_value(joyMsg->rightSwitch.home), vsc_get_button_value(joyMsg->rightSwitch.first),
			vsc_get_button_value(joyMsg->rightSwitch.second), vsc_get_button_value(joyMsg->rightSwitch.third));

	/* TODO: Add application specific code here to handle joystick messages */
}

void handleHeartbeatMsg(VscMsgType *recvMsg) {
	HeartbeatMsgType *msgPtr = (HeartbeatMsgType*) recvMsg->msg.data;

	printf("Heartbeat: E-Stop:  0x%x, VscMode: 0x%x, AutonomonyMode: 0x%x\n", msgPtr->EStopStatus, msgPtr->VscMode, msgPtr->AutonomonyMode);

	if (msgPtr->EStopStatus > 0) {
		EstopStatusType stopStatus;
		stopStatus.bytes = msgPtr->EStopStatus;

		if(stopStatus.bits.SRC) {
			printf("WARNING!  Received EMERGENCY STOP from the SRC\n");
		}

		if(stopStatus.bits.VSC) {
			printf("WARNING!  Received EMERGENCY STOP from the VSC\n");
		}

		if(stopStatus.bits.USER) {
			printf("WARNING!  Received EMERGENCY STOP from the USER\n");
		}
	}

	/* TODO: Add application specific code here to handle heartbeat messages */
}

void handleGpsMsg(VscMsgType *recvMsg) {
	GpsMsgType *msgPtr = (GpsMsgType*) recvMsg->msg.data;
	char message[100];

	strncpy(message, (char*)msgPtr->data, recvMsg->msg.length-1);
	message[recvMsg->msg.length-1] = '\0';
	printf("Received GPS Message (0x%x): %s\n", msgPtr->source, message);

	/* TODO: Add application specific code here to handle GPS messages */
}

void handleFeedbackMsg(VscMsgType *recvMsg) {
	UserFeedbackMsgType *msgPtr = (UserFeedbackMsgType*) recvMsg->msg.data;

	printf("Received Feedback Message.  Key: %i, Value %i\n", msgPtr->key, msgPtr->value);

	/* TODO: Add application specific code here to handle feedback messages */
}

void readFromVsc() {
	VscMsgType recvMsg;

	/* Read all messages */
	while (vsc_read_next_msg(vscInterface, &recvMsg) > 0) {
		/* Read next Vsc Message */
		switch (recvMsg.msg.msgType) {
		case MSG_VSC_HEARTBEAT:
			handleHeartbeatMsg(&recvMsg);

			break;
		case MSG_VSC_NMEA_STRING:
			handleGpsMsg(&recvMsg);

			break;
		case MSG_USER_FEEDBACK:
			handleFeedbackMsg(&recvMsg);

			break;
		case MSG_VSC_JOYSTICK:
			handleJoystickMsg(&recvMsg);

			break;
		default:
			printf("ERROR: Receive Error.  Invalid MsgType (0x%02X)\n",
					recvMsg.msg.msgType);
			break;
		}
	}

}

int main(int argc, char *argv[]) {
	struct timespec lastSent, timeNow, lastReceived, timeDiff;
	struct timeval timeout;
	int max_fd, vsc_fd, retval;
	int16_t testvalue;
	fd_set input;
	testvalue = -1234;

	/* Verify Arguments */
	if (argc != 3) {
		printf("Usage - program SerialPort BaudRate\n");
		printf("\t%s /dev/ttyUSB0 115200\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	/* Catch CTRL-C */
	signal(SIGINT, signal_handler);

	/* Open VSC Interface */
	vscInterface = vsc_initialize(argv[1], atoi(argv[2]));
	if (vscInterface == NULL) {
		printf("Opening VSC Interface failed.\n");
		exit(EXIT_FAILURE);
	}

	/* Initialize the input set */
	vsc_fd = vsc_get_fd(vscInterface);
	FD_ZERO(&input);
	FD_SET(vsc_fd, &input);
	max_fd = vsc_fd + 1;

	/* Reset timing values to the current time */
	clock_gettime(CLOCK_REALTIME, &lastSent);
	clock_gettime(CLOCK_REALTIME, &lastReceived);

	/* Send Heartbeat Message to VSC */
	vsc_send_heartbeat(vscInterface, ESTOP_STATUS_NOT_SET);

	/* Send Display Mode to VSC */
	vsc_send_user_feedback(vscInterface, VSC_USER_DISPLAY_MODE, DISPLAY_MODE_VALUES);

	/* Send User String Values Once to VSC */
	vsc_send_user_feedback_string(vscInterface, VSC_USER_FEEDBACK_KEY_1, "Counter");

	/* Loop Forever */
	while (1) {
		/* Get current clock time */
		clock_gettime(CLOCK_REALTIME, &timeNow);

		/* Send Heartbeat messages every 50 Milliseconds (20 Hz) */
		if (diffTime(lastSent, timeNow, &timeDiff) > 50000) {
			/* Get current clock time */
			lastSent = timeNow;

			/* Send Heartbeat */
			vsc_send_heartbeat(vscInterface, ESTOP_STATUS_NOT_SET);

			testvalue++;
			if(testvalue % 1500 == 0) {
				/* Send Motor Messages */
				vsc_send_user_feedback(vscInterface, VSC_USER_BOTH_MOTOR_INTENSITY, MOTOR_CONTROL_INTENSITY_HIGH);
			} else {
				/* Send User Feedback Messages */
				vsc_send_user_feedback(vscInterface, VSC_USER_FEEDBACK_KEY_1, testvalue);
			}
		}

		/* Initialize the timeout structure for 50 milliseconds*/
		timeout.tv_sec = 0;
		timeout.tv_usec = (50000 - (diffTime(lastSent, timeNow, &timeDiff) * .001));

		/* Perform select on serial port or Timeout */
		FD_ZERO(&input);
		FD_SET(vsc_fd, &input);
		max_fd = vsc_fd + 1;

		retval = select(max_fd, &input, NULL, NULL, &timeout);

		/* See if there was an error */
		if (retval < 0) {
			fprintf(stderr, "vsc_example: select failed");
		} else if (retval == 0) {
			/* No data received - Check to see when we last recieved data from the VSC */
			clock_gettime(CLOCK_REALTIME, &timeNow);
			diffTime(lastReceived, timeNow, &timeDiff);

			if(timeDiff.tv_sec > 0) {
				printf("vsc_example: WARNING: No data received from VSC in %li.%09li seconds!\n",
						timeDiff.tv_sec, timeDiff.tv_nsec);
			}
		} else {
			/* Input received, check to see if its from the VSC */
			if (FD_ISSET(vsc_fd, &input)) {
				/* Read from VSC */
				readFromVsc();

				/* Record the last time input was recieved from the VSC */
				clock_gettime(CLOCK_REALTIME, &lastReceived);
			} else {
				fprintf(stderr, "vsc_example: invalid fd set");
			}
		}

	}

	/* Clean up */
	printf("Shutting down.\n");
	vsc_cleanup(vscInterface);

	return 0;
}

