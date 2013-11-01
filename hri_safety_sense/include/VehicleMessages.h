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

#ifndef __VEHICLE_MESSAGES_INCLUDED__
#define __VEHICLE_MESSAGES_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/************************************************************
 * Application Notes:
 *   - Bit Fields in structs are compiler specific.  All
 *     Structs defined in this file are LSB first.
 *   - Structs need to be packed for alignment of
 *     serial interface data.
 ************************************************************/
#pragma pack(1)

#define VSC_MESSAGE_HEADER_1 	0x10
#define VSC_MESSAGE_HEADER_2 	0x01

#define VSC_MAX_MESSAGE_LENGTH 	248			/**  Maximum message length for the VSC  */
#define VSC_HEADER_OVERHEAD 	4			/**  4 for HEADERS, MsgType, and Length  */
#define VSC_FOOTER_OVERHEAD 	2			/**  2 for checksum						*/
#define VSC_MIN_MESSAGE_LENGTH (VSC_HEADER_OVERHEAD + VSC_FOOTER_OVERHEAD)

#define VSC_USER_FEEDBACK_STRING_LENGTH 20

/** VSC_STATES_TYPE
 * 	The enumerated values of VSC states.
 */
enum VSC_STATES_TYPE {
	VSC_STATE_SEARCHING = 0x01,
	VSC_STATE_LOCAL = 0x04,
	VSC_STATE_OPERATIONAL = 0x09,
	VSC_STATE_MENU = 0x0A,
	VSC_STATE_PAUSE = 0x0B
};

/** VSC_MESSAGE_TYPE
 * 	The enumerated values of VSC messages.
 */
enum VSC_MESSAGE_TYPE {
	MSG_VSC_JOYSTICK = 0x10,
	MSG_VSC_NMEA_STRING = 0x12,
	MSG_VSC_HEARTBEAT = 0x20,
	MSG_USER_HEARTBEAT = 0x21,
	MSG_USER_FEEDBACK = 0x30,
	MSG_USER_FEEDBACK_STRING = 0x31
};

/** VSC_USER_FEEDBACK_KEY_TYPE
 * 	The enumerated values of the available user feedback keys.
 */
enum VSC_USER_FEEDBACK_KEY_TYPE {
	VSC_USER_FEEDBACK_KEY_1 = 1,
	VSC_USER_FEEDBACK_KEY_2,
	VSC_USER_FEEDBACK_KEY_3,
	VSC_USER_FEEDBACK_KEY_4,
	VSC_USER_FEEDBACK_KEY_5,
	VSC_USER_FEEDBACK_KEY_6,
	VSC_USER_FEEDBACK_KEY_7,
	VSC_USER_FEEDBACK_KEY_8,
	VSC_USER_FEEDBACK_KEY_9,
	VSC_USER_LEFT_MOTOR_INTENSITY = 10,
	VSC_USER_RIGHT_MOTOR_INTENSITY = 11,
	VSC_USER_BOTH_MOTOR_INTENSITY = 12,
	VSC_USER_DISPLAY_ROW_1 = 90,
	VSC_USER_DISPLAY_ROW_2 = 91,
	VSC_USER_DISPLAY_ROW_3 = 92,
	VSC_USER_DISPLAY_ROW_4 = 93,
	VSC_USER_DISPLAY_MODE = 99,
};

/** VscMsgData
 * 	The union for data that is sent and received to the VSC.  This contains
 * 	the 16-bit header, message type, length, data fields and fletcher checksum.
 */
typedef union {
	uint8_t  buffer[VSC_MAX_MESSAGE_LENGTH + VSC_HEADER_OVERHEAD];
	struct {
		uint8_t  header_1;
		uint8_t  header_2;
		uint8_t  msgType;
		uint8_t  length;
		uint8_t  data[VSC_MAX_MESSAGE_LENGTH];
	};
} VscMsgData;

/** VscMsgType
 * 	The Structure for data that is sent and received by the VSC.
 */
typedef struct {
	VscMsgData  msg;
} VscMsgType;

/** JOYSTICK_STATUS_TYPE
 * 	The enumerated values of the joystick status values.
 */
enum JOYSTICK_STATUS_TYPE {
	STATUS_NOT_RECOGNIZED = 0x0,
	STATUS_SET = 0x1,
	STATUS_ERROR = 0x2,
	STATUS_NOT_AVAILABLE = 0x3
};

/** SwitchType
 * 	The Structure for the 1 byte packed data of the joystick button value.
 */
typedef struct {
	uint8_t home:2;				/* JOYSTICK_STATUS_TYPE */
	uint8_t first:2;			/* JOYSTICK_STATUS_TYPE */
	uint8_t second:2;			/* JOYSTICK_STATUS_TYPE */
	uint8_t third:2;			/* JOYSTICK_STATUS_TYPE */
} SwitchType;

/** JoystickType
 * 	The Structure for the 2 byte packed data of the joystick axis value.
 */
typedef struct {
	uint8_t neutral_status:2;	/* JOYSTICK_STATUS_TYPE */
	uint8_t negative_status:2;	/* JOYSTICK_STATUS_TYPE */
	uint8_t positive_status:2;	/* JOYSTICK_STATUS_TYPE */
	uint8_t mag_lsb:2;
	uint8_t magnitude;
} JoystickType;

/** JoystickMsgType
 * 	The Structure for the packed joystick message that is received from the VSC.
 */
typedef struct {
	JoystickType leftX;
	JoystickType leftY;
	JoystickType leftZ;
	JoystickType rightX;
	JoystickType rightY;
	JoystickType rightZ;
	SwitchType   leftSwitch;
	SwitchType   rightSwitch;
} JoystickMsgType;

/** ESTOP_STATUS_TYPE
 * 	The enumerated values of the emergency stop values.
 */
enum ESTOP_STATUS_TYPE {
	ESTOP_STATUS_NOT_SET = 0x0,
	ESTOP_STATUS_ACTIVATED = 0x1
};

/** EstopStatusType
 * 	The Structure for the 1 byte packed data of the joystick button value.
 */
typedef union {
	struct {
		uint8_t SRC:2;
		uint8_t VSC:2;
		uint8_t RSVD:2;
		uint8_t USER:2;
		uint8_t RSVD_2;
		uint8_t RSVD_3;
		uint8_t RSVD_4;
	} bits;
	uint32_t bytes;
} EstopStatusType;

/** UserHeartbeatMsgType
 * 	The Structure for the packed heartbeat message that is sent to the VSC.
 */
typedef struct {
	uint8_t EStopStatus;
} UserHeartbeatMsgType;

/** HeartbeatMsgType
 * 	The Structure for the packed heartbeat message that is received from the VSC.
 */
typedef struct {
	uint8_t VscMode;
	uint8_t AutonomonyMode;
	uint32_t EStopStatus;
} HeartbeatMsgType;

/** GpsMsgType
 * 	The Structure for the packed gps message that is received from the VSC.
 */
typedef struct {
	uint8_t source;
	uint8_t data[VSC_MAX_MESSAGE_LENGTH-1];
} GpsMsgType;

/** UserFeedbackMsgType
 * 	The Structure for the packed key value pair that is used to transmit the user
 * 	feedback data to the VSC.
 */
typedef struct {
	uint8_t key;
	int32_t value;
} UserFeedbackMsgType;

/** UserFeedbackStringMsgType
 * 	The Structure for the packed key value pair that is used to transmit the user
 * 	feedback data to the VSC.
 */
typedef struct {
	uint8_t key;
	char value[VSC_USER_FEEDBACK_STRING_LENGTH];
} UserFeedbackStringMsgType;

/** MOTOR_INTENSITY_TYPE
 * 	The enumerated values of the motor control intensities 
 */
enum DISPLAY_MODE_TYPE {
        DISPLAY_MODE_STANDARD = 0,
        DISPLAY_MODE_CUSTOM_TEXT,
        DISPLAY_MODE_TEXT_VALUE,
        DISPLAY_MODE_VALUES
};

/** MOTOR_INTENSITY_TYPE
 * 	The enumerated values of the motor control intensities 
 */
enum MOTOR_INTENSITY_TYPE {
        MOTOR_CONTROL_INTENSITY_OFF = 0,
        MOTOR_CONTROL_INTENSITY_HIGH = 1,
        MOTOR_CONTROL_INTENSITY_MAX
};

#ifdef __cplusplus
}
#endif


#endif
