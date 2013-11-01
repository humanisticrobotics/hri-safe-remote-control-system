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

/**
 * ROS Includes
 */
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt32.h"

/**
 * System Includes
 */
#include <errno.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <sys/resource.h>

/**
 * Includes
 */
#include "VscProcess.h"
#include "JoystickHandler.h"
#include "VehicleInterface.h"
#include "VehicleMessages.h"

using namespace hri_safety_sense;

VscProcess::VscProcess() :
	myEStopState(0)
{
	ros::NodeHandle nh("~");
	std::string serialPort = "/dev/ttyACM0";
	if(nh.getParam("serial", serialPort)) {
		ROS_INFO("Serial Port updated to:  %s",serialPort.c_str());
	}

	int  serialSpeed = 115200;
	if(nh.getParam("serial_speed", serialSpeed)) {
		ROS_INFO("Serial Port Speed updated to:  %i",serialSpeed);
	}

	/* Open VSC Interface */
	vscInterface = vsc_initialize(serialPort.c_str(),serialSpeed);
	if (vscInterface == NULL) {
		ROS_FATAL("Cannot open serial port! (%s, %i)",serialPort.c_str(),serialSpeed);
	} else {
		ROS_INFO("Connected to VSC on %s : %i",serialPort.c_str(),serialSpeed);
	}

	// Attempt to Set priority
	bool  set_priority = false;
	if(nh.getParam("set_priority", set_priority)) {
		ROS_INFO("Set priority updated to:  %i",set_priority);
	}

	if(set_priority) {
		if(setpriority(PRIO_PROCESS, 0, -19) == -1) {
			ROS_ERROR("UNABLE TO SET PRIORITY OF PROCESS! (%i, %s)",errno,strerror(errno));
		}
	}

	// Create Message Handlers
	joystickHandler = new JoystickHandler();

	// EStop callback
	estopServ = rosNode.advertiseService("safety/service/send_emergency_stop", &VscProcess::EmergencyStop, this);

	// KeyValue callbacks
	keyValueServ = rosNode.advertiseService("safety/service/key_value", &VscProcess::KeyValue, this);
	keyStringServ = rosNode.advertiseService("safety/service/key_string", &VscProcess::KeyString, this);

	// Publish Emergency Stop Status
	estopPub = rosNode.advertise<std_msgs::UInt32>("safety/emergency_stop", 10);

	// Main Loop Timer Callback
	mainLoopTimer = rosNode.createTimer(ros::Duration(1.0/VSC_INTERFACE_RATE), &VscProcess::processOneLoop, this);

	// Init last time to now
	lastDataRx = ros::Time::now();

	// Clear all error counters
	memset(&errorCounts, 0, sizeof(errorCounts));
}

VscProcess::~VscProcess()
{
    // Destroy vscInterface
	vsc_cleanup(vscInterface);

	if(joystickHandler) delete joystickHandler;
}

bool VscProcess::EmergencyStop(EmergencyStop::Request  &req, EmergencyStop::Response &res )
{
	myEStopState = (uint32_t)req.EmergencyStop;

	ROS_WARN("VscProcess::EmergencyStop: to 0x%x", myEStopState);

	return true;
}

bool VscProcess::KeyValue(KeyValue::Request  &req, KeyValue::Response &res )
{
	// Send heartbeat message to vehicle in every state
	vsc_send_user_feedback(vscInterface, req.Key, req.Value);

	ROS_INFO("VscProcess::KeyValue: 0x%x, 0x%x", req.Key, req.Value);

	return true;
}

bool VscProcess::KeyString(KeyString::Request  &req, KeyString::Response &res )
{
	// Send heartbeat message to vehicle in every state
	vsc_send_user_feedback_string(vscInterface, req.Key, req.Value.c_str());

	ROS_INFO("VscProcess::KeyValue: 0x%x, %s", req.Key, req.Value.c_str());

	return true;
}


void VscProcess::processOneLoop(const ros::TimerEvent&)
{
	// Send heartbeat message to vehicle in every state
	vsc_send_heartbeat(vscInterface, myEStopState);

	// Check for new data from vehicle in every state
	readFromVehicle();
}

int VscProcess::handleHeartbeatMsg(VscMsgType& recvMsg)
{
	int retVal = 0;

	if(recvMsg.msg.length == sizeof(HeartbeatMsgType)) {
		ROS_DEBUG("Received Heartbeat from VSC");

		HeartbeatMsgType *msgPtr = (HeartbeatMsgType*)recvMsg.msg.data;

		// Publish E-STOP Values
		std_msgs::UInt32 estopValue;
		estopValue.data = msgPtr->EStopStatus;
		estopPub.publish(estopValue);

		if(msgPtr->EStopStatus > 0) {
			ROS_WARN("Received ESTOP from the vehicle!!! 0x%x",msgPtr->EStopStatus);
		}

	} else {
		ROS_WARN("RECEIVED HEARTBEAT WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
				(unsigned int)sizeof(HeartbeatMsgType), recvMsg.msg.length);
		retVal = 1;
	}

	return retVal;
}

void VscProcess::readFromVehicle()
{
	VscMsgType recvMsg;

	/* Read all messages */
	while (vsc_read_next_msg(vscInterface, &recvMsg) > 0) {
		/* Read next Vsc Message */
		switch (recvMsg.msg.msgType) {
		case MSG_VSC_HEARTBEAT:
			if(handleHeartbeatMsg(recvMsg) == 0) {
				lastDataRx = ros::Time::now();
			}

			break;
		case MSG_VSC_JOYSTICK:
			if(joystickHandler->handleNewMsg(recvMsg) == 0) {
				lastDataRx = ros::Time::now();
			}

			break;

		case MSG_VSC_NMEA_STRING:
//			handleGpsMsg(&recvMsg);

			break;
		case MSG_USER_FEEDBACK:
//			handleFeedbackMsg(&recvMsg);

			break;
		default:
			errorCounts.invalidRxMsgCount++;
			ROS_ERROR("Receive Error.  Invalid MsgType (0x%02X)",recvMsg.msg.msgType);
			break;
		}
	}

	// Log warning when no data is received
	ros::Duration noDataDuration = ros::Time::now() - lastDataRx;
	if(noDataDuration > ros::Duration(.25)) {
		ROS_WARN_THROTTLE(.5, "No Data Received in %i.%09i seconds", noDataDuration.sec, noDataDuration.nsec );
	}

}

