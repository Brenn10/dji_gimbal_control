#include "dji_gimbal_control/dji_gimbal_control.h"

#include <unistd.h>

#include <iostream>

dji_gimbal_control::dji_gimbal_control(ros::NodeHandle& nh)
{
	// Configure the gimbal control parameters
	initializeParam();

	// Setup Subscribers
	gimbalAngleSub = nh.subscribe<geometry_msgs::Vector3Stamped>("/dji_sdk/gimbal_angle", 10, &dji_gimbal_control::gimbalAngleCallback, this);
	joySub = nh.subscribe("/joy", 10, &dji_gimbal_control::joyCallback, this);
	cameraInfoSub = nh.subscribe(cameraInfoTopic, 10, &dji_gimbal_control::cameraInfoCallback, this);
	tagPoseSub = nh.subscribe("/ar_pose_marker", 10, &dji_gimbal_control::tagCallback, this);

	// Setup Publishers
	gimbalSpeedPub = nh.advertise<geometry_msgs::Vector3Stamped>("/dji_sdk/gimbal_speed_cmd", 10);
	gimbalAnglePub = nh.advertise<dji_sdk::Gimbal>("/dji_sdk/gimbal_angle_cmd", 10);
}

void dji_gimbal_control::initializeParam()
{
	// Create private nodeHandle to read the launch file
	ros::NodeHandle nh_private("~");

	// Load values from launch file
	nh_private.param("track_tag", trackTag, false);
	nh_private.param("camera_info_topic", cameraInfoTopic, std::string("/dji_camera/camera_info"));
	nh_private.param("yaw_axis", yawAxis, 0);
	nh_private.param("pitch_axis", pitchAxis, 4);
	nh_private.param("roll_axis", rollAxis, 3);
	nh_private.param("reset_angle_btn", resetButton, 0);
	nh_private.param("face_down_btn", faceDownButton, 2);
	nh_private.param("toggle_track_btn", toggleButton, 3);

	// Initialize speed command
	speedCmd.vector.x = 0;
	speedCmd.vector.y = 0;
	speedCmd.vector.z = 0;

	// Initialize flags
	tagFound = false;
	velT = 0.75;
	invKp = 500.0;
}

void dji_gimbal_control::publishGimbalCmd()
{
	if (trackTag && tagFound)
	{
		speedCmd.vector.x = 0;
		speedCmd.vector.y = -tagY / invKp > velT ? velT : -tagY / invKp < -velT ? -velT : -tagY / invKp;
		speedCmd.vector.z = tagX / invKp > velT ? velT : tagX / invKp < -velT ? -velT : tagX / invKp;
	}
	gimbalSpeedPub.publish(speedCmd);
}

void dji_gimbal_control::resetGimbalAngle()
{
	// Prepare the reset command
	dji_sdk::Gimbal angleCmd;
	angleCmd.mode |= 0;
	angleCmd.mode |= 1; // for absolute angle
	angleCmd.ts    = 2;
	angleCmd.roll  = 0;
	angleCmd.pitch = 0;
	angleCmd.yaw   = 0;

	gimbalAnglePub.publish(angleCmd);

	sleep(2);
}

void dji_gimbal_control::faceDownwards()
{
	// Prepare the angle command
	dji_sdk::Gimbal angleCmd;
	angleCmd.mode |= 0;
	angleCmd.mode |= 1; // for absolute angle
	angleCmd.ts    = 2;
	angleCmd.roll  = 0;
	angleCmd.pitch = DEG2RAD(-90);
	angleCmd.yaw   = 0;

	gimbalAnglePub.publish(angleCmd);

	sleep(2);
}

// Callbacks
void dji_gimbal_control::gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
	gimbalAngle = *msg;
}

void dji_gimbal_control::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	// Toggle track flag
	if (msg->buttons[toggleButton] == 1)
		trackTag = !trackTag;

	if (msg->buttons[resetButton] == 1)
		resetGimbalAngle();
	else if (msg->buttons[faceDownButton] == 1)
		faceDownwards();
	else
	{
		// Update speed command
		speedCmd.vector.x = -msg->axes[rollAxis];
		speedCmd.vector.y = -msg->axes[pitchAxis];
		speedCmd.vector.z = -msg->axes[yawAxis];
	}
}

void dji_gimbal_control::cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
{
	// Get the focal length of the camera
	fx = msg.K[0];
	fy = msg.K[4];
}

void dji_gimbal_control::tagCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
	if(!msg.markers.empty())
	{
		// Get Tag center position
		double posTagX = msg.markers[0].pose.pose.position.x;
		double posTagY = msg.markers[0].pose.pose.position.y;
		double posTagZ = msg.markers[0].pose.pose.position.z;

		// Get tag pose in image coordinates
		tagX = fx*(posTagX/posTagZ);
		tagY = fy*(posTagY/posTagZ);

		tagFound = true;
	}
	else
		tagFound = false;
}

////////////////////////////////////////////////////////////
////////////////////////  Main  ////////////////////////////
////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dji_gimbal_control_node");
	ros::NodeHandle nh;

	dji_gimbal_control gimbalControl(nh);

	ros::Rate rate(10);

	while(ros::ok())
	{
		ros::spinOnce();
		gimbalControl.publishGimbalCmd();

		rate.sleep();
	}

	return 0;
}