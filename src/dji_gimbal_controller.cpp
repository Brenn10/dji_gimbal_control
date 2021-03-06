#include "dji_gimbal_control/dji_gimbal_controller.h"

#include <unistd.h>

#include <iostream>

dji_gimbal_controller::dji_gimbal_controller(ros::NodeHandle& nh)
{
	// Configure the gimbal control parameters
	initializeParam();

	// Setup Subscribers
	gimbalAngleSub = nh.subscribe<geometry_msgs::Vector3Stamped>("/dji_sdk/gimbal_angle", 10, &dji_gimbal_controller::gimbalAngleCallback, this);
	joySub = nh.subscribe("joy", 10, &dji_gimbal_controller::joyCallback, this);
	cameraInfoSub = nh.subscribe(cameraInfoTopic, 10, &dji_gimbal_controller::cameraInfoCallback, this);
	tagPoseSub = nh.subscribe(tagPoseTopic, 10, &dji_gimbal_controller::tagCallback, this);

	// Setup Publishers
	gimbalSpeedPub = nh.advertise<geometry_msgs::Vector3Stamped>("/dji_sdk/gimbal_speed_cmd", 10);
	gimbalAnglePub = nh.advertise<dji_sdk::Gimbal>("/dji_sdk/gimbal_angle_cmd", 10);
	
	// Setup Services
	facedownServ = nh.advertiseService("facedown", &dji_gimbal_controller::facedownCallback, this);
	faceupServ = nh.advertiseService("faceup", &dji_gimbal_controller::faceupCallback, this);
	setTrackingServ = nh.advertiseService("setGimbalTracking", &dji_gimbal_controller::setTrackingCallback, this);
}

void dji_gimbal_controller::initializeParam()
{
	// Create private nodeHandle to read the launch file
	ros::NodeHandle nh_private("~");

	// Load values from launch file
	nh_private.param("track_tag", trackTag, false);
	nh_private.param("camera_info_topic", cameraInfoTopic, std::string("dji_camera/camera_info"));
	nh_private.param("tag_pose_topic", tagPoseTopic, std::string("ar_pose_marker"));
	nh_private.param("yaw_axis", yawAxis, 0);
	nh_private.param("pitch_axis", pitchAxis, 4);
	nh_private.param("roll_axis", rollAxis, 3);
	nh_private.param("reset_angle_btn", resetButton, 0);
	nh_private.param("face_down_btn", faceDownButton, 2);
	nh_private.param("toggle_track_btn", toggleButton, 3);
	nh_private.param("Kp", Kp, .002);
	nh_private.param("Kd", Kd, 0.0);

	// Initialize speed command
	speedCmd.vector.x = 0;
	speedCmd.vector.y = 0;
	speedCmd.vector.z = 0;

	// Initialize flags
	tagFound = false;
	velT = 0.75;
	lastX=lastY=0;
}

void dji_gimbal_controller::publishGimbalCmd()
{
	if (trackTag && tagFound)
	{
		speedCmd.vector.x = 0;
		//corrections in x,y
		double cx,cy;
		cx = tagX * Kp - (lastX-tagX)*Kd;
		cy = -tagY * Kp+ (lastY-tagY)*Kd;
		//crop if outside range
		cx = cx>velT ? velT : (cx<-velT ? -velT : cx);
		cy = cy>velT ? velT : (cy<-velT ? -velT : cy);
		//add to vector
		speedCmd.vector.y = cy;
		speedCmd.vector.z = cx;

		gimbalSpeedPub.publish(speedCmd);

		// Reset Commands to zero
		speedCmd.vector.y = 0;
		speedCmd.vector.z = 0;
	}
	else
		gimbalSpeedPub.publish(speedCmd);
}

void dji_gimbal_controller::resetGimbalAngle()
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

void dji_gimbal_controller::faceDownwards()
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
}

// Callbacks
void dji_gimbal_controller::gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
	gimbalAngle = *msg;
}

void dji_gimbal_controller::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
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

void dji_gimbal_controller::cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
{
	// Get the focal length of the camera
	fx = msg.K[0];
	fy = msg.K[4];
}

void dji_gimbal_controller::tagCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
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

bool dji_gimbal_controller::facedownCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	faceDownwards();
	res.success = true;
	return true;
}

bool dji_gimbal_controller::faceupCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	resetGimbalAngle();
	res.success = true;
	return true;
}

bool dji_gimbal_controller::setTrackingCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	trackTag = req.data;
	res.success = true;
	return true;
}

////////////////////////////////////////////////////////////
////////////////////////  Main  ////////////////////////////
////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dji_gimbal_control_node");
	ros::NodeHandle nh;

	dji_gimbal_controller gimbalControl(nh);

	ros::Rate rate(30);

	while(ros::ok())
	{
		ros::spinOnce();
		gimbalControl.publishGimbalCmd();

		rate.sleep();
	}

	return 0;
}
