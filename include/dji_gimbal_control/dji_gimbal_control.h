#ifndef DJI_GIMBAL_CONTROL_H
#define DJI_GIMBAL_CONTROL_H

// DJI SDK includes
#include <dji_sdk/Gimbal.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Joy.h>

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

class dji_gimbal_control {
public:
	dji_gimbal_control(ros::NodeHandle& nh);
	~dji_gimbal_control(){};

	// Publish commands
	void publishGimbalCmd();

private:
	// Subscribers
	ros::Subscriber gimbalAngleSub;
  ros::Subscriber joySub;

	// Publishers
	ros::Publisher gimbalSpeedPub;
  ros::Publisher gimbalAnglePub;

	// Callbacks
	void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  // Functions
  void initializeJoy();
  void resetGimbalAngle();
  void faceDownwards();
  
	// Data
  int yawAxis, pitchAxis, rollAxis, resetButton, faceDownButton;
	geometry_msgs::Vector3Stamped gimbalAngle;
  geometry_msgs::Vector3Stamped speedCmd;
};

#endif //DJI_GIMBAL_CONTROL_H