#include <ros/ros.h>
#include <nodelet/loader.h>

#include "uvc_camera/camera.h"
#include "std_msgs/Int8MultiArray.h"
int8_t start_camera=0;

void start_callback(const std_msgs::Int8MultiArray::ConstPtr& array)
{
	ROS_INFO("data");
	start_camera=array->data[0];
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "robot_camera_node");
  ros::NodeHandle start_robot_cam_node;
  ros::Subscriber start_robot_cam_sub = start_robot_cam_node.subscribe("robot/robot_camera_start", 1, start_callback);
  //while(start_camera==0 && ros::ok())
  //ros::spinOnce();
  uvc_camera::Camera camera(ros::NodeHandle(), ros::NodeHandle("~"));
  while(start_camera!=1 && ros::ok())
  ros::spinOnce();
  return 0;
}

