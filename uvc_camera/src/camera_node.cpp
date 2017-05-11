#include <ros/ros.h>
#include <nodelet/loader.h>

#include "uvc_camera/camera.h"
#include "std_msgs/Int8MultiArray.h"
uint8_t terminate_camera=0;

void stop_callback(const std_msgs::Int8MultiArray::ConstPtr& array)
{
	terminate_camera=1;	
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "uvc_camera");

  uvc_camera::Camera camera(ros::NodeHandle(), ros::NodeHandle("~"));
  ros::NodeHandle stop_base_cam_node;
  ros::Subscriber stop_base_cam_sub = stop_base_cam_node.subscribe("base/base_camera_stop", 1, stop_callback);
  while(!terminate_camera && ros::ok())
  ros::spinOnce();
  return 0;
}

