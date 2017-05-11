#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Int8MultiArray.h"
//#include "std_msgs/MultiArrayDimension.h"
#include <sstream>


#define BIN 114

int8_t Array[BIN];

void chatterCallback(const std_msgs::Int8MultiArray::ConstPtr& array)
{
	int i;
	// print all the remaining numbers
	for(i=0; i<BIN; i++)
	{
		Array[i] = array->data[i];
		printf("%d, ",Array[i]);
	}
	ROS_INFO(" END/n");
	return;
}

int main(int argc, char **argv){
	//INIT ROS LISTENER
	ros::init(argc, argv, "serial_listener");
	//INIT ROS HANDLER
	ros::NodeHandle n;
	//INIT ROS HANDLER
	ros::Subscriber chatter_sub = n.subscribe("electronics", 1, chatterCallback);
	//SPIN
	ros::spin();
	return 0;
}