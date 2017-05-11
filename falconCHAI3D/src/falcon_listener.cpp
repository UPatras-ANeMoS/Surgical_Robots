#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <sstream>



float Array[3];
using namespace std;

void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& array){
	int i;
	// print all the remaining numbers
	for(i=0; i<3; i++){
		Array[i] = array->data[i];
	}
	cout<< "x "<< array->data[0] << " y "<< array->data[1] << " z "<< array->data[2] <<endl;
	//ROS_INFO(" END/n");
	return;
}

int main(int argc, char **argv){
	//INIT ROS LISTENER
	ros::init(argc, argv, "falcon_listener");
	//INIT ROS HANDLER
	ros::NodeHandle f;
	//INIT ROS HANDLER
	ros::Subscriber falcon_sub = f.subscribe("falcon", 1, chatterCallback);
	//SPIN
	ros::spin();
	return 0;
}
