#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>
#include "teensy_serial.h"
#include <inttypes.h>
#include <time.h>

#define SPEED 0
#define BIN 6
#define BOUT 1
#define PORT "/dev/Teensy"
#define RATE 500 //Hz

cserial_board* myserial;
using namespace std;
int end_program=0;

void chatterCallback(const std_msgs::String::ConstPtr& array_in){
    //ROS_INFO("%s",array_in->data.c_str());
	//cout<<(int) array_in->data[0]<<endl;
	//cout<<(int) array_in->data[0]<<endl;
	end_program=1;
}

int main(int argc, char **argv){
	//INIT ROS
	ros::init(argc, argv, "teensy_publisher");
	//INIT ROS HANDLER
	ros::NodeHandle n;
	//INIT PUBLISHING FUNCTION
	std_msgs::Int8MultiArray* array=new std_msgs::Int8MultiArray[BIN+1];
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int8MultiArray>("robot_limits", 2);
	//INIT ROS HANDLER
    //ros::NodeHandle dynamixel_in;
	//INIT LISTENING FUNCTION
    ros::Subscriber controller_sub = n.subscribe("teensy_stop", 1, chatterCallback);
	//rate
	ros::Rate loop_rate(RATE);
	//INIT SERIAL PORT
	char* port=new char[20];
	strcpy(port,PORT);
	myserial=new cserial_board(port, SPEED, BIN, BOUT);
    ROS_INFO("ready to open portttt!");
    cout<< "ready to open port" <<endl;
	//OPEN SERIAL PORT
	int openport=myserial->cserial_open();
	ROS_INFO("open port");
	if (openport>0){
	//START LOOP
		myserial->ch_out[0]='!';
		ROS_INFO("port open ");
		//get first values!
		if(myserial->cserial_write()){
				//serial read attempt
				if (myserial->cserial_read()){
					//ROS_INFO("I must publish something!")
					uint8_t i=0;
					while(i<BIN)	array->data.push_back((int8_t) (myserial->ch_in)[i++]);
					chatter_pub.publish(*array);
					//Let the world know
					ROS_INFO("I published something!");					
				}
				else{
					ROS_INFO("Publish problem");
				}
		}
		//Do this
		ros::spinOnce();
		//Added a delay so not to spam
		loop_rate.sleep();
		//Continious checking
		while (ros::ok() && !end_program)
		{
			if(myserial->cserial_write()){
				//serial read attempt
				if (myserial->cserial_read()){
                    ROS_INFO("read!");
					//ROS_INFO("I must publish something!");
					if((myserial->ch_in[0]!=array->data[0]) || (myserial->ch_in[1]!=array->data[1]) || (myserial->ch_in[2]!=array->data[2]) 
						|| (myserial->ch_in[3]!=array->data[3]) || (myserial->ch_in[4]!=array->data[4]) || (myserial->ch_in[5]!=array->data[5])){
					uint8_t i=0;
					array->data.clear();
					while(i<BIN)	array->data.push_back((int8_t) (myserial->ch_in)[i++]);
					chatter_pub.publish(*array);
					//Let the world know
					ROS_INFO("I published something!");
                    ROS_INFO("%c %c %c %c %c %c",array->data[0],array->data[1],array->data[2],array->data[3],array->data[4],array->data[5])	;
					//Clear array
				    	//array->data.clear();
					}
				}
				else{
					ROS_INFO("Publish problem");
				}
			}
			//Do this.
			ros::spinOnce();
			//Added a delay so not to spam
			loop_rate.sleep();
			//usleep(1000);
		}
		myserial->cserial_close();
		return 0;
	}
	else{
		ROS_INFO("Problem opening port");
		return(-1);
	}
}
