#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>
#include "serial_board.h"
#include <inttypes.h>
#include <time.h>

#define SPEED 0
#define BIN 108
#define BOUT 4
#define PORT "/dev/Koveos"
#define RATE 500 //Hz
uint32_t counter;
cserial_board* myserial;

using namespace std;

void chatterCallback(const std_msgs::String::ConstPtr& array_in){
	cout<< "Should write "<<endl;
	for(uint8_t i=0;i<array_in->data[1];i++){
	myserial->ch_out[i]=array_in->data[i];
	cout << (int) myserial->ch_out[i] << " "<<endl;
	//myserial->cserial_write();
	}
	myserial->ch_out[2]=0x06; //to avoid rewritting
}

int main(int argc, char **argv){
	counter=0;
	//INIT ROS
	ros::init(argc, argv, "serial_publisher");
	//INIT ROS HANDLER
	ros::NodeHandle n;
	//INIT PUBLISHING FUNCTION
	std_msgs::Int8MultiArray* array=new std_msgs::Int8MultiArray[BIN+1];
	ros::Publisher chatter_pub = n.advertise<std_msgs::Int8MultiArray>("electronics", 1);
	//INIT LISTENING FUNCTION
	ros::NodeHandle m;
	ros::Subscriber listen_arr = m.subscribe("board_commands",1,chatterCallback);
	//DEFINE PROGRAM RATE
	ros::Rate loop_rate(RATE);	
	//INIT SERIAL PORT
	char* port=new char[20];
	strcpy(port,PORT);
	myserial=new cserial_board(port, SPEED, BIN, BOUT);
	//prepare write
	//write open main switch
	myserial->ch_out[0]=0x21;
	myserial->ch_out[1]=0x04;
	myserial->ch_out[2]=0x00;
	myserial->ch_out[3]=(unsigned char) true;
	loop_rate.sleep();
	//OPEN SERIAL PORT
	if (myserial->cserial_open()){
		if(myserial->cserial_write() && myserial->cserial_read() && myserial->ch_in[BIN-2]==33 && myserial->ch_in[BIN-1]==33){
			ROS_INFO("Board connected");
			loop_rate.sleep();
			//START LOOP
			while (ros::ok() && counter++<100000){	
				//Do this.
				ros::spinOnce();
				//serial read attempt
				if (myserial->ch_out[2]!=0x06){
					myserial->ch_out[2]=0x06;
					myserial->ch_out[1]=0x03;
					myserial->cserial_write();
				}
				myserial->ch_out[2]=0x00;
				//ROS_INFO("I must publish something!");
				if(myserial->cserial_read()){
				//Clear array
				array->data.clear();
				uint8_t i=0;
				while(i<BIN){
					//cout<< (int) myserial->ch_in[i] << " ";
					array->data.push_back((int8_t) (myserial->ch_in)[i++]);
				}
				chatter_pub.publish(*array);
				//Let the world know
				ROS_INFO("I published something!");
				}
				ROS_INFO("%d",counter);
				//Added a delay so not to spam
				loop_rate.sleep();
			}
			//write close main switch
			myserial->ch_out[0]=0x21;
			myserial->ch_out[1]=0x04;
			myserial->ch_out[2]=0x00;
			myserial->ch_out[3]=(unsigned char) false; //NEVER REMOVE!
			myserial->cserial_write();
			myserial->cserial_read();
			myserial->cserial_close();
			return 0;
		}
		else{
			ROS_INFO("Problem first write port");
					myserial->cserial_close();
		}
	}
	else{
		ROS_INFO("Problem opening port");
		return(-1);
	}
}
