#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>
#include "serial_board.h"
#include <inttypes.h>
#include <time.h>

#define SPEED 0
#define BIN 88
#define BOUT 100
#define PORT "/dev/Koveos"
#define RATE 100 //Hz
cserial_board* myserial;

//TEENSY SERIAL
#define SPEED_TEENSY 0
#define BIN_TEENSY 16
#define BOUT_TEENSY 2
#define PORT_TEENSY "/dev/TeensyPOT"
cserial_board* teensy;

uint8_t read_only[3]={0x21,0x03,0x06};
bool board_status=false;

using namespace std;

struct smaStatus{
  float AD[8]; //32
  float ADrms[8]; //64
  //u_int16_t PWMval[8]; //48
  float Vbus; //68
  uint32_t time; //72
  //int16_t ADCV[16]; //88+32=120
  //u_int16_t ADraw; //+2
  uint16_t PWMval[8]; //88
  uint16_t ADCV[8]; //88
} smaState; //122byte

void chatterCallback(const std_msgs::UInt8MultiArray::ConstPtr& array_in){
  cout<< "Should write %d "<< array_in->data.size() << endl;
  for(uint8_t i=0;i<19;i++){
    myserial->ch_out[i]=array_in->data[i];
    //cout << (int) myserial->ch_out[i] << " ";
    //ROS_INFO("%d",*tmp);
  }
  myserial->ch_out[1]+=2;
  myserial->ch_out[19]=0;
  myserial->ch_out[20]=0;
  for(uint8_t i=0;i<21;i++){
    cout << (int) myserial->ch_out[i] << " ";
  }
  cout<<endl;
  myserial->cserial_write();
  ROS_INFO(" %d current of ch 8 ",*((uint16_t*) &myserial->ch_out[17]));
}

int main(int argc, char **argv){
  FILE *f;
  f=fopen("/home/odroid/Desktop/antagonistic_DOF68_1500_50_1000_1000_best","w");
  if(f==NULL){
    ROS_INFO("error opening file");
  }
  else{
    uint8_t counter=0;
    //INIT ROS
    ros::init(argc, argv, "serial_robot");
    //INIT ROS HANDLER
    ros::NodeHandle n;
    //INIT PUBLISHING FUNCTION
    std_msgs::UInt8MultiArray array;
    ros::Publisher chatter_pub = n.advertise<std_msgs::UInt8MultiArray>("board2ctrl", 1);
    //INIT LISTENING FUNCTION
    ros::Subscriber listen_arr = n.subscribe("ctrl2board",1,chatterCallback);
    //DEFINE PROGRAM RATE
    ros::Rate loop_rate(RATE);
    //INIT SERIAL PORT
    char* port=new char[20];
    strcpy(port,PORT);
    myserial=new cserial_board(port, SPEED, BIN, BOUT);
    strcpy(port,PORT_TEENSY);
    teensy=new cserial_board(port, SPEED_TEENSY, BIN_TEENSY, BOUT_TEENSY);
    //prepare write
    //write open main switch
    myserial->ch_out[0]=read_only[0];
    myserial->ch_out[1]=read_only[1];
    myserial->ch_out[2]=read_only[2];
    teensy->ch_out[0]=read_only[0];
    teensy->ch_out[1]=read_only[1];
    teensy->ch_out[2]=read_only[2];
    bool op1=myserial->cserial_open();
    bool op2=teensy->cserial_open();
    if(!op1 || !op2){
      if(!op1) ROS_INFO("ERROR open SMA board");
      if(!op2) ROS_INFO("ERROR open Tensy board");
    }
    else{
      myserial->ch_out[0]=0x21;
      myserial->ch_out[1]=0x04;
      myserial->ch_out[2]=0x00;
      myserial->ch_out[3]=(unsigned char) true;
      myserial->cserial_write();
      sleep(2);
      ROS_INFO("open main switch");
      myserial->ch_out[0]=read_only[0];
      myserial->ch_out[1]=read_only[1];
      myserial->ch_out[2]=read_only[2];
      bool error=myserial->cserial_write();
      bool error2=myserial->cserial_read();
      bool error3=teensy->cserial_write();
      bool error4=teensy->cserial_read();
      if(error && error2 && error3 && error4){
	ROS_INFO("Boards ready");
	board_status=true;
	//START LOOP
	ros::Time t_init=ros::Time::now();
	ros::Time t_current=t_init;
	uint8_t* ptr=(uint8_t*) &smaState;
	while (board_status && ros::ok()){
	  memcpy(&smaState,myserial->ch_in,88);
	  memcpy(smaState.ADCV,teensy->ch_in,16);
	  fprintf(f,"%f\t",t_current.toSec()-t_init.toSec());
	  for(uint8_t i=0;i<8;i++) fprintf(f,"%f\t",smaState.AD[i]); 
	  for(uint8_t i=0;i<8;i++) fprintf(f,"%f\t",smaState.ADrms[i]); 
	  fprintf(f,"%f\t",smaState.Vbus); 
	  fprintf(f,"%u\t",smaState.time); 
	  for(uint8_t i=0;i<8;i++) fprintf(f,"%d\t",smaState.PWMval[i]); 
	  for(uint8_t i=0;i<8;i++) fprintf(f,"%d\t",smaState.ADCV[i]); 
	  fprintf(f,"\n");
	  array.data.clear();
	  myserial->ch_out[0]=read_only[0];
	  myserial->ch_out[1]=read_only[1];
	  myserial->ch_out[2]=read_only[2];
	  //check if there is data from controller to write -- if not insert read command
	  bool error7=teensy->cserial_write();        
	  bool error5=myserial->cserial_write();
	  bool error6=myserial->cserial_read();
	  
	  bool error8=teensy->cserial_read();
	  if(error5 && error6 && error7 && error8){
	    //Clear array
	    //array.data.clear();
	    uint8_t i=0;
	    while(i<BIN){
	      //cout<< (int) myserial->ch_in[i] << " ";
	      array.data.push_back(myserial->ch_in[i++]);
	    }
	    i=0;
	    while(i<BIN_TEENSY){
	      //cout<< (int) teensy->ch_in[i] << " ";
	      array.data.push_back(teensy->ch_in[i++]);
	    }
	    uint16_t t_val[8];
	    memcpy(t_val,teensy->ch_in,16);
	    //for(i=0;i<8;i++) std::cout<< t_val[i]<< " "<< std::endl;
	    //std::cout<<std::endl;
	    // cout<< "endl" <<endl;
	    // 		      array.data.push_back(counter);
	    // 		      counter++;
	    chatter_pub.publish(array);
	    //ROS_INFO("I published something!");
	  }
	  else{
	    ROS_INFO("error read/write in main loop");
	    //board_status=false;
	  }
	  //myserial->ch_out[0]=0; //use to check if controller command arrived
	  ros::spinOnce();
	  loop_rate.sleep();
	  t_current=ros::Time::now();
	}
	ROS_INFO("close main switch");
	//write close main switch just in case something fails in controller and doesn't write
	myserial->ch_out[0]=0x21;
	myserial->ch_out[1]=0x04;
	myserial->ch_out[2]=0x00;
	myserial->ch_out[3]=(unsigned char) false; //NEVER REMOVE!
	myserial->cserial_write();
	sleep(0.5);
	myserial->ch_out[0]=0x21;
	myserial->ch_out[1]=21;
	myserial->ch_out[2]=0x01;
	for(uint8_t i=3;i<18;i++)
	  myserial->ch_out[i]=0x00;
	myserial->ch_out[19]=0x00;
	myserial->ch_out[20]=0x00;
	myserial->cserial_write();
	sleep(0.5);
      }
      else{
	ROS_INFO("close main switch");
	ROS_INFO("%d %d %d %d",error,error2,error3,error4);
	//write close main switch just in case something fails in controller and doesn't write
	myserial->ch_out[0]=0x21;
	myserial->ch_out[1]=21;
	myserial->ch_out[2]=0x01;
	for(uint8_t i=3;i<18;i++)
	  myserial->ch_out[i]=0x00;
	myserial->ch_out[19]=0x00;
	myserial->ch_out[20]=0x00;
	myserial->cserial_write();
	sleep(0.5);
	myserial->ch_out[0]=0x21;
	myserial->ch_out[1]=0x04;
	myserial->ch_out[2]=0x00;
	myserial->ch_out[3]=(unsigned char) false;
	myserial->cserial_write();
	sleep(0.5);
	myserial->cserial_close();
	teensy->cserial_close();
      }
    }
    
    myserial->cserial_close();
    teensy->cserial_close();
    fclose(f);
  }//close fopen
  return 0;
}
