/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/*Version 0.5 by Nick Evangeliou*/
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <time.h>

//#include "dynamixel_sdk/dxl_defs.h"
#include "dynamixel_sdk/dxl_functions.h"

//ROS STUFF
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>

//TEENSY SERIAL
#include "dynamixel_sdk/teensy_serial.h"

uint8_t num_actuators=NUM_ACTUATORS1+NUM_ACTUATORS2;
dynamixel_motor* dxl_motor=new dynamixel_motor[num_actuators];
dynamixel::PortHandler	*portHandler1,*portHandler2;
dynamixel::PacketHandler	*packetHandler1,*packetHandler2;
bool dxl_addparam_result1,dxl_addparam_result2,dxl_getdata_result1,dxl_getdata_result2;
uint8_t dxl_global_error;

//TEENSY SERIAL
#define SPEED 0
#define BIN 6
#define BOUT 2
#define PORT "/dev/TeensyDXL"
cserial_board* teensy;

#define RATE 25 //Hz - 40ms 
double s_time=1.0/((double) RATE); //seconds
double speedf12=SPEED_FACTOR12;
double speedf34=SPEED_FACTOR34;
double speedf5=SPEED_FACTOR5;

uint32_t lims[3]={0,0,0};

uint8_t rewrite_attempts=3; //num of attempts to rewrite before stopping program - not used for now
uint8_t executed_command=0;
uint8_t end_program=0;
uint8_t disable_torque=DISABLE_TORQUE;

bool use_sync_write=false;

struct timespec deadline;

bool init_process(){
    portHandler1=dynamixel::PortHandler::getPortHandler(DEVICENAME1);
    portHandler2=dynamixel::PortHandler::getPortHandler(DEVICENAME2);
    packetHandler1=dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    packetHandler2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dxl_motor[0].portHandler=dxl_motor[1].portHandler=portHandler1;
    dxl_motor[2].portHandler=dxl_motor[3].portHandler=dxl_motor[4].portHandler=portHandler2;
    dxl_motor[0].packetHandler=dxl_motor[1].packetHandler=packetHandler1;
    dxl_motor[2].packetHandler=dxl_motor[3].packetHandler=dxl_motor[4].packetHandler=packetHandler2;
    //START TEENSY----------------------------------------------------------
    //INIT SERIAL PORT
    char* port=new char[20];
    strcpy(port,PORT);
    teensy=new cserial_board(port, SPEED, BIN, BOUT);
    teensy->ch_out[0]='!';
    teensy->ch_out[1]='!';
    if(!teensy->cserial_open()){ROS_INFO("Failed to find Teensy");return false;}
    //START USB2RS485----------------------------------------------------------
    // Open port1
    if (!portHandler1->openPort()){ROS_INFO("Failed to open port 1!");return false;}
    // Open port2
    if (!portHandler2->openPort()){ROS_INFO("Failed to open port 2!");return false;}
    // Set port1 baudrate
    if (!portHandler1->setBaudRate(BAUDRATE)){ROS_INFO("Failed to change the baudrate of port 1!");return false;}
    // Set port2 baudrate
    if (!portHandler2->setBaudRate(BAUDRATE)){ROS_INFO("Failed to change the baudrate of port 2!");return false;}
    //SET STARTUP SETTINGS OF DYNAMIXEL----------------------------------------
    //PING FIRST
    for (uint8_t i=0;i<num_actuators;i++){
      uint16_t dxl_model_num; 
      uint8_t dxl_error=0;
      uint8_t result=dxl_motor[i].packetHandler->ping(dxl_motor[i].portHandler,dxl_motor[i].ID, &dxl_model_num, &dxl_error);
      while(result!=0){
	result=dxl_motor[i].packetHandler->ping(dxl_motor[i].portHandler,dxl_motor[i].ID, &dxl_model_num, &dxl_error);
	dxl_motor[i].packetHandler->printTxRxResult(result);
	sleep(0.05);
	ROS_INFO("Motor %d [ID:%.3d] ping error",i, dxl_motor[i].ID);
      }
      ROS_INFO("[ID:%.3d] Model No : %.5d \n pinged",dxl_motor[i].ID, dxl_model_num);
    }
    //TORQUE ON FOR ALL
    uint8_t TORQUE_ON[NUM_ACTUATORS2],num_motor[NUM_ACTUATORS2],num_motor2[NUM_ACTUATORS2];
    if(use_sync_write){
      for (uint8_t i=0;i<NUM_ACTUATORS2;i++){
	TORQUE_ON[i]=1; num_motor[i]=0+i; 
	num_motor2[i]=NUM_ACTUATORS1+i;
      }
      torqueONOFF_sync(TORQUE_ON, num_motor, NUM_ACTUATORS1, dxl_motor);
      torqueONOFF_sync(TORQUE_ON,num_motor2, NUM_ACTUATORS2, dxl_motor);
    }
    else{
      for(uint8_t i=0; i<num_actuators;i++){
	uint8_t dxl_error=-1;
	while(dxl_error!=0){
	  torqueONOFF_single(i,1,dxl_motor,&dxl_error);
	  ros::Duration(s_time).sleep();
	  ros::spinOnce();
	}
      }
    }
    ROS_INFO("position control enable");
    //SYNC WRITE POSITION CONTROL FOR ALL
    if(use_sync_write){
      set_position_control_sync(num_motor, NUM_ACTUATORS1, dxl_motor);
      set_position_control_sync(num_motor2, NUM_ACTUATORS2, dxl_motor);
    }
    else{
      for(uint8_t i=0; i<num_actuators;i++){
	ROS_INFO("%d",i);
	set_position_control_single(i,dxl_motor);
	ros::Duration(s_time).sleep();
	ros::spinOnce();
      }
    }
    //SET SPEED LIMITS FOR DXL 2,3 
    uint8_t err=0;
    write_uint16t(dxl_motor[2].packetHandler,dxl_motor[2].portHandler, dxl_motor[2].ID, P_SPEED_L ,
		  10, &err);
    write_uint16t(dxl_motor[3].packetHandler,dxl_motor[3].portHandler, dxl_motor[3].ID, P_SPEED_L ,
		  10, &err);
    //ALL TORUQES TO ?
    /*
    uint16_t torque=1023;
    uint8_t err=0;
    write_uint16t(packetHandler1, portHandler1,i,P_TORQUE_L,torque,&err);
    */
    //SYNC WRITE PID FOR ALL
    //P
    while(!write_uint16t(packetHandler2, portHandler2,2,P_TORQUE_L,512,&err));
    while(!write_uint16t(packetHandler2, portHandler2,1,P_TORQUE_L,128,&err));
    uint16_t values[NUM_ACTUATORS2];
    if(use_sync_write){
    for(uint8_t i=0;i<NUM_ACTUATORS1;i++) values[i]=dxl_motor[i].P;
    while(!SYNC_WRITE(num_motor, NUM_ACTUATORS1, P_P, 1, values,dxl_motor)){}
    for(uint8_t i=0;i<NUM_ACTUATORS2;i++) values[i]=dxl_motor[i+NUM_ACTUATORS1].P;
    while(!SYNC_WRITE(num_motor2, NUM_ACTUATORS2, P_P, 1, values,dxl_motor)){}
    sleep(0.5);
    }
    else{
      for(uint8_t i=0; i<num_actuators;i++){
	uint8_t dxl_error=-1;
	while(!write_uint8t(dxl_motor[i].packetHandler,dxl_motor[i].portHandler,dxl_motor[i].ID,P_P,dxl_motor[i].P,&dxl_error)){
	  ros::Duration(s_time).sleep();
	  ros::spinOnce();
	}
      }
    }
    //I
    if(use_sync_write){
      for(uint8_t i=0;i<NUM_ACTUATORS1;i++) values[i]=dxl_motor[i].I;
      while(!SYNC_WRITE(num_motor, NUM_ACTUATORS1, P_I, 1, values,dxl_motor)){}
      for(uint8_t i=0;i<NUM_ACTUATORS2;i++) values[i]=dxl_motor[i+NUM_ACTUATORS1].I;
      while(!SYNC_WRITE(num_motor2, NUM_ACTUATORS2, P_I, 1, values,dxl_motor)){}
      sleep(0.5);
    }
    else{
      for(uint8_t i=0; i<num_actuators;i++){
	uint8_t dxl_error=-1;
	while(!write_uint8t(dxl_motor[i].packetHandler,dxl_motor[i].portHandler,dxl_motor[i].ID,P_I,dxl_motor[i].I,&dxl_error)){
	  ros::Duration(s_time).sleep();
	  ros::spinOnce();
	}
      }
    }
    //D
    if(use_sync_write){
      for(uint8_t i=0;i<NUM_ACTUATORS1;i++) values[i]=dxl_motor[i].D;
      while(!SYNC_WRITE(num_motor, NUM_ACTUATORS1, P_D, 1, values, dxl_motor)){}
      for(uint8_t i=0;i<NUM_ACTUATORS2;i++) values[i]=dxl_motor[i+NUM_ACTUATORS1].D;
      while(!SYNC_WRITE(num_motor2, NUM_ACTUATORS2, P_D, 1, values,dxl_motor)){}
      sleep(0.5);
    }
    else{
      for(uint8_t i=0; i<num_actuators;i++){
	uint8_t dxl_error=-1;
	while(!write_uint8t(dxl_motor[i].packetHandler,dxl_motor[i].portHandler,dxl_motor[i].ID,P_D,dxl_motor[i].D,&dxl_error)){
	  ros::Duration(s_time).sleep();
	  ros::spinOnce();
	}
      }      
    }
    //SYNC WRITE ACCELERATION FOR ALL
    if(use_sync_write){
      for(uint8_t i=0;i<NUM_ACTUATORS1;i++) values[i]=dxl_motor[i].acceleration;
      while(!SYNC_WRITE(num_motor, NUM_ACTUATORS1, P_ACCELERATION, 1, values,dxl_motor)){}
      for(uint8_t i=0;i<NUM_ACTUATORS2;i++) values[i]=dxl_motor[i+NUM_ACTUATORS1].acceleration;
      while(!SYNC_WRITE(num_motor2, NUM_ACTUATORS2, P_ACCELERATION, 1, values,dxl_motor)){}
      sleep(0.5);
    }
    else{
      for(uint8_t i=0; i<num_actuators;i++){
	uint8_t dxl_error=-1;
	while(!write_uint8t(dxl_motor[i].packetHandler,dxl_motor[i].portHandler,dxl_motor[i].ID,P_ACCELERATION,dxl_motor[i].acceleration,&dxl_error)){
	  ros::Duration(s_time).sleep();
	  ros::spinOnce();
	}
      }    
    }
    return true;
}

void dxl_pos_update(uint8_t* motor_index,int sum){//updates position of all dynamixels
    //read previous measured positions
    uint16_t pos_prev[sum];
    for(uint8_t i=0;i<sum;i++) pos_prev[i]=dxl_motor[motor_index[i]].measured_position;
    uint8_t num_motor1[NUM_ACTUATORS1];
    uint8_t sum_sync=0;
    for(uint8_t i=0;i<sum;i++){
      if(dxl_motor[motor_index[i]].portHandler==portHandler1){
	num_motor1[sum_sync++]=motor_index[i];
      }
    }
    if(sum_sync>1) if(!read_position_bulk(num_motor1,sum_sync,dxl_motor)){
      ros::Duration(s_time).sleep();
      ros::spinOnce();
      for(uint8_t i=0;i<sum_sync;i++){
	while(!read_position_single(num_motor1[i],dxl_motor)){
	  ros::Duration(s_time).sleep();
	  ros::spinOnce();
	}
      }
    }
    else if(sum_sync==1) while(!read_position_single(num_motor1[0],dxl_motor)){
      ros::Duration(s_time).sleep();
      ros::spinOnce();
    }
    
    uint8_t num_motor2[NUM_ACTUATORS2];
    sum_sync=0;
    for(uint8_t i=0;i<sum;i++){
      if(dxl_motor[motor_index[i]].portHandler==portHandler2){
	num_motor2[sum_sync++]=motor_index[i];
      }
    }
    if(sum_sync>1) if(!read_position_bulk(num_motor2,sum_sync,dxl_motor)){
      ros::Duration(s_time).sleep();
      ros::spinOnce();
      for(uint8_t i=0;i<sum_sync;i++){
	while(!read_position_single(num_motor2[i],dxl_motor)){    
	  ros::Duration(s_time).sleep();
	  ros::spinOnce();
	}
      }
    }
    else if(sum_sync==1) while(!read_position_single(num_motor2[0],dxl_motor)){
      ros::Duration(s_time).sleep();
      ros::spinOnce();
    }

    int pos_diff;
    for(uint8_t i=0;i<sum;i++){
      switch(i){
	case 0:
	case 1:
	  	case 4:
	  //CCW rotation must increase position
	    pos_diff=pos_prev[i]-dxl_motor[motor_index[i]].measured_position;
	    if(abs(pos_diff)>3000){
	      if(pos_diff<0) dxl_motor[motor_index[i]].current_position+=4096;
	      else if(pos_diff>0 && int(dxl_motor[motor_index[i]].current_position)-4096>=0) dxl_motor[motor_index[i]].current_position-=4096;
	      else dxl_motor[motor_index[i]].current_position=0;
	    }
	    if(((int (dxl_motor[motor_index[i]].current_position))+pos_diff)<0) dxl_motor[motor_index[i]].current_position=0;
	    else dxl_motor[motor_index[i]].current_position+=pos_diff; 
	    break;
// 	    //CCW rotation must decrease position
// 	    pos_diff=pos_prev[i]-dxl_motor[motor_index[i]].measured_position;
// 	    if(abs(pos_diff)>3000){
// 	      if(pos_diff>0) dxl_motor[motor_index[i]].current_position+=4096;
// 	      else if(pos_diff<0 && int(dxl_motor[motor_index[i]].current_position)-4096>=0) dxl_motor[motor_index[i]].current_position+=4096;
// 	      else dxl_motor[motor_index[i]].current_position=0;
// 	    }
// 	    if(((int (dxl_motor[motor_index[i]].current_position))-pos_diff)<0) dxl_motor[motor_index[i]].current_position=0;
// 	    else dxl_motor[motor_index[i]].current_position-=pos_diff; 
// 	    break;
	case 2:
	case 3:
	    //CCW rotation must decrease position (this is the normal case)
	    pos_diff=pos_prev[i]-dxl_motor[motor_index[i]].measured_position;
	    if(abs(pos_diff)>4000){
	      if(pos_diff<0) dxl_motor[motor_index[i]].current_position+=4096;
	      else dxl_motor[motor_index[i]].current_position-=4096;
	    }
	    if(dxl_motor[motor_index[i]].current_position-pos_diff<0) dxl_motor[motor_index[i]].current_position=0;
	    else dxl_motor[motor_index[i]].current_position-=pos_diff;
	  break;
	default:
	  break;
      }
    }
}

void dxl_halt(int motor_index){//halts motor shown in index -mainly for velocity - torque control
    if(motor_index>=0){ROS_INFO("Halting %d",motor_index);}
    if(dxl_motor[motor_index].mode==POSITION_CONTROL && dxl_motor[motor_index].goal_position!=dxl_motor[motor_index].current_position){
      dxl_motor[motor_index].goal_position=dxl_motor[motor_index].current_position;
      while(!write_position_single(motor_index,dxl_motor)){
				ROS_INFO("Error halting motor %d in position mode",dxl_motor[motor_index].ID);
				ros::Duration(s_time).sleep();
				ros::spinOnce();
      }
    }
    else if(dxl_motor[motor_index].mode==VELOCITY_CONTROL){
      if(dxl_motor[motor_index].goal_speed>1024) dxl_motor[motor_index].goal_speed=1024;
      else if(dxl_motor[motor_index].goal_speed>0 && dxl_motor[motor_index].goal_speed<1024) dxl_motor[motor_index].goal_speed=0;
      while(!write_velocity_single(motor_index,dxl_motor)){
	ROS_INFO("Error halting motor %d in velocity mode",dxl_motor[motor_index].ID);
	ros::Duration(s_time).sleep();
	ros::spinOnce();
      }
    }
    else if(dxl_motor[motor_index].mode==TORQUE_CONTROL){}
}

void lim_switch_check(){//checks if motor has touched limit switches-if yes system halts
    if(teensy->cserial_update()){
      for(uint8_t i=0;i<num_actuators;i++){
	if(i!=2 && i!=3){
	  if(teensy->ch_in[dxl_motor[i].lim_high]==48 || teensy->ch_in[dxl_motor[i].lim_low]==48){
	    if(dxl_motor[i].mode==POSITION_CONTROL && dxl_motor[i].current_position!=dxl_motor[i].goal_position){
	      dxl_halt(i);
	    }
	    else if(dxl_motor[i].mode==VELOCITY_CONTROL){
	      if((dxl_motor[i].goal_speed>1024 && teensy->ch_in[dxl_motor[i].lim_high]==48) || 
		(dxl_motor[i].goal_speed>0 && dxl_motor[i].goal_speed<1024 && teensy->ch_in[dxl_motor[i].lim_low]==48)) dxl_halt(i);
	    }
	    else if(dxl_motor[i].mode==TORQUE_CONTROL){}
	  }
	}
      }
    }
    else{
      ROS_INFO("Teensy failed. Stopping all motors");
      //for(uint8_t i=0;i<num_actuators;i++)	dxl_halt(i);
      disable_torque=1;
      end_program=1;
    }
}

bool end_process(){
    if(disable_torque){
	//TORQUES OFF FOR ALL
	uint8_t TORQUE_ON1[NUM_ACTUATORS1];
	uint8_t num_motor[NUM_ACTUATORS1];
	for (uint8_t i=0;i<NUM_ACTUATORS1;i++){TORQUE_ON1[i]=0; num_motor[i]=0+i;}
	torqueONOFF_sync(TORQUE_ON1,num_motor, NUM_ACTUATORS1, dxl_motor);
	ROS_INFO("Torque DXL controller 1 Off");
	uint8_t TORQUE_ON2[NUM_ACTUATORS2];
	uint8_t num_motor2[NUM_ACTUATORS2];
	for (uint8_t i=0;i<NUM_ACTUATORS2;i++){TORQUE_ON2[i]=0; num_motor2[i]=NUM_ACTUATORS1+i;}
	torqueONOFF_sync(TORQUE_ON2,num_motor2, NUM_ACTUATORS2, dxl_motor);
	ROS_INFO("Torque DXL controller 2 Off");
    }
    // Close port1
    portHandler1->closePort();
    // Close port2
    portHandler2->closePort();
    // Close Teensy
    teensy->~cserial_board();
    end_program=1;
}

bool init_position(){//SET ALL DXL_MOTORS TO THEIR STARTUP POSITION
    //5 first because this function will be also called when stopping operation
    dxl_motor[4].goal_speed=0;
    if(teensy->ch_in[dxl_motor[4].lim_low]==49){
      //dxl_motor[4].goal_speed=50;
      dxl_motor[4].goal_speed=round((SPEED_FACTOR5*SPEED28_MAX/SPEED_QUANTIZATION)+0.5);
    }
    while(!write_velocity_single(4,dxl_motor)){
      ros::Duration(s_time).sleep(); 
      ros::spinOnce();
    }
    while(dxl_motor[4].goal_speed>0 && dxl_motor[4].goal_speed!=1024){
      lim_switch_check();
      ros::Duration(s_time).sleep();
      ros::spinOnce();
    }
    //3+4
    uint8_t num_motor[2]={2,3};
    dxl_motor[2].goal_position=DOF3_START_POS-DOF3_OFFSET;
    dxl_motor[3].goal_position=DOF4_START_POS-DOF4_OFFSET;
    //DO NOT DELETE - LIMITS VELOCITIES--------------------------------
    dxl_motor[2].goal_speed=dxl_motor[3].goal_speed=round((SPEED_FACTOR34*SPEED64_MAX/SPEED_QUANTIZATION)+0.5); //2% of max speed
    uint8_t err=0;
    while(!write_uint16t(dxl_motor[2].packetHandler,dxl_motor[2].portHandler, dxl_motor[2].ID, P_SPEED_L ,
		   dxl_motor[2].goal_speed, &err)){
			    ros::Duration(s_time).sleep();
		           ros::spinOnce();
		  }
    while(!write_uint16t(dxl_motor[3].packetHandler,dxl_motor[3].portHandler, dxl_motor[3].ID, P_SPEED_L ,
		   dxl_motor[3].goal_speed, &err)){
			    ros::Duration(s_time).sleep();
		           ros::spinOnce();
		  }
    //-----------------------------------------------------------------
    if(use_sync_write){
      while(!write_position_sync(num_motor,2,dxl_motor)){
	ros::Duration(s_time).sleep();
	ros::spinOnce();
      }
    }
    else{
      for(uint8_t i=2; i<4;i++){
	uint8_t dxl_error=-1;
	while(!write_position_single(i,dxl_motor)){
	  ros::Duration(s_time).sleep();
	  ros::spinOnce();
	}
      }
    }    
    //1+2
    num_motor[0]=0;
    num_motor[1]=1;
    dxl_motor[0].goal_speed=dxl_motor[1].goal_speed=0;
    if(teensy->ch_in[dxl_motor[0].lim_low]==49 || teensy->ch_in[dxl_motor[1].lim_low]==49){//someone must move...
      if(teensy->ch_in[dxl_motor[0].lim_low]==49)dxl_motor[0].goal_speed=round((SPEED_FACTOR12*SPEED64_MAX/SPEED_QUANTIZATION)+0.5); //50% //else dxl 0 already on lim switch
      if(teensy->ch_in[dxl_motor[1].lim_low]==49)dxl_motor[1].goal_speed=round((SPEED_FACTOR12*SPEED64_MAX/SPEED_QUANTIZATION)+0.5); //50% //else dxl 1 already on lim switch
    }
    if(use_sync_write){
      while(!write_velocity_sync(num_motor,2, dxl_motor)){
	ros::Duration(s_time).sleep();
	ros::spinOnce();
      }
    }
    else{
      //ros::Time now=ros::Time::now();
      for(uint8_t i=0; i<2;i++){
	while(!write_velocity_single(i,dxl_motor)){
	  ros::Duration(s_time).sleep();
	  ros::spinOnce();
	}
      }
    }
    while((dxl_motor[0].goal_speed>0 && dxl_motor[0].goal_speed!=1024)  
      || (dxl_motor[1].goal_speed>0 && dxl_motor[1].goal_speed!=1024)){
      lim_switch_check();
	ros::Duration(s_time).sleep();
	ros::spinOnce();
      }
    uint8_t num_motor_all[5]={0,1,2,3,4};
    dxl_pos_update(num_motor_all,num_actuators);
    dxl_motor[2].current_position=dxl_motor[2].position_start=dxl_motor[2].goal_position;
    dxl_motor[3].current_position=dxl_motor[3].position_start=dxl_motor[3].goal_position;
    dxl_motor[0].position_start=dxl_motor[0].measured_position;
    dxl_motor[1].position_start=dxl_motor[1].measured_position;
    dxl_motor[4].position_start=dxl_motor[4].measured_position;
    dxl_motor[0].current_position=dxl_motor[0].goal_position=0;
    dxl_motor[1].current_position=dxl_motor[1].goal_position=0;
    dxl_motor[4].current_position=dxl_motor[4].goal_position=0;
    return true;
}

void dxl_go_to_pos_notime(int first_run){//velocity for 1+2+5 -- position for 3+4
    uint8_t value_change[5]={0,0,0,0,0};
    for(uint8_t i=0;i<num_actuators;i++){
      if(i!=2 && i!=3){//1+2+5
	//CHECK FIRST RUN
	if(!first_run){
	    if(dxl_motor[i].current_position<dxl_motor[i].goal_position && teensy->ch_in[dxl_motor[i].lim_high]!=48){
	      ROS_INFO("%d %d %d in 0",i,dxl_motor[i].current_position,dxl_motor[i].goal_speed);
		if(i==4) dxl_motor[i].goal_speed=1024+round((speedf5*SPEED28_MAX/SPEED_QUANTIZATION)+0.5);
		else dxl_motor[i].goal_speed=1024+round((speedf12*SPEED64_MAX/SPEED_QUANTIZATION)+0.5);
		value_change[i]=1;
	    }
	    else if(dxl_motor[i].current_position>dxl_motor[i].goal_position && teensy->ch_in[dxl_motor[i].lim_low]!=48){
	      ROS_INFO("%d %d %d in 1",i,dxl_motor[i].current_position,dxl_motor[i].goal_speed);
	      if(i==4) dxl_motor[i].goal_speed=1024+round((speedf5*SPEED28_MAX/SPEED_QUANTIZATION)+0.5);
		else dxl_motor[i].goal_speed=1024+round((speedf12*SPEED64_MAX/SPEED_QUANTIZATION)+0.5);
	      value_change[i]=1;
	    }
	}//close first run
	else{
	   if(dxl_motor[i].current_position>=dxl_motor[i].goal_position && dxl_motor[i].goal_speed>1024){
	   ROS_INFO("Halt %d %d %d in 0",i,dxl_motor[i].current_position,dxl_motor[i].goal_speed);
	   dxl_motor[i].goal_speed=1024;
	   value_change[i]=1;
	   }
	   else if(dxl_motor[i].current_position<=dxl_motor[i].goal_position && dxl_motor[i].goal_speed>0 && dxl_motor[i].goal_speed<1024){
	   ROS_INFO("Halt %d %d %d in 1",i,dxl_motor[i].current_position,dxl_motor[i].goal_speed);
	   dxl_motor[i].goal_speed=0;
	   value_change[i]=1;
	  }
	}//close first run else
	//CHECK CHANGED VALUES
	if(value_change[0]==value_change[1] && value_change[0]==1){
	  if(use_sync_write){
	    uint8_t num_motor[2]={0,1};
	    while(!write_velocity_sync(num_motor,2,dxl_motor)){
	      ros::Duration(s_time).sleep();
	      ros::spinOnce();
	    }
	  }
	  else{
	    for(uint8_t i=0; i<2;i++){
	      while(!write_velocity_single(i,dxl_motor)){
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	      }
	    }
	  }
	}
	else if(value_change[0]==1){
	  while(!write_velocity_single(0,dxl_motor)){ 
	    ros::Duration(s_time).sleep();
	    ros::spinOnce();
	  }
	}
	else if(value_change[1]==1){
	  while(!write_velocity_single(1,dxl_motor)){
	    ros::Duration(s_time).sleep();
	    ros::spinOnce();
	  }
	}
	if(value_change[4]==1){
	  while(!write_velocity_single(4,dxl_motor)){
	    ROS_INFO("write 4 no time");
	    ros::Duration(s_time).sleep();
	    ros::spinOnce();
	  }
	}
      }//close if 1+2+5
      else{//3,4
	if(!first_run){
	  if(dxl_motor[2].goal_position!=dxl_motor[2].current_position && dxl_motor[3].goal_position!=dxl_motor[3].current_position){
	    if(use_sync_write){
	      uint8_t num_motor[2]={2,3};
	      while(!write_position_sync(num_motor,2,dxl_motor)){
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	      }
	    }
	    else{
	      for(uint8_t i=2; i<4;i++){
		while(!write_position_single(i,dxl_motor)){
		  ros::Duration(s_time).sleep();
		  ros::spinOnce();
		}
	      }
	    }
	  }
	  else if(dxl_motor[2].goal_position!=dxl_motor[2].current_position){
	    while(!write_position_single(2,dxl_motor)){
	      ros::Duration(s_time).sleep();
	      ros::spinOnce();
	    }
	  }
	  else if(dxl_motor[3].goal_position!=dxl_motor[3].current_position){
	    while(!write_position_single(3,dxl_motor)){
	      ros::Duration(s_time).sleep();
	      ros::spinOnce();
	    }
	  }
	} //close first run
      }//closing else 3+4
  }//close for all motors
}

void dxl_go_to_pos(int first_run){//velocity for 1+2+5 -- position for 3+4
  if(!first_run){
    double pos_diff[5]={0.0,0.0,0.0,0.0,0.0};
    double t_motion;
    double t_maximum;
    double t_minimum;
    uint16_t mt_new_speed[5]={0.0,0.0,0.0,0.0,0.0};
    //compute rotations
    for(uint8_t i=0;i<num_actuators;i++) pos_diff[i]=(double (dxl_motor[i].goal_position))-(double (dxl_motor[i].current_position));
    if(fabs(pos_diff[0])>=fabs(pos_diff[1]) && fabs(pos_diff[3])>=2.0){
      //ROS_INFO("C1");
      //motor 2 has more distance to cover 
      //check if speeds are manage-able
      t_maximum=((fabs(pos_diff[3])*2*C_PI)/(4096.0*SPEED_QUANTIZATION));
      t_minimum=((fabs(pos_diff[0])*2*C_PI)/(4096.0*SPEED64_MAX));
    }
    else if(fabs(pos_diff[1])>=fabs(pos_diff[0]) && fabs(pos_diff[2])>=2.0){
      //ROS_INFO("C2");
      //motor 1 has more distance to cover
      //check if speeds are manage-able
      t_maximum=((fabs(pos_diff[2])*2*C_PI)/(4096.0*SPEED_QUANTIZATION));
      t_minimum=((fabs(pos_diff[1])*2*C_PI)/(4096.0*SPEED64_MAX));
    }
    else{
      //we care only about position of motor 4!
      t_maximum=0.0;
      t_minimum=-1.0;
      ROS_INFO("times manually set to 0");
    }
    if(t_maximum<t_minimum){
      ROS_INFO("tOUGH LUCK! %f %f",t_maximum,t_minimum);
      ROS_INFO("tOUGH LUCK2! %f %f %f %f %f",fabs(pos_diff[0]), fabs(pos_diff[1]),fabs(pos_diff[2]), fabs(pos_diff[3]), fabs(pos_diff[4]));
      if(fabs(pos_diff[0])>=fabs(pos_diff[1])){ROS_INFO("C1");}
      else{ROS_INFO("C2");}
    }
    else{
      //equalize fastest linear motion to find rotational motion
      bool bool_change[5]={false,false,false,false,false};
      if(t_minimum>=0.0){
	t_motion=t_minimum;
	if(fabs(pos_diff[3])>=2.0 && fabs(pos_diff[0])>=2.0){
	  mt_new_speed[3]=round(((fabs(pos_diff[3])*2*C_PI)/(4096.0*SPEED_QUANTIZATION*t_motion))+0.5);
	  if(mt_new_speed[3]>=1024) mt_new_speed[3]=1023;
	  mt_new_speed[0]=round(((fabs(pos_diff[0])*2*C_PI)/(4096.0*SPEED_QUANTIZATION*t_motion))+0.5);
	  if(pos_diff[0]<0 && mt_new_speed[0]>=1024) mt_new_speed[0]=1023;
	  if(pos_diff[0]>0) mt_new_speed[0]+=1024;
	  if(pos_diff[0]>0 && mt_new_speed[0]>=1024+1023) mt_new_speed[0]=1024+1023;
	  bool_change[3]=bool_change[0]=true;
	}
	else{
	  ROS_INFO("B0");
	}
	if(fabs(pos_diff[2])>=2.0 && fabs(pos_diff[1])>=2.0){
	  mt_new_speed[2]=round(((fabs(pos_diff[2])*2*C_PI)/(4096.0*SPEED_QUANTIZATION*t_motion))+0.5);
	  if(mt_new_speed[2]>=1024) mt_new_speed[2]=1023;
	  mt_new_speed[1]=round(((fabs(pos_diff[1])*2*C_PI)/(4096.0*SPEED_QUANTIZATION*t_motion))+0.5);
	  if(pos_diff[1]<0 && mt_new_speed[1]>=1024) mt_new_speed[1]=1023;
	  if(pos_diff[1]>0) mt_new_speed[1]+=1024;
	  if(pos_diff[1]>0 && mt_new_speed[1]>=1024+1023) mt_new_speed[1]=1024+1023;
	  bool_change[2]=bool_change[1]=true;
	}
	else{
	  ROS_INFO("B1");
	}
	if(fabs(pos_diff[4])>=2.0){
	  mt_new_speed[4]=round(((fabs(pos_diff[4])*2*C_PI)/(4096.0*SPEED_QUANTIZATION*t_motion))+0.5);
	  if(pos_diff[4]<0 && mt_new_speed[4]>=1024) mt_new_speed[4]=100;
	  if(pos_diff[4]>0) mt_new_speed[4]+=1024;
	  if(pos_diff[4]>0 && mt_new_speed[4]>=1024+1023) mt_new_speed[4]=1024+100;
	  bool_change[4]=true;
	}
      }
      else{
	//we only care about 4
	t_motion=((fabs(pos_diff[4])*2*C_PI)/(4096.0*(SPEED28_MAX/10.0)));
	if(fabs(pos_diff[4])>=2.0){
	  mt_new_speed[4]=round(((fabs(pos_diff[4])*2*C_PI)/(4096.0*SPEED_QUANTIZATION*t_motion))+0.5);
	  if(pos_diff[4]<0 && mt_new_speed[4]>=1024) mt_new_speed[4]=100;
	  if(pos_diff[4]>0) mt_new_speed[4]+=1024;
	  if(pos_diff[4]>0 && mt_new_speed[4]>=1024+1023) mt_new_speed[4]=1024+100;
	  bool_change[4]=true;
	}
      }
      ROS_INFO("%d %d %d %d %d",dxl_motor[0].current_position,dxl_motor[1].current_position,dxl_motor[2].current_position,dxl_motor[3].current_position,dxl_motor[4].current_position);
      ROS_INFO("new speeds %f %d %d %d %d %d",t_motion, mt_new_speed[0],mt_new_speed[1],mt_new_speed[2],mt_new_speed[3],mt_new_speed[4]);
      for(uint8_t i=0;i<num_actuators;i++){
	if(bool_change[i] && dxl_motor[i].goal_speed!=mt_new_speed[i]){
 	  dxl_motor[i].goal_speed=mt_new_speed[i];
	}
	else bool_change[i]=false;
	//bool_change[i]=false;
      }
      //bool_change[0]=bool_change[1]=bool_change[2]=bool_change[3]=false;
      //APPLY SPEEDS AND THE CALL GO TO POS NO TIME! :)
//       ////////////////////////////////////////
//       for(uint8_t i=0;i<num_actuators;i++){
// 	if(bool_change[i] && dxl_motor[i].goal_speed!=mt_new_speed[i]){
// 	  dxl_motor[i].goal_speed=mt_new_speed[i];
// 	  bool_change[i]=true;
// 	}
// 	else bool_change[i]=false;
//       }
//       if(dxl_motor[3].goal_speed==0) {ROS_INFO("B0");dxl_motor[0].goal_speed=0; bool_change[3]=false; bool_change[0]=false;}
//       if(dxl_motor[2].goal_speed==0) {ROS_INFO("B1");dxl_motor[1].goal_speed=0; bool_change[1]=false; bool_change[2]=false;}
      uint8_t error=0;
      //1+2
      if(bool_change[0] && bool_change[1]){
	if(use_sync_write){
	  uint16_t tmp_speed[2]={dxl_motor[0].goal_speed,dxl_motor[1].goal_speed};
	  uint8_t num_motor[2]={0,1};
	  while(!write_velocity_sync(num_motor,2,dxl_motor)){
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	  }
	}
	else{
	  while(!write_velocity_single(0,dxl_motor)){
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	  }
	  while(!write_velocity_single(1,dxl_motor)){
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	  }
	}
      }
      else if(bool_change[0]){
	while(!write_velocity_single(0,dxl_motor)){
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	  }
      }
      else if(bool_change[1]){
	while(!write_velocity_single(1,dxl_motor)){
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	  }
      }
      else{}
      //3+4+5
      if(bool_change[2] && bool_change[3] && bool_change[4]){
	uint8_t num_motor[3]={2,3,4};
	if(use_sync_write){
	  uint16_t tmp_speed[3]={dxl_motor[2].goal_speed,dxl_motor[3].goal_speed,dxl_motor[4].goal_speed}; 
	  while(!SYNC_WRITE(num_motor,3,P_SPEED_L,2,tmp_speed,dxl_motor)){
		    ros::Duration(s_time).sleep();
		    ros::spinOnce();
	  }
	}
	else{
	    for(uint8_t i=0;i<3;i++){
	      ROS_INFO("D write");
	      while(!write_uint16t(dxl_motor[num_motor[i]].packetHandler,dxl_motor[num_motor[i]].portHandler,dxl_motor[num_motor[i]].ID,P_SPEED_L,dxl_motor[num_motor[i]].goal_speed,&error)){
		ROS_INFO("error D write %d",i);
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	      }
	    }
	}
      }
      else if(bool_change[2] && bool_change[3]){
	uint8_t num_motor[2]={2,3};
	if(use_sync_write){
	  uint16_t tmp_speed[2]={dxl_motor[2].goal_speed,dxl_motor[3].goal_speed};
	  while(!SYNC_WRITE(num_motor,2,P_SPEED_L,2,tmp_speed,dxl_motor)){
		  ros::Duration(s_time).sleep();
		  ros::spinOnce();
	  }
	}
	else{
	  for(uint8_t i=0;i<2;i++){
	      ROS_INFO("C write");
	      while(!write_uint16t(dxl_motor[num_motor[i]].packetHandler,dxl_motor[num_motor[i]].portHandler,dxl_motor[num_motor[i]].ID,P_SPEED_L,dxl_motor[num_motor[i]].goal_speed,&error)){
		ROS_INFO("error C write %d",i);
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	      }
	    }
	}
      }
      else if(bool_change[2] && bool_change[4]){
	uint8_t num_motor[2]={2,4};
	if(use_sync_write){
	  uint16_t tmp_speed[2]={dxl_motor[2].goal_speed,dxl_motor[4].goal_speed};
	  while(!SYNC_WRITE(num_motor,2,P_SPEED_L,2,tmp_speed,dxl_motor)){
		  ros::Duration(s_time).sleep();
		  ros::spinOnce();
	  }
	}
	else{
	  for(uint8_t i=0;i<2;i++){
	      ROS_INFO("B write");
	      while(!write_uint16t(dxl_motor[num_motor[i]].packetHandler,dxl_motor[num_motor[i]].portHandler,dxl_motor[num_motor[i]].ID,P_SPEED_L,dxl_motor[num_motor[i]].goal_speed,&error)){
		ROS_INFO("error B write %d",i);
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	      }
	  }
	}
      }
      else if(bool_change[3] && bool_change[4]){
	uint8_t num_motor[2]={3,4};
	if(use_sync_write){
	  uint16_t tmp_speed[2]={dxl_motor[3].goal_speed,dxl_motor[4].goal_speed};
	  while(!SYNC_WRITE(num_motor,2,P_SPEED_L,2,tmp_speed,dxl_motor)){
		  ros::Duration(s_time).sleep();
		  ros::spinOnce();
	  }
	}
	else{
	  for(uint8_t i=0;i<2;i++){
	      ROS_INFO("A write");
	      while(!write_uint16t(dxl_motor[num_motor[i]].packetHandler,dxl_motor[num_motor[i]].portHandler,dxl_motor[num_motor[i]].ID,P_SPEED_L,dxl_motor[num_motor[i]].goal_speed,&error)){
		ROS_INFO("error A write %d",i);
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	      }
	  }
	}
      }
      else if(bool_change[2]){
	while(!write_uint16t(dxl_motor[2].packetHandler,dxl_motor[2].portHandler,2,P_SPEED_L,dxl_motor[2].goal_speed,&error)){
		ROS_INFO("error write 2");
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	}
      }
      else if(bool_change[3]){
	while(!write_uint16t(dxl_motor[3].packetHandler,dxl_motor[3].portHandler,3,P_SPEED_L,dxl_motor[3].goal_speed,&error)){
		ROS_INFO("error write 3");
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	}
      }
      else if(bool_change[4]){
	ROS_INFO("write 4!");
	while(!write_velocity_single(4,dxl_motor)){
		ROS_INFO("error write");
		ros::Duration(s_time).sleep();
		ros::spinOnce();
	}
      }
      else{}
      ///////////////////////////
      //set positions for 3,4 /// 1,2,5 must be already moving now
      if(fabs(pos_diff[2])>=2.0 && fabs(pos_diff[3])>=2.0){
	ROS_INFO("write pos motor 2 and motor 3");
	if(use_sync_write){
	  uint8_t num_motor[2]={2,3};
	  while(!write_position_sync(num_motor,2,dxl_motor)){
	    ros::Duration(s_time).sleep();
	    ros::spinOnce();
	  }
	}
	else{
	  for(uint8_t i=2; i<4;i++){
	    while(!write_position_single(i,dxl_motor)){
	      ros::Duration(s_time).sleep();
	      ros::spinOnce();
	    }
	  }
	}
      }
      else if(fabs(pos_diff[2])>=2.0){
	  ROS_INFO("write pos motor 2");
	  while(!write_position_single(2,dxl_motor)){
	    ros::Duration(s_time).sleep();
	    ros::spinOnce();
	  }
	}
      else if(fabs(pos_diff[3])>=2.0){
	ROS_INFO("write pos motor 3");
	while(!write_position_single(3,dxl_motor)){
	  ros::Duration(s_time).sleep();
	  ros::spinOnce();
	}
      }
      //////////////////////////////
    } //close else rgarding time availability
  } //close if first first_run
  else{
    //since new speeds are applied dxl_no_time is called to monitor achievement
    //ROS_INFO("pos no time");
    dxl_go_to_pos_notime(1);
  }
}

void chatterCallback(const std_msgs::UInt32MultiArray::ConstPtr& dxl_in){
    switch((int) dxl_in->data[0]){
        case DYNAMIXEL_INIT_POSITION:{// initialize stuff and go to init position
	    ROS_INFO("[COMMAND] DYNAMIXEL_INIT_POSITION");
	    executed_command=DYNAMIXEL_INIT_POSITION;
	    break;
        }
        case DYNAMIXEL_COMPUTE_LIMITS:{ //computes limits of translational DoF
	    ROS_INFO("[COMMAND] DYNAMIXEL_COMPUTE_LIMITS");
	    executed_command=DYNAMIXEL_COMPUTE_LIMITS;
	    lims[0]=lims[1]=lims[2]=0;
	    //tha pigainw me init_position stin arxi (aka DYNAMIXEL_START_POS_PROCESS)
	    init_position();
	    //ola ta moter deksia me metrisi mesw tis DYNAMIXEL_GO_TO_POS_NOTIME me megales times
	    dxl_motor[0].goal_position=dxl_motor[1].goal_position=dxl_motor[4].goal_position=500000;
	    dxl_go_to_pos_notime(0);
	    //ksana init_position sto ros loop(aka DYNAMIXEL_START_POS_PROCESS)
	    break;
	}
	case DYNAMIXEL_GO_TO_POS:{ //will be used only when not passing through pivot point - aka robot out of site
	    executed_command=DYNAMIXEL_GO_TO_POS;
	   // ROS_INFO("[COMMAND] DYNAMIXEL_GO_TO_POS");
	    uint8_t i=1;
	    ROS_INFO("%d\t%d\t%d\t%d\t%d",dxl_in->data[2],dxl_in->data[1],dxl_in->data[3],dxl_in->data[4],dxl_in->data[5]);
	    dxl_motor[2].goal_position=dxl_in->data[3]-DOF3_OFFSET;
	    dxl_motor[3].goal_position=dxl_in->data[4]-DOF4_OFFSET;
	    dxl_motor[4].goal_position=dxl_in->data[5];
	    dxl_motor[0].goal_position=dxl_in->data[2];
	    dxl_motor[1].goal_position=dxl_in->data[1];
	    dxl_go_to_pos(0); //run once to decide initially over motion
	    break;
	}
	case DYNAMIXEL_GO_TO_POS_NOTIME:{ //will be used only when not passing through pivot point - aka robot out of site
	    executed_command=DYNAMIXEL_GO_TO_POS_NOTIME;
	    ROS_INFO("[COMMAND] DYNAMIXEL_GO_TO_POS_NOTIME");
	    uint8_t i=1;
	    ROS_INFO("%d\t%d\t%d\t%d\t%d",dxl_in->data[2],dxl_in->data[1],dxl_in->data[3],dxl_in->data[4],dxl_in->data[5]);
	    dxl_motor[2].goal_position=dxl_in->data[3]-DOF3_OFFSET;
	    dxl_motor[3].goal_position=dxl_in->data[4]-DOF4_OFFSET;
	    dxl_motor[4].goal_position=dxl_in->data[5];
	    dxl_motor[0].goal_position=dxl_in->data[2];
	    dxl_motor[1].goal_position=dxl_in->data[1];
	    dxl_go_to_pos_notime(0); //run once to decide initially over motion
	    break;
	}
	case DYNAMIXEL_MANUAL_EXTRAOPERATIVE_OPERATION:{
	    executed_command=DYNAMIXEL_MANUAL_EXTRAOPERATIVE_OPERATION;
	    //ROS_INFO("%d",dxl_in->data[1]);
	    //ROS_INFO("%d, %d, %d, %d, %d",dxl_in->data[1],dxl_in->data[2],dxl_in->data[3],dxl_in->data[4],dxl_in->data[5]);
	    uint8_t num_motor[2]={0,1};
	    if(dxl_in->data[2]==1){dxl_motor[0].goal_speed=512;}
	    else if(dxl_in->data[2]==2){dxl_motor[0].goal_speed=1024+512;}
	    else if(dxl_in->data[2]==0){dxl_motor[0].goal_speed=0;}
	    if(dxl_in->data[1]==1){dxl_motor[1].goal_speed=512;}
	    else if(dxl_in->data[1]==2){dxl_motor[1].goal_speed=1024+512;}
	    else if(dxl_in->data[1]==0){dxl_motor[1].goal_speed=0;}
	    while(!write_velocity_sync(num_motor,2,dxl_motor)){sleep(0.2);}
	    if(dxl_in->data[3]!=0){dxl_motor[2].goal_position=dxl_in->data[3];}
	    if(dxl_in->data[4]!=0){dxl_motor[3].goal_position=dxl_in->data[4];}
	    num_motor[0]=2; num_motor[1]=3;
	    while(!write_position_sync(num_motor,2,dxl_motor)){sleep(0.2);}
	    if(dxl_motor[4].goal_speed!=dxl_in->data[5]){
	    dxl_motor[4].goal_speed=dxl_in->data[5];
	    while(!write_velocity_single(4,dxl_motor)){sleep(0.2);}
	    }
	    break;
	}
        case DYNAMIXEL_STATUS:{
	    executed_command=DYNAMIXEL_STATUS;
            break;
        }
        case DYNAMIXEL_END_PROCESS:{
	    ROS_INFO("[COMMAND] END_PROCESS");
	    executed_command=DYNAMIXEL_END_PROCESS;
	    break;
        }
        default:
        break;
    }
}

int main(int argc, char** argv){
      ros::init(argc,argv,"myDXL_COM");
      sleep(0.5);
      ros::NodeHandle dynamixel_comm; //node that receives commands for dxl and informs controller about status
      sleep(0.5);
      ros::Rate loop_rate(RATE);
      //publish stuff
      std_msgs::UInt32MultiArray dynamixel_array_out;
      ros::Publisher dynamixel_out=dynamixel_comm.advertise<std_msgs::UInt32MultiArray>("dxl_2ctrl",2);
      sleep(0.5);
      //subscribe stuff
      ros::Subscriber dynamixel_in=dynamixel_comm.subscribe("dxl_from_ctrl",2,chatterCallback);
      sleep(0.5);
      while(dynamixel_out.getNumSubscribers()<1){
	ros::spinOnce();
	//loop_rate.sleep();
	ros::Duration(s_time).sleep();
      }
      //rate 
      ROS_INFO("Init OK");
    //Initialize actuator params
    for(uint8_t i=0;i<num_actuators;i++){
        if(i<NUM_ACTUATORS1)  dxl_motor[i].ID=i+1;
        else dxl_motor[i].ID=i-NUM_ACTUATORS1+1;
        dxl_motor[i].mode=0; //always start with position control
        dxl_motor[i].P=30;
        dxl_motor[i].I=0;
        dxl_motor[i].D=5;
        dxl_motor[i].acceleration=10;
	dxl_motor[i].position_start=0;
        dxl_motor[i].goal_position=0;
	dxl_motor[i].current_position=0;
	dxl_motor[i].measured_position=0;
	dxl_motor[i].goal_speed=0;
        dxl_motor[i].current_velocity=0;
        dxl_motor[i].current_torque=0;
        dxl_motor[i].moving=0;
	dxl_motor[i].type=64;
    }
    dxl_motor[0].ID=2;
    dxl_motor[1].ID=1;
    dxl_motor[4].type=32;
    //Deploy custom settings here
    dxl_motor[0].acceleration=dxl_motor[1].acceleration=dxl_motor[4].acceleration=100;
    //Define limit switch Teensy indexes for every motor 
    dxl_motor[0].lim_low=4; dxl_motor[0].lim_high=5;
    dxl_motor[1].lim_low=3; dxl_motor[1].lim_high=2;
    dxl_motor[4].lim_low=0; dxl_motor[4].lim_high=1;
	 
    //Start serial ports
    if(init_process()){
      uint8_t num_mot_all[5]={0,1,2,3,4};
      dynamixel_array_out.data.push_back(uint32_t (DYNAMIXEL_READY));
      //for(uint8_t i=0;i<num_actuators;i++) dynamixel_array_out.data.push_back(dxl_motor[i].current_position);
      dynamixel_out.publish(dynamixel_array_out);
      ros::spinOnce();
      //loop_rate.sleep();
      ros::Duration(s_time).sleep();
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /*
      init_position(); 	      ROS_INFO("dxl_status for D3 %d ",dxl_motor[4].current_position);

      uint8_t TORQUE_ON1[NUM_ACTUATORS1];
	uint8_t num_motor[NUM_ACTUATORS1];
	for (uint8_t i=0;i<NUM_ACTUATORS1;i++){TORQUE_ON1[i]=0; num_motor[i]=i;}
	torqueONOFF_sync(TORQUE_ON1,num_motor, NUM_ACTUATORS1, dxl_motor);
	ROS_INFO("Torque DXL controller 1 Off");
	uint8_t TORQUE_ON2[NUM_ACTUATORS2];
	uint8_t num_motor2[NUM_ACTUATORS2];
	for (uint8_t i=0;i<NUM_ACTUATORS2;i++){TORQUE_ON2[i]=0; num_motor2[i]=NUM_ACTUATORS1+i;}
	torqueONOFF_sync(TORQUE_ON2,num_motor2, NUM_ACTUATORS2, dxl_motor);
	ROS_INFO("Torque DXL controller 2 Off");
	
	*/
	//////////////////////////////////////////////////////////////////////////////////////////////////////
      while(ros::ok() && !end_program){
	dynamixel_array_out.data.clear();
	lim_switch_check();
	dxl_pos_update(num_mot_all,num_actuators);
	switch(int (executed_command)){
	    case DYNAMIXEL_GO_TO_POS:{
	      dxl_go_to_pos(1); //no time called but speeds are already fixed
	      if((dxl_motor[0].goal_speed==0 || dxl_motor[0].goal_speed==1024) 
		      && (dxl_motor[1].goal_speed==0 || dxl_motor[1].goal_speed==1024)
		      && (dxl_motor[4].goal_speed==0 || dxl_motor[4].goal_speed==1024)
		      && (abs(dxl_motor[2].current_position-dxl_motor[2].goal_position)<=ROT_DOF_TOLERANCE)
		      && (abs(dxl_motor[3].current_position-dxl_motor[3].goal_position)<=ROT_DOF_TOLERANCE)
	      ){
		ROS_INFO("publish");
		dynamixel_array_out.data.push_back(uint32_t (DYNAMIXEL_GO_TO_POS));
		executed_command=0;
		dynamixel_out.publish(dynamixel_array_out);
	      }
	      else{
	      ROS_INFO("%d\t%d\t%d\t%d\t%d\t%d",dxl_motor[0].goal_position,dxl_motor[0].current_position,dxl_motor[1].goal_position,dxl_motor[1].current_position,dxl_motor[4].goal_position,dxl_motor[4].current_position);
	      ROS_INFO("%d\t%d\t%d\t%d",dxl_motor[2].current_position,dxl_motor[2].goal_position,dxl_motor[3].current_position,dxl_motor[3].goal_position);
	      ROS_INFO("%d\t%d\t%d",dxl_motor[0].goal_speed,dxl_motor[1].goal_speed,dxl_motor[4].goal_speed);
	      }
	      break;
	    }
	    case DYNAMIXEL_GO_TO_POS_NOTIME:{
	      dxl_go_to_pos_notime(1);
	      //ROS_INFO("%d\t%d\t%d\t%d",dxl_motor[0].current_position,dxl_motor[0].measured_position,dxl_motor[1].current_position,dxl_motor[1].measured_position);
	      if((dxl_motor[0].goal_speed==0 || dxl_motor[0].goal_speed==1024) 
		      && (dxl_motor[1].goal_speed==0 || dxl_motor[1].goal_speed==1024)
		      && (dxl_motor[4].goal_speed==0 || dxl_motor[4].goal_speed==1024)
		      && (abs(dxl_motor[2].current_position-dxl_motor[2].goal_position)<=ROT_DOF_TOLERANCE)
		      && (abs(dxl_motor[3].current_position-dxl_motor[3].goal_position)<=ROT_DOF_TOLERANCE)
		){
		ROS_INFO("publish no time");
		dynamixel_array_out.data.push_back(uint32_t (DYNAMIXEL_GO_TO_POS_NOTIME));
		executed_command=0;
		dynamixel_out.publish(dynamixel_array_out);
		}
		else{
		  //ROS_INFO("%d\t%d\t%d\t%d\t%d",dxl_motor[0].goal_speed,dxl_motor[1].goal_speed,dxl_motor[4].goal_speed,abs(dxl_motor[2].current_position-dxl_motor[2].goal_position),abs(dxl_motor[3].current_position-dxl_motor[3].goal_position));  
		}
// 		else if((dxl_motor[0].goal_speed==0 || dxl_motor[0].goal_speed==1024) 
// 		      && (dxl_motor[1].goal_speed==0 || dxl_motor[1].goal_speed==1024)
// 		      && (dxl_motor[4].goal_speed==0 || dxl_motor[4].goal_speed==1024)){
// 			ROS_INFO("%d %d %d %d",dxl_motor[2].current_position,dxl_motor[3].current_position,
// 			  dxl_motor[2].goal_position,dxl_motor[3].goal_position);
// 		      }
	      break;
	    }
	    case DYNAMIXEL_COMPUTE_LIMITS:{
	      dxl_go_to_pos_notime(1);
	      if((dxl_motor[0].goal_speed==0 || dxl_motor[0].goal_speed==1024) 
		      && (dxl_motor[1].goal_speed==0 || dxl_motor[1].goal_speed==1024)
		      && (dxl_motor[4].goal_speed==0 || dxl_motor[4].goal_speed==1024)
		){
		lims[0]=dxl_motor[1].current_position;
		lims[1]=dxl_motor[0].current_position;  
		lims[2]=dxl_motor[4].current_position;
		dynamixel_array_out.data.push_back(uint32_t (DYNAMIXEL_COMPUTE_LIMITS));
		for(uint8_t i=0;i<3;i++) dynamixel_array_out.data.push_back(lims[i]);
		init_position();
		executed_command=0;
		dynamixel_out.publish(dynamixel_array_out);
	      }
	      break;
	    }
	    case DYNAMIXEL_MANUAL_EXTRAOPERATIVE_OPERATION:{
	      //dxl_go_to_pos_notime(1);
	      executed_command=0;
	      break;
	    }
	    case DYNAMIXEL_INIT_POSITION:{
	      init_position();
	      dynamixel_array_out.data.push_back(uint32_t (DYNAMIXEL_INIT_POSITION));
	      executed_command=0;
	      dynamixel_out.publish(dynamixel_array_out); 
	      break;
	    }
	    case DYNAMIXEL_END_PROCESS:{
	      init_position(); 
	      end_program=1;
	      executed_command=0; 
	      break;
	    }
	    //always publish position no matter what
	    case DYNAMIXEL_STATUS:
	    default:{
	      dynamixel_array_out.data.push_back(uint32_t (DYNAMIXEL_STATUS));
	      dynamixel_array_out.data.push_back(dxl_motor[1].current_position);
	      dynamixel_array_out.data.push_back(dxl_motor[0].current_position);
	      //fixed to account for offsets
	      dynamixel_array_out.data.push_back(dxl_motor[2].current_position+DOF3_OFFSET);
	      dynamixel_array_out.data.push_back(dxl_motor[3].current_position+DOF4_OFFSET);
	      dynamixel_array_out.data.push_back(dxl_motor[4].current_position);
	      executed_command=0;
	      dynamixel_out.publish(dynamixel_array_out);
	      break;
	    }
	}
	ros::spinOnce();
	//ros::Duration(s_time).sleep();
	loop_rate.sleep();
	//end_program=1;
      }
    }
    end_process();
    return 0;
}
