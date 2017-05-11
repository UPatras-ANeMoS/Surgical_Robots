#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>
#include "controller_defs.h"
#define RATE 100 //Hz - 10ms
double s_time=1.0/((double) RATE); //seconds

using namespace chai3d;
using namespace std;

//Kinematic parameters
double DH[DOF][4]={
  {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
  //Matrix containing the latest command for the 5 DXL_motors
  uint32_t dxl_command_matrix[5]={0,0,0,0,0};
  
  //assigned to global variables instead of (#DEFINES) because robot may recompute limits and impose them at any time
  double z_motion=Z_motion;
  double cart_motion=UPPER_Z_MOTION;
  double y_motion=Y_motion;
  //double operation_start_DH[5]={0.0,0.0,0.0,-C_PI_DIV_2/2.0,0.0};
  double operation_start_DH[9]={0.0,z_motion/2.0,0.0,-C_PI_DIV_2/2.0,0.0,0.0,0.0,0.0,0.0};
  
  transforms single_transforms;//4x4 transformation matrices from joint-to-joint
  transforms base_transforms;//4x4 transformation matrices w.r.t. base
  
  uint8_t dynamixel_ready=0;
  uint8_t dynamixel_init_position=0;
  uint8_t limits=1;
  uint8_t end_process=0;
  uint8_t shared_control=0; //set to 1 to enable shared_Control
  uint8_t operation_start=0; //used to depict that robot is initialized and in operation-ready pos
  uint8_t executed_command=0;
  uint8_t gui_command=0;
  
  uint16_t rms_current[8]={0,0,0,0,0,0,0,0};
  uint16_t rms_current_prev[8]={0,0,0,0,0,0,0,0};
  
  char kb_input;
  cVector3d pivot_point(0.0,0.0,0.0);
  cMatrix3d pivot_pose;
  double intraoperative_scaling=6.0; //amount of (down)-scaling from falcon workspace to robot. Set <1 to exceed falcon pos limits 
  double pivot_angle_lims[4]={-DOF3_ANGLE_LIMIT*C_DEG2RAD,DOF3_ANGLE_LIMIT*C_DEG2RAD,-2*DOF4_ANGLE_LIMIT*C_DEG2RAD,(DOF4_ANGLE_LIMIT/2.0)*C_DEG2RAD};
  //NOVINT FALCON STUFF
  // a haptic device handler
  cHapticDeviceHandler* handler;
  // a pointer to the current haptic device
  cGenericHapticDevicePtr hapticDevice;
  //Get device specs
  cHapticDeviceInfo hapticDeviceInfo;
  //A global variable to store the position [m] of the haptic device
  cVector3d hapticDevicePosition_prev;
  cVector3d hapticDevicePosition;
  //Global variables for buttons
  unsigned int userSwitches;
  bool buttons[4];
  bool *button_up=&buttons[2];
  bool *button_down=&buttons[0];
  bool *button_left=&buttons[1];
  bool *button_right=&buttons[3];
  bool button_value_change;
  
  //force field power
  double Kp=25;
  //damping term power
  double Kv=1.0;
  //specify falcon falcon_pos_tolerance
  double falcon_pos_tolerance=0.001;
  
  double aruco_position[3]={0.0,0.0,0.0};
  
  std_msgs::UInt32MultiArray dynamixel_array;//=new std_msgs::UInt32MultiArray[1];
  std_msgs::Float32MultiArray robot_position;//=new std_msgs::UInt32MultiArray[1];
  std_msgs::UInt8MultiArray board_feedback;
  std_msgs::UInt8MultiArray board_cmd; 
  
  int getch()
  {
    #ifdef __linux__
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
    #elif defined(_WIN32) || defined(_WIN64)
    return _getch();
    #endif
  }
  
  
  int kbhit(char* c)
  {
    #ifdef __linux__
    struct termios oldt, newt;
    int ch;
    int oldf;
    
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    
    ch = getchar();
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    
    if (ch != EOF)
    {
      //ungetc(ch, stdin);
      *c=ch;
      return 1;
    }
    
    return 0;
    #elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
    #endif
  }
  
  void help_menu(){
    ROS_INFO("Press button... for...."); 
  }
  
  void dxlCallback(const std_msgs::UInt32MultiArray::ConstPtr& dxl_in){
    switch((int) dxl_in->data[0]){
      case	DYNAMIXEL_READY:{
	dynamixel_ready=1;
	ROS_INFO("dxl_initialized");
	break;
      }
      case	DYNAMIXEL_INIT_POSITION:{
	dynamixel_init_position=1;
	ROS_INFO("robot in initial position");
	break;
      }
      case	DYNAMIXEL_COMPUTE_LIMITS:{
	ROS_INFO("Limits data changed");
	y_motion=(dxl_in->data[1]/4096.0)*T8pitch*4.0;
	z_motion=(dxl_in->data[2]/4096.0)*T8pitch*4.0;
	cart_motion=(dxl_in->data[3]/4096.0)*2.0*C_PI*BeltRadius;
	limits=1;
	break;
      }
      case	DYNAMIXEL_GO_TO_POS_NOTIME:{
	if(operation_start!=1){
	  operation_start=1; 
	  ROS_INFO("operation_start position achieved");
	}
	else{
	  ROS_INFO("GO TO POS NO TIME achieved");
	}
	
	break;
      }
      case DYNAMIXEL_GO_TO_POS:{
	if(executed_command==16){
	  executed_command=0; 
	  ROS_INFO("Reached POS point");
	}
	if(executed_command==24) executed_command=73;
	break;
      }
      case DYNAMIXEL_STATUS:{
	//calculate position using fkine by converting position to DH params
	double input[5];
	for(uint8_t i=0;i<5;i++){input[i]=dxl_in->data[i+1];}
	double dH_temp[9]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	DH_from_DXL_calculation(input,dH_temp);
	for(uint8_t i=5;i<9;i++)
	  dH_temp[i]=0;
	dH_temp[5]-=C_PI_DIV_2;
	DH_change(DH,dH_temp);
	//print_DH(DH);
	constructTransMat(&single_transforms,DH);
	computeFKine(&single_transforms,&base_transforms,DH);
	///////////////////////////////////////////////////////////////////////////////////////////////
	//for aruco verification
	//ROS_INFO("transform is:");
	///print_transform(&base_transforms,5);
	//
	//for aruco case
	robot_position.data.push_back(ROBOT_FEEDBACK);
	robot_position.data.push_back(base_transforms.trans_mat[DOF-2].x());
	robot_position.data.push_back(base_transforms.trans_mat[DOF-2].y());
	robot_position.data.push_back(base_transforms.trans_mat[DOF-2].z());
	robot_position.data.push_back(aruco_position[0]);
	robot_position.data.push_back(aruco_position[1]);
	robot_position.data.push_back(aruco_position[2]);
	if(kb_input==81){//piv point must be assigned
	  //assuming pivot point is when robot EE has reached it
	  pivot_point.set(base_transforms.trans_mat[DOF-1].x(),base_transforms.trans_mat[DOF-1].y(),base_transforms.trans_mat[DOF-1].z());
	  pivot_pose.copyfrom(base_transforms.rot_mat[DOF-1]);
	  ROS_INFO("Robot new pivot point is: %f %f %f",pivot_point.x(),pivot_point.y(),pivot_point.z());
	  //ROS_INFO("Input was: %d %d %d %d %d",dxl_in->data[1],dxl_in->data[2],dxl_in->data[3],dxl_in->data[4],dxl_in->data[5]);
	  ROS_INFO("Pivot DH is:");
	  print_DH(DH);
	  memcpy(operation_start_DH,dH_temp,9*sizeof(operation_start_DH[0])); //assign DH of pivot point
	  pivot_angle_lims[0]=-DOF3_ANGLE_LIMIT*C_DEG2RAD;
	  pivot_angle_lims[1]=-pivot_angle_lims[0];
	  pivot_angle_lims[2]=-2*DOF4_ANGLE_LIMIT*C_DEG2RAD;
	  pivot_angle_lims[3]=(DOF4_ANGLE_LIMIT/2.0)*C_DEG2RAD;
	  //computePivotLims(operation_start_DH,pivot_angle_lims,0.1,pivot_point);
	  executed_command=0;
	  kb_input=0;
	  ROS_INFO("%f %f %f %f",pivot_angle_lims[0],pivot_angle_lims[1],pivot_angle_lims[2],pivot_angle_lims[3]);
	}
	break;
      }
      default:{
	break;
      }
    }
  }
  
  void guiCallback(const std_msgs::UInt32MultiArray::ConstPtr& dxl_in){
    gui_command=true;
    switch(dxl_in->data[0]){
      case MAIN_SWITCH:{
	//open main switch in board
	break;
      }
      case SMA_CONTROLLER_TYPE:{
	//change controller for SMA (ONOFF,PID,SWITCHING MODE)
	if(operation_start==1){
	  
	}
	break;
      }
      case SMA_MANUAL_CONTROL:{
	//let user operate on SMA
	break;
      }
      case INTRAOPERATIVE:{
	//switch to intraoperative operation
	if(operation_start==1){
	  gui_command=73;
	}
	break;
      }
      case EXTRAOPERATIVE:{
	//switch to extraoperative operation
	if(operation_start==1){
	  gui_command=77;
	}
	break;
      }
      case GO_TO_PIVOT:{
	//go to pivot point position - remember to distinguish between extra and intraoperative
	//always assume intraoperative operation to avoid hurting patient here!
	gui_command=6;
	break;
      }
      case INIT_POSITION:{
	//go to init position (limit switches) - SMA must not hold current
	gui_command=INIT_POSITION;
	break;
      }
      case SHUTDOWN:{
	//shutdown operation 
	gui_command=81;
	break;
      }
      case HALT:{
	//stop at current position - SMAs must hold current
	ROS_INFO("halt received!");
	gui_command=27; //order dxl to halt!
	break;
      }
      default:{
	gui_command=0;
	break;
      }
      
    }
  }
  
  void arucoCallback(const std_msgs::Float32MultiArray::ConstPtr& aruco_in){
    aruco_position[0]=aruco_in->data[0];
    aruco_position[1]=aruco_in->data[1];
    aruco_position[2]=aruco_in->data[2];
  }
  
  void boardCallback(const std_msgs::UInt8MultiArray& board_in){
    //ROS_INFO("board callback");
    // for(uint8_t i=0;i<104;i++){
    //   board_data[i]=board_in.data[i];
    // }
    memcpy(&smaState,&board_in.data[0],104);
    //ROS_INFO("POT 7 %d",smaState.ADCV[5]);
  }
  
  int main(int argc, char** argv){
    //ROS FIRST
    ros::init(argc,argv,"controller");
    sleep(0.5);
    ros::NodeHandle dxl_comm;
    //INIT DXL PUBLISHING FUNCTION
    ros::Publisher dynamixel_pub=dxl_comm.advertise<std_msgs::UInt32MultiArray>("dxl_from_ctrl", 2);
    // while(dynamixel_pub.getNumSubscribers()<1){
    //   	ros::spinOnce();
    //	ros::Duration(s_time).sleep();
    //  }
    ros::Subscriber dynamixel_sub=dxl_comm.subscribe("dxl_2ctrl",2,dxlCallback);
    sleep(0.5);
    ros::NodeHandle gui_comm;
    ros::Publisher gui_pub=gui_comm.advertise<std_msgs::Float32MultiArray>("robot_info",1);
    sleep(0.5);
    ros::Subscriber gui_sub=gui_comm.subscribe("user_commands",1,guiCallback);
    sleep(0.5);
    ros::NodeHandle aruco_comm;
    ros::Subscriber aruco_sub=aruco_comm.subscribe("aruco_info",1,arucoCallback);
    sleep(0.5);
    ros::NodeHandle serial_board;
    ros::Publisher serial_pub=serial_board.advertise<std_msgs::UInt8MultiArray>("ctrl2board",1);
    ros::Subscriber serial_sub=serial_board.subscribe("board2ctrl",1,boardCallback);
    sleep(0.5);
    //rate
    ros::Rate loop_rate(RATE);
    //////Programma plaketas 1////////////////////////////////////////////////////////////
    ros::Time reference_time=ros::Time::now();
    ros::Time curr_time;
    double elapsed_timer=0.0;
    uint16_t val=0;
    uint16_t val_previous=0;
    ROS_INFO("reach here");
    while(ros::ok()){
      board_cmd.data.clear();
      //Experiment 1
      //Get hysteretic loop from SMA channel 8
      curr_time=ros::Time::now();
      elapsed_timer+=1000.0*(curr_time.toSec()-reference_time.toSec());
      // ROS_INFO("%f",elapsed_timer);
      reference_time=curr_time;
      for(uint8_t i=0;i<8;i++)
	rms_current[i]=50;
      if(elapsed_timer<1000.0)
	rms_current[7]=1200;
      else if(elapsed_timer<1750.0){
	rms_current[2]=1200;
      rms_current[7]=1200;
      }
      else if(elapsed_timer<4000.0)
	rms_current[5]=1200;
      else if(elapsed_timer<4750.0){
	rms_current[5]=1200;
	rms_current[4]=1200;
      }
      else {
	elapsed_timer=0;
      }
      //ROS_INFO("%d",val);
      bool write=false;
      for(uint8_t i=0;i<8;i++){
	if(rms_current[i]!=rms_current_prev[i]) write=true;
	rms_current_prev[i]=rms_current[i];
      }
      if(write){
	ROS_INFO("write");
	board_cmd.data.push_back(33);
	board_cmd.data.push_back(19);
	board_cmd.data.push_back(cmdIm);
	uint8_t * ptr=(uint8_t*) &rms_current[0];
	for(uint8_t i=0;i<16;i++) board_cmd.data.push_back(*ptr++);
	serial_pub.publish(board_cmd);
      } 
      //ROS_INFO("spin once");
      ros::spinOnce();
      //	    ROS_INFO("spin twice");
      loop_rate.sleep();
    }
    return -1;
    
    
    //-----------------------------------------------------------------------------
    //Init DH params and transformation matrices
    DH_init(DH);
    for(uint8_t i=0;i<DOF;i++){
      single_transforms.rot_mat[i].identity(); 
      single_transforms.trans_mat[i].zero();
      base_transforms.rot_mat[i].identity();
      base_transforms.trans_mat[i].zero();
    }
    //------------------------------------------------------------------------------
    //computeFKine(&single_transforms,&base_transforms,DH); //computes both transformation matrices and forward kinematics w.r.t. to base
    //print_transform(base_transforms,DOF-1);
    //for(uint8_t i=0;i<DOF;i++)print_transform(&single_transforms,i);
    //print_transform(&base_transforms,DOF-1);
    //Set DXL to initial position
    ROS_INFO("Waiting for dynamixel to be ready");
    while(ros::ok() && dynamixel_ready!=1){
      loop_rate.sleep();
      ros::spinOnce();
    }
    if(dynamixel_init_position!=1){
      ROS_INFO("Commanding dynamixel to go to initial position");
      dynamixel_array.data.push_back((uint32_t) DYNAMIXEL_INIT_POSITION);
      ros::spinOnce();
      loop_rate.sleep();
      dynamixel_pub.publish(dynamixel_array);
      while(ros::ok() && dynamixel_init_position!=1){
	loop_rate.sleep();
	ros::spinOnce();
	//dynamixel_init_position=1;
      }
    }
    //Check limits
    dynamixel_array.data.clear();
    if(limits!=1){
      ROS_INFO("Checking limits....");
      dynamixel_array.data.push_back((uint32_t) DYNAMIXEL_COMPUTE_LIMITS);
      dynamixel_pub.publish(dynamixel_array);
      while(ros::ok() && limits!=1){
	loop_rate.sleep();
	ros::spinOnce();
      }
    }
    //Set robot to operation start position - set your values here
    ROS_INFO("Moving to operation start");
    DH_change(DH,operation_start_DH);
    computeFKine(&single_transforms,&base_transforms,DH);
    //print_transform(&base_transforms,6);
    PositionCalculation(dxl_command_matrix,DH);
    dynamixel_array.data.clear();
    dynamixel_array.data.push_back((uint32_t) DYNAMIXEL_GO_TO_POS_NOTIME);
    for (uint8_t i=0;i<4;i++){
      dynamixel_array.data.push_back(dxl_command_matrix[i]);
    }
    dynamixel_array.data.push_back((uint32_t ) 0);
    dynamixel_pub.publish(dynamixel_array);
    while(ros::ok() && operation_start!=1){
      ros::spinOnce();
      loop_rate.sleep();
      //operation_start=1;
    }
    operation_start=0;
    ROS_INFO("Applying insertion position");
    dynamixel_array.data.clear();
    dynamixel_array.data.push_back((uint32_t) DYNAMIXEL_GO_TO_POS_NOTIME);
    for (uint8_t i=0;i<5;i++){
      dynamixel_array.data.push_back(dxl_command_matrix[i]);
    }
    dynamixel_pub.publish(dynamixel_array);
    while(ros::ok() && operation_start!=1){
      //operation_start=1;
      ros::spinOnce();
      loop_rate.sleep();
    }
    //To assign current position as first pivot point (if operator does not use pivot point manual)
    executed_command=80;
    kb_input=80;
    ROS_INFO("Robot must be to initial position. If not press m to switch to manual motion (see help for additional functionalities)");
    //Starting Falcon control
    // create a haptic device handler
    handler = new cHapticDeviceHandler();
    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);
    // open a connection to haptic device
    hapticDevice->open();
    // calibrate device (if necessary)
    hapticDevice->calibrate();
    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);
    //Get cHapticDeviceInfo
    hapticDeviceInfo=hapticDevice->getSpecifications();
    //Get first position
    updateHaptics();
    //hapticDevice->getPosition(hapticDevicePosition);
    //hapticDevicePosition_prev.copyfrom(hapticDevicePosition);
    ROS_INFO("Falcon ready.");
    operation_start=1;
    robot_position.data.clear();
    robot_position.data.push_back(ROBOT_READY);
    gui_pub.publish(robot_position);
    loop_rate.sleep();
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO("GUI INFORMED");
    robot_position.data.clear();
    //INFORM GUI ABOUT ROBOT READINESS
    //     char j=0;
    //     while(j!=106){
    //       if(kbhit(&j)==1){
    // 	robot_position.data.clear();
    // 	robot_position.data.push_back(ROBOT_READY);
    // 	gui_pub.publish(robot_position);
    //       }
    //       ros::spinOnce();
    //       loop_rate.sleep();
    //     }
    //-------------------------------------------------------------------------------
    while(ros::ok() && end_process==0){
      dynamixel_array.data.clear();
      updateHaptics();
      //CHECK FOR USER KEYBOARD INPUT
      if(kbhit(&kb_input)==1 || gui_command!=0){
	if(gui_command!=0) kb_input=gui_command;
	if(kb_input==109) kb_input=77; //m=M (extraoperative manual)
	else if(kb_input==113) kb_input=81; //q=Q(quit)
	else if(kb_input==104) kb_input=72; //h=H(help)
	else if(kb_input==112) kb_input=80; //p=P(get pivot point)
	else if(kb_input==105) kb_input=73; //i=I(intraoperative)
	if(executed_command==77 && kb_input==27){executed_command=80; kb_input=80;}
	else executed_command=kb_input;
	gui_command=0;
	switch(executed_command){
	  case 27:{
	    break;
	  }
	  case 72:{break;}//help menu
	  case 77:{ROS_INFO("Extraoperative mode ON. Move robot to pinhole!");break;}
	  case 81:{ROS_INFO("Quit pressed. Exiting..."); break;}
	  case 80:{
	    break;
	  }
	  case 73:{
	    //intraoperative mode
	    ROS_INFO("Intraoperative mode ON");
	    ROS_INFO("Move falcon fully out to initiate intraoperative operation");
	    updateHaptics();
	    while(hapticDevicePosition.x()<0.035 || abs(hapticDevicePosition.y())>0.01 || abs(hapticDevicePosition.z())>0.01){
	      updateHaptics();  
	      hapticDevicePosition_prev.set(hapticDevicePosition.x(),hapticDevicePosition.y(),hapticDevicePosition.z());
	      //ROS_INFO("%f \t %f \t %f",hapticDevicePosition.x(),hapticDevicePosition.y(),hapticDevicePosition.z());
	      ros::spinOnce();
	      loop_rate.sleep();  
	    }
	    hapticDevicePosition_prev.set(hapticDevicePosition.x(),hapticDevicePosition.y(),hapticDevicePosition.z());
	    break;
	  }
	  default: break;
	}
      }
      switch(executed_command){
	case 77:{//EXTRAOPERATIVE MANUAL OPERATION
	  bool write[3]={false,false,false};
	  if(abs(hapticDevicePosition.x()-hapticDevicePosition_prev.x())>falcon_pos_tolerance){
	    write[0]=true;
	    hapticDevicePosition_prev.set(hapticDevicePosition.x(),hapticDevicePosition_prev.y(),hapticDevicePosition_prev.z());
	  }
	  if(abs(hapticDevicePosition.y()-hapticDevicePosition_prev.y())>falcon_pos_tolerance){
	    write[1]=true;
	    hapticDevicePosition_prev.set(hapticDevicePosition_prev.x(),hapticDevicePosition.y(),hapticDevicePosition_prev.z());
	  }
	  if(abs(hapticDevicePosition.z()-hapticDevicePosition_prev.z())>falcon_pos_tolerance){
	    write[2]=true;
	    hapticDevicePosition_prev.set(hapticDevicePosition_prev.x(),hapticDevicePosition_prev.y(),hapticDevicePosition.z());
	  }
	  if( write[0]|| write[1]|| write[2] || button_value_change){
	    //Robot operates in manual mode
	    dynamixel_array.data.push_back((uint32_t) DYNAMIXEL_MANUAL_EXTRAOPERATIVE_OPERATION);
	    //1
	    if(*button_left && *button_right) dynamixel_array.data.push_back((uint32_t) 0); //to avoid assholes
	    else if(*button_left)dynamixel_array.data.push_back((uint32_t) 1);
	    else if(*button_right)dynamixel_array.data.push_back((uint32_t) 2);
	    else dynamixel_array.data.push_back((uint32_t) 0);
	    //2
	    if(*button_up && *button_down) dynamixel_array.data.push_back((uint32_t) 0); //to avoid assholes
	    else if(*button_up) dynamixel_array.data.push_back((uint32_t) 1);
	    else if(*button_down)dynamixel_array.data.push_back((uint32_t) 2);
	    else dynamixel_array.data.push_back((uint32_t) 0);
	    //3
	    if(write[1]) dynamixel_array.data.push_back((uint32_t) roundf(2048.0+(4096.0*operation_start_DH[2]/(2.0*C_PI))-((hapticDevicePosition.y()/0.06)*(DOF3_ANGLE_LIMIT*4096.0/360.0))+0.5));
	    else dynamixel_array.data.push_back((uint32_t) 0);
	    //4
	    uint32_t th_4=(uint32_t) roundf(2048.0+(4096.0*operation_start_DH[3]/(2.0*C_PI))+((hapticDevicePosition.z()/0.06)*(DOF4_ANGLE_LIMIT*4096.0/360.0))+0.5);
	    if(write[2]){
	      if(th_4>2048) th_4=2048;
	      else if(th_4<1024) th_4=1024;
	      dynamixel_array.data.push_back(th_4);
	    }
	    else dynamixel_array.data.push_back((uint32_t) 0);
	    //5
	    //if(write[0]) dynamixel_array.data.push_back((uint32_t) trnslate2rotationbitsBELT(hapticDevicePosition.x()*1000+operation_start_pos[4]+cart_motion/2.0));
	    //else dynamixel_array.data.push_back((uint32_t) 0);
	    if(abs(hapticDevicePosition.x())>0.005){
	      if(hapticDevicePosition.x()<0.0) dynamixel_array.data.push_back((uint32_t) roundf(1024+(abs(hapticDevicePosition.x())/0.06)*320+0.5));
	      else dynamixel_array.data.push_back((uint32_t) roundf((abs(hapticDevicePosition.x())/0.06)*320+0.5));  
	    }
	    else dynamixel_array.data.push_back((uint32_t) 0);
	    dynamixel_pub.publish(dynamixel_array);
	  }
	  break;
	}
	case 72:{
	  help_menu();
	  break;
	}
	case 73:{
	  //falcon motion is translated to robot motion w.r.t. pivot point (see my notes)
	  bool write[3]={false,false,false};
	  if(abs(hapticDevicePosition.x()-hapticDevicePosition_prev.x())>falcon_pos_tolerance){
	    write[0]=true;
	    hapticDevicePosition_prev.set(hapticDevicePosition.x(),hapticDevicePosition_prev.y(),hapticDevicePosition_prev.z());
	  }
	  if(abs(hapticDevicePosition.y()-hapticDevicePosition_prev.y())>falcon_pos_tolerance){
	    write[1]=true;
	    hapticDevicePosition_prev.set(hapticDevicePosition_prev.x(),hapticDevicePosition.y(),hapticDevicePosition_prev.z());
	  }
	  if(abs(hapticDevicePosition.z()-hapticDevicePosition_prev.z())>falcon_pos_tolerance){
	    write[2]=true;
	    hapticDevicePosition_prev.set(hapticDevicePosition_prev.x(),hapticDevicePosition_prev.y(),hapticDevicePosition.z());
	  }
	  if(write[0]==true || button_value_change){
	    cVector3d scaled_pos(hapticDevicePosition);
	    ROS_INFO("scaled pos x %f %f",scaled_pos.x(),hapticDevicePosition.x());
	    scaled_pos.x(0.042-scaled_pos.x()); //from meters to max motion input scaling
	    if(scaled_pos.x()<0.0) scaled_pos.x(0.0);
	    double dh_tmp[5]={0.0,0.0,0.0,0.0,0.0};
	    dh_tmp[4]=scaled_pos.x()*1000.0; //to mm
	    ROS_INFO("new insertion %f ",dh_tmp[4]);
	    double dh_prev[5];
	    dh_prev[0]=DH[1][0];
	    dh_prev[1]=DH[2][0]-((349.0-DH[0][0])-(z_motion/2.0));
	    dh_prev[2]=DH[3][3];
	    dh_prev[3]=DH[4][3]-C_PI_DIV_2;
	    dh_prev[4]=DH[5][0];
	    //theta 1
	    dh_tmp[2]=dh_prev[2];
	    ROS_INFO("d2 d3 %f %f",dh_prev[2],dh_prev[3]);
	    if(*button_left && !*button_right) dh_tmp[2]+=10*0.088*C_DEG2RAD;
	    else if(*button_right) dh_tmp[2]-=10*0.088*C_DEG2RAD;
	    if(dh_tmp[2]>pivot_angle_lims[1]) dh_tmp[2]=pivot_angle_lims[1];
	    else if(dh_tmp[2]<pivot_angle_lims[0]) dh_tmp[2]=pivot_angle_lims[0]; 
	    dh_tmp[3]=dh_prev[3];
	    if(*button_up && !*button_down) dh_tmp[3]+=10*0.088*C_DEG2RAD;
	    else if(*button_down) dh_tmp[3]-=10*0.088*C_DEG2RAD;
	    if(dh_tmp[3]>pivot_angle_lims[3]) dh_tmp[3]=pivot_angle_lims[3];
	    else if(dh_tmp[3]<pivot_angle_lims[2]) dh_tmp[3]=pivot_angle_lims[2];
	    robot_RCM2(dh_tmp,pivot_point);
	    dh_tmp[1]-=z_motion/2.0;
	    ROS_INFO("LIms %f %f %f %f",pivot_angle_lims[0],pivot_angle_lims[1],pivot_angle_lims[2],pivot_angle_lims[3]);
	    ROS_INFO("P %f \t %f \t %f",pivot_point.x(),pivot_point.y(),pivot_point.z());
	    ROS_INFO("D %f \t %f \t %f \t %f \t %f",dh_tmp[0],dh_tmp[1],dh_tmp[2]*C_RAD2DEG,dh_tmp[3]*C_RAD2DEG,dh_tmp[4]);
	    //dh_tmp[1] is motion from top so convert it w.r.t. to 0 position
	    double dH[9]={dh_tmp[0],dh_tmp[1],dh_tmp[2],dh_tmp[3],dh_tmp[4],DH[5][3],DH[6][3],DH[7][3],DH[8][3]};
	    DH_change(DH,dH);
	    //computeFKine(&single_transforms,&base_transforms,DH);
	    PositionCalculation(dxl_command_matrix,DH);
	    ROS_INFO("cmd %d \t %d \t %d \t %d \t %d",dxl_command_matrix[0],dxl_command_matrix[1],dxl_command_matrix[2],dxl_command_matrix[3],dxl_command_matrix[4]);
	    double dH2[9]={dh_prev[0],dh_prev[1],dh_prev[2],dh_prev[3],dh_prev[4],DH[5][3],DH[6][3],DH[7][3],DH[8][3]};
	    DH_change(DH,dH2);
	    dynamixel_array.data.push_back((uint32_t) DYNAMIXEL_GO_TO_POS);
	    for (uint8_t i=0;i<5;i++){
	      dynamixel_array.data.push_back(dxl_command_matrix[i]);
	    }
	    dynamixel_pub.publish(dynamixel_array);
	    executed_command=24;
	  }
	  //compute the new workspace based on falcon position
	  // 	  cVector3d scaled_pos(hapticDevicePosition);
	  // 	  scaled_pos.x(0.042-scaled_pos.x());
	  // 	  if(scaled_pos.x()<0.0) scaled_pos.x(0.0);
	  // 	  scaled_pos.y(-scaled_pos.y()/2.0);
	  // 	  scaled_pos.z(scaled_pos.z()/2.0);
	  // 	  //scaled_pos.mul(1000.0/intraoperative_scaling);
	  // 	  ROS_INFO("SPOSx %f %f %f",scaled_pos.x(),scaled_pos.y(),scaled_pos.z());
	  // 	  //invert x,y because falcon axes x and y are opossite to my frame system
	  // 	  //cVector3d target_pos=(pivot_pose*scaled_pos)+pivot_point;
	  // 	  double dh_tmp[5];
	  // 	  dh_tmp[2]=operation_start_DH[2];
	  // 	  if(scaled_pos.y()>=0) dh_tmp[2]+=(abs(scaled_pos.y()/0.06))*(pivot_angle_lims[1]-operation_start_DH[2]);
	  // 	  else dh_tmp[2]+=(abs(scaled_pos.y()/0.06))*(pivot_angle_lims[0]-operation_start_DH[2]);
	  // 	  dh_tmp[3]=operation_start_DH[3];
	  // 	  if(scaled_pos.z()>=0) dh_tmp[3]+=(abs(scaled_pos.z()/0.06))*(pivot_angle_lims[3]-operation_start_DH[3]);
	  // 	  else dh_tmp[3]+=(abs(scaled_pos.z()/0.06))*(pivot_angle_lims[2]-operation_start_DH[3]);
	  // 	  robot_RCM2(dh_tmp,pivot_point);
	  // 	  dh_tmp[4]=scaled_pos.x()/0.12;
	  // 	  if(dh_tmp[4]<0) dh_tmp[4]=0;
	  // 	  //Compute kinematics mathematically
	  // 	  robot_RCM2(dh_tmp,pivot_point);
	  // 	  //robot_RCM(pivot_point,target_pos,dH_temp);
	  // 	  //ROS_INFO("F %f \t %f \t %f",hapticDevicePosition.x(),hapticDevicePosition.y(),hapticDevicePosition.z());
	  // 	  //ROS_INFO("W %f \t %f \t %f",target_pos.x(),target_pos.y(),target_pos.z());
	  // 	  //ROS_INFO("P %f \t %f \t %f",pivot_point.x(),pivot_point.y(),pivot_point.z());
	  // 	  //ROS_INFO("D %f \t %f \t %f \t %f \t %f",dh_tmp[0],dh_tmp[1],dh_tmp[2]*C_RAD2DEG,dh_tmp[3]*C_RAD2DEG,dh_tmp[4]);
	  // 	  DH_change(DH,dh_tmp[0],dh_tmp[1],dh_tmp[2],dh_tmp[3],dh_tmp[4]);
	  // 	  //computeFKine(&single_transforms,&base_transforms,DH);
	  // 	  PositionCalculation(dxl_command_matrix,DH);
	  // 	  dynamixel_array.data.push_back((uint32_t) DYNAMIXEL_GO_TO_POS);
	  // 	  for (uint8_t i=0;i<5;i++){
	  // 		  dynamixel_array.data.push_back(dxl_command_matrix[i]);
	  // 	  }
	  // 	  dynamixel_pub.publish(dynamixel_array);
	  break;
	}
	case 24:{
	  //dummy case forcase 73 to wait until complete
	  break;
	}
	case 6:{ //go to pivot point using intraoperative mode
	  double dH[9]={operation_start_DH[0],operation_start_DH[1],operation_start_DH[2],operation_start_DH[3],operation_start_DH[4],DH[5][3],DH[6][3],DH[7][3],DH[8][3]};
	  DH_change(DH,dH);
	  //computeFKine(&single_transforms,&base_transforms,DH);
	  PositionCalculation(dxl_command_matrix,DH);
	  dynamixel_array.data.push_back((uint32_t) DYNAMIXEL_GO_TO_POS);
	  for (uint8_t i=0;i<5;i++){
	    dynamixel_array.data.push_back(dxl_command_matrix[i]);
	  }
	  dynamixel_pub.publish(dynamixel_array);
	  executed_command=16;
	  break;
	}
	case 16:{
	  //similar to case 6 but waitin to go to pivot point (see callback for case 16)
	  break;
	}
	case 81:{
	  end_process=1;
	  kb_input=0;
	  break;
	}
	case 80:{
	  if(kb_input=80){
	    dynamixel_array.data.push_back((uint32_t) DYNAMIXEL_STATUS);
	    dynamixel_pub.publish(dynamixel_array);
	    executed_command=0;
	    kb_input=81;
	  }
	  else{
	    //dynamixel always sends position if not ask here for position at your desired intervals
	  }
	  break;
	}
	case 4:{
	  ROS_INFO("go to init position called!");
	  dynamixel_array.data.push_back((uint32_t) INIT_POSITION);
	  dynamixel_pub.publish(dynamixel_array);
	  executed_command=0;
	}
	case 27:{
	  ROS_INFO("Escape pressed.Cancelling last operation mode. Chose another one from menu");
	  //must also halt robot here
	  executed_command=0;
	  break;
	}
	default:{
	  executed_command=0; 
	  break;
	}
      }
      if(robot_position.data.size()>0){
	//ROS_INFO("%f %f %f",base_transforms.trans_mat[DOF-2].x(),
	//base_transforms.trans_mat[DOF-2].y(),base_transforms.trans_mat[DOF-2].z());
	gui_pub.publish(robot_position);
	robot_position.data.clear();
      }
      ros::spinOnce();
      loop_rate.sleep();
    } //closing while ros::ok()
    //close dynamixel
    dynamixel_array.data.clear();
    dynamixel_array.data.push_back((uint32_t) DYNAMIXEL_END_PROCESS);
    dynamixel_pub.publish(dynamixel_array);
    dynamixel_array.data.clear();
    ros::spinOnce();
    loop_rate.sleep();
    hapticDevice->close();
    ROS_INFO("exit");
    return 0;
  }//closing main
  
  void PositionCalculation(uint32_t* command_matrix, double DH[][4]){
    //command matrix computes dxl motion commands in bits TAKING INTO ACCOUNT THE START POSITIONS OF THE DYNAMIXELS!
    command_matrix[0]=trnslate2rotationbits(DH[0][0]+y_motion/2.0);
    //command_matrix[1]=trnslate2rotationbits(z_motion/2.0-DH[2][0]-178.5);//144 due to standar addition param in DH[2][0]
    //command_matrix[1]=trnslate2rotationbits(378.5-DH[1][0]); //(z_motion/2-D2)
    command_matrix[1]=trnslate2rotationbits(z_motion-DH[1][0]); //(z_motion/2-D2)
    command_matrix[2]=uint32_t (roundf((4096.0*(DH[2][3])/(2.0*C_PI))+2048.0+0.5));
    command_matrix[3]=uint32_t (roundf((4096.0*(DH[3][3]-C_PI_DIV_2)/(2.0*C_PI))+2048.0+0.5));
    command_matrix[4]=trnslate2rotationbitsBELT(DH[4][0]+cart_motion/2.0);
  }
  
  void DH_from_DXL_calculation(double* DXL_Feedback,double* DH_val){
    DH_val[0]=(DXL_Feedback[0]*T8pitch*4.0/4096.0)-y_motion/2.0;
    //DH_val[1]=-(DXL_Feedback[1]*T8pitch*4.0/4096.0)+z_motion/2.0+144.0;
    //DH_val[1]=(349.0-DH[0][0])-(DXL_Feedback[1]*T8pitch*4.0/4096.0); //motion difference from midpoint cause full utilization is done in DH_change
    //DH_val[1]=-(DXL_Feedback[1]*T8pitch*4.0/4096.0)+z_motion/2.0; //motion difference from midpoint cause full utilization is done in DH_change
    DH_val[1]=d2_max-(DXL_Feedback[1]*T8pitch*4.0/4096.0);
    DH_val[2]=((DXL_Feedback[2]-2048.0)*2.0*C_PI)/4096.0;
    //DH_val[3]=((2048.0-DXL_Feedback[3])*2.0*C_PI/4096.0)+C_PI_DIV_2;
    DH_val[3]=((DXL_Feedback[3]-2048.0)*2.0*C_PI)/4096.0;
    //DH_val[4]=(DXL_Feedback[4]*2.0*C_PI*BeltRadius/4096.0)-cart_motion/2.0;
    DH_val[4]=(DXL_Feedback[4]*2.0*C_PI*BeltRadius/4096.0)-cart_motion/2.0;
  }
  
  void updateHaptics(void){
    hapticDevice->getPosition(hapticDevicePosition);
    bool buttons_prev[4];
    for(uint8_t i=0;i<4;i++) buttons_prev[i]=buttons[i];
    if(hapticDevice->getUserSwitches(userSwitches)){
      for(uint8_t i=0;i<4;i++) buttons[i]=cCheckBit(userSwitches,i);
    }
    button_value_change=false;
    for(uint8_t i=0;i<4;i++){
      if(buttons_prev[i]!=buttons[i]) button_value_change=true;
    }
    cVector3d force(0,0,0);
    cVector3d desiredPosition;
    desiredPosition.set(0.0,0.0,0.0);
    if(useForceField){
      cVector3d forceField=Kp* (desiredPosition-hapticDevicePosition);
      force.add(forceField);
    }
    if(useDamping){
      cVector3d forceDamping;
      hapticDevice->getLinearVelocity(forceDamping);
      forceDamping*=-Kv*hapticDeviceInfo.m_maxLinearDamping;
      force.add(forceDamping);
    }
    hapticDevice->setForce(force);
  }
  