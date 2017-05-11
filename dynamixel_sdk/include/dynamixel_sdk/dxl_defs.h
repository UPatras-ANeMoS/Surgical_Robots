//DXL STUFF
#include "dynamixel_sdk/dynamixel_sdk.h"                                 // Uses Dynamixel SDK library

#define DYNAMIXEL_POSITION_CONTROL 0 //PANT/TILT ACTIVE
#define DYNAMIXEL_VELOCITY_CONTROL 1
#define DYNAMIXEL_PROCEDURE_POS_CONTROL 2 //PAN/TILT INACTIVE -- ONLY ID 3 AND SMA MOVE
#define DYNAMIXEL_END_PROCESS 3
#define DYNAMIXEL_INIT_POSITION 4
#define DYNAMIXEL_GO_TO_PROCEDURE_START 5
#define DYNAMIXEL_GO_TO_POS 6
#define DYNAMIXEL_STATUS 7
#define DYNAMIXEL_READY 8
#define DYNAMIXEL_GO_TO_POS_NOTIME 9
#define DYNAMIXEL_COMPUTE_LIMITS	11
#define DYNAMIXEL_MANUAL_EXTRAOPERATIVE_OPERATION 12

#define PROCEDURE_THETA1 2048
#define PROCEDURE_THETA2 2048
#define PROCEDURE_THETA3 2048

#define DEVICENAME1                     "/dev/DXL2"      // Check which port is being used on your controller
#define DEVICENAME2                     "/dev/DXL1"      // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

// Protocol version
#define PROTOCOL_VERSION                1.0
#define BAUDRATE                        1000000

//Commands
#define P_GOAL_POSITION_L 30
#define P_GOAL_POSITION_H 31
#define P_PRESENT_POSITION_L  36
#define P_PRESENT_POSITION_H  37
#define P_MOVING    46
#define P_ACCELERATION    73
#define P_SPEED_L   32
#define P_SPEED_H   33
#define P_PRESENT_SPEED_L   38
#define P_PRESENT_SPEED_H   39
#define P_CW  6
#define P_CCW 8
#define P_P 28
#define P_I 27
#define P_D 26
#define P_TORQUE_L  34
#define P_TORQUE_H  35
#define ADDR_MX_TORQUE_ENABLE 24
#define DISABLE_TORQUE 0 //set to 1 to disable torque upon closing system

#define NUM_ACTUATORS1 2
#define NUM_ACTUATORS2 3

#define RESOLUTION_DIVIDER 2.0 //res divider for motors 1 and 2

#define DOF3_START_POS 2048 
#define DOF4_START_POS 2048//dont forget to change in controller!
//#define DOF4_OFFSET	24 //in bits from DOF4_START_POS
#define DOF3_OFFSET	0
#define DOF4_OFFSET	0
#define ROT_DOF_TOLERANCE 2

#define POSITION_CONTROL 0
#define VELOCITY_CONTROL 1
#define TORQUE_CONTROL	2

#define SPEED_QUANTIZATION 0.114 //rpm
#define SPEED64_MAX 63.0
#define SPEED28_MAX 55.0
#define SPEED_FACTOR12 0.5 //normalized max speed for motor 1,2 at start
#define SPEED_FACTOR34 0.02 //normalized max speed for motor 3,4 at start
#define SPEED_FACTOR5 0.5 //normalized max speed for motor 5 at start

#define C_PI 3.14159265358979323846


//to rs485 dxl2(deksia) sindeetai sta katw
//to rs485 dxl1(aristera) sindeetai sta panw

typedef struct{
    uint8_t ID;
    uint8_t mode; //0 position ,1 velocity, 2 torque
    uint8_t P;
    uint8_t I;
    uint8_t D;
    uint8_t acceleration;
    //Position variables
    uint16_t position_start; //initial position measurement while on limit switches
    uint32_t goal_position; //target position 
    uint32_t current_position; //current position(0-160000 cause multi-turn)
    uint16_t measured_position;	//current measured position from motor(0-4096)
    //Speed variables
    uint16_t goal_speed;//target speed from 0-1023 || 1024-2047
    uint16_t current_velocity; //current velocity measured from motor
    uint16_t current_torque;
    uint8_t moving;
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    uint8_t lim_low;
    uint8_t lim_high;
    uint8_t type;
}dynamixel_motor;

//double time2target(double , double , double , double , double , double);

