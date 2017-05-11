#include "kinodynamics.h"
#include "chai3d.h" //for Eigen convertions and stuff
#include "GL/glew.h"
#include "GL/glut.h"

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

#define REASSIGN_PIVOT_POINT 80 // similar to P

#define MAIN_SWITCH 0
#define SMA_CONTROLLER_TYPE 2
#define SMA_MANUAL_CONTROL 3
#define INTRAOPERATIVE 4
#define EXTRAOPERATIVE 5
#define GO_TO_PIVOT 6
#define INIT_POSITION 7
#define SHUTDOWN 8
#define ROBOT_READY 9
#define ROBOT_FEEDBACK 10
#define HALT 11

//Haptics
#define useForceField true
#define useDamping true
void PositionCalculation(uint32_t* , double [][4] );
void updateHaptics(void);
void DH_from_DXL_calculation(double* ,double* );

enum ctrlFlag{
        cmdOnOff=0x00,
        cmdPWM	=0x01,
        cmdVolt	=0x02,
        cmdIm	=0x03,
        cmdIrms	=0x04,
        cmdRate =0x05
    }ctrlFlg;
 /*
    struct smaStatus{
        float AD[8];
        float ADoff[8]; //64
        float ADrms[8]; //96
        float Vbus;
        float Vrf;
        float Vbat;
        u_int32_t time; //112
        int16_t ADCV[16]; //144
    } smaState; //144bytes
    */

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