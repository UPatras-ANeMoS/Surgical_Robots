#include "chai3d.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include <math.h>
#include <vector>
using namespace chai3d;
using namespace std;

#define T8pitch 2.0 //mm
#define BeltRadius	6.35 //6.5 mm
#define DOF 9 //number of fixed and non-fixed(aka DOF) transformation matrices

//#define Y_motion 225.0 //in mm
#define Y_motion 263.5 //in mm	
//#define Z_motion 240.0 //in mm
//#define Z_motion 195.8 //in mm
#define Z_motion 198.0 //in mm
#define UPPER_Z_MOTION 278.5//in mm
#define SHAFT_LENGTH 350 //in mm

#define DOF3_ANGLE_LIMIT 60 //+-60 degrees left right
#define DOF4_ANGLE_LIMIT 60//+-60 degrees left right

#define d1_min -131.75
#define d1_max 131.75
#define d2_min 173.5
#define d2_max 371.5
#define th3_min -C_PI_DIV_2
#define th3_max C_PI_DIV_2
#define th4_min -C_PI_DIV_2
#define th4_max 0
#define d5_min 245.15
#define d5_max 523.65

//completition order is
//D A a(alpha) theta
//all in mm and rad

struct transforms{
cMatrix3d rot_mat[DOF];
cVector3d trans_mat[DOF];
};

//constructs transformation matrices from DH params
void constructTransMat(transforms*,double [][4]);
//returns the rotate and translate matrix up to the DoF requested by the user
void computeFKine(transforms*,transforms*,double [][4]);
//computes inverse kinematics for the outer platform
void computeINVKine(transforms*);
//to do fkine and invkine for all DoF
void DH_init(double [][4]);
void DH_change(double [][4], double*);
void print_DH(double [][4]);
void print_transform(transforms* , uint8_t );
//void robot_RCM(cVector3d, cVector3d, double*);
void robot_RCM2(double*,cVector3d);
void computePivotLims(double*, double*, double, cVector3d);

inline uint32_t trnslate2rotationbits(double translate){/*1 turn is 360 degrees and 2mm pitch*/ return((uint32_t) roundf((4096.0*translate/(T8pitch*4.0))+0.5));} //to convert to degrees multiply this amount by 360.0

inline uint32_t trnslate2rotationbitsBELT(double translate){uint32_t tmp=(uint32_t) roundf(((4096.0*translate)/(2.0*C_PI*BeltRadius))+0.5); return tmp;} //to convert to degrees multiply this amount by 360.0
