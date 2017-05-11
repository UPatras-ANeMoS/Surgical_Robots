//------------------------------------------------------------------------------
#include "chai3d.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "time.h"
#include <sstream>
#include <inttypes.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;



//------------------------------------------------------------------------------
#include "GL/glut.h"

#define RATE 1000 //Hz

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// frequency counter to measure the simulation haptic rate
//cFrequencyCounter frequencyCounter;

// cPrecision clock
//cPrecisionClock myclock;

//precision sleep
//struct timespec rqv;
//long sleep_t;


int main(int argc, char* argv[])
{
    //ROS stuff
	ros::init(argc, argv, "falcon_publisher");

    //INIT ROS HANDLER
    ros::NodeHandle f;
    //INIT PUBLISHING FUNCTION
    std_msgs::Float32MultiArray* array=new std_msgs::Float32MultiArray[3];
    ros::Publisher falcon_pub = f.advertise<std_msgs::Float32MultiArray>("falcon", 1);
    //DEFINE PROGRAM RATE
    ros::Rate loop_rate(RATE);

	//--------------------------------------------------------------------------
    // WSPACE INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "[1] - Enable/Disable potential field" << endl;
    cout << "[2] - Enable/Disable damping" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;

    //--------------------------------------------------------------------------
    // HAPTIC DEVICE - CHAI3D
    //--------------------------------------------------------------------------

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

    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts thex main haptics rendering loop
    //frequencyCounter.reset();
    //myclock.start();
    //rqv={0};
    //int y,z;
    //int y_previous,z_previous;
    //y=0;
    //z=0;
    //y_previous=0;
    //z_previous=0;
    //int diff=100;
    while(ros::ok()){
        //myclock.reset(0);
        //Clear array
        array->data.clear();
        //Get falcon pos
        hapticDevice->getPosition(hapticDevicePosition);
        //Assign
        array->data.push_back(hapticDevicePosition.x());
        array->data.push_back(hapticDevicePosition.y());
        array->data.push_back(hapticDevicePosition.z());
        //Publish
        cout<< "x "<< array->data[0] << " y "<< array->data[1] << " z "<< array->data[2] <<endl; 
        falcon_pub.publish(*array);

        //y=-(hapticDevicePosition.y()/0.05)*2000;
        //z=(hapticDevicePosition.z()/0.05)*2000;
        //cout<<"Device position \t" << hapticDevicePosition.x() << "\t" << hapticDevicePosition.y() << "\t" << hapticDevicePosition.z() << "\t" << endl;
        //cout<<"Pan Position \t" << y << "\t" << "TIlt Position \t" << z << endl;
        //if(abs(y-y_previous)>diff){
        //set_desired(PAN, POSITION, (PTU_PARM_PTR *) &y, ABSOLUTE);
        //y_previous=y;
        //}
        //if(abs(z-z_previous)>diff){
        //set_desired(TILT, POSITION, (PTU_PARM_PTR *) &z, ABSOLUTE);
        //z_previous=z;
        //}
        //usleep(20000-myclock.getCurrentTimeSeconds()*1000000); //17 milliseconds to nanoseconds and the same for Currenttime
    	//usleep(1000-myclock.getCurrentTimeSeconds()*1000000);
        ros::spinOnce();
        //rqv.tv_nsec=1000000-1000000*myclock.getCurrentTimeSeconds();
        //nanosleep(&rqv, NULL);
    	//nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);
        //Added a delay so not to spam
        loop_rate.sleep();
    }

    // exit
    hapticDevice->close();
    delete handler;
	return 0;
}

