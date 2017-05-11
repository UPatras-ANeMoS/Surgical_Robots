#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include "std_msgs/Int8MultiArray.h"
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
//#include <Eigen/Dense>
#include <Eigen/Core>
//#include <Eigen/LU>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <time.h>

//#define m_size 0.0795
#define m_size 159.5

//117.28
using namespace Eigen;
using namespace aruco;
MarkerDetector MDetector, MDetector2;
MarkerPoseTracker PTracker, PTracker2;
vector<Marker> Markers,  Markers2;

bool first_detect=true;
bool imswitch=false;
CameraParameters aruco_cam_param, aruco_cam_param2;

Matrix4d base2camera, base2marker, marker2camera, camera2camera, camera2marker, marker2robot, base2robot;
std::string sep="\n--------------------------------------------------\n";
Matrix4d test_cam;



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

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  try
  {
    cv::Mat imag=cv_bridge::toCvShare(msg, "bgr8")->image;
    if(!imswitch){
       //MDetector.detect(imag,Markers,aruco_cam_param,m_size5, false);
       Markers=MDetector.detect(imag);
       //ROS_INFO("%d MARKERS",Markers.size());
       if(Markers.size()==1){
	for (unsigned int i=0;i<Markers.size();i++){
	Markers[i].draw(imag,cv::Scalar(0,0,255),2);
	//Markers[i].calculateExtrinsics (m_size5, aruco_cam_param, false);
	}
	if(PTracker.estimatePose(Markers[0],aruco_cam_param,m_size5,1)){
	  aruco::CvDrawingUtils::draw3dAxis(imag, aruco_cam_param , PTracker.getRvec(),PTracker.getTvec(),m_size5);
	  counter++;
	  ROS_INFO("%d", counter);
	  cv::Mat pose=PTracker.getRTMatrix();
	  cv::cv2eigen(pose.inv(),marker2camera);
	  base2camera=base2marker*marker2camera;
	  //cv::imshow("view", MDetector.getThresholdedImage());
	}
       }
    }
    else{
       //MDetector2.detect(imag,Markers2,aruco_cam_param2,m_size, false);
       Markers2=MDetector2.detect(imag);
       //ROS_INFO("%d MARKERS 2",Markers2.size());
       if(Markers2.size()==1){
	for (unsigned int i=0;i<Markers2.size();i++){
	Markers2[i].draw(imag,cv::Scalar(0,0,255),2);
	//Markers2[i].calculateExtrinsics (m_size, aruco_cam_param2, false);
	}
	if(PTracker2.estimatePose(Markers2[0],aruco_cam_param2,m_size,1)){
	  aruco::CvDrawingUtils::draw3dAxis(imag, aruco_cam_param2, PTracker2.getRvec(),PTracker2.getTvec(),m_size);   
	  cv::Mat pose2=PTracker2.getRTMatrix();
	  cv::cv2eigen(pose2,camera2marker);
	  base2robot=base2camera*camera2camera;
	  cout<< "B2C" << base2camera <<sep<< endl;
	  cout<<"B2C2" << base2robot <<sep <<endl;
	  base2robot=base2robot*camera2camera*camera2marker;
	  cout<<"B2R" << base2robot <<sep <<endl;
	  cout<< "POse2"<< pose2<< sep<< endl;
	  //cout<<"B2C" << base2camera<< sep<< endl;
	  //cv::imshow("view", MDetector2.getThresholdedImage());
	}
       }
    }
    if(Markers.size()>0){
      cv::imshow("view", MDetector.getThresholdedImage());
    }
    else if(Markers2.size()>0){
      cv::imshow("view", MDetector2.getThresholdedImage());
    }
    Markers.clear();
    Markers2.clear();
    //cv::imshow("view", imag);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  base2marker<< 0, 1, 0, 0.052, -1, 0, 0, 0, 0, 0, 1, 0.005, 0, 0, 0, 1;
  camera2camera<< 1, 0, 0, 0, 0, 0.866, -0.5, 0.0290885, 0, 0.5, 0.866, 0.0083301, 0, 0, 0, 1;
  marker2robot<<1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1;
  MDetector.setDictionary(Dictionary::ARUCO_MIP_36h12);
  MDetector2.setDictionary(Dictionary::ARUCO_MIP_36h12);
  //MDetector.setThresholdParams(7,7);
  //MDetector.setThresholdParamRange(2,0);
  //MDetector2.setThresholdParams(7,7);
  //MDetector2.setThresholdParamRange(2,0);
//   base2camera<< 0.99787, 0.0271947, -0.0592875, -0.0605119, 
// 	     0.0652191, -0.430121, 0.900412, -0.658758,
// 	     -0.00101441, -0.902361, -0.430979, 0.472846,
// 	     0, 0, 0, 1;
  aruco_cam_param.readFromXMLFile("/home/odroid/ros_SMArobot/src/camera_grab/yaml_files/yaml1/cam_calib_aruco_large.yml");
  //aruco_cam_param.readFromXMLFile("/home/odroid/ros_SMArobot/src/camera_grab/yaml_files/yaml1/cam_calib_aruco_large_cam_robot_v2.yml");
  aruco_cam_param2.readFromXMLFile("/home/odroid/ros_SMArobot/src/camera_grab/yaml_files/yaml1/cam_calib_aruco_large_cam_robot_v2.yml");
  //aruco_cam_param.readFromXMLFile("/home/odroid/ros_SMArobot/src/camera_grab/yaml_files/yaml1/cam_calib_aruco.yml");
  //aruco_cam_param2.readFromXMLFile("/home/odroid/ros_SMArobot/src/camera_grab/yaml_files/yaml1/cam_calib_aruco.yml");
  ROS_INFO("read_Succesfull");
  //fs["camera_matrix"]             >> intrinsics;
  //cout << "intrinsics = "<< endl << " "  << intrinsics << endl << endl;
  //fs["distortion_coefficients"]   >> distortion; //changed to 0 in cam_aruco.yaml because i get rectified image from imag_proc
  //distortion=cv::Mat();
  ros::init(argc, argv, "image_listener");
  std_msgs::Int8MultiArray* array=new std_msgs::Int8MultiArray[2];
  cv::namedWindow("view");
  cv::startWindowThread();
  ros::NodeHandle bi;
  image_transport::ImageTransport ib(bi);
  image_transport::Subscriber Bsub = ib.subscribe("base/image_raw", 1, imageCallback);
  ros::NodeHandle bp;
  ros::Publisher Bpub=bp.advertise<std_msgs::Int8MultiArray>("base/base_camera_stop",1);
  while(counter<20){
	ros::spinOnce();
	sleep(0.1);
  }
  array->data.push_back(0);
  Bpub.publish(*array);
  ros::spinOnce();
  sleep(1);
  array->data.clear();
  //ros::NodeHandle ri;
  //image_transport::ImageTransport ir(ri);
  //image_transport::Subscriber Rsub = ir.subscribe("robot/image_raw", 1, imageCallback);
  //ros::NodeHandle rp;
  //ros::Publisher Rpub=rp.advertise<std_msgs::Int8MultiArray>("robot/robot_camera_start",1);
  Bsub = ib.subscribe("robot/image_raw", 1, imageCallback);
  Bpub=bp.advertise<std_msgs::Int8MultiArray>("robot/robot_camera_start",1);
  sleep(1);
  //Bsub = ib.subscribe("robot/image_raw", 1, imageCallback);
  //Bpub=bp.advertise<std_msgs::Int8MultiArray>("robot/robot_camera_start",1);
  array->data.push_back(1);
  Bpub.publish(*array);
  ros::spinOnce();
  array->data.clear();
  //ros::NodeHandle ri;
  //image_transport::ImageTransport ir(ri);
  //image_transport::Subscriber Rsub = ir.subscribe("robot/image_raw", 1, imageCallback);

  //image_transport::CameraSubscriber sub = it.subscribeCamera("camera/image_raw_filtered", 1, imageCallback);
  char c;
  while(ros::ok() && kbhit(&c)!=1){
  ros::spinOnce();
  }
  array->data.push_back(0);
  Bpub.publish(*array);
  ros::spinOnce();
  sleep(1);
  array->data.clear();
  cv::destroyWindow("view");
}
