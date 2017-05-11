#define FPS 5

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include <cv_bridge/cv_bridge.h>

//EIGEN -- always before aruco/eigen includes!!!
#include <eigen3/Eigen/Eigen>

//ARUCO
#include <aruco/aruco.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

//NON_BLOCKING TERM_INPUT
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <time.h>

//MARKER SIZE
#define m_size 159.5

//117.28
using namespace Eigen;
using namespace aruco;
MarkerDetector MDetector;
MarkerPoseTracker PTracker,PTracker2;
vector<Marker> Markers;

bool first_detect=true;
uint8_t counter=0;
CameraParameters aruco_cam_param;

Matrix4d basemarker, robotmarker, base2marker, marker2inst, position, tmp_Eigen;
std::string sep="\n--------------------------------------------------\n";
Matrix4d result;

bool detected_base=false;
bool detected_marker=false;
bool pub_thresholded_image=false;
bool first_call=true;
bool new_image=false;
bool close_program=false;

uint8_t median_counter=0;

cv_bridge::CvImagePtr out_msg;

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
    //imag=cv_bridge::toCvShare(msg, "bgr8")->image;
    out_msg=cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat imag=out_msg->image;
    new_image=true;
    //MDetector.detect(imag,Markers,aruco_cam_param,m_size5, false);
    ros::Time now,then;
    now=ros::Time::now();
    Markers=MDetector.detect(imag);
    then=ros::Time::now();
    ROS_INFO_STREAM(then-now);
    //ROS_INFO("%d MARKERS",Markers.size());
    if(Markers.size()>0){
      for (unsigned int i=0;i<Markers.size();i++){
	Markers[i].draw(imag,cv::Scalar(0,0,255),2);
      }
      //Markers[i].calculateExtrinsics (m_size5, aruco_cam_param, false);
      if(counter<10){
	for (unsigned int i=0;i<Markers.size();i++){
	  if(Markers[i].id==101){
	    if(PTracker.estimatePose(Markers[i],aruco_cam_param,m_size,10)){
	      counter++;
	      aruco::CvDrawingUtils::draw3dAxis(imag, aruco_cam_param , PTracker.getRvec(),PTracker.getTvec(),m_size);
	      cv::Mat tmp=PTracker.getRTMatrix();
	      cv::cv2eigen<double>(tmp.inv(),tmp_Eigen);
	      if(first_call){basemarker=tmp_Eigen; first_call=false;}
	      else{basemarker+=tmp_Eigen; basemarker/=2.0;}
	      //cout<<"BM "<<basemarker<<endl;
	      //cout<< basemarker.inverse()<<sep << endl;
	      
	    }
	  }
	}
	if(counter>=10) detected_base=true;
      }
      else{
	for (unsigned int i=0;i<Markers.size();i++){
	  if(Markers[i].id==100){
	  if(PTracker2.estimatePose(Markers[i],aruco_cam_param,m_size,1)){
	    aruco::CvDrawingUtils::draw3dAxis(imag, aruco_cam_param , PTracker2.getRvec(),PTracker2.getTvec(),m_size);
	    cv::cv2eigen<double>(PTracker2.getRTMatrix(),robotmarker);
	    //cout<<"RM "<<robotmarker<<endl;
	    position=base2marker*basemarker*robotmarker*marker2inst;
	    cout<<"BM "<<basemarker<<endl;
	    cout<<"RM "<<robotmarker<<endl;
	    cout<< "POS " <<position<<endl;
	    detected_marker=true;
	  }
	  else detected_marker=false;
	  }
	}
      }
    imag.copyTo(out_msg->image);//=imag;
  }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void commandsCallback(const std_msgs::Int8MultiArray& array){ 
  //ROS_INFO("%d %d %d",array.data[0],array.data[1],array.data[2]);
  if(array.data[0]==array.data[1] && array.data[0]==0){
    close_program=true;
    new_image=false;
  }
  else{
  if(array.data[2]==1) pub_thresholded_image=true;
  else pub_thresholded_image=false;
  MDetector.setThresholdParams(array.data[0],array.data[1]);
  }
}

int main(int argc, char **argv)
{
  base2marker<< 1, 0, 0, 90, 0, 0, -1, 675, 0, 1, 0, 110, 0, 0, 0, 1; //mm
  marker2inst<< 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1; //rotation to match robot
  position<<	1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1; //only for init
  MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
  MDetector.setDictionary(Dictionary::ARUCO_MIP_36h12);
  aruco_cam_param.readFromXMLFile("/home/odroid/ros_SMArobot/src/camera_grab/yaml_files/yaml1/aruco_calib_1280_1024_base.yml");
  ros::init(argc, argv, "image_listener");
  std_msgs::Int8MultiArray array;
  std_msgs::Float32MultiArray aruco_position;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  ros::NodeHandle bi;
  image_transport::ImageTransport ib(bi);
  image_transport::Subscriber Bsub = ib.subscribe("robot/image_raw", 1, imageCallback);
  image_transport::Publisher  filtered_image_pub = ib.advertise("robot/image_raw_filt",1);
  ros::NodeHandle bp;
  ros::Publisher Bpub=bp.advertise<std_msgs::Int8MultiArray>("robot/robot_camera_start",1);
  ros::Publisher Bpub2=bp.advertise<std_msgs::Float32MultiArray>("aruco_info",1);
  ros::Subscriber settings_sub=bp.subscribe("aruco_commands",1,commandsCallback);
  ros::Rate loop_rate(FPS);
  char c;
  while(ros::ok() && kbhit(&c)!=1 && !close_program){
    ros::spinOnce();
    if(new_image){
      //Check if we publish mono or color image
      if(pub_thresholded_image){
      out_msg->image=MDetector.getThresholdedImage();
      out_msg->encoding=sensor_msgs::image_encodings::MONO8;
      }
      //Publish image
      filtered_image_pub.publish(out_msg->toImageMsg());
      new_image=false;
      //Calculate new position and send to controller
      if(detected_marker){
      double pos[16];
      Map<Matrix4d>(pos,4,4)=position;
      aruco_position.data.clear();
      aruco_position.data.push_back(pos[12]);
      aruco_position.data.push_back(pos[13]);
      aruco_position.data.push_back(pos[14]);
      Bpub2.publish(aruco_position);
      }
    }
    loop_rate.sleep();
  }
  array.data.clear();
  array.data.push_back(1);
  Bpub.publish(array);
  ros::spinOnce();
  sleep(1);
  array.data.clear();
  //cv::destroyWindow("view");
}
