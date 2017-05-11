#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>

#define m_size 0.0795
#define m_size3 0.156
#define m_size2 0.159
#define m_size4 0.078

using namespace aruco;
MarkerDetector MDetector;
MarkerPoseTracker PTracker;
vector<Marker> Markers;
image_geometry::PinholeCameraModel cam_model;
uint8_t cam_model_found=0;
CameraParameters aruco_cam_param;
//cv::FileStorage fs("cam_aruco.yaml", cv::FileStorage::READ);
//cv::Mat intrinsics, distortion;
//cv::Mat transf_mat[4];
void imageCallback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  try
  {
    cv::Mat imag=cv_bridge::toCvShare(msg, "bgr8")->image;
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(!cam_model_found){
    cam_model.fromCameraInfo(info_msg);
    cv::Mat zero_dist= cv::Mat::zeros(4, 1, CV_64FC1);
    aruco_cam_param.setParams(cv::Mat(cam_model.intrinsicMatrix()), zero_dist ,cv::Size(cam_model.fullResolution()));
    //aruco_cam_param.setParams(cv::Mat(cam_model.intrinsicMatrix()), cv::Mat(cam_model.distortionCoeffs()) ,cv::Size(cam_model.reducedResolution()));
    cam_model_found=1;
    }
    //cv::Mat intrinsics, distortion;
    //intrinsics = cv::Mat(cam_model.intrinsicMatrix());
    //distortion = cv::Mat();
    MDetector.detect(imag,Markers,aruco_cam_param.CameraMatrix, cv::Mat(),m_size);
    if(Markers.size()==1){
    //for (unsigned int i=0;i<Markers.size();i++){
      Markers[0].calculateExtrinsics (m_size, aruco_cam_param, false);
      if(PTracker.estimatePose(Markers[0],aruco_cam_param ,m_size , 2)){
        cv::Mat pose=PTracker.getRTMatrix();
        cout<< pose<< endl;
      }
      //cout<<Markers[i]<<endl;
      Markers[0].draw(imag,cv::Scalar(0,0,255),2);
      aruco::CvDrawingUtils::draw3dAxis(imag, aruco_cam_param , PTracker.getRvec(),PTracker.getTvec(),m_size);
    }
    //drawAxis(imag, intrinsics, distortion, rvec, tvec, 0.2);
    cv::imshow("view", imag);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void infoCallback(const sensor_msgs::ImageConstPtr& msg){

}

int main(int argc, char **argv)
{
  //fs["camera_matrix"]             >> intrinsics;
  //cout << "intrinsics = "<< endl << " "  << intrinsics << endl << endl;
  //fs["distortion_coefficients"]   >> distortion; //changed to 0 in cam_aruco.yaml because i get rectified image from imag_proc
  //distortion=cv::Mat();
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nt;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nt);
  image_transport::CameraSubscriber sub = it.subscribeCamera("camera/image_rect", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}