#include "opencv2/opencv.hpp"
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <exception>

using namespace cv;
using namespace std;

//640x480
cv::Mat K_1 = (Mat_<double>(3, 3) <<601.1819494061277, 0, 329.1305015302001, 0.000000, \
 600.0406404279748, 238.3837396948256, 0.000000, 0.000000, 1.000000);
cv::Mat D_1 = (Mat_<double>(4,1) << -0.0541737, -0.761986, 3.11624, -3.69709);

//1280x720
cv::Mat K_2 = (Mat_<double>(3, 3) <<907.3179347878757, 0, 622.1428470715612, 0.000000, \
 910.7995408784911, 330.5347846342667, 0.000000, 0.000000, 1.000000);
cv::Mat D_2 = (Mat_<double>(4,1) << -0.120183, 0.0311701, -0.0129947, -0.00799496);

//1920x1080
cv::Mat K_3 = (Mat_<double>(3, 3) <<1398.526435675626, 0, 974.6271798798929,0, \
 1399.471439435285, 533.3491663893035,0, 0, 1);
cv::Mat D_3 = (Mat_<double>(4,1) << -0.101938, -0.0501564, 0.0323029, 0.0226299);

class Opencv_cam
{
public:
  Opencv_cam(ros::NodeHandle nh,ros::NodeHandle nh_local);
  ~Opencv_cam();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  string topic_image_;
  int video_index_;
  bool distortion_correct_;
  int param_width_;
  int param_height_;
  double new_intrinsic_mat_x_;
  double new_intrinsic_mat_y_;
  void initialize();
  int pubFrames();
};
