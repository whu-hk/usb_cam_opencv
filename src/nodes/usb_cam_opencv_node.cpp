#include <usb_cam_opencv.h>

int main(int argc,char **argv)
{
	ros::init(argc, argv, "usb_cam_opencv_node", ros::init_options::NoRosout);
    ros::NodeHandle nh("");
	ros::NodeHandle nh_local("~");
    try {
	  ROS_INFO("[usb_cam_opencv]: Initializing node");
	  Opencv_cam od(nh,nh_local);
    }
    catch (const char* s) {
	  ROS_FATAL_STREAM("[usb_cam_opencv]: "  << s);
    }
    catch (...) {
	  ROS_FATAL_STREAM("[usb_cam_opencv]: Unexpected error");
    }
    return 0;
}