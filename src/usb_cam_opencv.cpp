#include<usb_cam_opencv.h>

Opencv_cam::Opencv_cam(ros::NodeHandle nh,ros::NodeHandle nh_local):nh_(nh),nh_local_(nh_local)
{
    initialize();
	pubFrames();
}

Opencv_cam::~Opencv_cam() 
{
    nh_local_.deleteParam("Topic_image");
    nh_local_.deleteParam("Video_index");
    nh_local_.deleteParam("Distortion_correct");
}

void Opencv_cam::initialize()
{
	nh_local_.param<string>("Topic_image",topic_image_,"camera/bgr/image_raw");
	nh_local_.param<int>("Video_index",video_index_,0);
	nh_local_.param<bool>("Distortion_correct",distortion_correct_,true);
	nh_local_.param<int>("Param_width",param_width_,640);
	nh_local_.param<int>("Param_height",param_height_,480);
}

int count_fps()
{
    static int fps = 0;
 	static int lastTime = cv::getTickCount(); // ms
 	static int frameCount = 0;  //加了static使得每次函数执行:frameCount只进行一次初始化
 	++frameCount;
 	int curTime = cv::getTickCount();
 	if ((curTime - lastTime) / cv::getTickFrequency() > 1.0) // 取固定时间间隔为1秒
 	{
 		fps = frameCount;
 		frameCount = 0;
 		lastTime = curTime;
 	}
 	return fps;
}

int Opencv_cam::pubFrames()
{
    image_transport::ImageTransport it(nh_);
    image_transport::Publisher pub = it.advertise(topic_image_, 10);
    cv::VideoCapture cap(video_index_);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, param_width_);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, param_height_);
	if(param_width_ > 640 && param_height_ > 480)
	    std::cout<<"Set code:"<<cap.set(CV_CAP_PROP_FOURCC, \
	                CV_FOURCC('M', 'J', 'P', 'G'))<<std::endl;
	//cap.set(CV_CAP_PROP_FPS, 30);
    std::cout<<"&&&  FPS:"<< cap.get(CV_CAP_PROP_FPS)<<std::endl;
    int codec = cap.get(CV_CAP_PROP_FOURCC);
    std::cout<<"&&&  Code:"<<std::to_string(codec&0xFF)+ \
                         std::to_string((codec>>8)&0xFF)+ \
                         std::to_string((codec>>16)&0xFF)+ \
                         std::to_string((codec>>24)&0xFF)<<std::endl;
    std::cout<<"&&&  Resolution:"<<cap.get(CV_CAP_PROP_FRAME_WIDTH)<< \
                "x"<<cap.get(CV_CAP_PROP_FRAME_HEIGHT)<<std::endl;
    if (!cap.isOpened())
    {
        cout << "!!!  Could not open the input video: "<<video_index_<<std::endl;
        return -1;
    }
    else
        std::cout<<"&&&  Open video"<<video_index_<<" success!"<<std::endl;
    cv::Mat frame;
    cv::Mat undistort_frame;
	if(distortion_correct_)
		std::cout << "!!!  Distortion corrected" << std::endl;    
	else
		std::cout << "!!!  Distortion exist" << std::endl;
	
	cv::Mat K,D;
    cv::Mat new_intrinsic_mat;
    if(param_width_ == 640 && param_height_ == 480)
    {
        K = K_1;
        D = D_1;
    }
    else if(param_width_ == 1280 && param_height_ == 720)
    {
        K = K_2;
        D = D_2;
    }
    else if(param_width_ == 1920 && param_height_ == 1080)
    {
        K = K_3;
        D = D_3;
    }
    else
    {
        std::cout<<"error resolution"<<std::endl;
        return 0;
    }
    K.copyTo(new_intrinsic_mat);
    //调整输出校正图的视场，归一化焦距
    //new_intrinsic_mat.at<double>(0, 0) *= 1; 
    //0.86使得640x480横向像素正好完全填充，0.8为标定时的视场
    //new_intrinsic_mat.at<double>(1, 1) *= 1;
    
    new_intrinsic_mat.at<double>(0, 0) *= 0.8; 
    new_intrinsic_mat.at<double>(1, 1) *= 0.8;
    
    //调整输出校正图的中心,主光点
    new_intrinsic_mat.at<double>(0, 2) += 0.5 * frame.cols;
    new_intrinsic_mat.at<double>(1, 2) += 0.5 * frame.rows;
    int num = 0;
    cv::Size2i image_size(param_width_,param_height_);
    Mat mapx = Mat(image_size, CV_32FC1);
    Mat mapy = Mat(image_size, CV_32FC1);
    Mat R = Mat::eye(3, 3, CV_32F);
    cv::fisheye::initUndistortRectifyMap(K,D,R,getOptimalNewCameraMatrix(K,D,image_size,1,image_size,0),image_size,CV_32FC1,mapx,mapy);
    while(nh_.ok()) 
    {  
        if(!cap.read(frame)) 
		{
            std::cout << "!!!  No frame" << std::endl;
            break;
        }
		cv::imshow("origin",frame);
		if(distortion_correct_)
		{  
		    //if(param_width_ > 640 && param_height_ > 480)
		    //{
                cv::remap(frame,undistort_frame,mapx,mapy,INTER_LINEAR);
		    //}
		    //else  //640x480用第一种去畸变没有输出,用第一种方法对K,D等有很高的精度要求
		    //    cv::fisheye::undistortImage(frame,undistort,K,D,new_intrinsic_mat);
		    cv::imshow("Undistort",undistort_frame);
		}
        char key = cv::waitKey(1);
        if(key == 'q' || key == 'Q' || key == 27)
          break;
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = "head_camera";
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();     
        pub.publish(msg);
        int fps = count_fps();
        if(num % 80 == 0)
            std::cout<<"FPS:"<<fps<<std::endl;
        num++;  
    }
	return 0;
}
