#include <ros/ros.h> //ros标准库头文件
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <iostream> //C++标准输入输出库
 
//OpenCV2标准头文件

// #include<opencv2/core/core.hpp>
// #include<opencv2/highgui/highgui.hpp>
// #include<opencv2/imgproc/imgproc.hpp>
// #include <opencv2/videoio/videoio.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mp42topic"); 
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    string path_mp4_;
    string topic_advertise_;
    string frame_id_;
    bool Bool_restart_;
    
    nh.param<std::string>("path_mp4", path_mp4_, "1.mp4");
    nh.param<std::string>("topic_advertise", topic_advertise_, "/camera/image_raw");
    nh.param<std::string>("frame_id", frame_id_, "camera_frame");
    nh.param<bool>("Bool_restart", Bool_restart_, false);

    // Read a mp4
    cv::VideoCapture cap(path_mp4_);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video file." << std::endl;
        return 1;
    }


    image_transport::Publisher image_pub = it.advertise(topic_advertise_, 1);


    double frameRate = cap.get(cv::CAP_PROP_FPS);
    ros::Rate loop_rate(frameRate);

    cv::Mat frame;

    int frame_id = 0;
    while (ros::ok()) {
        if (!cap.read(frame)) {
            ROS_INFO("End of video.");
            if(Bool_restart_ == true) {
                ROS_INFO("Restarting...");
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            }
            else
            {
                break;
            }
        }
        // 处理帧
        // 创建ROS图像消息
        
        double timeDiff = frame_id / frameRate;

        
        sensor_msgs::ImagePtr img_msg;  
        img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        img_msg->header.stamp = ros::Time::now();
        img_msg->header.frame_id = frame_id_; // 适应您的需求

        //bag.write(topic_advertise_, img_msg->header.stamp, img_msg);
        image_pub.publish(img_msg);
        
        // 输出帧数
        ROS_INFO("Frame %d published.", frame_id);
        frame_id++;

        ros::spinOnce();
        loop_rate.sleep();

    }

    cap.release(); // 释放视频捕获对象

    return 0;
    //ros::spin();
}
