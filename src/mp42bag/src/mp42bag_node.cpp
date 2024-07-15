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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mp42bag"); 
    ros::NodeHandle nh("~");

    string path_mp4_;
    string path_bag_;
    string topic_advertise_;
    string frame_id_;
    string compress_type_;
    
    nh.param<std::string>("path_mp4", path_mp4_, "1.mp4");
    nh.param<std::string>("path_bag", path_bag_, "1.bag");
    nh.param<std::string>("topic_advertise", topic_advertise_, "/camera/image_raw");
    nh.param<std::string>("frame_id", frame_id_, "camera_frame");
    nh.param<std::string>("compress_type", compress_type_, "raw");

    // Read a mp4
    cv::VideoCapture cap(path_mp4_);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video file." << std::endl;
        return 1;
    }
    // Create a ROS bag
    rosbag::Bag bag;
    bag.open(path_bag_, rosbag::bagmode::Write);


    double frameRate = cap.get(cv::CAP_PROP_FPS);
    const ros::Time baseTime = ros::Time::now();

    cv::Mat frame;

    int frame_id = 0;
    while (ros::ok()) {
        if (!cap.read(frame)) {
            break;
        }
        // 处理帧
        // 创建ROS图像消息
        
        double timeDiff = frame_id / frameRate;

        if(compress_type_ == "raw") {
            
            sensor_msgs::ImagePtr img_msg;  
            img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            img_msg->header.stamp = baseTime + ros::Duration(timeDiff);
            img_msg->header.frame_id = frame_id_; // 适应您的需求

            // 将图像消息保存到ROS bag
            bag.write(topic_advertise_, img_msg->header.stamp, img_msg);
        }
        else {
            std::vector<uchar> compressed_data;
            cv::imencode(".jpg", frame, compressed_data);
            sensor_msgs::CompressedImage compressed_image_msg;
            compressed_image_msg.format = "jpeg";
            compressed_image_msg.data = compressed_data;

            
            compressed_image_msg.header.stamp = baseTime + ros::Duration(timeDiff);
            compressed_image_msg.header.frame_id = frame_id_; // 适应您的需求

            bag.write(topic_advertise_, baseTime + ros::Duration(timeDiff), compressed_image_msg);
        }
        

        // 输出帧数
        ROS_INFO("Frame %d saved.", frame_id);
        frame_id++;



        // Convert the OpenCV image to a ROS sensor_msgs::Image
        //sensor_msgs::Image image_msg;
        // image_msg.header.stamp = ros::Time::now();
        // image_msg.height = frame.rows;
        // image_msg.width = frame.cols;
        // image_msg.encoding = "bgr8";
        // image_msg.is_bigendian = false;
        // image_msg.data.resize(frame.step * frame.rows);
        //memcpy(image_msg.data.data(), frame.data, frame.step * frame.rows);

        // Write the image message to the ROS bag
        //bag.write(topic_advertise_, ros::Time::now(), image_msg);



    }

    cap.release(); // 释放视频捕获对象

    return 0;
    //ros::spin();
}
