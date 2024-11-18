/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>


#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> // 用于将 PCL 点云转换为 ROS 点云消息
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include<opencv2/core/core.hpp>

#include "System.h"

using namespace std;

int frame_num = 0;
ros::Publisher pub;

sensor_msgs::PointCloud2 PreparePointCloud2Message(const std::string &, const int);
void publish_pointcloud(const ORB_SLAM2::System* pSLAM);



class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    ros::NodeHandle nodeHandler("~");
 
    /**************** modify v1 **********/   
    cout<< endl << "Executed in path:" << argv[0] << endl;

    string Mono_image_topic_;
    nodeHandler.param<string>("Mono_image_topic", Mono_image_topic_, "/camera/color/image_raw");
    string path_to_vocabulary_, path_to_settings_;
    nodeHandler.param<string>("path_to_vocabulary", path_to_vocabulary_, "ORBvoc.txt");
    nodeHandler.param<string>("path_to_settings", path_to_settings_, "Monocular/settings.yaml");

    /*
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    */

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(path_to_vocabulary_, path_to_settings_, ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);




    ros::Subscriber sub = nodeHandler.subscribe(Mono_image_topic_, 1, &ImageGrabber::GrabImage, &igb);
    pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/points", 1);



    // int last_frame_num = frame_num;

// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // while (ros::ok()) {
    //     // 执行ROS相关操作
    //     if(last_frame_num != frame_num) {
    //         last_frame_num = frame_num;
            
    //         std::cout << "Frame number: " << frame_num << std::endl;
    //         std::cout << "-- Ground: " << SLAM.mpMap->mvGroundPlaneNormal << std::endl;

    //         end_time = std::chrono::high_resolution_clock::now();
    //         duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    //         start_time = end_time;
    //         std::cout << "-- cost: " << duration.count() << " ms." << std::endl;
        
    //         publish_pointcloud(SLAM.mpMap->GetAllMapPoints());
    //     }
    //     // 一次性处理所有待处理的ROS事件
    //     ros::spinOnce();
    // }

    ros::spin();

    string path_to_output_;
    nodeHandler.param<string>("path_to_output", path_to_output_, "/root/assets/");

    string filename = path_to_output_ + "Points.csv";
    //获取地图中的所有地图点
    vector<ORB_SLAM2::MapPoint*> all_points = SLAM.mpMap->GetAllMapPoints();
    cout << endl << "Saving Map Point to " << filename  << endl;
    //文件写入的准备操作
    ofstream f;
    f.open(filename.c_str());
    f << fixed;
    f << "Pos_x,Pos_y,Pos_z," << "NormalVec_0,NormalVec_1,NormalVec_2," << "Observations,isBad,GetFound,isGround," 
        << "minDis,maxDis," << "mnId,mnFirstKFid,mnFirstFrame,mnTrackReferenceForFrame,mnLastFrameSeen" << endl;
    for(auto& pSinglePoint:all_points) {
        cv::Mat pos = pSinglePoint->GetWorldPos();
        cv::Mat norm = pSinglePoint->GetNormal();
        int obs = pSinglePoint->Observations();
        bool bad = pSinglePoint->isBad();
        int mnFound = pSinglePoint->GetFound();
        bool bGround = pSinglePoint->mbGround;

        float minDis = pSinglePoint->GetMinDistanceInvariance()/0.8f;
        float maxDis = pSinglePoint->GetMaxDistanceInvariance()/1.2f;

        f << setprecision(8) << pos.at<float>(0) << "," << pos.at<float>(1) << "," << pos.at<float>(2) << ","
            << norm.at<float>(0) << "," << norm.at<float>(1) << "," << norm.at<float>(2) << "," 
            << obs << "," << std::noboolalpha << bad << "," << mnFound << "," << bGround << ","
            << minDis << "," << maxDis << ","
            << pSinglePoint->mnId << "," << pSinglePoint->mnFirstKFid << "," << pSinglePoint->mnFirstFrame << "," 
            << pSinglePoint->mnTrackReferenceForFrame << "," << pSinglePoint->mnLastFrameSeen << endl;
        
    }
    //关闭文件
    f.close();
    cout << endl << "Map Point saved!" << endl;





    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    filename = path_to_output_ + "KeyFrameTrajectory.txt";
    SLAM.SaveKeyFrameTrajectoryTUM(filename);

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // debug 
    std::cout << "Frame number: " << frame_num << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    frame_num ++;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());


    // debug 
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "-- GrabImage cost: " << duration.count() << " ms." << std::endl<< std::endl;


    start_time = std::chrono::high_resolution_clock::now();
    publish_pointcloud(mpSLAM);
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "-- Publish cost: " << duration.count() << " ms." << std::endl;





}


sensor_msgs::PointCloud2 PreparePointCloud2Message(const std::string &frame_id, const int num_points)
{
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = num_points;
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = 12 * msg.width;
    msg.is_dense = true;
    msg.data.resize(12 * num_points);
    return msg;
}

void publish_pointcloud(const ORB_SLAM2::System* pSLAM)
{


    // 可以直接获取 std::set<MapPoint*> mspMapPoints;
    vector<ORB_SLAM2::MapPoint*>&& all_points = pSLAM->mpMap->GetAllMapPoints();
    if(!all_points.size())
        return;

    cv::Mat Normal = pSLAM->mpMap->mvGroundPlaneNormal;

    cv::Mat P0 = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    cv::Mat P1 = (cv::Mat_<float>(3, 1) << 0, 0, 1);
    // 计算投影点
    float square_sum = sum(Normal.mul(Normal))[0];
    float t0 = -(P0.dot(Normal) + 1) / square_sum;
    cv::Mat Pn0 = P0 + t0 * Normal; // 投影点
    float t1 = -(P1.dot(Normal) + 1) / square_sum;
    cv::Mat Pn1 = P1 + t1 * Normal;
    // 计算 x_1, y_1, z_1
    cv::Mat x_1 = Pn1 - Pn0;
    x_1 = x_1 / norm(x_1);
    cv::Mat z_1 = Normal / norm(Normal);
    cv::Mat y_1 = z_1.cross(x_1);

    // 旋转矩阵 R
    cv::Mat R(4, 4, CV_32F);
    x_1.copyTo(R.col(0).rowRange(0,3));
    y_1.copyTo(R.col(1).rowRange(0,3));
    z_1.copyTo(R.col(2).rowRange(0,3));
    R.at<float>(3, 3) = 1;

    // 平移矩阵 T
    cv::Mat T = cv::Mat::eye(4, 4, CV_32F);
    Pn0.copyTo(T.col(3).rowRange(0,3));
    cv::Mat T_inv = T.inv();

    // // Calculate P2 using transformation matrix T
    // P2 = T * R * P1.t();

    // Calculate PP2 using rotation matrix R and translation vector T
    cv::Mat PP1, PP2;
    PP2 = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
    PP1 = R.t() * (T_inv * PP2);
    cout << "-- height: " << PP1.at<float>(2) << endl;; // 当前相机高度

    auto msg = PreparePointCloud2Message("map", all_points.size());
    ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());


    for (const auto &temp_point : all_points)
    {
        // && !temp_point->mbGround
        if(!temp_point->isBad() && !temp_point->mbGround) {
            cv::Mat new_point;
            vconcat(temp_point->GetWorldPos(), cv::Mat::ones(1, 1, CV_32F) , new_point);
            new_point = R.t() * (T_inv * new_point);


            stream.next(new_point.at<float>(0));
            stream.next(new_point.at<float>(1));
            stream.next(new_point.at<float>(2));
        }
    }
    pub.publish(msg);
}



