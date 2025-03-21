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

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include<opencv2/core/core.hpp>

#include "System.h"
#include "Converter.h"

using namespace std;


bool enable_scale_recovery = true; // 设置为 true 开启尺度恢复，false 关闭
bool is_initialized = false;
double scale_factor = 1.0; // 尺度因子，默认为 1.0
double h_init_true;

int frame_num = 0;
ros::Publisher pub;

sensor_msgs::PointCloud2 PreparePointCloud2Message(const std::string &, const int);
void publish_pointcloud(const ORB_SLAM2::System* pSLAM);
void publish_Pose(const ORB_SLAM2::System* pSLAM);


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
    nodeHandler.param<double>("h_init_true", h_init_true, 0.8);
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
    std::cout << "-- GrabImage cost: " << duration.count() << " ms." << std::endl;


    start_time = std::chrono::high_resolution_clock::now();
    if(is_initialized) {publish_Pose(mpSLAM);}
    publish_pointcloud(mpSLAM);
    end_time = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::chrono::duration<double, std::milli> duration2 = end_time - start_time;
    std::cout << "-- Publish cost: " << duration2.count() << " ms." << std::endl;


    std::cout << std::endl;


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


    // 可以直接获取 std::set<MapPoint*> mspMapPoints;    pSLAM->mpMap->GetReferenceMapPoints()
    vector<ORB_SLAM2::MapPoint*>& all_points = pSLAM->mpTracker->mCurrentFrame.mvpMapPoints;
    if(!all_points.size() || pSLAM->mpMap->mspGroundPoints.size() < 500)
        return;

    cv::Mat Normal = pSLAM->mpMap->mvGroundPlaneNormal;

    static double h_init;

    if (!is_initialized) {
        h_init = 1.0 / norm(Normal);
        scale_factor = h_init_true / h_init;
        is_initialized = true;
        std::cout << "h_init: " << h_init << " h_init_true: " << h_init_true << " scale_factor: " << scale_factor << std::endl;
    }

    // 如果尺度恢复功能关闭，则将尺度因子设为 1.0
    if (!enable_scale_recovery) {
        scale_factor = 1.0;
    }

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
    cv::Mat R = cv::Mat::eye(4, 4, CV_32F);
    x_1.copyTo(R.col(0).rowRange(0,3));
    y_1.copyTo(R.col(1).rowRange(0,3));
    z_1.copyTo(R.col(2).rowRange(0,3));
    R.at<float>(3, 3) = 1;

    // 平移矩阵 T
    cv::Mat T = cv::Mat::eye(4, 4, CV_32F);
    Pn0.copyTo(T.col(3).rowRange(0,3));
    cv::Mat T_inv = T.inv();

    // Calculate PP2 using rotation matrix R and translation vector T
    cv::Mat PP1, PP2;
    PP2 = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
    PP1 = R.t() * (T_inv * PP2);
    cout << "-- height: " << PP1.at<float>(2) << endl; // 当前相机高度

    // static const float scale = 0.9 / norm(Pn0);
    // cout << "-- scale: " << scale << endl;; // 当前缩放尺度



    // 使用临时容器存储满足条件的点
    std::vector<ORB_SLAM2::MapPoint*> valid_points;
    valid_points.reserve(all_points.size());

    for (const auto &temp_point : all_points)
    {
        if (temp_point && !temp_point->isBad() && !temp_point->mbGround && temp_point->Observations() >= 4)
        {
            valid_points.push_back(temp_point);
        }
    }
    int actual_num_points = valid_points.size();
    if (actual_num_points == 0)
        return;


    // 获取当前帧的相机位姿
    cv::Mat Tcw = pSLAM->mpTracker->mCurrentFrame.mTcw.clone(); // 当前帧的相机位姿
    cv::Mat Twc = Tcw.inv(); // T_wc = T_cw^{-1}
      cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3); 
      cv::Mat twc = Twc.rowRange(0, 3).col(3); 
      twc *= scale_factor;
    Tcw = Twc.inv();

    // 使用向量收集点数据
    std::vector<float> points_data;
    points_data.reserve(actual_num_points * 3); // 预分配空间

    for (const auto &temp_point : valid_points) 
    {
        cv::Mat new_point;
        vconcat(scale_factor * temp_point->GetWorldPos(), cv::Mat::ones(1, 1, CV_32F) , new_point);
        // new_point = R.t() * (T_inv * new_point);
        new_point =  Tcw * new_point;

        // stream.next(new_point.at<float>(0));
        // stream.next(new_point.at<float>(1));
        // stream.next(new_point.at<float>(2));
        points_data.push_back(new_point.at<float>(0)); // x
        points_data.push_back(new_point.at<float>(1)); // y
        points_data.push_back(new_point.at<float>(2)); // z
    }
    // 将向量数据赋值给 msg.data
    auto msg = PreparePointCloud2Message("camera", actual_num_points);
    std::copy(points_data.begin(), points_data.end(), reinterpret_cast<float*>(msg.data.data()));
    // pub.publish(msg);





    static tf2_ros::TransformBroadcaster tf_broadcaster;
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "initial";
    transform_stamped.child_frame_id = "map";
    transform_stamped.transform.translation.x = Pn0.at<float>(0);
    transform_stamped.transform.translation.y = Pn0.at<float>(1);
    transform_stamped.transform.translation.z = Pn0.at<float>(2);
    transform_stamped.transform.rotation.x = q[0];
    transform_stamped.transform.rotation.y = q[1];
    transform_stamped.transform.rotation.z = q[2];
    transform_stamped.transform.rotation.w = q[3];

    // 广播变换
    tf_broadcaster.sendTransform(transform_stamped);

    pub.publish(msg);

}

void publish_Pose(const ORB_SLAM2::System* pSLAM)
{
    // vector<ORB_SLAM2::MapPoint*>& all_points = pSLAM->mpTracker->mCurrentFrame.mvpMapPoints;
    // if(!all_points.size() || pSLAM->mpMap->mspGroundPoints.size() < 200)
    //     return;

    
    static tf2_ros::TransformBroadcaster tf_broadcaster;

    ORB_SLAM2::Tracking* tracker = pSLAM->mpTracker;
    if (!tracker || tracker->mState != ORB_SLAM2::Tracking::OK)
        return;
    cv::Mat t = tracker->mCurrentFrame.GetCameraCenter() * scale_factor;
    cv::Mat R = tracker->mCurrentFrame.GetRotationInverse();
    
    //抽取旋转部分和平移部分，前者使用四元数表示
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);

    // 创建一个变换
    // 设置旋转矩阵
    // tf2::Matrix3x3 rotation_matrix(
    //     R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
    //     R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
    //     R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2)
    // );
    // transform.setBasis(rotation_matrix);
    // tf2::Transform transform;
    // transform.setOrigin(tf2::Vector3(t.at<float>(0), t.at<float>(1), t.at<float>(2)));
    
    // 创建一个变换消息
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "initial";
    transform_stamped.child_frame_id = "camera";
    transform_stamped.transform.translation.x = t.at<float>(0);
    transform_stamped.transform.translation.y = t.at<float>(1);
    transform_stamped.transform.translation.z = t.at<float>(2);
    transform_stamped.transform.rotation.x = q[0];
    transform_stamped.transform.rotation.y = q[1];
    transform_stamped.transform.rotation.z = q[2];
    transform_stamped.transform.rotation.w = q[3];

    // 广播变换
    tf_broadcaster.sendTransform(transform_stamped);


}


