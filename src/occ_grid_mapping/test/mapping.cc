// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <iostream>
#include <ros/ros.h>


#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <nav_msgs/Odometry.h>
#include <occ_grid_mapping/grid_map.h>
#include <occ_grid_mapping/grid_mapper.h>

/* Global */
GridMap* g_map;
GridMapper* g_gmapper;
ros::Subscriber g_odom_suber;
ros::Publisher g_map_puber;
Pose2d g_robot_pose;
tf2_ros::Buffer* g_tf_buffer;  // TF 缓冲区

/* 新增全局变量 */
std::vector<std::pair<ros::Time, geometry_msgs::Pose>> g_camera_trajectory; // 用于存储camera的轨迹（时间戳+位姿）
double g_camera_trajectory_length = 0.0; // camera的三维轨迹总长度

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

// 更新camera轨迹函数
void updateCameraTrajectory(const ros::Time& timestamp, const geometry_msgs::Pose& pose) {
    if (!g_camera_trajectory.empty()) {
        // 计算新位姿与上一位姿之间的三维欧几里得距离
        const geometry_msgs::Pose& last_pose = g_camera_trajectory.back().second;
        double dx = pose.position.x - last_pose.position.x;
        double dy = pose.position.y - last_pose.position.y;
        double dz = pose.position.z - last_pose.position.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        g_camera_trajectory_length += distance; // 累加轨迹长度
    }
    g_camera_trajectory.emplace_back(timestamp, pose); // 将新位姿加入轨迹
    ROS_INFO("Current camera trajectory length: %.2f meters", g_camera_trajectory_length);
}


// 保存轨迹到TUM格式文件
void saveTrajectoryToTUMFile(const std::string& file_path) {
    std::ofstream tum_file(file_path);
    if (!tum_file.is_open()) {
        ROS_ERROR("Failed to open file for writing: %s", file_path.c_str());
        return;
    }

    // 写入TUM格式数据
    for (const auto& [timestamp, pose] : g_camera_trajectory) {
        // 时间戳转换为秒
        double time_seconds = timestamp.toSec();

        // 平移部分
        double tx = pose.position.x;
        double ty = pose.position.y;
        double tz = pose.position.z;

        // 旋转部分（四元数）
        double qx = pose.orientation.x;
        double qy = pose.orientation.y;
        double qz = pose.orientation.z;
        double qw = pose.orientation.w;

        // 写入一行数据
        tum_file << std::fixed << std::setprecision(6)
                 << time_seconds << " "
                 << tx << " " << ty << " " << tz << " "
                 << qx << " " << qy << " " << qz << " " << qw << "\n";
    }

    tum_file.close();
    ROS_INFO("Trajectory saved to: %s", file_path.c_str());
}

int main ( int argc, char **argv )
{
    /***** 初始化ROS *****/
    ros::init ( argc, argv, "GridMapping" );
    ros::NodeHandle nh;

    /***** 加载参数 *****/
    int map_sizex, map_sizey, map_initx, map_inity;
    double map_cell_size;
    /* TODO 错误处理 */
    nh.getParam ( "/mapping/map/sizex", map_sizex );
    nh.getParam ( "/mapping/map/sizey", map_sizey );
    nh.getParam ( "/mapping/map/initx", map_initx );
    nh.getParam ( "/mapping/map/inity", map_inity );
    nh.getParam ( "/mapping/map/cell_size", map_cell_size );

    Pose2d T_r_l;
    double x, y, theta;
    double P_occ, P_free, P_prior;
    /* TODO 错误处理 */
    nh.getParam ( "/mapping/robot_laser/x", x );
    nh.getParam ( "/mapping/robot_laser/y", y );
    nh.getParam ( "/mapping/robot_laser/theta", theta );
    T_r_l = Pose2d ( x, y, theta );

    nh.getParam ( "/mapping/sensor_model/P_occ", P_occ );
    nh.getParam ( "/mapping/sensor_model/P_free", P_free );
    nh.getParam ( "/mapping/sensor_model/P_prior", P_prior );
    
    /* 地图保存地址 */
    std::string map_image_save_dir, map_config_save_dir;
    nh.getParam ( "/mapping/map_image_save_dir", map_image_save_dir );
    nh.getParam ( "/mapping/map_config_save_dir", map_config_save_dir );

    /***** 初始化地图和构图器 *****/
    g_map = new GridMap ( map_sizex, map_sizey,  map_initx, map_inity, map_cell_size, P_prior);
    g_gmapper = new GridMapper ( g_map, T_r_l, P_occ, P_free, P_prior );

    
    // 初始化 TF 监听器
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    g_tf_buffer = &tf_buffer;  // 将 TF 缓冲区赋值给全局变量
    ros::Subscriber cloud_suber = nh.subscribe("/points", 1, pointCloudCallback);
    g_map_puber = nh.advertise<nav_msgs::OccupancyGrid> ( "mapping/grid_map", 1 );
    
    ros::spin();

    // 保存轨迹到文件
    std::string output_file_path = "/mnt/d/WslSystem/share/dataset/rosout/camera_trajectory.txt";
    saveTrajectoryToTUMFile(output_file_path);

    /* TODO 保存地图 */
    g_map->saveMap(map_image_save_dir, map_config_save_dir);
    
    std::cout << "\nMap saved\n";
}


// PointCloud2 回调
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    geometry_msgs::Transform transform_map_camera;
    // TF 变换回调
    try {
        // 获取 camera 相对于 map 的变换
        geometry_msgs::TransformStamped transform = g_tf_buffer->lookupTransform("map", "camera", ros::Time(0));
        transform_map_camera = transform.transform;

        
        // 提取时间戳和位姿
        ros::Time timestamp = transform.header.stamp;

        // 检查时间戳是否相同
        if (!g_camera_trajectory.empty()) {
            const auto& [last_timestamp, last_pose] = g_camera_trajectory.back();
            if (timestamp == last_timestamp) {
                return;
            }
        }

        geometry_msgs::Pose pose;
        pose.position.x = transform.transform.translation.x;
        pose.position.y = transform.transform.translation.y;
        pose.position.z = transform.transform.translation.z;
        pose.orientation = transform.transform.rotation;

        // 更新 camera 轨迹
        updateCameraTrajectory(timestamp, pose);


    } catch (tf2::TransformException& ex) {
        ROS_WARN("TF 变换异常: %s", ex.what());
        return;  // 如果 TF 变换失败，直接返回
    }

    // 更新地图
    g_gmapper->updateMap(cloud_msg, transform_map_camera);

    // 用 OpenCV 显示地图
    // cv::Mat map = g_map->toCvMat();
    // cv::imshow("map", map);
    // cv::waitKey(1);

    // 发布地图
    nav_msgs::OccupancyGrid occ_map;
    g_map->toRosOccGridMap("map", occ_map);
    g_map_puber.publish(occ_map);
}
