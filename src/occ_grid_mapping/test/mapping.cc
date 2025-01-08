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

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
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
    g_map = new GridMap ( map_sizex, map_sizey,  map_initx, map_inity, map_cell_size );
    g_gmapper = new GridMapper ( g_map, T_r_l, P_occ, P_free, P_prior );

    
    // 初始化 TF 监听器
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    g_tf_buffer = &tf_buffer;  // 将 TF 缓冲区赋值给全局变量
    /***** 初始Topic *****/
    // g_odom_suber = nh.subscribe ( "/mbot/odometry", 1, odometryCallback );
    // ros::Subscriber laser_suber = nh.subscribe ( "/scan", 1, laserCallback );
    ros::Subscriber cloud_suber = nh.subscribe("/points", 1, pointCloudCallback);
    g_map_puber = nh.advertise<nav_msgs::OccupancyGrid> ( "mapping/grid_map", 1 );
    
    ros::spin();

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

        // 提取姿态（四元数）
        geometry_msgs::Quaternion quat = transform.transform.rotation;
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quat, tf_quat);

        // 提取 camera 的 z 轴方向（在 map 坐标系下的方向）
        tf2::Vector3 z_axis_camera = tf2::quatRotate(tf_quat, tf2::Vector3(0, 0, 1));
        double theta = atan2(z_axis_camera.y(), z_axis_camera.x());
        ROS_INFO("Current yaw angle (theta): %f radians", theta * 180.0 / M_PI);

    } catch (tf2::TransformException& ex) {
        ROS_WARN("TF 变换异常: %s", ex.what());
        return;  // 如果 TF 变换失败，直接返回
    }

    // 更新地图
    g_gmapper->updateMap(cloud_msg, transform_map_camera);

    // 用 OpenCV 显示地图
    cv::Mat map = g_map->toCvMat();
    cv::imshow("map", map);
    cv::waitKey(1);

    // 发布地图
    nav_msgs::OccupancyGrid occ_map;
    g_map->toRosOccGridMap("map", occ_map);
    g_map_puber.publish(occ_map);
}
