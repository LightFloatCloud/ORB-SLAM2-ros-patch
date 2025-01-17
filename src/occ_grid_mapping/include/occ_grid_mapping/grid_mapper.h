// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef GRID_MAPPER_H
#define GRID_MAPPER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <occ_grid_mapping/grid_map.h>
#include <occ_grid_mapping/Pose2d.h>


class GridMapper{
public:
    GridMapper(GridMap* map,  Pose2d& T_r_l,  double& P_occ, double& P_free, double& P_prior);
    void updateMap(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, geometry_msgs::Transform& transform); //根据当前机器人的位姿和相机点云更新一次地图
    void updateGrid(const Eigen::Vector2d& grid, const double& pmzx, const double& length);
    double laserInvModel(const double& r, const double& R, const double& cell_size);
    
private:
    GridMap* map_;
    Pose2d T_r_l_; // robot坐标系到sensor坐标系的变换
    double P_occ_, P_free_, P_prior_;
    
}; //class GridMapper

#endif

