// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <occ_grid_mapping/grid_mapper.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // 包含 tf2::fromMsg

GridMapper::GridMapper ( GridMap* map, Pose2d& T_r_l, double& P_occ, double& P_free, double& P_prior):
map_(map), T_r_l_(T_r_l), P_occ_(P_occ), P_free_(P_free), P_prior_(P_prior)
{
    
}

void GridMapper::updateMap (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, geometry_msgs::Transform& transform) 
{

    // 获取地图的分辨率
    const double& cell_size = map_->getCellSize();
    const double inc_step = 1.0 * cell_size;
    // 使用 PointCloud2Iterator 遍历点云
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    tf2::Transform tf_transform;
    tf2::fromMsg(transform, tf_transform);
    
    // 提取相机的平移部分（camera 在 map 坐标系下的位置）
    double camera_x = transform.translation.x;
    double camera_y = transform.translation.y;
    double camera_z = transform.translation.z;

    // 遍历点云中的每个点
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        // 获取当前点的坐标
        double x_c = *iter_x;
        double y_c = *iter_y;
        double z_c = *iter_z;
        
        // 将点从 camera 坐标系转换到 map 坐标系
        tf2::Vector3 point_camera(x_c, y_c, z_c);
        tf2::Vector3 point_map = tf_transform * point_camera;
        
        // 计算点在 map 坐标系下的相对位置（减去相机的位置）
        double x_relative = point_map.x() - camera_x;
        double y_relative = point_map.y() - camera_y;
        
        // 将点云数据转换到世界坐标系
        Eigen::Vector2d p_relative(x_relative, y_relative);  // camera投影下的坐标
        // 计算点到相机的距离
        double R = p_relative.norm();  

        // 沿着激光射线更新地图
        Eigen::Vector2d last_grid(Eigen::Infinity, Eigen::Infinity);  // 上一步更新的 grid 位置
        for (double r = 0; r < R + cell_size; r += inc_step) {
            Eigen::Vector2d p_l(
                r * (x_relative / R), 
                r * (y_relative / R)
            );

            // 转换到世界坐标系
            Eigen::Vector2d p_w = Eigen::Vector2d(camera_x, camera_y) + p_l;

            // 避免重复更新
            if (p_w == last_grid) continue;

            // 更新栅格
            updateGrid(p_w, laserInvModel(r, R, cell_size));

            last_grid = p_w;
        }
    }
}

void GridMapper::updateGrid ( const Eigen::Vector2d& grid, const double& pmzx )
{
    /* TODO 这个过程写的太低效了 */
    double log_bel;
    if(  ! map_->getGridLogBel( grid(0), grid(1), log_bel )  ) //获取log的bel
        return;
    log_bel += log( pmzx / (1.0 - pmzx) ); //更新
    map_->setGridLogBel( grid(0), grid(1), log_bel  ); //设置回地图
}

double GridMapper::laserInvModel ( const double& r, const double& R, const double& cell_size )
{
    if(r < ( R - 0.5*cell_size) )
        return P_free_;
    
    if(r > ( R + 0.5*cell_size) )
        return P_prior_;
    
    return P_occ_;
}

