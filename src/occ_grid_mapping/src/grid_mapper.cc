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

    tf2::Transform tf_transform;
    tf2::fromMsg(transform, tf_transform);
    
    // 提取相机的平移部分（camera 在 map 坐标系下的位置）
    double camera_x = transform.translation.x;
    double camera_y = transform.translation.y;
    double camera_z = transform.translation.z;
    
    // 获取camera的朝向向量在map坐标系下的投影
    tf2::Vector3 camera_forward(0, 0, 1);
    tf2::Quaternion rotation = tf_transform.getRotation();
    camera_forward = tf2::quatRotate(rotation, camera_forward); // 使用 quatRotate 旋转向量
    camera_forward.setZ(0); // 投影到map的xy平面
    double camera_heading_angle = atan2(camera_forward.y(), camera_forward.x());


    // 定义激光扫描的参数
    // const double angle_min = -M_PI / 180 * 35.2;  // 最小角度    -35.2
    // const double angle_max = M_PI / 180 * 35.2;   // 最大角度     35.2
    // const double angle_increment = M_PI / 180.0 * 1;  // 角度增量（1度）  一个像素[0.12]
    const double range_min = 0.1;    // 最小距离
    const double range_max = 10.0;   // 最大距离
    const double z_max = camera_z * 1.25;        // 最大高度阈值
    
    // 初始化 angle_min 和 angle_max
    double angle_min = std::numeric_limits<double>::max();
    double angle_max = std::numeric_limits<double>::lowest();
    
    // 存储每个点的 relative_angle 和 range
    std::vector<std::pair<double, double>> point_data;

    // 使用 PointCloud2Iterator 遍历点云
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    // 遍历点云中的每个点
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        // 获取当前点的坐标
        double x_c = *iter_x;
        double y_c = *iter_y;
        double z_c = *iter_z;
        
        
        // 将点从 camera 坐标系转换到 map 坐标系
        tf2::Vector3 point_camera(x_c, y_c, z_c);
        tf2::Vector3 point_map = tf_transform * point_camera;

        // 过滤高度超过 z_max 的点
        if (point_map.z() > z_max) {
            ROS_INFO("z_c: %.3f, z_max: %.3f",z_c, z_max);
            continue;
        }
        
        // 计算点在 map 坐标系下的相对位置（减去相机的位置）
        double x_relative = point_map.x() - camera_x;
        double y_relative = point_map.y() - camera_y;
        // 计算点相对于camera投影方向的角度
        double point_angle = atan2(y_relative, x_relative);
        double relative_angle = point_angle - camera_heading_angle;
        relative_angle = atan2(sin(relative_angle), cos(relative_angle)); // 将角度映射到[-pi, pi]

        // 更新 angle_min 和 angle_max
        angle_min = std::min(angle_min, relative_angle);
        angle_max = std::max(angle_max, relative_angle);

        // 计算点的距离
        double range = hypot(x_relative, y_relative);

        // 过滤无效点
        if (range < range_min || range > range_max) {
            ROS_WARN("relative_angle: %.3f, range: %.3f", relative_angle / PI * 180, range);
            continue;
        }

        // 存储 relative_angle 和 range
        point_data.emplace_back(relative_angle, range);
    }

    // 计算激光束的数量和角度增量
    const double angle_increment = M_PI / 180.0 * 1;  // 角度增量（1度）  一个像素[0.12]
    size_t num_beams = std::ceil((angle_max - angle_min) / angle_increment);
    // 初始化每个激光束的最小距离
    std::vector<double> ranges(num_beams, std::numeric_limits<double>::infinity());

    // 遍历存储的点数据以更新 ranges
    for (const auto& [relative_angle, range] : point_data) {
        // 找到对应的激光束索引
        int index = static_cast<int>((relative_angle - angle_min) / angle_increment);
        if (index < 0 || index >= num_beams) {
            continue;
        }
        // 更新最小距离
        if (range < ranges[index]) {
            ranges[index] = range;
        }
    }
    
    // 遍历每个激光束，更新地图
    for (size_t i = 0; i < num_beams; ++i) {
        double R = ranges[i];
        if (R >= range_max || R < range_min) {
            continue;
        }
        
        // 计算当前激光束的角度（相对于camera投影坐标系）
        double beam_angle = angle_min + i * angle_increment;
        double beam_angle_global = beam_angle + camera_heading_angle;
        beam_angle_global = atan2(sin(beam_angle_global), cos(beam_angle_global)); // 将角度映射到[-pi, pi]

        double cangle = cos(beam_angle_global);
        double sangle = sin(beam_angle_global);

        // 沿着激光射线更新地图
        Eigen::Vector2d last_grid(Eigen::Infinity, Eigen::Infinity);  // 上一步更新的 grid 位置
        for (double r = 0; r < R + cell_size; r += inc_step) {
            Eigen::Vector2d p_l(
                r * cangle,
                r * sangle
            );  // 在相机投影坐标系下的坐标


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

