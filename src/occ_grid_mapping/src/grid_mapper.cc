// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <occ_grid_mapping/grid_mapper.h>

#include <sensor_msgs/point_cloud2_iterator.h>

GridMapper::GridMapper ( GridMap* map, Pose2d& T_r_l, double& P_occ, double& P_free, double& P_prior):
map_(map), T_r_l_(T_r_l), P_occ_(P_occ), P_free_(P_free), P_prior_(P_prior)
{
    
}

std::vector<std::pair<Eigen::Vector2i, double>> GridMapper::AmanatidesWoo(
    const Eigen::Vector2d& start, const Eigen::Vector2d& end, double cell_size, Eigen::Vector2i& last_cell)
{
    std::vector<std::pair<Eigen::Vector2i, double>> crossedCells;
    Eigen::Vector2d direction = end - start;
    double length = direction.norm(); // 射线的总长度
    if (length == 0) return crossedCells;

    direction.normalize(); // 归一化方向向量
    int step_x = (direction.x() >= 0) ? 1 : -1;
    int step_y = (direction.y() >= 0) ? 1 : -1;

    Eigen::Vector2i current_cell(cvFloor(start.x() / cell_size), cvFloor(start.y() / cell_size));
    double t_max_x = (step_x > 0)
        ? (std::ceil(start.x() / cell_size) * cell_size - start.x()) / direction.x()
        : (start.x() - std::floor(start.x() / cell_size) * cell_size) / (-direction.x());
    double t_max_y = (step_y > 0)
        ? (std::ceil(start.y() / cell_size) * cell_size - start.y()) / direction.y()
        : (start.y() - std::floor(start.y() / cell_size) * cell_size) / (-direction.y());

    double delta_t_x = (step_x > 0)
        ? cell_size / direction.x()
        : cell_size / (-direction.x());
    double delta_t_y = (step_y > 0)
        ? cell_size / direction.y()
        : cell_size / (-direction.y());

    double t = 0.0;
    while (t < length)
    {
        crossedCells.emplace_back(current_cell, std::min(t_max_x, t_max_y) - t);
        if (t_max_x < t_max_y)
        {
            t = t_max_x;
            t_max_x += delta_t_x;
            current_cell.x() += step_x;
        }
        else
        {
            t = t_max_y;
            t_max_y += delta_t_y;
            current_cell.y() += step_y;
        }
    }
    
    // 返回最后一个栅格，弹出最后一个元素并赋给 last_cell
    if (!crossedCells.empty()) {
        last_cell = crossedCells.back().first;
        crossedCells.pop_back();
    } else {
        // 处理 crossedCells 为空的情况
        last_cell = current_cell; // 或者其他适当的默认值
    }
    return crossedCells;
}
void GridMapper::updateMap (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, geometry_msgs::Transform& transform) 
{

    // 获取地图的分辨率
    const double& cell_size = map_->getCellSize();
    const double inc_step = 1.0 * cell_size;

    tf2::Transform tf_transform;
    tf2::fromMsg(transform, tf_transform);
    
    // 提取相机的平移部分（camera 在 map 坐标系下的位置）
    tf2::Vector3 camera_translation = tf_transform.getOrigin();
    double camera_x = camera_translation.x();
    double camera_y = camera_translation.y();
    double camera_z = camera_translation.z();
    
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
    
    // 初始化 angle_min 和 angle_max
    double angle_min = std::numeric_limits<double>::max();
    double angle_max = std::numeric_limits<double>::lowest();
    
    // 存储每个点的 relative_angle 和 range
    std::vector<std::pair<double, double>> point_data = 
        preprocessPointCloud(cloud_msg, tf_transform, camera_translation, camera_heading_angle, angle_min, angle_max);

    
    // 计算激光束的数量和角度增量
    const double angle_increment = M_PI / 180.0 * 1;  // 角度增量（1度）  一个像素[0.12]

    // 处理激光束数据
    std::vector<double> ranges = processLaserBeams(point_data, angle_min, angle_max, angle_increment);
    
    // 遍历每个激光束，更新地图
    for (size_t i = 0; i < ranges.size(); ++i) {
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

        Eigen::Vector2d p_w = Eigen::Vector2d(camera_x, camera_y) + R * Eigen::Vector2d(cangle, sangle);

        // 获取穿过的栅格和长度
        Eigen::Vector2i last_cell;
        std::vector<std::pair<Eigen::Vector2i, double>> crossedCells =
            AmanatidesWoo(Eigen::Vector2d(camera_x, camera_y), p_w, cell_size, last_cell);

        for (const auto& cell : crossedCells)
        {
            updateGrid(cell.first, P_free_, cell.second / cell_size);
        }
        
        // 更新最后一个栅格为占据状态        
        if(R <= 2) {
            updateGrid(last_cell, P_occ_, 1.0);
            continue;
        } 
        // 高斯分布更新
        updateGridWithGaussian(p_w, R, cell_size);
    }
}

/**
 * @brief 处理激光束数据，计算每个激光束的最小距离
 * @param point_data 点云数据，包含每个点的相对角度和距离
 * @param angle_min 最小角度
 * @param angle_max 最大角度
 * @param angle_increment 角度增量
 * @return 每个激光束的最小距离
 */
std::vector<double> GridMapper::processLaserBeams(
    const std::vector<std::pair<double, double>>& point_data,
    double angle_min, double angle_max, double angle_increment) 
{
    // 计算激光束的数量
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

    return ranges;
}


// 点云数据预处理
std::vector<std::pair<double, double>> GridMapper::preprocessPointCloud(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const tf2::Transform& tf_transform, 
    const tf2::Vector3& camera_translation, const double& camera_heading_angle,
    double& angle_min, double& angle_max) {

    double camera_x = camera_translation.x();
    double camera_y = camera_translation.y();
    double camera_z = camera_translation.z();

    std::vector<std::pair<double, double>> point_data;
    // 定义激光扫描的参数
    // const double angle_min = -M_PI / 180 * 35.2;  // 最小角度    -35.2
    // const double angle_max = M_PI / 180 * 35.2;   // 最大角度     35.2
    // const double angle_increment = M_PI / 180.0 * 1;  // 角度增量（1度）  一个像素[0.12]
    const double range_min = 0.1;    // 最小距离
    const double range_max = 10.0;   // 最大距离
    const double z_max = camera_z * 1.25; // 最大高度阈值

    // 使用 PointCloud2Iterator 遍历点云
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
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

        // 计算点的距离
        double range = hypot(x_relative, y_relative);

        // 过滤无效点
        if (range < range_min || range > range_max) {
            ROS_WARN("relative_angle: %.3f, range: %.3f", relative_angle / PI * 180, range);
            continue;
        }
        // 更新 angle_min 和 angle_max
        angle_min = std::min(angle_min, relative_angle);
        angle_max = std::max(angle_max, relative_angle);
        // 存储 relative_angle 和 range
        point_data.emplace_back(relative_angle, range);
    }

    return point_data;
}


void GridMapper::updateGridWithGaussian(const Eigen::Vector2d& p_w, double range, double cell_size) {
    
    // 高斯分布更新
    // 2m 对应 1grid(0.05 m) = 2标准差
    const double sigma = range * (cell_size / 2 / 2.0); // 高斯分布的标准差
    const double two_sigma = 2.0 * sigma; // 2 个标准差的范围

    // 计算圆的边界
    double x_min = p_w.x() - two_sigma;
    double x_max = p_w.x() + two_sigma;
    double y_min = p_w.y() - two_sigma;
    double y_max = p_w.y() + two_sigma;

    // 获取 p_w 所在的栅格坐标
    // Eigen::Vector2i p_w_cell(cvFloor(p_w.x() / cell_size), cvFloor(p_w.y() / cell_size));

    // 计算需要遍历的栅格范围
    int x_min_cell = cvRound(x_min / cell_size);
    int x_max_cell = cvRound(x_max / cell_size);
    int y_min_cell = cvRound(y_min / cell_size);
    int y_max_cell = cvRound(y_max / cell_size);

    // 遍历可能被圆覆盖的栅格
    for (int x = x_min_cell; x <= x_max_cell; ++x) {
        for (int y = y_min_cell; y <= y_max_cell; ++y) {
            Eigen::Vector2i cell(x, y);

            // 计算栅格中心点到 p_w 的距离
            Eigen::Vector2d cell_center = cell.cast<double>() * cell_size + Eigen::Vector2d(cell_size / 2.0, cell_size / 2.0);
            double distance = (cell_center - p_w).norm();

            // 如果距离在 2 个标准差以内，则更新栅格
            if (distance <= two_sigma) {
                // 计算高斯分布的概率体积
                double gaussian_value = std::exp(-0.5 * (distance * distance) / (sigma * sigma)) / (2 * M_PI * sigma * sigma);
                double volume = gaussian_value * cell_size * cell_size;
                // 更新栅格
                updateGrid(cell, P_occ_, volume);
            }
        }
    }
}


void GridMapper::updateGrid(const Eigen::Vector2i& grid, const double& pmzx, const double& unit)
{
    double log_bel;
    if (!map_->getGridLogBel(grid(0), grid(1), log_bel))
        return;
    // std::cout << "unit: " << unit << std::endl;
    double log_odds = log(pmzx / (1.0 - pmzx)) * unit;
    log_bel += log_odds;
    map_->setGridLogBel(grid(0), grid(1), log_bel);
}

