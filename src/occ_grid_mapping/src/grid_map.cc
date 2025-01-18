// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <occ_grid_mapping/grid_map.h>

GridMap::GridMap ( const int& size_x, const int& size_y, const int& init_x, const int& init_y, const double& cell_size , const double& init_bel ) :
    size_x_ ( size_x ), size_y_ ( size_y ), init_x_ ( init_x ), init_y_ ( init_y ), cell_size_ ( cell_size )
{
    bel_data_.resize ( size_x_, size_y_ );
    // bel_data_.setOnes() *= init_bel; //全部设为0.5的概率
    bel_data_.setConstant(init_bel); // 直接设置所有元素为 init_bel
    
}

bool GridMap::getIdx ( const double& x, const double& y, Eigen::Vector2i& idx )
{
    int xidx = cvFloor( x / cell_size_ ) + init_x_;
    int yidx  = cvFloor( y /cell_size_ )+ init_y_;
    
    // TODO 动态扩张地图
    if((xidx < 0) || (yidx < 0) || (xidx >= size_x_) || (yidx >= size_y_))
        return false;
    idx << xidx , yidx;
    return true;
}
bool GridMap::getIdx(const int& x, const int& y, Eigen::Vector2i& idx)
{
    int xidx = x + init_x_;
    int yidx = y+ init_y_;
    // TODO 动态扩张地图
    if((xidx < 0) || (yidx < 0) || (xidx >= size_x_) || (yidx >= size_y_))
        return false;
    idx << xidx , yidx;
    return true;
}

template <typename T>
bool GridMap::getGridBel ( const T& x, const T& y, double& bel)
{
    Eigen::Vector2i idx;
    if(!getIdx(x, y, idx))
        return false;
    bel = bel_data_(idx(0), idx(1));
    return true;
}

template <typename T>
bool GridMap::setGridBel ( const T& x, const T& y, const double& bel )
{
    Eigen::Vector2i idx;
    if(!getIdx(x, y, idx))
        return false;
    bel_data_(idx(0), idx(1)) = bel;
    return true;
}

template <typename T>
bool GridMap::getGridLogBel ( const T& x, const T& y, double& log_bel )
{
    double bel;
    if(!getGridBel(x, y, bel))
        return false;
    log_bel = log( bel / (1.0-bel));
    return true;
}

template <typename T>
bool GridMap::setGridLogBel ( const T& x, const T& y, const double& log_bel )
{
    double bel = 1.0 - 1.0 / (1 + exp(log_bel));
    if (!setGridBel(x, y, bel)) {
        std::cerr << "Failed to set grid belief at (" << x << ", " << y << ")" << std::endl;
        return false;
    }
    return true;
}

double GridMap::getCellSize()
{
    return cell_size_;
}

cv::Mat GridMap::toCvMat()
{
    /* 构造opencv mat */
    cv::Mat map(cv::Size(size_x_, size_y_), CV_64FC1);
    
    // 使用指针访问数据，提高访问速度
    double* map_ptr = map.ptr<double>(0);
    const double* bel_data_ptr = bel_data_.data();

    for (int i = 0; i < size_x_ * size_y_; ++i) {
        map_ptr[i] = 1.0 - bel_data_ptr[i]; // 翻转数值
    }

    /* 翻转 */
    cv::flip(map, map, 0);

    return map;
}


void GridMap::toRosOccGridMap(const std::string& frame_id, nav_msgs::OccupancyGrid& occ_grid)
{
    // 设置头信息
    occ_grid.header.frame_id = frame_id;
    occ_grid.header.stamp = ros::Time::now();

    // 设置地图信息
    occ_grid.info.width = size_x_;
    occ_grid.info.height = size_y_;
    occ_grid.info.resolution = cell_size_;
    occ_grid.info.origin.position.x = -init_x_ * cell_size_;
    occ_grid.info.origin.position.y = -init_y_ * cell_size_;

    // 获取网格总数
    const int N = size_x_ * size_y_;

    // 确保 bel_data_.data() 的大小与预期一致
    if (N != static_cast<int>(bel_data_.size())) {
        throw std::runtime_error("Size mismatch between grid and belief data");
    }

    // 清空 occ_grid.data 并预分配内存
    occ_grid.data.clear();
    occ_grid.data.reserve(N);

    // 定义一个小的容差值用于浮点数比较
    const double epsilon = 1e-6;

    // 预先将所有值设为 -1
    occ_grid.data.assign(N, -1);

    // 填充 occupancy grid 数据
    for (size_t i = 0; i < N; ++i) {
        double value = bel_data_.data()[i];
        if (std::abs(value - 0.5) >= epsilon) {
            occ_grid.data[i] = static_cast<int8_t>(value * 100);
        }
    }
}

void GridMap::saveMap ( const std::string& img_dir, const std::string& cfg_dir )
{
    /* 保存图片 */
    cv::Mat img = toCvMat();
    img = img * 255;
    cv::imwrite(img_dir, img);
    
    /* 保存配置 */
    std::ofstream  file;
    file.open(cfg_dir); 
    file << "map:"<< std::endl
    << "  size_x: " << size_x_ << std::endl
    << "  size_y: " << size_y_ << std::endl
    << "  init_x: " << init_x_ << std::endl
    << "  init_y: " << init_y_ << std::endl
    << "  cell_size: " << cell_size_ << std::endl;
}


// 显式实例化模板函数
template bool GridMap::getGridBel<int>(const int&, const int&, double&);
template bool GridMap::setGridBel<int>(const int&, const int&, const double&);
template bool GridMap::getGridLogBel<int>(const int&, const int&, double&);
template bool GridMap::setGridLogBel<int>(const int&, const int&, const double&);

template bool GridMap::getGridBel<double>(const double&, const double&, double&);
template bool GridMap::setGridBel<double>(const double&, const double&, const double&);
template bool GridMap::getGridLogBel<double>(const double&, const double&, double&);
template bool GridMap::setGridLogBel<double>(const double&, const double&, const double&);





