# ORB-SLAM2-ros-patch

这是一个基于ORB-SLAM2的ROS包集合，用于实现单目视觉SLAM、点云处理、栅格地图构建等功能。
需与 https://github.com/LightFloatCloud/ORB-SLAM2-with_GroundPlane 进行搭配

## 用法
- **编译**
  ```bash
  catkin build
  ```
  或
  ```bash
  catkin_make
  ```
- **运行launch文件**（需要配置路径）
  - 启动单目SLAM + 栅格地图
    ```bash
    source devel/setup.bash
    roslaunch orbslam-ros all.launch
    ```
  - 通过mp4启动单目SLAM
    ```bash
    source devel/setup.bash
    roslaunch orbslam-ros intergrated.launch
    ```

## 功能包介绍

### orbslam-ros
- **作用**: 实现单目视觉SLAM功能。通过ORB特征点检测和匹配，构建相机的位姿估计和地图构建。
- **主要文件**:
  - `src/ros_mono.cc`: 单目SLAM的主要实现代码。
  - `launch/*.launch`: 各种启动文件，用于配置和启动SLAM系统。
  - `param/*.yaml`: 配置文件，包含相机参数、ORB特征提取参数等。

#### Launch文件用法
- **Mono.launch**
  - **作用**: 单目SLAM系统各参数配置文件。
  - **参数**:
    - `Mono_image_topic`: 输入的单目图像话题，默认为`/camera/color/image_raw`。
    - `path_to_vocabulary`: ORB词汇表文件路径，默认为`/root/ORB_SLAM2/Vocabulary/ORBvoc.bin`。
    - `path_to_settings`: SLAM配置文件路径，默认为`/root/catkin_ws/src/orbslam-ros/launch/normal.yaml`
    - `path_to_output`: 输出路径

- **bagMono.launch**
  - **作用**: 从ROS bag文件中读取单目图像数据并启动单目SLAM系统。
  - **参数**:
    - `dataname`: 数据集名称
    - `path_to_settings`: SLAM配置文件路径
    - `path_to_output`: 输出路径
    - `Mono_image_topic`: 输入的单目图像话题，默认为`/kitti/camera_color_left/image_raw`。
  - **示例**:
    ```bash
    roslaunch orbslam-ros bagMono.launch
    ```

- **intergrated.launch**
  - **作用**: 从mp4文件中读取单目图像数据并启动单目SLAM系统。
  - **参数**:
    - `dataname`: 数据集名称
    - `path_to_settings`: SLAM配置文件路径
    - `path_to_output`: 输出路径
    - `start_time`: 从视频第n秒播放
  - **示例**:
    ```bash
    roslaunch orbslam-ros intergrated.launch
    ```

- **all.launch**
  - **作用**: 启动集成的SLAM系统，包括单目SLAM、点云处理和栅格地图构建。
  - **包含的launch文件**:
    - `intergrated.launch`: 集成的SLAM系统启动文件。
    - `mapping.launch`: 地图构建启动文件。
  - **示例**:
    ```bash
    roslaunch orbslam-ros all.launch
    ```

### occ_grid_mapping
- **作用**: 实现基于占用网格的地图构建。通过点云数据更新地图，生成栅格地图。
- **主要文件**:
  - `src/grid_map.cc`: 网格地图的数据结构实现。
  - `src/grid_mapper.cc`: 地图更新算法实现。
  - `launch/mapping.launch`: 启动文件，用于配置和启动地图构建节点。
  - `config/default.yaml`: 配置文件，包含地图参数等。

#### Launch文件用法
- **mapping.launch**
  - **作用**: 启动地图构建节点，并使用RViz可视化地图。
  - **配置文件**:
    - `config/default.yaml`: 地图和传感器模型的配置参数。
  - **示例**:
    ```bash
    roslaunch occ_grid_mapping mapping.launch
    ```

### pcd2pgm（弃用）
- **作用**: 将点云数据转换为栅格地图，并发布为ROS话题。
- **主要文件**:
  - `src/pcd2pgm.cpp`: 主要实现代码，负责点云滤波和栅格地图生成。
  - `launch/mp42topic.launch`: 启动文件，用于配置和启动点云转栅格地图节点。

### mp42bag
- **作用**: 将MP4视频文件转换为ROS bag文件，或发布为ROS话题。
- **主要文件**:
  - `src/mp42bag_node.cpp`: 将MP4视频转换为ROS bag文件。
  - `src/mp42topic_node.cpp`: 将MP4视频发布为ROS话题。
  - `launch/mp42rosbag.launch`: 启动文件，用于配置和启动MP4转ROS bag节点。
  - `launch/mp42topic.launch`: 启动文件，用于配置和启动MP4转ROS话题节点。


#### Launch文件用法
- **mp42rosbag.launch**
  - **作用**: 将MP4视频文件转换为ROS bag文件。
  - **参数**:
    - `path_mp4`: MP4视频文件路径。
    - `path_bag`: 输出的ROS bag文件路径。
    - `topic_advertise`: 发布的话题名称，默认为`/camera/image_raw`。
    - `frame_id`: 帧ID，默认为`camera_frame`。
    - `compress_type`: 压缩类型，默认为`jpg`。
  - **示例**:
    ```bash
    roslaunch mp42bag mp42rosbag.launch path_mp4:=/mnt/test.mp4 path_bag:=/mnt/test.bag
    ```

- **mp42topic.launch**
  - **作用**: 将MP4视频文件发布为ROS话题。
  - **参数**:
    - `path_mp4`: MP4视频文件路径。
    - `topic_advertise`: 发布的话题名称，默认为`/camera/color/image_raw`。
    - `frame_id`: 帧ID，默认为`camera_frame`。
    - `Bool_restart`: 是否循环播放视频，默认为`false`。
    - `Bool_Info_show`: 是否显示帧数信息，默认为`true`。
    - `start_time`: 视频起始时间，默认为`0.0`秒。
    - `Output_FrameRate`: 输出帧率，默认为`30.0`。
  - **示例**:
    ```bash
    roslaunch mp42bag mp42topic.launch
    ```

