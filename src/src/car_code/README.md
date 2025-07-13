# UVPTCCS 机器人系统 README 文档

## 项目概述
UVPTCCS (Unmanned Vehicle Positioning Tracking Control and Communication System) 是一个基于ROS2的无人车定位、追踪、控制和通信系统。该系统提供了完整的机器人功能包集合，包括底盘驱动、视觉追踪、导航建图、SLAM、位姿发布等功能。

## 系统架构
```
UVPTCCS/
├── car_astra/          # 颜色追踪与识别包
├── car_bringup/        # 机器人底盘驱动包
├── car_nav/            # 机器人导航包
├── car_slam/           # SLAM与点云处理包
├── robot_pose_publisher_ros2/  # 机器人位姿发布包
└── laserscan_to_point_pulisher/  # 激光雷达点云转换包
```

## 包功能说明

### 1. car_astra - 颜色追踪与识别包
**主要功能**:
- 基于深度摄像头的颜色目标识别
- HSV颜色空间参数调整
- 实时颜色目标追踪和跟随
- PID控制器优化运动控制

**核心节点**:
- `colorHSV`: 颜色识别和HSV参数调整
- `colorTracker`: 基于深度信息的目标追踪

**应用场景**: 目标跟随、颜色识别、视觉导航

### 2. car_bringup - 机器人底盘驱动包
**主要功能**:
- 支持多种机器人型号(X1、X3、R2)
- 底层运动控制和传感器数据发布
- RGB灯光和蜂鸣器控制
- 自主巡逻和校准功能

**核心节点**:
- `Mcnamu_driver_X3`: X3型机器人驱动
- `Mcnamu_driver_x1`: X1型机器人驱动
- `Ackman_driver_R2`: R2型阿克曼转向机器人驱动
- `patrol_*`: 各种巡逻功能节点

**应用场景**: 机器人底层控制、传感器集成、自主巡逻

### 3. car_nav - 机器人导航包
**主要功能**:
- 多种SLAM建图算法(Gmapping、Cartographer、RTABMap)
- 路径规划和自主导航(DWA、TEB)
- 激光雷达数据处理和滤波
- 地图保存和加载

**核心节点**:
- `scan_filter`: 激光雷达数据滤波

**应用场景**: 自主导航、环境建图、路径规划

### 4. car_slam - SLAM与点云处理包
**主要功能**:
- 基于ORB-SLAM的视觉SLAM
- RGB-D相机点云建图
- 八叉树地图构建
- 实时点云处理和可视化

**核心节点**:
- `pointcloud_mapping`: 点云建图节点

**应用场景**: 视觉SLAM、3D建图、环境感知

### 5. robot_pose_publisher_ros2 - 机器人位姿发布包
**主要功能**:
- 基于TF变换的位姿计算
- 实时机器人位姿发布
- 支持多种消息格式

**核心节点**:
- `robot_pose_publisher`: 位姿发布节点

**应用场景**: 位姿监控、导航状态反馈、数据记录

### 6. laserscan_to_point_pulisher - 激光雷达点云转换包
**主要功能**:
- 激光雷达数据转换为点云
- 坐标系转换和数据处理
- 实时数据可视化

**核心节点**:
- `laserscanToPointPublish`: 激光雷达点云转换节点

**应用场景**: 数据转换、可视化、环境感知

## 系统要求

### 硬件要求
- 支持的机器人平台: X1、X3、R2系列
- 传感器: 激光雷达、RGB-D相机、IMU
- 计算平台: 支持ROS2的Linux系统

### 软件依赖
- **操作系统**: Ubuntu 20.04/22.04
- **ROS版本**: ROS2 Foxy/Galactic/Humble
- **核心库**:
  - OpenCV (计算机视觉)
  - PCL (点云处理)
  - Nav2 (导航功能)
  - tf2_ros (坐标变换)
  - Cartographer (SLAM)
  - RTABMap (视觉SLAM)

## 快速开始

### 1. 环境准备
```bash
# 安装ROS2
sudo apt update
sudo apt install ros-humble-desktop-full

# 安装依赖
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-rtabmap-ros
```

### 2. 编译系统
```bash
# 创建工作空间
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# 克隆代码
git clone <repository_url>

# 编译
cd ~/robot_ws
colcon build --symlink-install

# 设置环境
source install/setup.bash
```

### 3. 启动系统

#### 启动机器人底盘
```bash
# X3机器人
ros2 launch car_bringup car_bringup_X3_launch.py

# R2机器人
ros2 launch car_bringup car_bringup_R2_launch.py
```

#### 启动颜色追踪
```bash
ros2 launch car_astra colorTracker_X3.launch.py
```

#### 启动导航系统
```bash
# SLAM建图
ros2 launch car_nav map_gmapping_launch.py

# 自主导航
ros2 launch car_nav navigation_dwa_launch.py
```

#### 启动SLAM系统
```bash
# 点云建图
ros2 launch car_slam orbslam_pcl_map_launch.py

# 八叉树建图
ros2 launch car_slam orbslam_pcl_octomap_launch.py
```

## 使用指南

### 1. 机器人控制
```bash
# 手动控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 查看机器人状态
ros2 topic echo /robot_pose
ros2 topic echo /voltage
```

### 2. 建图导航
```bash
# 开始建图
export RPLIDAR_TYPE=a1
ros2 launch car_nav map_gmapping_launch.py

# 保存地图
ros2 launch car_nav save_map_launch.py

# 开始导航
ros2 launch car_nav navigation_dwa_launch.py
```

### 3. 视觉功能
```bash
# 颜色识别
ros2 run car_astra colorHSV

# 目标追踪
ros2 run car_astra colorTracker
```

### 4. 数据可视化
```bash
# 启动RViz
ros2 run rviz2 rviz2

# 查看点云
ros2 launch car_slam display_pcl_launch.py

# 查看导航
ros2 launch car_nav display_nav_launch.py
```

## 配置说明

### 1. 机器人参数配置
根据实际机器人型号修改对应的参数文件:
- `car_bringup/param/`: 机器人底盘参数
- `car_nav/params/`: 导航算法参数
- `car_slam/params/`: SLAM算法参数

### 2. 传感器校准
- 激光雷达: 使用`calibrate_*`节点进行校准
- 相机: 修改`car_slam/params/`中的相机参数
- IMU: 调整`car_bringup/param/imu_filter_param.yaml`

### 3. 坐标系配置
确保以下坐标系正确配置:
- `base_link`: 机器人基座
- `base_footprint`: 机器人地面投影
- `laser`: 激光雷达坐标系
- `camera`: 相机坐标系
- `map`: 全局地图坐标系
- `odom`: 里程计坐标系

## 故障排除

### 1. 常见问题
- **TF变换错误**: 检查坐标系配置和时间同步
- **激光雷达无数据**: 检查设备连接和驱动
- **相机图像异常**: 检查相机参数和USB连接
- **导航失败**: 检查地图质量和参数配置

### 2. 调试工具
```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 检查话题
ros2 topic list
ros2 topic echo /topic_name

# 查看节点状态
ros2 node list
ros2 node info /node_name

# 参数调试
ros2 param list /node_name
ros2 param get /node_name parameter_name
```

## 开发指南

### 1. 代码结构
- 遵循ROS2标准包结构
- 使用Python或C++开发节点
- 添加适当的错误处理和日志

### 2. 测试验证
- 单元测试: 使用pytest或gtest
- 集成测试: 验证节点间通信
- 性能测试: 监控CPU和内存使用

### 3. 文档维护
- 更新README文档
- 添加代码注释
- 记录参数说明

## 许可证
本项目遵循开源许可证，具体请查看LICENSE文件。

## 贡献指南
欢迎提交问题报告和改进建议。请遵循以下步骤:
1. Fork项目
2. 创建特性分支
3. 提交更改
4. 发起Pull Request

## 联系方式
如有问题或建议，请通过以下方式联系:
- 邮箱: [维护者邮箱]
- 项目地址: [GitHub链接]
- 文档地址: [文档链接] 