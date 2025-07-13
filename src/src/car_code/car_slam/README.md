# ROS2 SLAM与点云处理包 README 文档

## 包名称
**car_slam**

## 功能描述
该包提供视觉SLAM和点云处理功能，主要基于ORB-SLAM和点云建图技术。包含点云生成、滤波、可视化、八叉树地图构建等功能，适用于机器人三维环境感知和建图。

## 节点列表

### 1. pointcloud_mapping (点云建图节点)
- **节点名称**: `pointcloud_mapping`
- **可执行文件**: `point_cloud_main`
- **功能**: 基于RGB-D相机的实时点云建图

#### 发布接口
| 话题名称 | 消息类型 | 描述 |
|:---------|:---------|:-----|
| `/Global/PointCloudOutput` | `sensor_msgs/msg/PointCloud2` | 发布全局点云地图 |
| `/Local/PointCloudOutput` | `sensor_msgs/msg/PointCloud2` | 发布局部点云地图 |

#### 订阅接口
| 话题名称 | 消息类型 | 回调函数 | 描述 |
|:---------|:---------|:---------|:-----|
| `/RGBD/RGB/Image` | `sensor_msgs/msg/Image` | `callback` | 订阅RGB图像数据 |
| `/RGBD/Depth/Image` | `sensor_msgs/msg/Image` | `callback` | 订阅深度图像数据 |
| `/RGBD/CameraPose` | `geometry_msgs/msg/PoseStamped` | `callback` | 订阅相机位姿数据 |

#### 参数配置
| 参数名称 | 类型 | 默认值 | 描述 |
|:---------|:-----|:-------|:-----|
| `topicColor` | string | "/RGBD/RGB/Image" | RGB图像话题名称 |
| `topicDepth` | string | "/RGBD/Depth/Image" | 深度图像话题名称 |
| `topicTcw` | string | "/RGBD/CameraPose" | 相机位姿话题名称 |
| `local_frame_id` | string | "camera" | 局部坐标系ID |
| `global_frame_id` | string | "camera" | 全局坐标系ID |
| `node_path` | string | "/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_slam/pcl/" | 点云文件保存路径 |
| `use_viewer` | bool | false | 是否启用PCL可视化 |
| `fx` | float | 517.306408 | 相机内参fx |
| `fy` | float | 516.469215 | 相机内参fy |
| `cx` | float | 318.643040 | 相机内参cx |
| `cy` | float | 255.313989 | 相机内参cy |
| `resolution` | float | 0.05 | 点云分辨率 |
| `depthfactor` | float | 1000.0 | 深度图缩放因子 |
| `queueSize` | int | 10 | 消息队列大小 |
| `buseExact` | bool | false | 是否使用精确时间同步 |

## 启动文件

### 1. orbslam_ros_launch.py
- **功能**: 启动ORB-SLAM系统
- **包含组件**:
  - ORB-SLAM节点
  - 相机驱动节点

### 2. orbslam_base_launch.py
- **功能**: 启动ORB-SLAM基础系统

### 3. orbslam_pose_launch.py
- **功能**: 启动ORB-SLAM位姿估计

### 4. orbslam_pcl_map_launch.py
- **功能**: 启动ORB-SLAM点云建图
- **包含组件**:
  - ORB-SLAM节点
  - 点云建图节点

### 5. orbslam_pcl_octomap_launch.py
- **功能**: 启动ORB-SLAM点云八叉树建图
- **包含组件**:
  - ORB-SLAM节点
  - 点云建图节点
  - 八叉树地图节点

### 6. camera_octomap_launch.py
- **功能**: 启动相机八叉树建图

### 7. octomap_server_launch.py
- **功能**: 启动八叉树地图服务器

### 8. display_octomap_launch.py
- **功能**: 显示八叉树地图
- **RViz配置**: `rviz/octomap.rviz`

### 9. display_pcl_launch.py
- **功能**: 显示点云地图
- **RViz配置**: `rviz/orbslam_pcl.rviz`

## 配置文件

### 相机参数文件

#### 1. mono.yaml
- **功能**: 单目相机参数配置
- **包含参数**:
  - 相机内参矩阵
  - 畸变系数
  - 图像尺寸

#### 2. rgbd.yaml
- **功能**: RGB-D相机参数配置
- **包含参数**:
  - RGB相机内参
  - 深度相机内参
  - 基线距离

#### 3. usb_cam_param.yaml
- **功能**: USB相机参数配置

### SLAM参数文件

#### 4. ORBvoc.txt
- **功能**: ORB特征词汇表
- **用途**: 用于回环检测和重定位

### 点云处理参数文件

#### 5. pointcloud_map.yaml
- **功能**: 点云建图参数配置
- **包含参数**:
  - 点云分辨率
  - 滤波参数
  - 可视化参数

#### 6. pointcloud_octomap.yaml
- **功能**: 点云八叉树建图参数配置
- **包含参数**:
  - 八叉树分辨率
  - 占用概率阈值
  - 更新参数

## 点云数据文件

### resultPointCloudFile.pcd
- **功能**: 保存的点云地图文件
- **格式**: PCD (Point Cloud Data)
- **用途**: 离线点云分析和可视化

## 使用说明

### 1. 启动ORB-SLAM系统
```bash
# 启动基础ORB-SLAM系统
ros2 launch car_slam orbslam_base_launch.py

# 启动带位姿估计的ORB-SLAM
ros2 launch car_slam orbslam_pose_launch.py

# 启动完整的ORB-SLAM系统
ros2 launch car_slam orbslam_ros_launch.py
```

### 2. 启动点云建图
```bash
# 启动点云建图
ros2 launch car_slam orbslam_pcl_map_launch.py

# 启动点云八叉树建图
ros2 launch car_slam orbslam_pcl_octomap_launch.py
```

### 3. 启动八叉树地图
```bash
# 启动八叉树地图服务器
ros2 launch car_slam octomap_server_launch.py

# 启动相机八叉树建图
ros2 launch car_slam camera_octomap_launch.py
```

### 4. 可视化
```bash
# 显示点云地图
ros2 launch car_slam display_pcl_launch.py

# 显示八叉树地图
ros2 launch car_slam display_octomap_launch.py
```

### 5. 单独运行点云建图节点
```bash
# 运行点云建图节点
ros2 run car_slam point_cloud_main

# 设置参数
ros2 param set /pointcloud_mapping use_viewer true
ros2 param set /pointcloud_mapping resolution 0.05
```

## 主要功能

### 1. 点云生成
- 基于RGB-D图像生成彩色点云
- 支持实时点云生成和处理
- 自动去除无效点和噪声

### 2. 点云滤波
- 体素滤波降低点云密度
- 通过滤波器去除离群点
- 支持自定义滤波参数

### 3. 点云变换
- 基于相机位姿变换点云到全局坐标系
- 支持多帧点云融合
- 实时更新全局点云地图

### 4. 点云可视化
- 支持PCL可视化器实时显示
- 发布ROS点云消息供RViz显示
- 支持点云保存和加载

### 5. 八叉树建图
- 将点云转换为八叉树表示
- 支持占用概率更新
- 适用于路径规划和碰撞检测

## 依赖项
- ROS2 (机器人操作系统)
- PCL (点云处理库)
- OpenCV (计算机视觉库)
- Eigen3 (线性代数库)
- ORB-SLAM2/3 (视觉SLAM算法)
- OctoMap (八叉树地图库)
- message_filters (消息同步)
- cv_bridge (OpenCV与ROS图像转换)

## 注意事项
1. 确保RGB-D相机正常工作并已校准
2. 根据实际相机参数修改配置文件
3. 点云建图需要足够的计算资源
4. 建图过程中保持相机稳定移动
5. 定期保存点云地图以防数据丢失
6. 大规模点云建图时注意内存使用
7. 八叉树分辨率影响地图精度和存储空间 