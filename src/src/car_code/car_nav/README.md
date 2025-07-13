# ROS2 机器人导航包 README 文档

## 包名称
**car_nav**

## 功能描述
该包提供机器人导航和建图的完整解决方案，包括SLAM建图、路径规划、自主导航等功能。支持多种建图算法(Gmapping、Cartographer、RTABMap)和导航算法(DWA、TEB)。

## 节点列表

### 1. scan_filter (激光雷达滤波节点)
- **节点名称**: `scan_dilute`
- **可执行文件**: `scan_filter`
- **功能**: 激光雷达数据降采样和滤波处理

#### 发布接口
| 话题名称 | 消息类型 | 描述 |
|:---------|:---------|:-----|
| `/downsampled_scan` | `sensor_msgs/msg/LaserScan` | 发布降采样后的激光雷达数据 |

#### 订阅接口
| 话题名称 | 消息类型 | 回调函数 | 描述 |
|:---------|:---------|:---------|:-----|
| `/scan` | `sensor_msgs/msg/LaserScan` | `laserCallback` | 订阅原始激光雷达数据 |

#### 参数配置
| 参数名称 | 类型 | 默认值 | 描述 |
|:---------|:-----|:-------|:-----|
| `multiple` | int | 2 | 降采样倍数 |

## 启动文件

### 建图相关启动文件

#### 1. map_gmapping_launch.py
- **功能**: 启动Gmapping SLAM建图
- **支持的雷达类型**: A1、S2、4ROS
- **环境变量**: `RPLIDAR_TYPE`

#### 2. map_gmapping_a1_launch.py
- **功能**: 基于A1激光雷达的Gmapping建图

#### 3. map_gmapping_4ros_s2_launch.py
- **功能**: 基于4ROS/S2激光雷达的Gmapping建图

#### 4. map_cartographer_launch.py
- **功能**: 启动Cartographer SLAM建图
- **配置文件**: `lds_2d.lua`

#### 5. map_rtabmap_launch.py
- **功能**: 启动RTABMap SLAM建图
- **包含组件**:
  - 激光雷达启动
  - RTABMap同步节点

#### 6. save_map_launch.py
- **功能**: 保存建图结果
- **默认保存路径**: `maps/yahboomcar`

### 导航相关启动文件

#### 7. navigation_dwa_launch.py
- **功能**: 基于DWA算法的自主导航
- **参数文件**: `dwa_nav_params.yaml`
- **地图文件**: `maps/yahboomcar.yaml`

#### 8. navigation_teb_launch.py
- **功能**: 基于TEB算法的自主导航
- **参数文件**: `teb_nav_params.yaml`
- **地图文件**: `maps/yahboomcar.yaml`

#### 9. navigation_rtabmap_launch.py
- **功能**: 基于RTABMap的自主导航
- **包含组件**:
  - 激光雷达启动
  - RTABMap定位
  - RTABMap导航

#### 10. rtabmap_nav_launch.py
- **功能**: RTABMap导航系统
- **参数文件**: `rtabmap_nav_params.yaml`

### 定位相关启动文件

#### 11. rtabmap_localization_launch.py
- **功能**: RTABMap定位系统

#### 12. rtabmap_sync_launch.py
- **功能**: RTABMap数据同步

### 显示相关启动文件

#### 13. display_map_launch.py
- **功能**: 显示建图结果
- **RViz配置**: `rviz/map.rviz`

#### 14. display_nav_launch.py
- **功能**: 显示导航过程
- **RViz配置**: `rviz/nav.rviz`

#### 15. display_rtabmap_map_launch.py
- **功能**: 显示RTABMap建图结果
- **RViz配置**: `rviz/rtabmap_map.rviz`

#### 16. display_rtabmap_nav_launch.py
- **功能**: 显示RTABMap导航过程
- **RViz配置**: `rviz/rtabmap_nav.rviz`

### 其他启动文件

#### 17. laser_bringup_launch.py
- **功能**: 启动激光雷达驱动

#### 18. occupancy_grid_launch.py
- **功能**: 启动占用栅格地图

#### 19. rtabmap_viz_launch.py
- **功能**: RTABMap可视化

## 配置文件

### 导航参数文件

#### 1. dwa_nav_params.yaml
- **功能**: DWA导航算法参数配置
- **包含组件**:
  - bt_navigator (行为树导航器)
  - controller_server (控制器服务器)
  - local_costmap (局部代价地图)
  - global_costmap (全局代价地图)
  - planner_server (路径规划器)
  - map_server (地图服务器)

#### 2. teb_nav_params.yaml
- **功能**: TEB导航算法参数配置

#### 3. rtabmap_nav_params.yaml
- **功能**: RTABMap导航参数配置

### 建图参数文件

#### 4. lds_2d.lua
- **功能**: Cartographer 2D建图参数配置
- **主要参数**:
  - `map_frame`: "map"
  - `tracking_frame`: "base_footprint"
  - `published_frame`: "odom"
  - `use_odometry`: true
  - `num_laser_scans`: 1

## 地图文件

### 1. yahboomcar.yaml/yahboomcar.pgm
- **分辨率**: 0.05m/pixel
- **原点**: [-2.3, -8.95, 0]
- **占用阈值**: 0.65
- **空闲阈值**: 0.25

### 2. yahboomcar2.yaml/yahboomcar2.pgm
- **分辨率**: 0.05m/pixel
- **原点**: [-10, -10, 0]

## 使用说明

### 1. SLAM建图

#### Gmapping建图
```bash
# 设置激光雷达类型
export RPLIDAR_TYPE=a1

# 启动建图
ros2 launch car_nav map_gmapping_launch.py

# 保存地图
ros2 launch car_nav save_map_launch.py
```

#### Cartographer建图
```bash
# 启动Cartographer建图
ros2 launch car_nav map_cartographer_launch.py

# 保存地图
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/path/to/map.bag'}"
```

#### RTABMap建图
```bash
# 启动RTABMap建图
ros2 launch car_nav map_rtabmap_launch.py
```

### 2. 自主导航

#### DWA导航
```bash
# 启动DWA导航
ros2 launch car_nav navigation_dwa_launch.py
```

#### TEB导航
```bash
# 启动TEB导航
ros2 launch car_nav navigation_teb_launch.py
```

#### RTABMap导航
```bash
# 启动RTABMap导航
ros2 launch car_nav navigation_rtabmap_launch.py
```

### 3. 可视化

#### 查看建图过程
```bash
# 显示建图结果
ros2 launch car_nav display_map_launch.py
```

#### 查看导航过程
```bash
# 显示导航过程
ros2 launch car_nav display_nav_launch.py
```

### 4. 激光雷达滤波
```bash
# 启动激光雷达滤波节点
ros2 run car_nav scan_filter
```

## 依赖项
- ROS2 (机器人操作系统)
- Nav2 (导航功能包)
- Cartographer (谷歌SLAM算法)
- RTABMap (实时外观建图)
- Gmapping (SLAM建图算法)
- RViz2 (可视化工具)
- TF2 (坐标变换库)

## 注意事项
1. 建图前确保激光雷达正常工作
2. 建图过程中保持机器人缓慢移动
3. 导航前确保已有准确的地图
4. 根据环境特点选择合适的导航算法
5. 定期校准里程计和IMU数据
6. 注意机器人安全，避免碰撞障碍物 