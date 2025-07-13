# ROS2 机器人位姿发布包 README 文档

## 包名称
**robot_pose_publisher_ros2**

## 功能描述
该包提供机器人位姿发布功能，基于TF变换计算机器人在地图坐标系中的位置和姿态，并以标准ROS消息格式发布。主要用于机器人定位、导航状态监控和上层应用的位姿获取。

## 节点列表

### 1. robot_pose_publisher (机器人位姿发布节点)
- **节点名称**: `robot_pose_publisher`
- **可执行文件**: `robot_pose_publisher`
- **功能**: 实时发布机器人在地图坐标系中的位姿信息

#### 发布接口
| 话题名称 | 消息类型 | 描述 |
|:---------|:---------|:-----|
| `/robot_pose` | `geometry_msgs/msg/Pose` | 发布机器人位姿(非时间戳版本) |
| `/robot_pose` | `geometry_msgs/msg/PoseStamped` | 发布机器人位姿(时间戳版本) |

#### 订阅接口
无直接订阅接口，通过TF监听器获取坐标变换信息

#### 参数配置
| 参数名称 | 类型 | 默认值 | 描述 |
|:---------|:-----|:-------|:-----|
| `map_frame` | string | "map" | 地图坐标系名称 |
| `base_frame` | string | "base_link" | 机器人基座坐标系名称 |
| `is_stamped` | bool | false | 是否发布带时间戳的位姿消息 |

#### 定时器配置
- **发布频率**: 20Hz (每50ms发布一次)
- **定时器类型**: Wall Timer (实时定时器)

## 启动文件

### robot_pose_publisher_launch.py
- **功能**: 启动机器人位姿发布节点
- **包含组件**:
  - robot_pose_publisher节点
- **可配置参数**:
  - 地图坐标系名称
  - 基座坐标系名称
  - 输出消息类型

## 核心功能

### 1. TF监听
- 监听地图坐标系到机器人基座坐标系的变换
- 实时获取机器人位置和姿态信息
- 自动处理坐标变换异常

### 2. 位姿计算
- 基于TF变换计算机器人位姿
- 支持位置(x, y, z)和姿态(四元数)信息
- 提供高精度的位姿数据

### 3. 消息发布
- 支持两种消息格式：Pose和PoseStamped
- 可配置发布频率和消息类型
- 自动添加时间戳和坐标系信息

### 4. 异常处理
- 处理TF变换查找失败的情况
- 在坐标变换不可用时跳过发布
- 提供稳定的位姿数据流

## 使用说明

### 1. 启动位姿发布节点
```bash
# 使用默认参数启动
ros2 run robot_pose_publisher_ros2 robot_pose_publisher

# 使用启动文件启动
ros2 launch robot_pose_publisher_ros2 robot_pose_publisher_launch.py
```

### 2. 配置参数启动
```bash
# 自定义坐标系名称
ros2 run robot_pose_publisher_ros2 robot_pose_publisher \
  --ros-args \
  -p map_frame:=map \
  -p base_frame:=base_footprint \
  -p is_stamped:=true
```

### 3. 查看位姿信息
```bash
# 查看位姿消息(非时间戳版本)
ros2 topic echo /robot_pose

# 查看位姿消息(时间戳版本)
ros2 topic echo /robot_pose
```

### 4. 监控发布频率
```bash
# 查看话题发布频率
ros2 topic hz /robot_pose

# 查看话题信息
ros2 topic info /robot_pose
```

### 5. 参数动态配置
```bash
# 查看当前参数
ros2 param list /robot_pose_publisher

# 修改参数
ros2 param set /robot_pose_publisher map_frame odom
ros2 param set /robot_pose_publisher base_frame base_link
ros2 param set /robot_pose_publisher is_stamped true
```

## 消息格式

### geometry_msgs/msg/Pose
```yaml
position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
```

### geometry_msgs/msg/PoseStamped
```yaml
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "map"
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

## 应用场景

### 1. 机器人导航
- 为导航系统提供实时位姿信息
- 支持路径规划和轨迹跟踪
- 用于导航状态监控

### 2. 定位系统
- 与SLAM系统配合使用
- 提供机器人定位结果
- 支持多传感器融合定位

### 3. 数据记录
- 记录机器人运动轨迹
- 用于离线分析和评估
- 支持数据可视化

### 4. 上层应用
- 为应用层提供位姿接口
- 支持远程监控和控制
- 用于任务规划和执行

## 依赖项
- ROS2 (机器人操作系统)
- tf2_ros (坐标变换库)
- geometry_msgs (几何消息类型)
- rclcpp (ROS2 C++客户端库)

## 注意事项
1. 确保TF树完整，map到base_link的变换可用
2. 根据实际机器人配置修改坐标系名称
3. 合理设置发布频率，避免过高的CPU占用
4. 在SLAM或定位系统启动后再启动此节点
5. 注意坐标系的一致性，避免混淆不同的坐标系
6. 监控TF变换的时效性，避免使用过时的数据
7. 根据应用需求选择合适的消息类型(Pose或PoseStamped)